import json
import logging
import threading
import time
from pathlib import Path
from time import sleep

import paho.mqtt.client as mqtt
import pigpio
import yaml

from encode_tables import NICE_FLOR_S_TABLE_KI, NICE_FLOR_S_TABLE_ENCODE

# Logging setup
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
_LOGGER = logging.getLogger(__name__)

# Constants
CONF_GPIO = "gpio"
CONF_SERIAL = "serial"
CONF_PIGPIO_HOST = "pigpio_host"
CONF_START_CODE = "start_code"
CONF_MQTT = "mqtt"
CONF_TIME_FROM_OPEN_TO_CLOSE = "time_from_open_to_close"

# Button IDs for Nice Flor-S protocol
BUTTON_ID_OPEN = 1
BUTTON_ID_STOP = 2
BUTTON_ID_CLOSE = 4
GPIO_NONE = 0


class NextCodeEntity:
    def __init__(self, start_code: int, config_file: str,code_file: str) -> None:
        self._attr_native_value = start_code
        self._config_file = config_file
        self._code_file = code_file

    def increase(self):
        self._attr_native_value = self._attr_native_value + 1
        self._save_to_config()

    def _save_to_config(self):
        try:
            with open(self._code_file, "w") as code_file:
                code_file.write(str(self._attr_native_value))
            _LOGGER.debug(f"Updated start_code to {self._attr_native_value}")
        except Exception as e:
            _LOGGER.error(f"Error saving NextCodeEntity value to config: {e}")


class RFDevice:
    def __init__(
        self,
        gpio: int,
        pi: pigpio,
    ) -> None:
        self._pi = pi
        self._gpio = gpio
        #self.tx_pulse_short = 250  # Halved values
        #self.tx_pulse_long = 500   # Halved values
        #self.tx_pulse_sync = 750   # Halved values
        #self.tx_pulse_gap = 7500   # Halved values
        self.tx_pulse_short = 500
        self.tx_pulse_long = 1000
        self.tx_pulse_sync = 1500
        self.tx_pulse_gap = 15000
        self.tx_length = 52

    def tx_code(self, code: int):
        wf = []
        wf.extend(self.tx_sync())
        rawcode = format(code, "#0{}b".format(self.tx_length))[2:]
        for bit in range(0, self.tx_length):
            if rawcode[bit] == "1":
                wf.extend(self.tx_l0())
            else:
                wf.extend(self.tx_l1())
        wf.extend(self.tx_sync())
        wf.extend(self.tx_gap())

        while self._pi.wave_tx_busy():
            time.sleep(0.1)

        self._pi.wave_clear()
        self._pi.wave_add_generic(wf)
        wave = self._pi.wave_create()
        self._pi.wave_send_once(wave)

        while self._pi.wave_tx_busy():
            time.sleep(0.1)

        self._pi.wave_delete(wave)

    def tx_l0(self):
        return [
            pigpio.pulse(self._gpio, GPIO_NONE, self.tx_pulse_short),
            pigpio.pulse(GPIO_NONE, self._gpio, self.tx_pulse_long),
        ]

    def tx_l1(self):
        return [
            pigpio.pulse(self._gpio, GPIO_NONE, self.tx_pulse_long),
            pigpio.pulse(GPIO_NONE, self._gpio, self.tx_pulse_short),
        ]

    def tx_sync(self):
        return [
            pigpio.pulse(self._gpio, GPIO_NONE, self.tx_pulse_sync),
            pigpio.pulse(GPIO_NONE, self._gpio, self.tx_pulse_sync),
        ]

    def tx_gap(self):
        return [
            pigpio.pulse(GPIO_NONE, self._gpio, self.tx_pulse_gap),
        ]


class NiceBlindController:
    def __init__(self, config_file: str, code_file: str) -> None:
        # Load configuration
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        # RF Device Setup
        self.pi = pigpio.pi(self.config.get(CONF_PIGPIO_HOST, "127.0.0.1"))
        gpio_pin = self.config.get(CONF_GPIO, 27)
        self.rf_device = RFDevice(gpio=1 << gpio_pin, pi=self.pi)

        if Path(code_file).is_file():
            if Path(code_file).read_text() != "":
                # Next Code Entity
                self.next_code_entity = NextCodeEntity(
                    int(Path(code_file).read_text()),
                    config_file,
                    code_file
                )
            else:
                # Next Code Entity
                self.next_code_entity = NextCodeEntity(
                    self.config.get(CONF_START_CODE, 0x111),
                    config_file,
                    code_file
                )
        else:
            # Next Code Entity
            self.next_code_entity = NextCodeEntity(
                self.config.get(CONF_START_CODE, 0x111),
                config_file,
                code_file
            )

        # Serial Number
        self.serial_number = self.config.get(CONF_SERIAL, 0xabcdef0)

        # MQTT Configuration
        mqtt_config = self.config.get(CONF_MQTT, {})
        self.mqtt_broker = mqtt_config.get('broker', 'localhost')
        self.mqtt_port = mqtt_config.get('port', 1883)
        self.mqtt_topic_base = mqtt_config.get('topic_base', 'homeassistant/cover/nice_blind')

        # MQTT Client Setup - Updated for new Paho MQTT version
        self.mqtt_client = mqtt.Client(
            client_id=f"nice_blind_controller_{gpio_pin}",
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2
        )
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message

        # Current state tracking
        self.current_position = 100  # Fully open
        self.single_button_pressed = False
        self.startup_time : float

    def send(self, serial: int, button_id: int):
            code = int(self.next_code_entity._attr_native_value)
            self._send_repeated(serial, button_id, code)
            self.next_code_entity.increase()
            time.sleep(0.5)

    def _send_repeated(self, serial: int, button_id: int, code: int):
        for repeat in range(1, 7):
            tx_code = self._nice_flor_s_encode(serial, code, button_id, repeat)
            _LOGGER.info(
                "serial %s, button_id %i, code %i, tx_code %s",
                hex(serial),
                button_id,
                code,
                hex(tx_code),
            )
            self.rf_device.tx_code(tx_code)

    def _nice_flor_s_encode(
        self, serial: int, code: int, button_id: int, repeat: int
    ) -> int:
        snbuff = [None] * 4
        snbuff[0] = serial & 0xFF
        snbuff[1] = (serial & 0xFF00) >> 8
        snbuff[2] = (serial & 0xFF0000) >> 16
        snbuff[3] = (serial & 0xFF000000) >> 24

        encbuff = [None] * 7
        enccode = NICE_FLOR_S_TABLE_ENCODE[code]
        ki = NICE_FLOR_S_TABLE_KI[code & 0xFF] ^ (enccode & 0xFF)

        encbuff[0] = button_id & 0x0F
        encbuff[1] = ((repeat ^ button_id ^ 0x0F) << 4) | ((snbuff[3] ^ ki) & 0x0F)
        encbuff[2] = enccode >> 8
        encbuff[3] = enccode & 0xFF
        encbuff[4] = snbuff[2] ^ ki
        encbuff[5] = snbuff[1] ^ ki
        encbuff[6] = snbuff[0] ^ ki

        encoded = 0
        encoded |= ((encbuff[6] << 0x4) & 0xF0) << 0
        encoded |= (((encbuff[5] & 0x0F) << 4) | ((encbuff[6] & 0xF0) >> 4)) << 8
        encoded |= (((encbuff[4] & 0x0F) << 4) | ((encbuff[5] & 0xF0) >> 4)) << 16
        encoded |= (((encbuff[3] & 0x0F) << 4) | ((encbuff[4] & 0xF0) >> 4)) << 24
        encoded |= (((encbuff[2] & 0x0F) << 4) | ((encbuff[3] & 0xF0) >> 4)) << 32
        encoded |= (((encbuff[1] & 0x0F) << 4) | ((encbuff[2] & 0xF0) >> 4)) << 40
        encoded |= (((encbuff[0] & 0x0F) << 4) | ((encbuff[1] & 0xF0) >> 4)) << 48
        encoded = encoded ^ 0xFFFFFFFFFFFFF0
        encoded = encoded >> 4

        return encoded

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """Handle MQTT connection."""
        if reason_code == 0:
            _LOGGER.info("Connected to MQTT Broker")

            # Subscribe to command topic
            client.subscribe(f"{self.mqtt_topic_base}/set")
            client.subscribe(f"{self.mqtt_topic_base}/set_position")
            # Publish discovery information for Home Assistant
            self._publish_discovery()

            # Publish initial state
            self._publish_state()
        else:
            _LOGGER.error(f"Failed to connect to MQTT Broker. Reason: {reason_code}")

    def _publish_discovery(self):
        """Publish MQTT discovery information for Home Assistant."""
        discovery_topic = f"{self.mqtt_topic_base}/config"
        discovery_payload = {
            "name": "Nice Blind",
            "unique_id": f"nice_blind_{self.config.get(CONF_GPIO, 27)}",
            "device_class": "blind",
            "command_topic": f"{self.mqtt_topic_base}/set",
            "state_topic": f"{self.mqtt_topic_base}/state",
            "position_topic": f"{self.mqtt_topic_base}/position",
            "set_position_topic": f"{self.mqtt_topic_base}/set_position"
        }

        self.mqtt_client.publish(discovery_topic, json.dumps(discovery_payload), retain=True)

    def _publish_state(self):
        """Publish current state to MQTT."""
        state_topic = f"{self.mqtt_topic_base}/state"
        position_topic = f"{self.mqtt_topic_base}/position"

        # Determine state based on current position
        if self.current_position == 100:
            state = "closed"
        elif self.current_position == 0:
            state = "open"
        else:
            state = "partially_open"

        self.mqtt_client.publish(state_topic, state)
        self.mqtt_client.publish(position_topic, str(self.current_position))

    def _on_mqtt_message(self, client, userdata, message):
        """Handle incoming MQTT messages."""
        topic = message.topic
        payload = message.payload.decode()
        thingy_time = int(self.config.get(CONF_TIME_FROM_OPEN_TO_CLOSE, 0))
        if (time.time() - self.startup_time < thingy_time) and thingy_time > 0:
            return #do not process commands when booting up


        _LOGGER.info(f"Received message on {topic}: {payload}")

        # Handle different command topics
        if topic.endswith("/set"):
            if payload == "OPEN":
                self.open_blind()
            elif payload == "CLOSE":
                self.close_blind()
            elif payload == "STOP":
                self.stop_blind()

        # Handle set position topic
        elif topic.endswith("/set_position"):
            thingy_time = int(self.config.get(CONF_TIME_FROM_OPEN_TO_CLOSE, 0))
            is_position_enabled = thingy_time > 0
            if not is_position_enabled:
                return
            try:
                position = int(payload)
                self.set_position(position,thingy_time)
            except ValueError:
                _LOGGER.error(f"Invalid position value: {payload}")

    def start(self):
        """Start the MQTT client and connect to broker."""
        # Connect to MQTT broker
        mqtt_config = self.config.get(CONF_MQTT, {})
        if 'username' in mqtt_config and 'password' in mqtt_config:
            self.mqtt_client.username_pw_set(
                mqtt_config['username'],
                mqtt_config['password']
            )

        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)

        # Start MQTT loop in a separate thread
        mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever)
        mqtt_thread.daemon = True
        mqtt_thread.start()

        #going to up position at boot
        self.close_blind()
        self.startup_time = time.time()


    def cleanup(self):
        """Cleanup resources."""
        self.mqtt_client.disconnect()
        self.pi.stop()

    def close_blind(self, no_change_position=False):
        if not no_change_position:
            self.current_position = 100
        self._send_repeated(self.serial_number,BUTTON_ID_CLOSE,self.next_code_entity._attr_native_value)
        self._publish_state()
        self.next_code_entity.increase()
        self.single_button_pressed = True

    def open_blind(self, no_change_position=False):
        if not no_change_position:
            self.current_position = 0
        self._send_repeated(self.serial_number,BUTTON_ID_OPEN,self.next_code_entity._attr_native_value)
        self._publish_state()
        self.next_code_entity.increase()
        self.single_button_pressed = True

    def stop_blind(self):
        self._send_repeated(self.serial_number,BUTTON_ID_STOP,self.next_code_entity._attr_native_value)
        self._publish_state()
        self.next_code_entity.increase()
        self.single_button_pressed = True

    def set_position(self, position, thingy_time):
        if self.single_button_pressed:
            self.close_blind() # fully open to reset unknown state
        delta = abs(self.current_position - position)
        time_to_execute = (delta * thingy_time) / 100
        if position > self.current_position:
            self.close_blind(no_change_position=True)
            time.sleep(time_to_execute)
            self.stop_blind()
        else:
            self.open_blind(no_change_position=True)
            time.sleep(time_to_execute)
            self.stop_blind()
        self.current_position = position
        self._publish_state()
        self.single_button_pressed = False



def main():
    # Configuration file path
    config_file = Path.cwd() / "nice_blind_config.yaml"
    code_file = Path.cwd() / "code.txt"
    code_file.touch(exist_ok=True)

    # Create blind controller
    blind_controller = NiceBlindController(str(config_file),str(code_file))

    try:
        # Start the blind controller
        blind_controller.start()

        # Keep the main thread running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        _LOGGER.info("Shutting down...")
    finally:
        blind_controller.cleanup()


if __name__ == "__main__":
    main()