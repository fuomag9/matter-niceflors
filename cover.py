import logging
import time
import yaml
import os
import json
import paho.mqtt.client as mqtt
import pigpio
import threading

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

# Button IDs for Nice Flor-S protocol
BUTTON_ID_OPEN = 1
BUTTON_ID_STOP = 2
BUTTON_ID_CLOSE = 4


class NextCodeEntity:
    def __init__(self, start_code: int, config_file: str) -> None:
        self._attr_native_value = start_code
        self._config_file = config_file

    def increase(self):
        self._attr_native_value = self._attr_native_value + 1
        self._save_to_config()

    def _save_to_config(self):
        """Save the current code value back to the config file."""
        try:
            with open(self._config_file, 'r') as file:
                config = yaml.safe_load(file)

            config[CONF_START_CODE] = self._attr_native_value

            with open(self._config_file, 'w') as file:
                yaml.dump(config, file, default_flow_style=False)

            _LOGGER.debug(f"Updated start_code to {self._attr_native_value}")
        except Exception as e:
            _LOGGER.error(f"Error saving NextCodeEntity value to config: {e}")


class RFDevice:
    def __init__(self, gpio: int, pi: pigpio.pi) -> None:
        self._pi = pi
        self._gpio = gpio
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
            pigpio.pulse(self._gpio, 0, self.tx_pulse_short),
            pigpio.pulse(0, self._gpio, self.tx_pulse_long),
        ]

    def tx_l1(self):
        return [
            pigpio.pulse(self._gpio, 0, self.tx_pulse_long),
            pigpio.pulse(0, self._gpio, self.tx_pulse_short),
        ]

    def tx_sync(self):
        return [
            pigpio.pulse(self._gpio, 0, self.tx_pulse_sync),
            pigpio.pulse(0, self._gpio, self.tx_pulse_sync),
        ]

    def tx_gap(self):
        return [
            pigpio.pulse(0, self._gpio, self.tx_pulse_gap),
        ]


class NiceBlindController:
    def __init__(self, config_file: str):
        # Load configuration
        with open(config_file, 'r') as file:
            self.config = yaml.safe_load(file)

        # RF Device Setup
        self.pi = pigpio.pi(self.config.get(CONF_PIGPIO_HOST, "127.0.0.1"))
        gpio_pin = self.config.get(CONF_GPIO, 27)
        self.rf_device = RFDevice(gpio=1 << gpio_pin, pi=self.pi)

        # Next Code Entity
        self.next_code_entity = NextCodeEntity(
            self.config.get(CONF_START_CODE, 0x111),
            config_file
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
        self.target_position = 100
        self.is_moving = False

    def _send_rf_command(self, button_id):
        """Send RF command with code increment."""
        current_code = int(self.next_code_entity._attr_native_value)

        # Perform repeated transmission
        for repeat in range(1, 7):
            tx_code = self._nice_flor_s_encode(
                self.serial_number,
                current_code,
                button_id,
                repeat
            )
            _LOGGER.info(
                f"Sending command: serial {hex(self.serial_number)}, "
                f"button_id {button_id}, code {current_code}"
            )
            self.rf_device.tx_code(tx_code)
            time.sleep(0.1)

        # Increment code
        self.next_code_entity.increase()

    def _nice_flor_s_encode(
            self, serial: int, code: int, button_id: int, repeat: int
    ) -> int:
        # Existing Nice Flor-S encoding logic from previous implementation
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

    def open_blind(self):
        """Open the blind fully."""
        self._send_rf_command(BUTTON_ID_OPEN)
        self.current_position = 100
        self._publish_state()

    def close_blind(self):
        """Close the blind fully."""
        self._send_rf_command(BUTTON_ID_CLOSE)
        self.current_position = 0
        self._publish_state()

    def stop_blind(self):
        """Stop the blind's current movement."""
        self._send_rf_command(BUTTON_ID_STOP)
        # Retain the last known position
        self._publish_state()

    def set_position(self, position: int):
        """Set blind to a specific position."""
        if position < 0 or position > 100:
            _LOGGER.error(f"Invalid position: {position}")
            return

        # Determine the sequence of commands to reach the desired position
        if position == 100:
            self.open_blind()
        elif position == 0:
            self.close_blind()
        else:
            # If already at desired position, do nothing
            if abs(self.current_position - position) < 5:
                return

            # Strategy to reach an intermediate position
            if position > self.current_position:
                # Need to open more
                self.open_blind()
            else:
                # Need to close more
                self.close_blind()

            # Stop at approximate position
            time.sleep(abs(self.current_position - position) * 0.5)
            self.stop_blind()

            # Update current position
            self.current_position = position
            self._publish_state()

    def _on_mqtt_connect(self, client, userdata, flags, reason_code, properties=None):
        """Handle MQTT connection."""
        if reason_code == 0:
            _LOGGER.info("Connected to MQTT Broker")

            # Subscribe to command topic
            command_topic = f"{self.mqtt_topic_base}/set"
            client.subscribe(command_topic)

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
            state = "open"
        elif self.current_position == 0:
            state = "closed"
        else:
            state = "partially_open"

        self.mqtt_client.publish(state_topic, state)
        self.mqtt_client.publish(position_topic, str(self.current_position))

    def _on_mqtt_message(self, client, userdata, message):
        """Handle incoming MQTT messages."""
        topic = message.topic
        payload = message.payload.decode()

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
            try:
                position = int(payload)
                self.set_position(position)
            except ValueError:
                _LOGGER.error(f"Invalid position value: {payload}")

    # ... (rest of the methods remain the same as in the previous implementation)

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

    def cleanup(self):
        """Cleanup resources."""
        self.mqtt_client.disconnect()
        self.pi.stop()


def main():
    # Configuration file path
    config_file = os.path.join(os.path.dirname(__file__), 'nice_blind_config.yaml')

    # Create blind controller
    blind_controller = NiceBlindController(config_file)

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