from __future__ import annotations

import logging
import pigpio
import time
import yaml
import os

from threading import Lock

from encode_tables import NICE_FLOR_S_TABLE_KI, NICE_FLOR_S_TABLE_ENCODE

_LOGGER = logging.getLogger(__name__)

CONF_GPIO = "gpio"
CONF_SERIAL = "serial"
CONF_PIGPIO_HOST = "pigpio_host"
CONF_START_CODE = "start_code"
CONF_CONFIG_FILE = "config_file"
DEVICE_CLASS = "shutter"

GPIO_NONE = 0
BUTTON_ID_OPEN = 1
BUTTON_ID_STOP = 2
BUTTON_ID_CLOSE = 4


class RestoreNumber:
    pass


class NextCodeEntity(RestoreNumber):
    def __init__(self, start_code: int, config_file: str) -> None:
        self._attr_unique_id = "next_code"
        self._attr_native_value = start_code
        self._attr_icon = "mdi:remote"
        self._attr_name = "Nice Flor-S Next Code"
        self._config_file = config_file

    def increase(self):
        self._attr_native_value = self._attr_native_value + 1
        # Save the updated code to the yaml file
        self._save_to_config()

    def _save_to_config(self):
        """Save the current code value back to the config file."""
        if not self._config_file:
            _LOGGER.warning("No config file specified for saving NextCodeEntity value")
            return

        try:
            # Load the current config
            with open(self._config_file, 'r') as file:
                config = yaml.safe_load(file)

            # Update the start_code value
            config[CONF_START_CODE] = self._attr_native_value

            # Write back to the file
            with open(self._config_file, 'w') as file:
                yaml.dump(config, file, default_flow_style=False)

            _LOGGER.debug(f"Updated {CONF_START_CODE} to {self._attr_native_value} in {self._config_file}")
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


class PigpioNotConnected(BaseException):
    """Exception raised when connection to pigpio daemon fails."""


class NiceHub:
    def __init__(
            self,
            pigpio_host: str | None,
            gpio: int,
            next_code: NextCodeEntity,
    ) -> None:
        self._gpio = gpio
        self._next_code = next_code
        self._lock = Lock()

        self._pi = pigpio.pi(pigpio_host)
        _LOGGER.info("Connecting to pigpio on %s, please wait...", pigpio_host)

        i = 0
        while not self._pi.connected:
            i += 1
            _LOGGER.info("...")
            time.sleep(1)
            if i > 9:
                raise PigpioNotConnected()
        self._pi.set_mode(gpio, pigpio.OUTPUT)

        rfdevice = RFDevice(gpio=1 << gpio, pi=self._pi)
        self._rfdevice = rfdevice

    def cleanup(self):
        with self._lock:
            self._pi.stop()

    def send(self, serial: int, button_id: int):
        with self._lock:
            code = int(self._next_code._attr_native_value)
            self._send_repeated(serial, button_id, code)
            self._next_code.increase()
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
            self._rfdevice.tx_code(tx_code)

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


def load_config(config_file: str) -> dict:
    """Load configuration from YAML file."""
    try:
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        return config
    except Exception as e:
        _LOGGER.error(f"Error loading config file {config_file}: {e}")
        return {}


def main():
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Default config file location
    config_file = os.path.join(os.path.dirname(__file__), 'nice_config.yaml')

    # Load config
    config = load_config(config_file)

    # Get values from config with defaults
    pigpio_host = config.get(CONF_PIGPIO_HOST, "127.0.0.1")
    gpio_pin = config.get(CONF_GPIO, 27)
    serial_number = config.get(CONF_SERIAL, 0xdeadbef)
    start_code = config.get(CONF_START_CODE, 0x0)

    # Initialize the NextCodeEntity with the config file for persistence
    next_code_entity = NextCodeEntity(start_code, config_file)

    # Initialize the hub
    hub = NiceHub(pigpio_host, gpio_pin, next_code_entity)

    try:
        # Example: send STOP command
        hub.send(serial_number, BUTTON_ID_CLOSE)
        _LOGGER.info(f"Command sent successfully. Next code will be: {next_code_entity._attr_native_value}")
    finally:
        # Clean up
        hub.cleanup()


if __name__ == "__main__":
    main()