<img src="https://www.home-assistant.io/images/home-assistant-logo.svg" alt="Home Assistant" width="200"/><br>
This integration allows you to control Nice Flor-S shutters with a Raspberry Pi and a 433 MHz transmitter via MQTT on home assistant

## Installation 
1. Install requirements.txt
2. Buy FS1000A and connect to GRND, 5V and GPIO pin 17 (or use another and change the config)
2. Pair remote by following the flor-s-programming-instructions.pdf (use a flipper zero or follow the master repository or do a hack, if you are using this you know how)
6. It will now work, take note that it will automatically open when service is started to sync the status (as there is no way for the software to retrieve it afaik)
## Troubleshooting
- Make sure that the GPIO pin is correctly configured
- Make sure that the transmitter is correctly connected to the GPIO pin
- Make sure that the shutter is in pairing mode
- Make sure that the serial number of the shutter is correct
- Make sure that the code is correctly transmitted (debug via flipper zero/SDR)
## Issues
If you encounter any issues, please create an issue on GitHub.
## License
This integration is licensed under the MIT License. Many thanks to https://github.com/niklas-sparfeld/homeassistant-niceflors for the original code

## Matter
I wanted to use matter for this project but as far as I am aware there is no python sdk for making this a standalone device, I will try with C++ and call python code directly (I am absolutely NOT rewriting it in C++) but it's a WIP for now
