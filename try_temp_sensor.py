# Add dtoverlay=w1-gpio to /boot/firmware/config.txt (or /boot/firmware/usercfg.txt) and reboot the Pi before trying this.
# Make sure it is wired to GPIO4.
# Github: https://github.com/timofurrer/w1thermsensor

import time
from w1thermsensor import W1ThermSensor

sensor = W1ThermSensor()

while True:
    temperature = sensor.get_temperature()
    print(f"The temperature is {temperature}")
    time.sleep(1)

