#!/usr/bin/env python3
# Adapted from: https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/
# GPIO code from: https://hub.libre.computer/t/how-to-control-gpio-via-python-3/601

import gpiod
import time
import sys

DEFAULT_TIMEOUT = 0.0075
PING_SLEEP = 0.00001
SPEED_OF_SOUND = 34300

def chip_from_num(num):
    return f'gpiochip{num}'

class Sonar:
    """
    Each Sonar object corresponds to a physical sonar. 

    Attributes:
        chip (int): which GPIO chip is used for the trigger and echo pins
                    Note: Both pins have to be on the same chip
        trig_line (int): Trigger pin line number
        echo_line (int): Echo pin line number
        timeout (float): How much time elapses before we assume no ping 
                         will be heard by the echo.

        Pins:
        *  7: 1 98
        *  8: 1 91
        * 10: 1 92
        * 11: 0  8
        * 12: 0  6
        * 13: 0  9
        * 15: 0 10
        * 16: 1 93
        * 18: 1 94
        * 22: 1 79
        * 29: 1 96
        * 31: 1 97
        * 32: 1 95
        * 33: 1 85
        * 35: 1 86
        * 36: 1 81
        * 37: 1 84
        * 38: 1 82
        * 40: 1 83

        Current circuit:
        *  8, 10 -> 1 91 92
        * 11, 13 -> 1 93 94
        * 29, 31 -> 1 96 97
        
    """

    def __init__(self, chip, trig_line, echo_line, timeout=DEFAULT_TIMEOUT):
        self.chip = chip
        self.trig_line = trig_line
        self.echo_line = echo_line
        self.timeout = timeout

    def read(self):
        """Read this Sonar once, returning a distance in centimeters."""
        with gpiod.Chip(chip_from_num(self.chip)) as chip:
            trig_line = self.get_line(chip, self.trig_line, gpiod.LINE_REQ_DIR_OUT)
            echo_line = self.get_line(chip, self.echo_line, gpiod.LINE_REQ_DIR_IN)

            self.send_ping(trig_line)
            duration = self.listen_for_return(echo_line)

            return duration * SPEED_OF_SOUND / 2

    def get_line(self, chip, line_num, req_dir):
        line = chip.get_line(line_num)
        line.request(consumer=sys.argv[0], type=req_dir)
        return line

    def send_ping(self, trig_line):
        trig_line.set_value(1)
        time.sleep(PING_SLEEP)
        trig_line.set_value(0)

    def listen_for_return(self, echo_line):
        init = start = stop = time.time()
        while echo_line.get_value() == 0 and time.time() - init < self.timeout:
            start = time.time()

        while echo_line.get_value() == 1 and time.time() - init < self.timeout:
            stop = time.time()

        return stop - start


class SonarArray:
    def __init__(self, sonar_list=None):
        self.sonar_list = [] if sonar_list is None else sonar_list

    def add(self, sonar):
        self.sonar_list.append(sonar)

    def read(self):
        return [sonar.read() for sonar in self.sonar_list]


if __name__ == '__main__':
    if len([arg for arg in sys.argv if arg.startswith("-s")]) == 0:
        print("Usage: sonar.py [-s:chip:trig_line:echo_line] [-t:timeout] [-p:num_pings]")
    else:
        sonars = SonarArray()
        timeout = DEFAULT_TIMEOUT
        num_pings = None
        for arg in sys.argv:
            if arg.startswith("-t:"):
                timeout = float(arg[3:])
            elif arg.startswith("-p:"):
                num_pings = int(arg[3:])
            elif arg.startswith("-s:"):
                tag, chip, trig_line, echo_line = arg.split(":")
                sonars.add(Sonar(int(chip), int(trig_line), int(echo_line), timeout))

        count = 0
        while num_pings is None or count < num_pings:
            count += 1
            print(sonars.read())
            time.sleep(0.2)
