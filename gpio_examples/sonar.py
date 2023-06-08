#!/usr/bin/env python3
# Adapted from: https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/

import gpiod
import time
import sys


SPEED_OF_SOUND = 34300

def chip_from_num(num):
    return f'gpiochip{num}'

class Sonar:
    def __init__(self, chip, trig_line, echo_line, timeout):
        self.chip = chip
        self.trig_line = trig_line
        self.echo_line = echo_line
        self.timeout = timeout


    def read(self):
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
        time.sleep(0.00001)
        trig_line.set_value(0)

    def listen_for_return(self, echo_line):
        init = start = stop = time.time()
        while echo_line.get_value() == 0 and time.time() - init < self.timeout:
            start = time.time()

        while echo_line.get_value() == 1 and time.time() - init < self.timeout:
            stop = time.time()

        return stop - start


if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("Usage: sonar.py chip trig_line echo_line timeout [num_pings]")
    else:
        sonar = Sonar(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), float(sys.argv[4]))
        num_measures = None if len(sys.argv) < 6 else int(sys.argv[5])
        count = 0
        while num_measures is None or count < num_measures:
            count += 1
            print(sonar.read())
