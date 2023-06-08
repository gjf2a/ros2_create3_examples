# Adapted from: https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/

import gpiod
import time

# trigger is pin 8 - chip 1, line 91
# echo is pin 10 - chip 1, line 92

max_wait = 1


if __name__ == '__main__':

    with gpiod.Chip('gpiochip1') as chip:
        trg_line = chip.get_line(91)
        trg_line.request(consumer='hi', type=gpiod.LINE_REQ_DIR_OUT)
        ech_line = chip.get_line(92)
        ech_line.request(consumer='bye', type=gpiod.LINE_REQ_DIR_IN)

        while True:
            trg_line.set_value(1)
            time.sleep(0.00001)
            trg_line.set_value(0)
            
            init = start = stop = time.time()

            while ech_line.get_value() == 0 and time.time() - init < max_wait:
                start = time.time()

            while ech_line.get_value() == 1 and time.time() - init < max_wait:
                stop = time.time()


            duration = stop - start
            distance = (duration * 34300) / 2

            print(distance)

