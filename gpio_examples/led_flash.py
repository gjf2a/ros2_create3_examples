import gpiod
import time

led = (0, 6)

if __name__ == '__main__':

    with gpiod.Chip(f'gpiochip{led[0]}') as chip:
        line = chip.get_line(led[1])
        line.request(consumer='hello', type=gpiod.LINE_REQ_DIR_OUT)
        for i in range(4):
            line.set_value(1)
            print("on")
            time.sleep(1)
            line.set_value(0)
            print("off")
            time.sleep(1)
