import subprocess
import time

def lgpio(*args):
    subprocess.call(('lgpio',) + args)

led = 12

if __name__ == '__main__':
    for i in range(4):
        print("on")
        lgpio('set', f'{led}=1')
        time.sleep(1)
        print("off")
        lgpio('set', f'{led}=0')
        time.sleep(1)
