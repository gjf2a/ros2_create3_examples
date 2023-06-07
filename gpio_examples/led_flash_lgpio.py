import subprocess
import time

if __name__ == '__main__':
    for i in range(4):
        print("on")
        subprocess.call(['lgpio', 'set', '8=1'])
        time.sleep(1)
        print("off")
        subprocess.call(['lgpio', 'set', '8=0'])
        time.sleep(1)
