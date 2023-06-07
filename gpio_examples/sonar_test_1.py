import subprocess
import time


def lgpio(*args):
    return subprocess.call(('lgpio',) + args)


if __name__ == '__main__':

    trg = 8
    ech = 10
    #lgpio('mode', trg, 'out')
    #lgpio('mode', ech, 'in')

    for i in range(4):
        lgpio('set', f'{trg}=1')
        time.sleep(0.00001)
        lgpio('set', f'{trg}=0')
        
        start = time.time()
        while lgpio('get', f'{ech}') == 0 and time.time() - start < 1:
            pass

        stop = time.time()
        print(f'{stop - start}')
