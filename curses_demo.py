from curses import wrapper
from sub_odom import OdometrySubscriber

def main(stdscr):
    stdscr.clear()

    keystrokes = 0

    stdscr.addstr(0, 0, 'Enter q to quit')
    stdscr.addstr(1, 50, 'huh?')
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        else:
            keystrokes += 1
            stdscr.addstr(1, 0, f"keystrokes: {keystrokes}") 
            stdscr.refresh()
    

if __name__ == '__main__':
    wrapper(main)
