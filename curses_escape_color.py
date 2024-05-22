import curses

def main(stdscr):
    curses.curs_set(0)
    stdscr.clear()

    keystrokes = 0

    stdscr.addstr(0, 0, '\033[31mEnter q to quit')
    stdscr.addstr(1, 50, '\033[92mhuh?')
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        else:
            keystrokes += 1
            stdscr.addstr(1, 0, f"\033[35mkeystrokes: {keystrokes} ({k}) {' ' * 20}") 
            if k.startswith("KEY"):
                stdscr.addstr(2, 0, "\033[36mNon-printable keystroke")
            else: 
                stdscr.addstr(2, 0, ' ' * 30)
            stdscr.refresh()
    

if __name__ == '__main__':
    curses.wrapper(main)
