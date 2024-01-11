from curses import wrapper, curs_set

def main(stdscr):
    curs_set(0)
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
            stdscr.addstr(1, 0, f"keystrokes: {keystrokes} ({k}) {' ' * 20}") 
            if k.startswith("KEY"):
                stdscr.addstr(2, 0, "Non-printable keystroke")
            else: 
                stdscr.addstr(2, 0, ' ' * 30)
            stdscr.refresh()
    

if __name__ == '__main__':
    wrapper(main)
