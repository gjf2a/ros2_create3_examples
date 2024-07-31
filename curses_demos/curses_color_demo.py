import curses

def main(stdscr):
    curses.curs_set(0)
    stdscr.clear()

    color_list = [curses.COLOR_BLACK, curses.COLOR_RED, curses.COLOR_GREEN, curses.COLOR_YELLOW, curses.COLOR_BLUE, curses.COLOR_MAGENTA, curses.COLOR_CYAN, curses.COLOR_WHITE]

    for i, color in enumerate(color_list):
        curses.init_pair(i + 1, color, color_list[(i + 1) % len(color_list)])

    keystrokes = 0

    stdscr.addstr(0, 0, 'Enter q to quit', curses.color_pair(1))
    stdscr.addstr(1, 50, 'huh?', curses.color_pair(3))
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        else:
            keystrokes += 1
            stdscr.addstr(1, 0, f"keystrokes: {keystrokes} ({k}) {' ' * 20}", curses.color_pair(5)) 
            if k.startswith("KEY"):
                stdscr.addstr(2, 0, "Non-printable keystroke", curses.color_pair(7))
            else: 
                stdscr.addstr(2, 0, ' ' * 30)
            stdscr.refresh()
    

if __name__ == '__main__':
    curses.wrapper(main)
