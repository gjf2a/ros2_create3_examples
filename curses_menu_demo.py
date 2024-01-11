from curses import wrapper, curs_set, A_REVERSE

items = ['bell', 'book', 'candle', 'exit']

def main(stdscr):
    curs_set(0)
    stdscr.clear()

    current = 0

    while True:
        stdscr.clear()
        for i, item in enumerate(items):
            if current == i:
                stdscr.addstr(i, 0, item, A_REVERSE)
            else:
                stdscr.addstr(i, 0, item)
        stdscr.refresh()

        key = stdscr.getkey()
        if key == '\n' and current == len(items) - 1:
            break
        elif key == 'KEY_DOWN':
            current = (current + 1) % len(items)
        elif key == 'KEY_UP':
            current = (current - 1) % len(items)
    

if __name__ == '__main__':
    wrapper(main)
