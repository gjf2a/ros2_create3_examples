from curses import wrapper, curs_set, A_REVERSE, error


class MenuItems:
    def __init__(self, start_row, items):
        self.items = items
        self.start_row = start_row
        self.current = 0
        self.longest = max(len(item) for item in items)

    def show(self, stdscr):
        for i, item in enumerate(self.items):
            item = f"{item}{' ' * (self.longest - len(item))}"
            if self.current == i:
                stdscr.addstr(self.start_row + i, 0, item, A_REVERSE)
            else:
                stdscr.addstr(self.start_row + i, 0, item)

    def update_from_key(self, key):
        if key == '\n':
            return self.items[self.current]
        elif key == 'KEY_DOWN':
            self.current = (self.current + 1) % len(self.items)
        elif key == 'KEY_UP':
            self.current = (self.current - 1) % len(self.items)



def main(stdscr):
    curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    menu = MenuItems(3, ['bell', 'book', 'candle', 'exit'])
    update = True

    stdscr.clear()
    while True:
        if update:
            menu.show(stdscr)
            stdscr.refresh()
            update = False

        try:
            key = stdscr.getkey()
            update = True
            selection = menu.update_from_key(key)
            if selection == 'exit':
                break
        except error:
            stdscr.addstr(0, 0, "caught no-key")
    

if __name__ == '__main__':
    wrapper(main)
