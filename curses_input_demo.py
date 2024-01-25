from curses_menu_demo import MenuItems
import curses


# From: https://stackoverflow.com/questions/21784625/how-to-input-a-word-in-ncurses-screen

def main(stdscr):
    stdscr = curses.initscr()
    stdscr.clear()
    choice = my_raw_input(stdscr, 2, 3, "cool or hot?").lower().strip()
    choice = choice.decode('utf-8')
    stdscr.addstr(1, 0, f"Entered [{choice}]")
    if choice == "cool":
        stdscr.addstr(5,3,"Super cool!")
    elif choice == "hot":
        stdscr.addstr(5, 3," HOT!") 
    else:
        stdscr.addstr(5, 3," Invalid input") 
    stdscr.refresh()
    stdscr.getch()

def my_raw_input(stdscr, row, col, prompt_string):
    curses.echo() 
    stdscr.addstr(row, col, prompt_string)
    stdscr.refresh()
    text = stdscr.getstr(row + 1, col, 20)
    return text       
    

if __name__ == '__main__':
    curses.wrapper(main)
