import threading, subprocess, sys, math, curses, pickle, datetime

from pyhop_anytime import *

from queue import Queue

graph = Graph()
last_position = (1, 2)


# From: https://stackoverflow.com/questions/21784625/how-to-input-a-word-in-ncurses-screen
def my_raw_input(stdscr, row, col, prompt_string):
    curses.echo()
    stdscr.addstr(row, col, prompt_string)
    stdscr.refresh()
    text = stdscr.getstr(row, col + len(prompt_string) + 1, 20)
    return text


def main(stdscr):
    global graph, last_position

    curses.curs_set(0)
    stdscr.clear()

    stdscr.addstr(0, 0, 'WASD to move; R to reset position; X to record location; Q to quit')
    stdscr.refresh()
    
    while True:
        k = stdscr.getkey()
        if k == 'q':
            break
        elif k == 'x':
            name = my_raw_input(stdscr, 8, 0, "Enter name:").lower().strip()
            name = name.decode('utf-8')
            stdscr.addstr(8, 0, f"Using {name}                                     ")
            graph.add_node(name, last_position)
            last_position = (last_position[0] + 1, last_position[1] + 2)
        elif k == 'r':
            stdscr.addstr(1, 0, f"Waiting for reset...{' ' * 30}")
            result = 0
            if result.returncode == 0:
                stdscr.addstr(1, 0, "Reset complete.     ")
            else:
                stdscr.addstr(1, 0, "Trouble with reset. ")
            stdscr.refresh()
    

if __name__ == '__main__':
    curses.wrapper(main)
    with open(f"map_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}", 'wb') as file:
        pickle.dump(graph, file)
