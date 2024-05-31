import pickle, sys, curses

def main(stdscr):
    with open(sys.argv[1], 'rb') as f:
        map_data = pickle.load(f)
        map_data.rotate = True
        map_graph = map_data.square_graph()

        curses.curs_set(0)

        running = True
        message = ""

        while running:
            stdscr.clear()

            stdscr.addstr(0, 0, 'Enter location name to see coordinate; "quit" to exit.')
            stdscr.addstr(2, 0, message)
        
            map_str = map_data.square_name_str()
            row = 3
            for i, line in enumerate(map_str.split('\n')):
                stdscr.addstr(row + i, 0, line)
        
            choice = my_raw_input(stdscr, 1, 0, "> ")

            if choice == 'quit':
                running = False
            elif map_graph.has_node(choice):
                message = f"{map_graph.node_value(choice)}"
            else:
                message = "Unrecognized"
        

def my_raw_input(stdscr, row, col, prompt_string):
    curses.echo()
    stdscr.addstr(row, col, prompt_string)
    stdscr.refresh()
    text = stdscr.getstr(row, col + len(prompt_string) + 1, 20)
    text = text.decode('utf-8')
    return text


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: curses_map.py pickled_map_file")
    else:
        curses.wrapper(main)
            
