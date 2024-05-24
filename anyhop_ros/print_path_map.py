import pickle, sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: print_map.py pickled_map_file")
    else:
        with open(sys.argv[1], 'rb') as f:
            g = pickle.load(f)
            g.rotate = True
            print(g.occupancy_str())
            print()
            print(g.path_str())
            print()
            print(g.square_name_str())
            print()
            g.square_graph().print_graph()
