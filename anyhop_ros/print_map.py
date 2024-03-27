import pickle, sys

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: print_map.py pickled_map_file")
    else:
        with open(sys.argv[1], 'rb') as f:
            g = pickle.load(f)
            g.print_graph()
