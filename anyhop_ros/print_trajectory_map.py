import trajectories

import sys, pickle

with open(sys.argv[1], 'wb') as map_file:
    tmap = pickle.load(map_file)
    print(tmap.all_points())
