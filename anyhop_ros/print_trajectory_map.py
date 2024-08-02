from trajectories import TrajectoryMap
import sys

with open(sys.argv[1], 'r') as map_file:
    tmap = eval(map_file.read())
    print(tmap.all_points())
