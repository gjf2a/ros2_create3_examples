from trajectories import TrajectoryMap
import sys

with open(sys.argv[1], 'r') as map_file:
    tmap = eval(map_file.read())
    print(tmap.all_points())
    max_name_length = max(len(name) for name in tmap.named_locations)
    for name, location in tmap.named_locations.items():
        print(f"{name}{' ' * (max_name_length - len(name) + 4)}({location[0]:5.2f}, {location[1]:5.2f})")
