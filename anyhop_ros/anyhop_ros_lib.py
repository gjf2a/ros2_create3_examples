from pyhop_anytime import *
import copy

class MapInfo:
    def __init__(self, filename=None):
        self.adjacency = {}
        self.locations = {}
        self.name = "Unnamed"
        if filename:
            with open(filename) as file:
                self.name = filename
                for line in file:
                    self.add_row(line)

    def __contains__(self, location):
        return location in self.locations

    def all_locations(self):
        return list(self.locations)

    def coordinates_of(self, location_name):
        return self.locations[location_name]

    def start(self):
        for location in self.locations:
            return location

    def add_row(self, row_str):
        name, location, adjacency = row_str.split(':')
        name = name.strip()
        x, y = location.strip()[1:-1].split(',')
        self.locations[name] = (float(x), float(y))
        for adj in adjacency.split(','):
            self.add_adjacent(name, adj)

    def add_adjacent(self, a, b):
        a = a.strip()
        b = b.strip()
        if a not in self.adjacency:
            self.adjacency[a] = []
        if b not in self.adjacency[a]:
            self.adjacency[a].append(b)
        if b not in self.adjacency or a not in self.adjacency[b]:
            self.add_adjacent(b, a)

    def __str__(self):
        s = ""
        for name in self.locations:
            s += f"{name}: {self.locations[name]}: "
            for adj in self.adjacency[name]:
                s += f'{adj}, '
            s = f"{s[:-2]}\n"
        return s

    def pyhop_state(self):
        state = pyhop.State(self.name)
        state.connected = copy.deepcopy(self.adjacency)
        state.visited = {'robot': set()}
        state.location = {'robot': self.start()}    
        return state


def make_planner():
    planner = pyhop.Planner()
    planner.declare_operators(go)
    planner.declare_methods(find_route)
    return planner


def go_pre(state, robot, start, end):
    return state.location[robot] == start and end in state.connected[start]


def go(state, robot, start, end):
    if go_pre(state, robot, start, end):
        state.location[robot] = end
        state.visited[robot].add(end)
        return state


def find_route(state, robot, start, end):
    if start == end:
        state.visited = {robot: set()}
        return TaskList(completed=True)
    elif go_pre(state, robot, start, end) and end not in state.visited[robot]:
        return TaskList([('go', robot, start, end)])
    else:
        tasks = []
        for neighbor in state.connected[start]:
            tasks.append([('go', robot, start, neighbor), 
                          ('find_route', robot, neighbor, end)])
        return TaskList(tasks)

