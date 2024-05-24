START_CHAR = 65

class PathwayGrid:
    def __init__(self, meters_per_square=1):
        self.meters_per_square = meters_per_square
        self.visited = set()
        self.pathway = []

        self.x_min_square = None
        self.x_max_square = None

        self.y_min_square = None
        self.y_max_square = None

        self.rotate = False

    def empty(self):
        return len(self.visited) == 0

    def visit(self, x_meters, y_meters):
        x = int(x_meters / self.meters_per_square)
        y = int(y_meters / self.meters_per_square)

        if self.empty():
            self.x_min_square = self.x_max_square = x
            self.y_min_square = self.y_max_square = y
        else:
            self.x_min_square = min(self.x_min_square, x)
            self.x_max_square = max(self.x_max_square, x)
            self.y_min_square = min(self.y_min_square, y)
            self.y_max_square = max(self.y_max_square, y)

        self.visited.add((x, y))
        self.pathway.append((x, y))

    def squares_wide(self):
        return self.x_max_square - self.x_min_square + 1

    def squares_high(self):
        return self.y_max_square - self.y_min_square + 1

    def console_grid_points(self):
        points = []
        for y in range(self.y_max_square, self.y_min_square - 1, -1):
            row = []
            for x in range(self.x_min_square, self.x_max_square + 1):
                row.append((x, y))
            points.append(row)
        return points

    def rotated_grid_points(self):
        points = []
        for x in range(self.x_min_square, self.x_max_square + 1):
            row = []
            for y in range(self.y_min_square, self.y_max_square + 1):
                row.append((x, y))
            points.append(row)
        return points

    def point2console(self, console_point_list):
        result = {}
        for y, row in enumerate(console_point_list):
            for x, pos in enumerate(row):
                result[pos] = (x, y)
        return result

    def encode_point(self, x, y):
        if (x, y) == (0, 0):
            return "$$"

        x_spot = x - self.x_min_square + START_CHAR
        y_spot = y - self.y_min_square + START_CHAR
        return f"{chr(x_spot)}{chr(y_spot)}"

    def encoded_point_in_meters(self, encoded):
        if encoded == "$$":
            return (0, 0)

        x = ord(encoded[0]) - START_CHAR + self.x_min_square
        y = ord(encoded[1]) - START_CHAR + self.y_min_square
        return (x * self.meters_per_square, y * self.meters_per_square)

    def occupancy_str(self):
        if self.empty():
            return "No visits"
        else:
            result = ''
            points = self.rotated_grid_points() if self.rotate else self.console_grid_points()
            for row in points:
                for point in row:
                    if point == (0, 0):
                        result += 'X'
                    else:
                        result += '.' if point in self.visited else ' '
                result += '\n'
            return result

    def path_str(self):
        if self.empty():
            return "No path"
        else:
            points = self.rotated_grid_points() if self.rotate else self.console_grid_points()
            chars = [[' ' for pos in row] for row in points]
            point2console = self.point2console(points)
            for i in range(len(self.pathway)):
                start = point2console[self.pathway[i]]
                end = point2console[self.pathway[min(len(self.pathway) - 1, i + 1)]]
                chars[start[1]][start[0]] = point_dir_char(start, end)
            rows = [''.join(row) for row in chars]
            return '\n'.join(rows)

    def square_name_str(self):
        if self.empty():
            return "No map"
        else:
            result = ''
            points = self.rotated_grid_points() if self.rotate else self.console_grid_points()
            for row in points:
                for point in row:
                    if point in self.visited:
                        result += self.encode_point(point[0], point[1]) + " "
                    else:
                        result += '   '
                result += '\n'
            return result                
                    
                    
def point_dir_char(start, end):
    xdiff = end[0] - start[0]
    ydiff = end[1] - start[1]
    if xdiff == 1:
        if ydiff == 1:
            return '\\'
        elif ydiff == -1:
            return '/'
        else:
            return '>'
    elif xdiff == -1:
        if ydiff == 1:
            return ','
        elif ydiff == -1:
            return '`'
        else:
            return '<'
    else:
        if ydiff == 1:
            return 'v'
        elif ydiff == -1:
            return '^'
        else:
            return '*'
