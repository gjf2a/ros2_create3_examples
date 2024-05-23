class OccupancyGrid:
    def __init__(self, meters_per_square=1):
        self.meters_per_square = meters_per_square
        self.visited = set()

        self.x_min_square = None
        self.x_max_square = None

        self.y_min_square = None
        self.y_max_square = None

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

        

    def squares_wide(self):
        return self.x_max_square - self.x_min_square + 1

    def squares_high(self):
        return self.y_max_square - self.y_min_square + 1

    def __str__(self):
        if self.empty():
            return "No visits"
        else:
            result = ''
            for y in range(self.y_max_square, self.y_min_square - 1, -1):
                for x in range(self.x_min_square, self.x_max_square + 1):    
                    if (x, y) == (0, 0):
                        result += 'X'
                    else:
                        result += '.' if (x, y) in self.visited else '*'
                result += '\n'
            return result
