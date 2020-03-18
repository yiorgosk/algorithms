from operator import attrgetter


class Cell:
    def __init__(self, x, y, distance):
        self.x = x
        self.y = y
        self.distance = distance

    def __lt__(self, other):
        if self.distance < other.distance:
            if self.x != other.x:
                return self.x < other.x
            else:
                return self.y < other.y
        return self.distance < other.distance


class ShortestPath:
    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.matrix = [[0 for i in range(self.col)]
                       for j in range(self.row)]

        self.dist = [[100000 for i in range(self.col)]
                     for j in range(self.row)]

        self.visited = [[False for i in range(self.col)]
                        for j in range(self.row)]

        self.grid = [['*' for i in range(self.col)]
                     for j in range(self.row)]
        self.shortest_path_tree = []

    def PrintSolution(self, x, y):
        for row in self.grid:
            print(row)
        print("\nThe destination point is " + str(self.matrix[x][y]) + " and it's weight is " + str(self.dist[x][y]))
        print(self.shortest_path_tree)

    def isInsideGrid(self, i, j):
        return 0 <= i < self.col and 0 <= j < self.row

    def Dijkstra(self, source, destination):

        dx = [0, 1, 0, -1]
        dy = [-1, 0, 1, 0]

        priority_list = [Cell(0, 0, 0)]

        for i in range(self.row):
            for j in range(self.col):
                if self.matrix[i][j] == 0:
                    self.visited[i][j] = True
                    self.grid[i][j] = '#'
                else:
                    self.visited[i][j] = False

        for i in range(self.row):
            for j in range(self.col):
                if self.matrix[i][j] == source:
                    self.dist[i][j] = 0
                    self.grid[i][j] = 's'
                    self.visited[i][j] = True

        while priority_list:

            cell = min(priority_list, key=attrgetter('distance'))
            self.shortest_path_tree.append(cell.distance)
            priority_list.remove(cell)

            for i in range(4):

                x = cell.x + dx[i]
                y = cell.y + dy[i]

                if not self.isInsideGrid(x, y):
                    continue

                if self.dist[x][y] > self.dist[cell.x][cell.y] + self.matrix[x][y] and self.visited[x][y] == False:
                    if self.dist[x][y] != 100000:
                        priority_list.remove(Cell(x, y, self.dist[x][y]))
                    self.dist[x][y] = self.dist[cell.x][cell.y] + self.matrix[x][y]
                    priority_list.append(Cell(x, y, self.dist[x][y]))
                    self.visited[x][y] = True
                    self.grid[x][y] = str(self.dist[x][y])

                    if self.matrix[x][y] == destination:
                        self.grid[x][y] = 'd'
                        return self.PrintSolution(x, y)


if __name__ == '__main__':
    grid = [[31, 100, 0, 12, 18],
            [10, 13, 0, 157, 6],
            [100, 113, 0, 11, 33],
            [88, 124, 0, 21, 140],
            [99, 32, 11, 41, 20]]

    g = ShortestPath(5, 5)
    g.matrix = grid
    print(g.Dijkstra(31, 20))
