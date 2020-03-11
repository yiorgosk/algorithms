row = 5
col = 5


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


def isInsideGrid(i, j):
    return 0 <= i < col and 0 <= j < row


class ShortestPath:
    def __init__(self):
        self.__matrix = [[0 for i in range(col)]
                       for j in range(row)]

        self.dist = [[100000 for i in range(col)]
                     for j in range(row)]

        self.shortest_path_tree = [[False for i in range(col)]
                                   for j in range(row)]

        self.grid = [['*' for i in range(col)]
                     for j in range(row)]

    def PrintSolution(self, x, y):
        for row in self.grid:
            print(row)
        print("\nThe destination point is " + str(self.matrix[x][y]) + " and it's weight is " + str(self.dist[x][y]))

    def Dijkstra(self, source, destination):

        dx = [-1, 0, 1, 0]
        dy = [0, 1, 0, -1]

        st = set()
        st.add(Cell(0, 0, 0))

        for i in range(row):
            for j in range(col):
                if self.matrix[i][j] == 0:
                    self.shortest_path_tree[i][j] = True
                    self.grid[i][j] = '#'
                else:
                    self.shortest_path_tree[i][j] = False

        for i in range(row):
            for j in range(col):
                if self.matrix[i][j] == source:
                    self.dist[i][j] = self.matrix[i][j]
                    self.grid[i][j] = 's'
                    self.shortest_path_tree[i][j] = True

        while st:

            k = st.pop()

            for i in range(4):

                x = k.x + dx[i]
                y = k.y + dy[i]

                if not isInsideGrid(x, y):
                    continue

                if self.dist[x][y] > self.dist[k.x][k.y] + self.matrix[x][y] and self.shortest_path_tree[x][y] == False:
                    self.dist[x][y] = self.dist[k.x][k.y] + self.matrix[x][y]
                    st.add(Cell(x, y, self.dist[x][y]))
                    self.shortest_path_tree[x][y] = True
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

    g = ShortestPath()
    g.matrix = grid
    print(g.Dijkstra(31, 20))
