from operator import attrgetter  # method for working with specific attributes of an object


class ShortestPath:  # main class
    def __init__(self, row, col, grid):
        self.__row = row
        self.__col = col

        self.__matrix = [[0 for i in range(self.__col)]  # matrix for grid input
                         for j in range(self.__row)]

        self.__matrix = grid

        self.__dist = [[100000 for i in range(self.__col)]  # matrix for applying 'infinite' weight for each cell that has not been visited yet by Dijkstra function
                       for j in range(self.__row)]

        self.__visited = [[False for i in range(self.__col)]  # matrix for the distinction of whether a cell has been visited or not, indicated by True or False
                          for j in range(self.__row)]

        self.__grid = [['*' for i in range(self.__col)]  # matrix for representing the final grid, with the start point, the weight value for each cell that has been visited and the destination point
                       for j in range(self.__row)]

        self.__shortest_path_tree = []  # list for storing the shortest path leading to the destination point

    class __Cell:  # class for storing the cell coordinates and the weight of distance leading to that cell
        def __init__(self, x, y, distance):
            self.x = x
            self.y = y
            self.distance = distance

    def _PrintSolution(self, x, y):  # prints the final grid and shortest path
        for row in self.__grid:
            print(row)
        print(
            "\nThe destination point is " + str(self.__matrix[x][y]) + " and it's weight is " + str(self.__dist[x][y]))
        print(self.__shortest_path_tree)

    def _isInsideGrid(self, i, j):  # indicating whether a cell is inside the boundaries of the grid or not
        return 0 <= i < self.__col and 0 <= j < self.__row

    def Dijkstra(self, xs, ys, xd, yd):  # Dijkstra Algorithm

        dx = [0, 1, 0, -1, 1, -1, 1, -1]  # lists dx and dy store the pairs of movement that the algorithm makes in order to define the distance from the source cell to it's neighbors (up, down, right, left, diagonal)
        dy = [-1, 0, 1, 0, 1, 1, -1, -1]

        priority_list = [ShortestPath.__Cell(0, 0, 0)]  # priority list that stores the cell that have been visited by the algorithm

        for i in range(self.__row):  # initializing the cells that will act as obstacles, their value is zero (0)
            for j in range(self.__col):
                if self.__matrix[i][j] == 0:
                    self.__visited[i][j] = True
                    self.__grid[i][j] = '#'
                else:
                    self.__visited[i][j] = False

        for i in range(self.__row):  # initializing the source cell
            for j in range(self.__col):
                if i == xs and j == ys:
                    self.__matrix[i][j] = 0
                    self.__dist[i][j] = 0
                    self.__grid[i][j] = 's'
                    self.__visited[i][j] = True

        while priority_list:  # main loop

            cell = min(priority_list, key=attrgetter('distance'))  # the cell with the lowest value is becoming the source cell and is stored in the shortest path list, this cell can not be visited again by the algorithm so it's value
            self.__shortest_path_tree.append(cell.distance)  # is set to True in the visited matrix
            self.__visited[cell.x][cell.y] = True
            priority_list.remove(cell)

            for i in range(8):

                x = cell.x + dx[i]
                y = cell.y + dy[i]

                if not self._isInsideGrid(x, y):  # checks whether the cell is within the boundaries of the matrix
                    continue

                if not self.__visited[x][y]:  # if the cell is not contained inside self.__visited matrix then proceed
                    if self.__dist[x][y] > self.__dist[cell.x][cell.y] + self.__matrix[x][y]:  # if the weight (distance) of the cell in check is greater than the value of the source cell plus the cell in check in the main matrix the proceed
                        self.__dist[x][y] = self.__dist[cell.x][cell.y] + self.__matrix[x][y]  # update the weight (distance) of the cell in  check with the value of the source cell plus the value of the cell in check in the main matix
                        priority_list.append(ShortestPath.__Cell(x, y, self.__dist[x][y]))  # append this cell in the priority list
                        self.__grid[x][y] = str(self.__dist[x][y])  # update the weight of the target cell in the grid

                        if x == xd and y == yd:  # if the coordinates of the target cell are the same as those of the destination cell
                            self.__grid[x][y] = 'd'  # update the value of the target cell with the letter 'd' indicating destination
                            return self._PrintSolution(x, y)  # return the _PrintSolution method in order to print the final grid and the shortest path leading to the destination cell

if __name__ == '__main__':
    grid = [[31, 100, 0, 12, 18],  # main grid
            [10, 13, 0, 157, 6],
            [100, 113, 0, 11, 33],
            [88, 0, 0, 21, 140],
            [99, 32, 111, 11, 20]]

    g = ShortestPath(5, 5, grid)  # instantiation of ShortestPath class
    g.Dijkstra(0, 0, 4, 4)  # Dijkstra Algorithm
