from operator import attrgetter  # method for working with specific attributes of an object
import numpy as np
D = 1  # heuristic function weights
D2 = 2  # with 2 works better


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

        self.__closed_list = []  # list for storing the shortest path leading to the destination point

    class __Cell:  # class for storing the cell coordinates, the weight of distance leading to that cell and the f value
        def __init__(self, x, y, distance, fscore):
            self.x = x
            self.y = y
            self.distance = distance
            self.fscore = fscore

    def Heuristic(self, xn, yn, xg, yg):  # Manhattan heuristic function in order to find the heuristic value that represents the distance from he current node to the destination node
        dx = abs(xn - xg)
        dy = abs(yn - yg)
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def _PrintSolution(self, x, y, fscore):  # prints the final grid and shortest path
        for row in self.__grid:
            print(row)
        print("\nThe destination point is " + str(self.__matrix[x][y]) + " and it's weight is " + str(fscore))
        print(self.__closed_list)

    def _isInsideGrid(self, i, j):  # indicating whether a cell is inside the boundaries of the grid or not
        return 0 <= i < self.__col and 0 <= j < self.__row

    def Astar(self, xs, ys, xd, yd):  # A Star Algorithm

        dx = [0, 1, 0, -1, 1, -1, 1,
              -1]  # lists dx and dy store the pairs of movement that the algorithm makes in order to define the distance from the source cell to it's neighbors (up, down, right, left, diagonal)
        dy = [-1, 0, 1, 0, 1, 1, -1, -1]

        open_list = [ShortestPath.__Cell(xs, ys, 0, self.__dist[xs][ys] + self.Heuristic(xs, ys, xd, yd))]  # priority list that stores the cell that have been visited by the algorithm

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

        while open_list:  # main loop

            current_node = min(open_list, key=attrgetter('fscore'))  # the cell with the lowest value is becoming the source cell and is stored in the shortest path list, this cell can not be visited again by the algorithm so it's value
            self.__closed_list.append(current_node.fscore)  # is set to True in the visited matrix
            self.__visited[current_node.x][current_node.y] = True
            open_list.remove(current_node)

            if current_node.x == xd and current_node.y == yd:  # if the coordinates of the target cell are the same as those of the destination cell
                self.__grid[current_node.x][
                    current_node.y] = 'd'  # update the value of the target cell with the letter 'd' indicating destination
                return self._PrintSolution(current_node.x, current_node.y, current_node.fscore)  # return the _PrintSolution method in order to print the final grid and the shortest path leading to the destination cell

            for i in range(8):

                x = current_node.x + dx[i]
                y = current_node.y + dy[i]

                if not self._isInsideGrid(x, y):  # checks whether the cell is within the boundaries of the matrix
                    continue

                if not self.__visited[x][y]:  # if the cell is not contained inside self.__visited matrix then proceed
                    if self.__dist[x][y] > self.__dist[current_node.x][current_node.y] + self.__matrix[x][y]:  # if the weight (distance) of the cell in check is greater than the value of the source cell plus the cell in check in the main matrix the proceed
                        self.__dist[x][y] = self.__dist[current_node.x][current_node.y] + self.__matrix[x][y]  # update the weight (distance) of the cell in  check with the value of the source cell plus the value of the cell in check in the main matix
                        fscore = self.__dist[x][y] + self.Heuristic(x, y, xd, yd)  # calculate the f value of the cell/node
                        open_list.append(ShortestPath.__Cell(x, y, self.__dist[x][y], fscore))  # append this cell/node in the priority list
                        self.__grid[x][y] = str(fscore)  # update the weight of the target cell/node in the grid


if __name__ == '__main__':
    grid = [[1, 1, 1, 1, 1, 0, 1, 1, 1, 1],  # main grid
            [1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 0, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]

    g = ShortestPath(10, 10, grid)  # instantiation of ShortestPath class
    g.Astar(0, 0, 9, 9)  # A Star Algorithm
