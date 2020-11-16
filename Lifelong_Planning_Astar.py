from operator import attrgetter  # method for working with specific attributes of an object
import numpy as np

D = 1  # heuristic function weights
D2 = 2  # with 2 works better


class Lifelong_Planning_Astar:  # main class

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

        self.__grid = [['*****' for i in range(self.__col)]  # matrix for representing the final grid, with the start point, the weight value for each cell that has been visited and the destination point
                       for j in range(self.__row)]

        self.__closed_list = []  # list for storing the shortest path leading to the destination point

        self.__open_list = []  # list for storing the cell nodes

    class __Cell:  # class for storing the cell coordinates, the weight of distance leading to that cell and the f value

        def __init__(self, x, y, gscore, rhs, key, parent):

            self.x = x
            self.y = y
            self.gscore = gscore
            self.rhs = rhs
            self.key = key
            self.parent = parent

    def __Heuristic(self, xn, yn, xg, yg):  # Manhattan heuristic function in order to find the heuristic value that represents the distance from he current node to the destination node

        dx = abs(xn - xg)
        dy = abs(yn - yg)

        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def Calculatekey(self, rhs, gs, heuristic):  # Calculates the algorithm key

        return [min(rhs, gs) + heuristic, min(rhs, gs)]

    def Update(self, current_node, goal):  # Updates the weights of neightboring nodes

        if current_node != goal:

            dx = [0, 1, 0, -1, 1, -1, 1, -1]  # lists dx and dy store the pairs of movement that the algorithm makes in order to define the distance from the source cell to it's neighbors (up, down, right, left, diagonal)
            dy = [-1, 0, 1, 0, 1, 1, -1, -1]

            for i in range(8):

                x = current_node.x + dx[i]
                y = current_node.y + dy[i]

                if not self.__isInsideGrid(x, y):  # checks whether the cell is within the boundaries of the matrix
                    continue

                if not self.__visited[x][y]:  # if the cell is not contained inside self.__visited matrix then proceed
                    if self.__dist[x][y] > current_node.gscore + self.__matrix[x][y]:  # if the weight (distance) of the cell in check is greater than the value of the source cell plus the cell in check in the main matrix the proceed
                        current_node.rhs = current_node.gscore + self.__matrix[x][y]  # Applies the g(s') + c(s, s') to rhs value
                        current_node.rhs = min(current_node.rhs, self.__dist[x][y])
                        key = self.Calculatekey(current_node.rhs, self.__dist[x][y], self.__Heuristic(x, y, goal.x, goal.y))  # Calculates the key

                        node = Lifelong_Planning_Astar.__Cell(x, y, self.__dist[x][y], current_node.rhs, key, current_node)  # Creates a new node

                        if node in self.__open_list:
                            self.__open_list.remove(node)
                        if node.rhs != node.gscore:
                            self.__open_list.append(node)  # append this cell/node in the priority list

    def __PrintSolution(self, x, y, fscore):  # prints the final grid and shortest path

        for row in self.__grid:
            print(row)
        print("\nThe destination point is " + str(self.__matrix[x][y]) + " and it's weight is " + str(fscore))

        i = 0
        for node in self.__closed_list:
            try:
                print(i, node.gscore, node.rhs, node.key, node.parent.key)
                i += 1
            except AttributeError:
                pass

    def __isInsideGrid(self, i, j):  # indicating whether a cell is inside the boundaries of the grid or not
        return 0 <= i < self.__col and 0 <= j < self.__row

    def ComputePath(self, xs, ys, xd, yd):  # A Star Algorithm

        for i in range(self.__row):  # initializing the cells that will act as obstacles, their value is zero (0)
            for j in range(self.__col):
                if self.__matrix[i][j] == 0:
                    self.__visited[i][j] = True
                    self.__grid[i][j] = '#####'

        for i in range(self.__row):  # initializing the source cell
            for j in range(self.__col):
                if i == xs and j == ys:
                    self.__matrix[i][j] = 0
                    self.__dist[i][j] = 100000
                    self.__grid[i][j] = 's'

        start_node = self.__Cell(xs, ys, self.__dist[xs][ys], 0, self.Calculatekey(0, self.__dist[xs][ys], self.__Heuristic(xs, ys, xs, ys)), None)
        goal_node = self.__Cell(xd, yd, self.__dist[xd][yd], 100000, self.Calculatekey(100000, self.__dist[xd][yd], self.__Heuristic(xd, yd, xs, ys)), None)
        self.__open_list = [start_node]  # priority list that stores the cell that have been visited by the algorithm
        current_node = min(self.__open_list, key=attrgetter('key'))

        while current_node.key < self.Calculatekey(goal_node.rhs, goal_node.gscore, self.__Heuristic(goal_node.x, goal_node.y, goal_node.x, goal_node.y)) or goal_node.rhs != goal_node.gscore:  # main loop

            current_node = min(self.__open_list, key=attrgetter('key'))
            self.__closed_list.append(current_node)  # is set to True in the visited matrix

            if current_node in self.__open_list:
                self.__open_list.remove(current_node)
            self.__grid[current_node.x][current_node.y] = str(current_node.key)  # update the weight of the target cell/node in the grid

            if self.__dist[current_node.x][current_node.y] > current_node.rhs:
                current_node.gscore = current_node.rhs
                self.__dist[current_node.x][current_node.y] = current_node.rhs
                self.Update(current_node, goal_node)
            else:
                current_node.gscore = 100000
                self.__dist[current_node.x][current_node.y] = 100000
                self.Update(current_node, goal_node)

            if current_node.x == xd and current_node.y == yd:  # if the coordinates of the target cell are the same as those of the destination cell
                self.__grid[current_node.x][current_node.y] = 'dest'  # update the value of the target cell with the letter 'd' indicating destination
                current_node.gscore = current_node.rhs
                return self.__PrintSolution(current_node.x, current_node.y, current_node.key)  # return the _PrintSolution method in order to print the final grid and the shortest path leading to the destination cell
            


if __name__ == '__main__':
    grid = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],  # main grid
            [1, 1, 0, 0, 1, 1, 1, 1, 1, 1],
            [1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
            [1, 1, 0, 0, 0, 0, 0, 0, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]

    g = Lifelong_Planning_Astar(10, 10, grid)  # instantiation of ShortestPath class
    g.ComputePath(0, 0, 9, 9)  # Lifelong A Star Algorithm
