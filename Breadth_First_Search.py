import collections


class ShortestPath:

    def __init__(self, row, column, weight):

        self.row = row
        self.column = column
        self.distance = weight
        self.graph = [[0 for col in range(4)]
                      for i in range(4)]

    def Breadth_First_Search(self):

        visited = [[False for col in range(4)]
                   for row in range(4)]

        for i in range(4):
            for j in range(4):
                if self.graph[i][j] == '0':
                    visited[i][j] = True
                else:
                    visited[i][j] = False

                if self.graph[i][j] == 's':
                    self.row = i
                    self.column = j

        q = collections.deque([[self.row, self.column, self.distance]])

        visited[self.row][self.column] = True

        while q:

            path = q.pop()

            if self.graph[path[0]][path[1]] == 'd':
                return path[2]

            if self.row - 1 >= 0 and visited[self.row - 1][self.column] == False:
                q.append([self.row - 1, self.column, self.distance + 1])
                visited[self.row - 1][self.column] = True
                self.row = self.row - 1
                self.distance = self.distance + 1

            if self.row + 1 < 4 and visited[self.row + 1][self.column] == False:
                q.append([self.row + 1, self.column, self.distance + 1])
                visited[self.row + 1][self.column] = True
                self.row = self.row + 1
                self.distance = self.distance + 1

            if self.column - 1 >= 0 and visited[self.row][self.column - 1] == False:
                q.append([self.row, self.column - 1, self.distance + 1])
                visited[self.row][self.column - 1] = True
                self.column = self.column - 1
                self.distance = self.distance + 1

            if self.column + 1 < 4 and visited[self.row][self.column + 1] == False:
                q.append([self.row, self.column + 1, self.distance + 1])
                visited[self.row][self.column + 1] = True
                self.column = self.column + 1
                self.distance = self.distance + 1

            else:
                for i in range(4):
                    for j in range(4):
                        if self.graph[i][j] == '0':
                            visited[i][j] = True
                        else:
                            visited[i][j] = False


if __name__ == '__main__':
    g = ShortestPath(0, 0, 0)
    g.graph = [['0', '*', '0', '*'],
               ['*', '0', '*', 's'],
               ['0', '*', '*', '*'],
               ['d', '*', '*', '*']]
    print(g.Breadth_First_Search())
