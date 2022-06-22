import numpy
from heapq import *
import time


class Slider:

    def heuristic(self, a, b):
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2

    def __init__(self):
        self.oheap = []

    def compute(self, length, array, start, goal):
        self.gscore = {start:0}
        self.fscore = {start:self.heuristic(start, goal)}
        self.close_set = set()
        self.came_from = {}
        self.array = array


        heappush(self.oheap, (self.fscore[start], start))

        while self.oheap:
            current = heappop(self.oheap)[1]

            if abs(current[0] - goal[0]) <= length/2 and abs(current[1] - goal[1]) <= length/2:
                data = []
                while current in self.came_from:
                    data.append(current)
                    current = self.came_from[current]
                return data

            self.close_set.add(current)
            self.slide_algorithm(current, length, self.array, goal)

        return False


    def a_star_computation(self, current, neighbor, goal):
        tentative_g_score = self.gscore[current] + self.heuristic(current, neighbor)
        if neighbor == [-1,-1]:
            return
        if 0 <= neighbor[0] < self.array.shape[0]:
            if 0 <= neighbor[1] < self.array.shape[1]:
                if self.array[neighbor[0]][neighbor[1]] == 1:
                    return
            else:
                # array bound y walls
                return
        else:
            # array bound x walls
            return

        if (neighbor[0], neighbor[1]) in self.close_set and tentative_g_score >= self.gscore.get((neighbor[0], neighbor[1]), 0):
            return

        if  tentative_g_score < self.gscore.get((neighbor[0], neighbor[1]), 0) or (neighbor[0],neighbor[1]) not in [i[1]for i in self.oheap]:
            self.came_from[(neighbor[0], neighbor[1])] = current
            self.gscore[(neighbor[0], neighbor[1])] = tentative_g_score
            self.fscore[(neighbor[0], neighbor[1])] = tentative_g_score + self.heuristic(neighbor, goal)
            heappush(self.oheap, (self.fscore[(neighbor[0], neighbor[1])], (neighbor[0], neighbor[1])))


    def slide_algorithm(self, center, l, map, goal):
        mat = [[center[0] - l, center[1] - l], [center[0]-l, center[1]], [center[0] -l, center[1] +l],
                [center[0], center[1]-l], [center[0], center[1] + l], [center[0] + l, center[1] - l],
                [center[0] + l, center[1]], [center[0] + l, center[1] + l]]

        for a in range(0,8):
            grid = []
            l_hat = mat[a]
            for j in range(0, l):
                grid_t = []
                for i in range(0, l):
                    if (l_hat[0] - int(l/2) + j < len(map) and l_hat[1] - int(l/2) + i < len(map[0])
                        and l_hat[0] - int(l/2) + j >= 0 and l_hat[1] - int(l/2) + i >= 0):
                        grid_t.append(map[l_hat[0] - int(l/2) + j][l_hat[1] - int(l/2) + i])
                grid.append(grid_t)

            neighbor = center

            if a == 0 or a == 2:
                neighbor = self.cross_slide_helper(grid, map, center, l_hat, l, 0)
            elif a == 5 or a == 7:
                neighbor = self.cross_slide_helper(grid, map, center, l_hat, l, 1)
            if a % 5 == 0:
                neighbor = self.cross_slide_helper(grid, map, center, l_hat, l, 2)
            elif a % 5 == 2:
                neighbor = self.cross_slide_helper(grid, map, center, l_hat, l, 3)
            elif a % 5 == 1:
                neighbor = self.cross_slide_helper(grid, map, center, l_hat, l, 3)
                neighbor = self.cross_slide_helper(grid, map, center, l_hat, l, 2)
            elif a == 3 or a == 4:
                neighbor = self.cross_slide_helper(grid, map, center, l_hat, l, 1)
                neighbor = self.cross_slide_helper(grid, map, center, l_hat, l, 0)

            self.a_star_computation(center, neighbor, goal)


    def cross_slide_helper(self, grid, map, origin, center_n, l, dir):
        truisms = [1 in grid[i] for i in range(0, 8)]
        if (True in truisms):
            center = center_n
            for k in range(0, int(l/2)):
                grid_t = []
                if dir == 0:
                    center[0] += k
                elif dir == 1:
                    center[0] -= k
                elif dir == 2:
                    center[1] += k
                else:
                    center[1] -= k

                for j in range(0, l):
                    grid_t2 = []
                    for k in range(0, l):
                        if (center[0] - int(l/2) + j < len(map) and center[1] - int(l/2) + k < len(map[0])
                            and center[0] - int(l/2) + j >= 0 and center[1] - int(l/2) + k >= 0):
                            grid_t2.append(map[center[0] - int(l/2) + j][center[1] - int(l/2) + k])
                    grid_t.append(grid_t2)

                grid_truism = [1 in grid_t[i] for i in range(0, len(grid_t))]
                if (not any(grid_truism)):
                    center_temp = False
                    limiter = 0
                    while (not center_temp and limiter < l):
                        limiter += 1
                        stuff = self.cross_slide_inwards(map, origin, center, l)
                        center_temp = stuff[1]
                        center = stuff[0]
                    if (limiter == l - 1):
                        return [-1,-1]
                    return  center


        else:
            return  center_n
        return [-1,-1]

    def cross_slide_inwards(self, map, center, center_n, l):
        diff_y = center_n[0] - center[0]
        diff_x = center_n[1] - center[1]
        checked = False
        slope = diff_y / diff_x
        direction = -1 if diff_x < 0 else 1

        for i in range(1 * direction, diff_x, direction):
            l_hat = [int(center_n[0] + slope * i), int(center_n[0] + i)]
            for j in range(0, l):
                grid_t = []
                for k in range(0, l):
                    if (l_hat[0] - int(l/2) + j < len(map) and l_hat[1] - int(l/2) + k < len(map[0])
                        and l_hat[0] - int(l/2) + j >= 0 and l_hat[1] - int(l/2) + k >= 0):
                        grid_t.append(map[l_hat[0] - int(l/2) + j][l_hat[1] - int(l/2) + k])
                if (1 in grid_t):
                    if (abs(diff_x) >= abs(diff_y) and diff_x > 0):
                        center_n[1] -= 1

                    elif (abs(diff_x) >= abs(diff_y) and diff_x <= 0):
                        center_n[1] += 1

                    elif (abs(diff_y) >= abs(diff_x) and diff_y > 0):
                        center_n[0] -=1

                    else:
                        center_n[0] += 1
                    return [center_n, False]

        return [center_n, True]




def setup_map(map):
        padding = [0,0,0,0]

        for obstacle in (obstacles):
            top_y = obstacle[0] + padding[0]
            top_x = obstacle[1] + padding[1]
            width = obstacle[2] + padding[2]
            height = obstacle[3] + padding[3]

            for i in range(top_y, top_y + height):
                for j in range(top_x, top_x + width):
                    if i >= 0 and i < len(map) and j >= 0 and j < len(map[i]):
                        map[i][j] = 1




#obstacles = [[200, 40, 60, 320],[250, 350, 60, 60],[120, 400, 100, 10],[30, 230, 70, 50],[40, 260, 60, 60],[330, 220, 120, 40],[330,150, 5, 70],[440, 150, 5, 70],[1999,5332,1000,400],[444,2424,555,1344]]
#obstacles = [[200, 40, 60, 320],[250, 350, 60, 60],[120, 400, 100, 10],  [30, 230, 70, 50],[40, 260, 60, 60],[330, 220, 120, 40],[330,150, 5, 70],[440, 150, 5, 70],[0,0,499, 1], [0,0,1,499], [498,0,1,499], [0,497,499,2]]
obstacles = [[200, 40, 60, 320],[55, 50, 40, 8],[5000,5000,1000,2000],[758,300,1000,5666],[250, 350, 60, 60],[120, 400, 100, 10],[30, 230, 70, 50],[40, 260, 60, 60],[330, 220, 120, 40],[330,150, 5, 70],[440, 150, 5, 70],[1999,5332,1000,400],[444,2424,555,1344]]




nmap = numpy.array([[0 for x in range(10000)] for y in range(10000)])
setup_map(nmap)

start = time.time()
print (Slider().compute(10, nmap, (20,20), (8045,4000)))

print ("Elapsed: ", time.time()-start)
