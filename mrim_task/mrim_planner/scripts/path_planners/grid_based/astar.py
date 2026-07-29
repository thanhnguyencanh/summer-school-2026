"""
A-star path planner
@author: F. Nekovar
@maintainer: P. Petracek
"""

import heapq, math, time
import numpy as np
from numpy import sqrt

# # #{ class AStar
class AStar():

    def __init__(self, grid, safety_distance, timeout, straighten=True):
        self.grid            = grid
        self.safety_distance = safety_distance
        self.straighten      = straighten
        self.timeout         = timeout

        # 26-neighborhood with precomputed step costs
        self.neighborhood = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    self.neighborhood.append((dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz)))

        # mild greediness (weighted A*) speeds up the search considerably while the
        # path straightening removes most of the incurred suboptimality afterwards
        self.heuristic_weight = 1.1

    # # #{ dist()
    def dist(self, first, second):
        a = first[0] - second[0]
        b = first[1] - second[1]
        c = first[2] - second[2]
        return math.sqrt(a**2 + b**2 + c**2)
    # # #}

    # # #{ nearestFreeCell()
    def nearestFreeCell(self, idx, max_radius=10):
        '''
        Returns the given cell if it is free, otherwise the nearest free cell within
        max_radius cells (or None if no free cell is found).
        '''
        occ  = self.occ
        dims = occ.shape

        idx = tuple(int(np.clip(idx[i], 0, dims[i] - 1)) for i in range(3))

        if not occ[idx]:
            return idx

        for r in range(1, max_radius + 1):
            x0, x1 = max(0, idx[0] - r), min(dims[0], idx[0] + r + 1)
            y0, y1 = max(0, idx[1] - r), min(dims[1], idx[1] + r + 1)
            z0, z1 = max(0, idx[2] - r), min(dims[2], idx[2] + r + 1)

            free = np.argwhere(~occ[x0:x1, y0:y1, z0:z1])
            if free.size:
                cells = free + np.array([x0, y0, z0])
                d2    = np.sum((cells - np.array(idx))**2, axis=1)
                return tuple(int(v) for v in cells[int(np.argmin(d2))])

        return None
    # # #}

    # # #{ generatePath()
    def generatePath(self, m_start, m_goal):

        print("[INFO] A*: Searching for path from [{:.2f}, {:.2f}, {:.2f}] to [{:.2f}, {:.2f}, {:.2f}].".format(m_start[0], m_start[1], m_start[2], m_goal[0], m_goal[1], m_goal[2]))

        self.occ = np.asarray(self.grid.array, dtype=bool)

        start = self.grid.metricToIndex(m_start)
        goal  = self.grid.metricToIndex(m_goal)

        # snap the endpoints to the nearest free cells (the exact metric endpoints are restored below)
        start = self.nearestFreeCell(start)
        goal  = self.nearestFreeCell(goal)

        if start is None or goal is None:
            print("[ERROR] A*: start or goal cell is occupied and there is no free cell nearby!")
            return None, None

        path = self.searchPath(start, goal)

        if path is None:
            print("[ERROR] A* did not find any path!")
            return None, None

        if self.straighten:
            path = self.halveAndTest(path)
            path = self.greedyShortcut(path)

        path_m = [self.grid.indexToMetric(node) for node in path]

        # replace the path endpoints with the exact metric coordinates (incl. headings)
        start_hdg = m_start[3] if len(m_start) > 3 else None
        goal_hdg  = m_goal[3]  if len(m_goal)  > 3 else None

        path_m[0] = (m_start[0], m_start[1], m_start[2], start_hdg)
        if len(path_m) > 1:
            path_m[-1] = (m_goal[0], m_goal[1], m_goal[2], goal_hdg)
        else:
            path_m.append((m_goal[0], m_goal[1], m_goal[2], goal_hdg))

        distance = 0.0
        for i in range(1, len(path_m)):
            distance += self.dist(path_m[i - 1], path_m[i])

        return path_m, distance
    # # #}

    # # #{ searchPath()
    def searchPath(self, start, goal):

        start_time = time.time()

        occ  = self.occ
        dims = occ.shape

        g_grid = np.full(dims, np.inf, dtype=np.float64)
        closed = np.zeros(dims, dtype=bool)

        gx, gy, gz = goal
        w          = self.heuristic_weight

        g_grid[start] = 0.0
        came          = {}
        heap          = [(w * self.dist(start, goal), start)]

        pops = 0
        while heap:

            f, pos = heapq.heappop(heap)

            if pos == goal:
                # reconstruct the path from goal to start
                path = [pos]
                while pos in came:
                    pos = came[pos]
                    path.append(pos)
                path.reverse()
                return path

            if closed[pos]:
                continue
            closed[pos] = True

            pops += 1
            if pops % 2048 == 0 and time.time() - start_time > self.timeout:
                print("[ERROR] A*: Timeout limit in searchPath() exceeded ({:.1f} s > {:.1f} s). Ending.".format(time.time() - start_time, self.timeout))
                return None

            x, y, z = pos
            g_pos   = g_grid[pos]

            for dx, dy, dz, c in self.neighborhood:
                nx, ny, nz = x + dx, y + dy, z + dz

                if nx < 0 or ny < 0 or nz < 0 or nx >= dims[0] or ny >= dims[1] or nz >= dims[2]:
                    continue

                npos = (nx, ny, nz)
                if occ[npos] or closed[npos]:
                    continue

                ng = g_pos + c
                if ng < g_grid[npos]:
                    g_grid[npos] = ng
                    came[npos]   = pos

                    # Euclidean distance to goal: admissible and consistent heuristic
                    h = math.sqrt((nx - gx)**2 + (ny - gy)**2 + (nz - gz)**2)
                    heapq.heappush(heap, (ng + w * h, npos))

        print("[ERROR] A*: open node queue is empty, could not find path!")
        return None
    # # #}

    # # #{ halveAndTest()
    def halveAndTest(self, path):
        '''
        Recursively straightens the path: if the straight connection of the path endpoints
        is collision-free, the path is replaced by it; otherwise, the path is divided in
        half and both halves are straightened recursively.
        '''
        if len(path) <= 2:
            return path

        pt1 = path[0]
        pt2 = path[-1]

        if self.grid.obstacleBetween(pt1, pt2):
            mid  = len(path) // 2
            seg1 = self.halveAndTest(path[:mid + 1])
            seg2 = self.halveAndTest(path[mid:])
            return seg1[:-1] + seg2

        return [pt1, pt2]
    # # #}

    # # #{ greedyShortcut()
    def greedyShortcut(self, path):
        '''
        Greedy forward pass over the (already straightened) path: from each node, jump to
        the farthest node visible in a straight, collision-free line.
        '''
        if len(path) <= 2:
            return path

        out = [path[0]]
        i   = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and self.grid.obstacleBetween(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j

        return out
    # # #}

# # #}
