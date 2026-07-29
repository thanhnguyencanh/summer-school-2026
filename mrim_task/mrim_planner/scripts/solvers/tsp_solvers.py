"""
Various types of TSP utilizing local planners for distance estimation and path planning
@author: P. Petracek & V. Kratky & P.Vana & P.Cizek & R.Penicka
"""

import time
import numpy as np

from random import randint
from itertools import permutations

from sklearn.cluster import KMeans

try:
    from scipy.spatial.kdtree import KDTree
except ImportError:
    from scipy.spatial import KDTree

from utils import *
from path_planners.grid_based.grid_3d import Grid3D
from path_planners.grid_based.astar   import AStar
from path_planners.sampling_based.rrt import RRT

from solvers.LKHInvoker import LKHInvoker

class TSPSolver3D():

    ALLOWED_PATH_PLANNERS               = ('euclidean', 'astar', 'rrt', 'rrtstar')
    ALLOWED_DISTANCE_ESTIMATION_METHODS = ('euclidean', 'euclidean_time', 'astar', 'rrt', 'rrtstar')
    GRID_PLANNERS                       = ('astar')

    def __init__(self):
        self.lkh         = LKHInvoker()
        self._cost_cache = {}

    # # #{ setup()
    def setup(self, problem, path_planner, viewpoints, build_grid=True):
        """setup objects required in path planning methods"""

        if path_planner is None:
            return

        assert path_planner['path_planning_method'] in self.ALLOWED_PATH_PLANNERS, 'Given method to compute path (%s) is not allowed. Allowed methods: %s' % (path_planner, self.ALLOWED_PATH_PLANNERS)
        assert path_planner['distance_estimation_method'] in self.ALLOWED_DISTANCE_ESTIMATION_METHODS, 'Given method for distance estimation (%s) is not allowed. Allowed methods: %s' % (path_planner, self.ALLOWED_DISTANCE_ESTIMATION_METHODS)

        # Setup environment: KD tree for collision queries and environment bounds
        # (built once, they are shared by all the planning methods)
        if 'obstacles_kdtree' not in path_planner:
            obstacles_array = np.array([[opt.x, opt.y, opt.z] for opt in problem.obstacle_points])
            path_planner['obstacles_kdtree'] = KDTree(obstacles_array)

        if 'bounds' not in path_planner:
            xs = [p.x for p in problem.safety_area]
            ys = [p.y for p in problem.safety_area]
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            path_planner['bounds'] = Bounds(Point(x_min, y_min, problem.min_height), Point(x_max, y_max, problem.max_height))

        # Setup 3D grid for grid-based planners
        if build_grid and (path_planner['path_planning_method'] in self.GRID_PLANNERS or path_planner['distance_estimation_method'] in self.GRID_PLANNERS):

            # construct grid
            x_list = [opt.x for opt in problem.obstacle_points]
            x_list.extend([vp.pose.point.x for vp in viewpoints])
            y_list = [opt.y for opt in problem.obstacle_points]
            y_list.extend([vp.pose.point.y for vp in viewpoints])
            z_list = [opt.z for opt in problem.obstacle_points]
            z_list.extend([vp.pose.point.z for vp in viewpoints])

            min_x = np.min(x_list) - path_planner['safety_distance']
            max_x = np.max(x_list) + path_planner['safety_distance']
            min_y = np.min(y_list) - path_planner['safety_distance']
            max_y = np.max(y_list) + path_planner['safety_distance']
            min_z = problem.min_height
            max_z = problem.max_height

            dim_x = int(np.floor((max_x - min_x) / path_planner['astar/grid_resolution']))+1
            dim_y = int(np.floor((max_y - min_y) / path_planner['astar/grid_resolution']))+1
            dim_z = int(np.floor((max_z - min_z) / path_planner['astar/grid_resolution']))+1

            path_planner['grid'] = Grid3D(idx_zero = (min_x, min_y,min_z), dimensions=(dim_x,dim_y,dim_z), resolution_xyz=path_planner['astar/grid_resolution'])
            path_planner['grid'].setObstacles(problem.obstacle_points, path_planner['safety_distance'])

            # block the keepout zones (e.g., the start location of the other UAV where it may be parked)
            for kp in path_planner.get('extra_keepout', []):
                path_planner['grid'].addKeepoutSphere(kp[0:3], kp[3])

    # # #}

    # #{ plan_tour()

    def plan_tour(self, problem, viewpoints, path_planner=None):
        '''
        Solve TSP on viewpoints with given goals and starts

        Parameters:
            problem (InspectionProblem): task problem
            viewpoints (list[Viewpoint]): list of Viewpoint objects
            path_planner (dict): dictionary of parameters

        Returns:
            path (list): sequence of points with start equaling the end
        '''

        # Setup 3D grid for grid-based planners and KDtree for sampling-based planners
        self.setup(problem, path_planner, viewpoints)

        n              = len(viewpoints)
        self.distances = np.zeros((n, n))
        self.paths = {}

        dist_method = path_planner['distance_estimation_method'] if path_planner is not None else 'euclidean'

        # Estimate the distances between the viewpoints for the TSP
        if dist_method in ('euclidean', 'euclidean_time'):

            # closed-form estimates: no pairwise path planning is required, the
            # (possibly time-based) estimates are computed directly from the poses
            for a in range(n):
                for b in range(a + 1, n):

                    g1 = viewpoints[a].pose
                    g2 = viewpoints[b].pose

                    if dist_method == 'euclidean':
                        cost = distEuclidean(g1, g2)
                    else:
                        cost = self.estimatePairCost(g1, g2, path_planner)

                    self.distances[a][b] = cost
                    self.distances[b][a] = cost

                    self.paths[(a, b)] = [g1, g2]
                    self.paths[(b, a)] = [g2, g1]

        else:

            # find path between each pair of goals (a, b)
            for a in range(n):
                for b in range(n):
                    if a == b:
                        continue

                    # get poses of the viewpoints
                    g1 = viewpoints[a].pose
                    g2 = viewpoints[b].pose

                    # estimate distances between the viewpoints
                    path, distance = self.compute_path(g1, g2, path_planner, path_planner['distance_estimation_method'])

                    # store paths/distances in matrices
                    self.paths[(a, b)]   = path
                    self.distances[a][b] = distance

        # compute TSP tour
        path = self.compute_tsp_tour(viewpoints, path_planner)

        return path

    # #}

    # # #{ estimatePairCost()

    def estimatePairCost(self, g1, g2, path_planner):
        '''
        Estimates the flight time between two poses. Straight-line connections
        obstructed by obstacles are penalized by a constant detour factor to
        bias the TSP towards unobstructed hops. The results are cached.

        Parameters:
            g1, g2 (Pose): the two poses
            path_planner (dict): dictionary of parameters

        Returns:
            cost (float): estimated flight time in seconds
        '''

        key = (round(g1.point.x, 3), round(g1.point.y, 3), round(g1.point.z, 3),
               round(g2.point.x, 3), round(g2.point.y, 3), round(g2.point.z, 3))

        if key in self._cost_cache:
            return self._cost_cache[key]

        cost = estimateFlightTime(g1, g2, path_planner['dynamics'])

        if self.lineCollides(g1, g2, path_planner):
            cost *= path_planner.get('detour_penalty', 1.5)

        self._cost_cache[key] = cost
        self._cost_cache[(key[3], key[4], key[5], key[0], key[1], key[2])] = cost

        return cost

    # # #}

    # # #{ lineCollides()

    def lineCollides(self, g1, g2, path_planner, step=0.5):
        '''
        Checks whether the straight line between two poses passes closer than the
        planning safety distance to any obstacle.
        '''

        if 'obstacles_kdtree' not in path_planner:
            return False

        p1 = np.array(g1.point.asList())
        p2 = np.array(g2.point.asList())

        dist  = np.linalg.norm(p2 - p1)
        n_pts = max(2, int(np.ceil(dist / step)) + 1)
        pts   = np.linspace(p1, p2, n_pts)

        dists, _ = path_planner['obstacles_kdtree'].query(pts, k=1)

        return bool(np.min(dists) < path_planner['safety_distance'])

    # # #}

    # # #{ compute_path()

    def compute_path(self, p_from, p_to, path_planner, path_planner_method):
        '''
        Computes collision-free path (if feasible) between two points.
        If the primary method fails, the remaining methods are tried as fallbacks
        such that a single unlucky planning failure does not kill the whole mission.

        Parameters:
            p_from (Pose): start
            p_to (Pose): to
            path_planner (dict): dictionary of parameters
            path_planner_method (string): method of path planning

        Returns:
            path (list[Pose]): sequence of points
            distance (float): length of path
        '''

        # Use Euclidean metric
        if path_planner is None or path_planner_method in ('euclidean', 'euclidean_time'):
            return [p_from, p_to], distEuclidean(p_from, p_to)

        # primary method first, then the fallbacks
        methods = [path_planner_method]
        for fallback in ('astar', 'rrt', 'euclidean'):
            if fallback in methods:
                continue
            if fallback == 'astar' and 'grid' not in path_planner:
                continue
            methods.append(fallback)

        path, distance = None, float('inf')

        for method in methods:

            if method == 'euclidean':
                print('[WARN] compute_path(): all path planners failed, falling back to straight-line connection. The path may violate the obstacle distance constraint!')
                path, distance = [p_from, p_to], distEuclidean(p_from, p_to)

            # Plan with A*
            elif method == 'astar':

                astar = AStar(path_planner['grid'], path_planner['safety_distance'], path_planner['timeout'], path_planner['straighten'])
                path, distance = astar.generatePath(p_from.asList(), p_to.asList())
                if path:
                    path = [Pose(p[0], p[1], p[2], p[3]) for p in path]

            # Plan with RRT/RRT*
            elif method.startswith('rrt'):

                rrt = RRT()
                path, distance = rrt.generatePath(p_from.asList(), p_to.asList(), path_planner, rrtstar=(method == 'rrtstar'), straighten=path_planner['straighten'])
                if path:
                    path = [Pose(p[0], p[1], p[2], p[3]) for p in path]

            if path:

                # enforce the exact endpoint poses (incl. headings): the grid-based planners
                # would otherwise end up to half a grid cell off the viewpoints
                path[0]  = Pose(p_from.point.x, p_from.point.y, p_from.point.z, p_from.heading)
                path[-1] = Pose(p_to.point.x, p_to.point.y, p_to.point.z, p_to.heading)

                if method != path_planner_method:
                    print('[WARN] compute_path(): method {:s} failed, fallback {:s} succeeded.'.format(path_planner_method, method))

                return path, distance

        rospy.logerr('No path found. Shutting down.')
        rospy.signal_shutdown('No path found. Shutting down.');
        exit(-2)

    # # #}

    # #{ compute_tsp_tour()

    def compute_tsp_tour(self, viewpoints, path_planner):
        '''
        Compute the shortest tour based on the distance matrix (self.distances) and connect the path throught waypoints

        Parameters:
            viewpoints (list[Viewpoint]): list of VPs
            path_planner (dict): dictionary of parameters

        Returns:
            path (list[Poses]): sequence of points with start equaling the end
        '''

        # compute the shortest sequence given the distance matrix
        sequence = self.compute_tsp_sequence()

        path = []
        n    = len(self.distances)

        for a in range(n):
            b     = (a + 1) % n
            a_idx = sequence[a]
            b_idx = sequence[b]

            # if the paths are already computed
            if path_planner['distance_estimation_method'] == path_planner['path_planning_method']:
                actual_path = self.paths[(a_idx, b_idx)]
            # if the path planning and distance estimation methods differ, we need to compute the path
            else:
                actual_path, _ = self.compute_path(viewpoints[a_idx].pose, viewpoints[b_idx].pose, path_planner, path_planner['path_planning_method'])

            # join paths
            path = path + actual_path[:-1]

            # force flight to end point
            if a == (n - 1):
                path = path + [viewpoints[b_idx].pose]
        return path

    # #}

    # # #{ compute_tsp_sequence()

    def compute_tsp_sequence(self):
        '''
        Compute the shortest sequence based on the distance matrix (self.distances) using LKH

        Returns:
            sequence (list): sequence of viewpoints ordered optimally w.r.t the distance matrix
        '''

        n = len(self.distances)

        # trivial tours do not require the LKH solver
        if n <= 3:
            return list(range(n))

        try:
            fname_tsp = "problem"
            user_comment = "a comment by the user"
            self.lkh.writeTSPLIBfile_FE(fname_tsp, self.distances, user_comment)
            self.lkh.run_LKHsolver_cmd(fname_tsp, silent=True)
            sequence = self.lkh.read_LKHresult_cmd(fname_tsp)
        except Exception as e:
            # LKH failed (missing binary, I/O error, ...): fall back to the built-in
            # nearest-neighbor + 2-opt solver rather than killing the whole mission
            print('[WARN] compute_tsp_sequence(): LKH failed ({:s}), falling back to the built-in 2-opt solver.'.format(str(e)))
            _, sequence = self._solveSmallTSP(self.distances)
            return sequence

        if len(sequence) > 0 and sequence[0] is not None:
            for i in range(len(sequence)):
                if sequence[i] is None:
                    new_sequence = sequence[i:len(sequence)] + sequence[:i]
                    sequence = new_sequence
                    break

        # rotate the sequence such that it starts at the start viewpoint (index 0)
        if 0 in sequence and sequence[0] != 0:
            i        = sequence.index(0)
            sequence = sequence[i:] + sequence[:i]

        return sequence

    # # #}

    # #{ clusterViewpoints()

    def clusterViewpoints(self, problem, viewpoints, method, forced_radius=0.0):
        '''
        Clusters viewpoints into K (number of robots) clusters.

        Parameters:
            problem (InspectionProblem): task problem
            viewpoints (list): list of Viewpoint objects
            method (string): method ('random', 'kmeans')
            forced_radius (float): viewpoints closer than this radius to a robot start
                                   position are forced to that robot (their vicinity is
                                   unreachable by the other robot due to the keepout zones)

        Returns:
            clusters (Kx list): clusters of points indexed for each robot:
        '''
        k = problem.number_of_robots

        starts = np.array([[problem.start_poses[r].position.x,
                            problem.start_poses[r].position.y,
                            problem.start_poses[r].position.z] for r in range(k)])

        ## | ------------------- K-Means clustering ------------------- |
        if method == 'kmeans' and len(viewpoints) >= k:

            # Prepare positions of the viewpoints in the world
            positions = np.array([vp.pose.point.asList() for vp in viewpoints])

            # cluster into k groups (deterministic given the fixed random_state)
            kmeans     = KMeans(n_clusters=k, n_init=10, random_state=42).fit(positions)
            raw_labels = list(kmeans.labels_)
            centers    = kmeans.cluster_centers_

            # map the cluster labels to the robots by minimizing the total distance
            # between the cluster centers and the robot start positions
            best_perm, best_cost = None, None
            for perm in permutations(range(k)):
                cost = sum(np.linalg.norm(centers[c] - starts[perm[c]]) for c in range(k))
                if best_cost is None or cost < best_cost:
                    best_perm, best_cost = perm, cost

            labels = [best_perm[l] for l in raw_labels]

        elif method == 'kmeans':

            # fewer shared viewpoints than robots: assign each to the closest robot start
            labels = [int(np.argmin([np.linalg.norm(np.array(vp.pose.point.asList()) - starts[r]) for r in range(k)])) for vp in viewpoints]

        ## | -------------------- Random clustering ------------------- |
        else:
            labels = [randint(0, k - 1) for vp in viewpoints]

        # force viewpoints in the close vicinity of a robot start position to that robot
        if forced_radius > 0.0:
            for i in range(len(viewpoints)):
                dists = [np.linalg.norm(np.array(viewpoints[i].pose.point.asList()) - starts[r]) for r in range(k)]
                r_min = int(np.argmin(dists))
                if dists[r_min] < forced_radius:
                    labels[i] = r_min

        # Store as clusters (2D array of viewpoints)
        clusters = []
        for r in range(k):
            clusters.append([])

            for label in range(len(labels)):
                if labels[label] == r:
                    clusters[r].append(viewpoints[label])

        return clusters

    # #}

    # #{ balanceViewpoints()

    def balanceViewpoints(self, problem, viewpoints, movable_idxs, path_planner, rounds=30, time_budget=20.0):
        '''
        Re-balances the shared viewpoints between the two robots to minimize the makespan
        (the maximum estimated tour time over the robots). Iteratively moves the best
        shared viewpoint from the robot with the longer tour to the other robot as long
        as the makespan estimate improves.

        Parameters:
            problem (InspectionProblem): task problem
            viewpoints (list[list[Viewpoint]]): per-robot viewpoint lists (incl. the start pose at index 0)
            movable_idxs (set[int]): indices of the viewpoints which may be moved between the robots
            path_planner (dict): dictionary of parameters
            rounds (int): maximum number of moves
            time_budget (float): maximum time in seconds

        Returns:
            viewpoints (list[list[Viewpoint]]): re-balanced per-robot viewpoint lists
        '''

        if len(viewpoints) != 2 or not movable_idxs:
            return viewpoints

        # the estimator needs the collision KD tree
        all_vps = [vp for vps in viewpoints for vp in vps]
        self.setup(problem, path_planner, all_vps, build_grid=False)

        t_start = time.time()

        times = [self.estimateTourTime(vps, path_planner) for vps in viewpoints]
        print('[BALANCING VIEWPOINTS] initial tour time estimates: {:.1f} s / {:.1f} s'.format(times[0], times[1]))

        for _ in range(rounds):

            if time.time() - t_start > time_budget:
                print('[BALANCING VIEWPOINTS] time budget exceeded, stopping.')
                break

            longer  = int(np.argmax(times))
            shorter = 1 - longer

            best = None
            for vp in viewpoints[longer][1:]:

                if vp.idx not in movable_idxs:
                    continue

                cand_long  = [v for v in viewpoints[longer] if v.idx != vp.idx]
                cand_short = viewpoints[shorter] + [vp]

                t_long  = self.estimateTourTime(cand_long, path_planner)
                t_short = self.estimateTourTime(cand_short, path_planner)

                makespan = max(t_long, t_short)
                if best is None or makespan < best[0]:
                    best = (makespan, vp, t_long, t_short)

            if best is None or best[0] >= max(times) - 1e-3:
                break

            _, vp, t_long, t_short = best
            viewpoints[longer]  = [v for v in viewpoints[longer] if v.idx != vp.idx]
            viewpoints[shorter] = viewpoints[shorter] + [vp]
            times               = [t_long, t_short] if longer == 0 else [t_short, t_long]

            print('[BALANCING VIEWPOINTS] moved VP {:d} from robot {:d} to robot {:d}, tour time estimates: {:.1f} s / {:.1f} s'.format(vp.idx, longer, shorter, times[0], times[1]))

        return viewpoints

    # #}

    # #{ estimateTourTime()

    def estimateTourTime(self, viewpoints, path_planner):
        '''
        Estimates the tour time over the given viewpoints (closed tour starting and
        ending at the viewpoint with index 0 in the list) using a nearest-neighbor
        construction followed by a 2-opt improvement on the flight time estimates.

        Parameters:
            viewpoints (list[Viewpoint]): viewpoints (the first is the start)
            path_planner (dict): dictionary of parameters

        Returns:
            cost (float): estimated tour time in seconds
        '''

        n = len(viewpoints)
        if n <= 1:
            return 0.0

        M = np.zeros((n, n))
        for a in range(n):
            for b in range(a + 1, n):
                cost = self.estimatePairCost(viewpoints[a].pose, viewpoints[b].pose, path_planner)
                M[a][b] = M[b][a] = cost

        cost, _ = self._solveSmallTSP(M)
        return cost

    # #}

    # #{ _solveSmallTSP()

    def _solveSmallTSP(self, M):
        '''
        Quickly solves a small TSP instance given the distance matrix M: greedy
        nearest-neighbor tour construction followed by a 2-opt improvement.

        Returns:
            cost (float): tour cost
            order (list[int]): the tour (closed, starting at index 0)
        '''

        n = len(M)
        if n <= 2:
            return float(2.0 * sum(M[0][1:2])), list(range(n))

        # nearest-neighbor construction
        order     = [0]
        unvisited = set(range(1, n))
        while unvisited:
            last = order[-1]
            nxt  = min(unvisited, key=lambda j: M[last][j])
            order.append(nxt)
            unvisited.remove(nxt)

        # 2-opt improvement
        improved = True
        while improved:
            improved = False
            for i in range(1, n - 1):
                for j in range(i + 1, n):
                    a, b = order[i - 1], order[i]
                    c, d = order[j], order[(j + 1) % n]
                    if a == c or b == d:
                        continue
                    delta = M[a][c] + M[b][d] - M[a][b] - M[c][d]
                    if delta < -1e-9:
                        order[i:j + 1] = order[i:j + 1][::-1]
                        improved       = True

        cost = sum(M[order[i]][order[(i + 1) % n]] for i in range(n))
        return float(cost), order

    # #}
