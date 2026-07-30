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

        # inspection points by index (needed by the shell-pose optimization)
        self._ip_by_idx = {ip.idx: ip for ip in problem.inspection_points}

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

        n = len(self.distances)

        # choose the actual inspection poses: either the nominal viewpoints, or
        # (with shell_dp enabled) the time-optimal poses on the inspection
        # tolerance shells found by dynamic programming along the fixed sequence
        shell_dp = bool(path_planner.get('shell_dp', False))
        if shell_dp:
            tour_poses = self.optimizeShellPoses(viewpoints, sequence, path_planner)
        else:
            tour_poses = [viewpoints[idx].pose for idx in sequence]

        path = []

        for a in range(n):
            b = (a + 1) % n

            # if the paths are already computed (and the tour poses are the nominal viewpoints)
            if not shell_dp and path_planner['distance_estimation_method'] == path_planner['path_planning_method']:
                actual_path = self.paths[(sequence[a], sequence[b])]
            # otherwise, plan the leg between the chosen tour poses
            else:
                actual_path, _ = self.compute_path(tour_poses[a], tour_poses[b], path_planner, path_planner['path_planning_method'])

            # join paths
            path = path + actual_path[:-1]

            # force flight to end point
            if a == (n - 1):
                path = path + [tour_poses[0]]
        return path

    # #}

    # #{ optimizeShellPoses()

    def optimizeShellPoses(self, viewpoints, sequence, path_planner):
        '''
        For the fixed tour sequence, chooses for every inspection point the
        time-optimal pose on its inspection-tolerance shell by dynamic programming.

        The candidates lie EXACTLY on the nominal inspection sphere (full distance
        tolerance is kept as margin) with the EXACT inspection heading (full heading
        tolerance kept as margin) — only the position on the sphere within a cone
        around the nominal viewpoint direction varies. The nominal viewpoint is
        always among the candidates, hence the result can never be worse than the
        nominal tour.

        Parameters:
            viewpoints (list[Viewpoint]): the viewpoints (index 0 = start pose)
            sequence (list[int]): tour sequence over the viewpoints
            path_planner (dict): dictionary of parameters

        Returns:
            tour_poses (list[Pose]): chosen pose for each tour position (start first)
        '''

        vp_dist    = path_planner.get('viewpoints_distance', 4.0)
        cone_angle = path_planner.get('shell_dp/cone_angle', 0.55)

        n = len(sequence)

        # candidate poses per tour position (start pose is fixed)
        layers = [[viewpoints[sequence[0]].pose]]
        for k in range(1, n):
            vp = viewpoints[sequence[k]]
            ip = self._ip_by_idx.get(vp.idx)
            layers.append(self.shellCandidates(vp, ip, vp_dist, cone_angle, path_planner))

        # positions as arrays for the vectorized transition costs
        arrays = [np.array([[p.point.x, p.point.y, p.point.z] for p in layer]) for layer in layers]

        # heading of each layer (constant within a layer)
        headings = [layer[0].heading for layer in layers]

        (v_x, v_y, v_z), (a_x, a_y, a_z), hdg_rate = path_planner['dynamics']

        # forward DP over the layers, closing the tour back at the start
        costs   = [np.zeros(len(layers[0]))]
        parents = [None]
        for k in range(1, n + 1):
            src = arrays[k - 1]
            dst = arrays[k] if k < n else arrays[0]

            T = self._pairTimeMatrix(src, dst, (v_x, v_y, v_z), (a_x, a_y, a_z))

            # the heading-rotation time is a lower bound common to all candidate pairs
            h_from = headings[k - 1]
            h_to   = headings[k] if k < n else headings[0]
            if h_from is not None and h_to is not None and hdg_rate > 1e-3:
                T = np.maximum(T, abs(angleDiff(h_from, h_to)) / hdg_rate)

            total     = costs[-1][:, None] + T
            parent    = np.argmin(total, axis=0)
            costs.append(np.min(total, axis=0))
            parents.append(parent)

        # reconstruct the chosen candidate indices (the tour ends at the single start pose)
        chosen     = [0] * n
        best_last  = int(parents[n][0])
        chosen[n - 1] = best_last
        for k in range(n - 1, 0, -1):
            chosen[k - 1] = int(parents[k][chosen[k]])

        tour_poses = [layers[k][chosen[k]] for k in range(n)]

        return tour_poses

    # #}

    # #{ shellCandidates()

    def shellCandidates(self, vp, ip, vp_dist, cone_angle, path_planner):
        '''
        Generates collision-free candidate poses on the inspection-tolerance shell
        of the given viewpoint: points on the sphere of the exact nominal radius
        around the inspection point, within a cone around the nominal viewpoint
        direction, all with the exact inspection heading.
        '''

        nominal = Pose(vp.pose.point.x, vp.pose.point.y, vp.pose.point.z, vp.pose.heading)

        if ip is None:
            return [nominal]

        ip_pos = np.array([ip.position.x, ip.position.y, ip.position.z])
        vp_pos = np.array([vp.pose.point.x, vp.pose.point.y, vp.pose.point.z])

        axis_len = np.linalg.norm(vp_pos - ip_pos)
        if axis_len < 1e-6:
            return [nominal]
        axis = (vp_pos - ip_pos) / axis_len

        # orthonormal basis perpendicular to the nominal direction
        ref = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.95 else np.array([1.0, 0.0, 0.0])
        e1  = np.cross(axis, ref)
        e1 /= np.linalg.norm(e1)
        e2  = np.cross(axis, e1)

        # rings of directions around the nominal axis
        dirs = [axis]
        for alpha in (0.5 * cone_angle, cone_angle):
            for az in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False):
                d = np.cos(alpha) * axis + np.sin(alpha) * (np.cos(az) * e1 + np.sin(az) * e2)
                dirs.append(d / np.linalg.norm(d))

        positions = ip_pos + vp_dist * np.vstack(dirs)

        # filter the candidates: obstacle clearance, bounds, keepout zones
        candidates = [nominal]
        safety     = path_planner['safety_distance']
        bounds     = path_planner.get('bounds')
        keepouts   = path_planner.get('extra_keepout', [])

        clearances, _ = path_planner['obstacles_kdtree'].query(positions, k=1)

        for i in range(1, len(positions)):
            p = positions[i]
            if clearances[i] <= safety:
                continue
            if bounds is not None and not bounds.valid(Point(p[0], p[1], p[2])):
                continue
            if any(np.linalg.norm(p - np.array(kp[0:3])) < kp[3] + 0.3 for kp in keepouts):
                continue
            candidates.append(Pose(float(p[0]), float(p[1]), float(p[2]), nominal.heading))

        return candidates

    # #}

    # #{ _pairTimeMatrix()

    def _pairTimeMatrix(self, src, dst, v_lims, a_lims):
        '''
        Vectorized straight-line flight-time estimates between all pairs of the
        given position arrays (trapezoidal per-axis velocity profiles).

        Parameters:
            src (np.array [n, 3]), dst (np.array [m, 3])

        Returns:
            T (np.array [n, m]): time estimates in seconds
        '''

        def trapezoid(dist, v_max, a_max):
            d_ramps = v_max * v_max / a_max
            return np.where(dist >= d_ramps,
                            dist / v_max + v_max / a_max,
                            2.0 * np.sqrt(np.maximum(dist, 0.0) / a_max))

        diff = dst[None, :, :] - src[:, None, :]
        dxy  = np.sqrt(diff[:, :, 0]**2 + diff[:, :, 1]**2)
        dz   = np.abs(diff[:, :, 2])

        t_xy = trapezoid(dxy, min(v_lims[0], v_lims[1]), min(a_lims[0], a_lims[1]))
        t_z  = trapezoid(dz, v_lims[2], a_lims[2])

        return np.maximum(t_xy, t_z)

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
