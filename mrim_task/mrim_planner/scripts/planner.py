#!/usr/bin/env python3
"""
Custom planner for multi-robot inspection
@author: P. Petracek, V. Kratky, R. Penicka, T. Baca
"""

import rospy, rospkg
import numpy as np
import time

from utils import *

from solvers.tsp_solvers import *
from trajectory import Trajectory, TrajectoryUtils

class MrimPlanner:

    ALLOWED_COLLISION_AVOIDANCE_METHODS = ['none', 'delay_2nd_till_1st_UAV_finishes', 'delay_till_no_collisions_occur']

    # # #{ __init__()
    def __init__(self):

        rospy.init_node('mrim_planner', anonymous=True)

        ## | ---------------------- load problem ---------------------- |
        problem_filename = rospy.get_param('~problem/name')
        session_problem = rospy.get_param('~session_problem')

        # prefer the session-defined problem if it is an actual problem file
        # (the launch scripts also pass run-type tags like 'offline' here)
        if session_problem is not None and str(session_problem).endswith('.problem'):
            print('[MrimPlanner] Using session problem: {:s}'.format(str(session_problem)))
            problem_filename = str(session_problem)

        problem_filepath = rospkg.RosPack().get_path('mrim_resources') + "/problems/" + problem_filename

        problem, log_msg = ProblemLoader().loadProblem(problem_filepath)

        if problem is None:
            rospy.logerr(log_msg)
            rospy.signal_shutdown(log_msg)
            exit(-1)

        ## |  load parameters from ROS custom config (mrim_task/mrim_planner/config/custom_config.yaml)  |
        self._viewpoints_distance    = rospy.get_param('~viewpoints/distance', 3.0)
        self._plot                     = rospy.get_param('~problem/plot', False)
        self._trajectory_dt            = rospy.get_param('~trajectories/dt', 0.2)
        self._smoothing_sampling_step  = rospy.get_param('~path_smoothing/sampling_step', 0.1)
        self._smoothing_distance       = rospy.get_param('~path_smoothing/lookahead_dist', 0.3)
        self._heading_hold_dist        = rospy.get_param('~path_smoothing/heading_hold_dist', 0.6)
        self._sample_with_stops        = rospy.get_param('~trajectory_sampling/with_stops', True)
        self._global_frame             = rospy.get_param('~global_frame', "gps_origin")
        self._tsp_clustering_method    = rospy.get_param('~tsp/clustering', 'random')
        self._balance_viewpoints       = rospy.get_param('~tsp/balance_viewpoints', False)
        self._balance_time_budget      = rospy.get_param('~tsp/balance_time_budget', 20.0)
        self._detour_penalty           = rospy.get_param('~tsp/detour_penalty', 1.5)

        # safety factor scaling the dynamic constraints used for planning: leaves
        # a margin for numerical effects of the discrete trajectory sampling such
        # that the strict constraint checks of the evaluator are always satisfied
        self._dynamics_safety_factor   = rospy.get_param('~trajectories/dynamics_safety_factor', 0.95)
        sf                             = self._dynamics_safety_factor

        max_vel_x                      = rospy.get_param('~dynamic_constraints/max_velocity/x', 1.0)
        max_vel_y                      = rospy.get_param('~dynamic_constraints/max_velocity/y', 1.0)
        max_vel_z                      = rospy.get_param('~dynamic_constraints/max_velocity/z', 1.0)
        max_acc_x                      = rospy.get_param('~dynamic_constraints/max_acceleration/x', 1.0)
        max_acc_y                      = rospy.get_param('~dynamic_constraints/max_acceleration/y', 1.0)
        max_acc_z                      = rospy.get_param('~dynamic_constraints/max_acceleration/z', 1.0)
        self._max_velocity             = (sf * max_vel_x, sf * max_vel_y, sf * max_vel_z)
        self._max_acceleration         = (sf * max_acc_x, sf * max_acc_y, sf * max_acc_z)
        self._max_heading_rate         = sf * rospy.get_param('~dynamic_constraints/max_heading_rate', 1.0)
        self._max_heading_acceleration = sf * rospy.get_param('~dynamic_constraints/max_heading_rate_acceleration', 1.0)

        ## | ---------------- setup collision avoidance --------------- |
        self._safety_distance_mutual = rospy.get_param('~trajectories/min_distance/mutual')
        self._collision_avoidance    = rospy.get_param('~collision_avoidance/method', 'none')
        self._mutual_collision_margin= rospy.get_param('~collision_avoidance/mutual_margin', 0.3)

        ## | ------------------- setup path planner ------------------- |
        self._path_planner = {}
        self._path_planner['timeout']                       = rospy.get_param('~path_planner/timeout', 1.0)
        self._path_planner['path_planning_method']          = rospy.get_param('~path_planner/method', 'rrt')
        self._path_planner['safety_distance']               = rospy.get_param('~trajectories/min_distance/obstacles') + rospy.get_param('~path_planner/obstacle_margin', 0.0)
        self._path_planner['distance_estimation_method']    = rospy.get_param('~tsp/distance_estimates', 'euclidean')
        self._path_planner['straighten']                    = rospy.get_param('~path_planner/straighten_paths')

        # dynamics used by the flight-time estimates in the TSP
        self._path_planner['dynamics']       = (self._max_velocity, self._max_acceleration, self._max_heading_rate)
        self._path_planner['detour_penalty'] = self._detour_penalty

        # optimization of the inspection poses on the tolerance shells
        self._shell_dp                             = rospy.get_param('~tsp/shell_dp/enabled', False)
        self._path_planner['shell_dp']             = self._shell_dp
        self._path_planner['shell_dp/cone_angle']  = rospy.get_param('~tsp/shell_dp/cone_angle', 0.55)
        self._path_planner['shell_dp/radius_slack']  = rospy.get_param('~tsp/shell_dp/radius_slack', 0.0)
        self._path_planner['shell_dp/heading_slack'] = rospy.get_param('~tsp/shell_dp/heading_slack', 0.0)
        self._path_planner['viewpoints_distance']  = self._viewpoints_distance

        # evaluation limit on the obstacle distance (nominal viewpoints violating it
        # are relocated on their tolerance spheres by the shell-pose optimization)
        self._path_planner['eval_obstacle_limit'] = rospy.get_param('~trajectories/check/obstacles', 1.5)

        # last-resort safety net state: viewpoints sacrificed to avoid a zero score
        self._exclude_vp_idxs = set()
        self._dropped_vp_count = 0

        # clearance-aware corner rounding during path smoothing
        self._corner_arcs       = rospy.get_param('~path_smoothing/corner_arcs', False)
        self._corner_arc_radius = rospy.get_param('~path_smoothing/corner_arc_radius', 4.0)

        # load the parameters of all the planners (any of them may be used as a fallback)
        self._path_planner['astar/grid_resolution'] = rospy.get_param('~path_planner/astar/grid_resolution', 0.4)
        self._path_planner['rrt/branch_size']       = rospy.get_param('~path_planner/rrt/branch_size', 3.0)
        self._path_planner['rrt/sampling/method']   = rospy.get_param('~path_planner/rrt/sampling/method', 'uniform')
        self._path_planner['rrtstar/neighborhood']  = rospy.get_param('~path_planner/rrt/star/neighborhood', 1.0)

        if self._path_planner['rrt/sampling/method'] == 'gaussian':
            self._path_planner['rrt/sampling/gaussian/stddev_inflation']  = rospy.get_param('~path_planner/rrt/sampling/gaussian/stddev_inflation', 0.2)

        ## | -------------- print out general parameters -------------- |
        # print('using parameters:')
        # print(' viewpoints distance:', self._viewpoints_distance)
        # print(' max velocity:', self._max_velocity)
        # print(' max acceleration:', self._max_acceleration)
        # print(' max heading rate:', self._max_heading_rate)
        # print(' max heading acceleration:', self._max_heading_acceleration)
        # print(' smoothing lookahead distance:', self._smoothing_distance)
        # print(' smoothing sampling step:', self._smoothing_sampling_step)
        # print(' plot:', self._plot)
        # print(' trajectory dT:', self._trajectory_dt)
        # print(' path planning method:', pp_method)
        # print(' distance estimation method:', de_method)

        ## | ----------------- initiate ROS publishers ---------------- |
        self.publisher_trajectory_1 = rospy.Publisher("~trajectory_1_out", TrajectoryReference, queue_size=1, latch=True)
        self.publisher_trajectory_2 = rospy.Publisher("~trajectory_2_out", TrajectoryReference, queue_size=1, latch=True)
        self.problem_publisher      = rospy.Publisher("~problem_out", InspectionProblem, queue_size=1, latch=True)

        rate = rospy.Rate(1.0)
        rate.sleep()

        ## | --------------------- publish problem -------------------- |
        self.problem_publisher.publish(problem)

        ## | -------------------- plan trajectories ------------------- |
        trajectories, plotter = self.planTrajectories(problem)

        # # | -------------- convert to ROS trajectories -------------- |
        ros_trajectory_1 = trajectoryToRosMsg(trajectories[0].getPoses(), self._global_frame)
        ros_trajectory_2 = trajectoryToRosMsg(trajectories[1].getPoses(), self._global_frame)

        ## | ------------------ publish trajectories ------------------ |
        self.publisher_trajectory_1.publish(ros_trajectory_1)
        self.publisher_trajectory_2.publish(ros_trajectory_2)

        plotter.show(legend=True)

        rospy.loginfo('Trajectories published, staying on, the publishers are latched.')
        rospy.spin()
    # # #}

    # # #{ planTrajectories()
    def planTrajectories(self, problem):

        stopwatch_start = time.time()

        # refresh the enhancement switches (they may have been disabled by the
        # conservative-replanning safety net)
        self._path_planner['shell_dp'] = self._shell_dp

        ## | --------------- create visualization object -------------- |
        plotter = ProblemPlotter(self._plot)
        plotter.addProblem(problem)

        ## | ----- initialize objects for TSP and trajectory utils ---- |
        tsp_solver       = TSPSolver3D()
        trajectory_utils = TrajectoryUtils(self._max_velocity, self._max_acceleration, self._trajectory_dt, heading_hold_dist=self._heading_hold_dist)

        # # #{ Cluster target locations

        print('[ASSIGNING VIEWPOINTS TO UAVs]')

        viewpoints       = []
        nonclustered_vps = []

        for r in range(problem.number_of_robots):

            # add starting pose of the robot
            start_vp = Viewpoint(0, Pose(problem.start_poses[r].position.x, problem.start_poses[r].position.y, problem.start_poses[r].position.z, problem.start_poses[r].heading))
            viewpoints.append([start_vp])

            # get robot ID
            robot_id = problem.robot_ids[r]
            for ip in problem.inspection_points:

                # viewpoints sacrificed by the last-resort safety net
                if ip.idx in self._exclude_vp_idxs:
                    continue

                # convert IP to VP [id x y z heading]
                viewpoint = inspectionPointToViewPoint(ip, self._viewpoints_distance)

                # if inspectability of IP is unique for robot with this ID, add it
                if len(ip.inspectability) == 1 and robot_id in ip.inspectability:
                    viewpoints[r].append(viewpoint)

                # if inspectability of IP is arbitrary, store it for clustering
                elif len(ip.inspectability) != 1 and ip.idx not in [nips.idx for nips in nonclustered_vps]:
                    nonclustered_vps.append(viewpoint)

        # keepout radius around the start position of the other UAV (it may be parked there)
        keepout_radius = self._safety_distance_mutual + self._mutual_collision_margin

        # Cluster the rest of the viewpoints into two separate groups
        clusters = tsp_solver.clusterViewpoints(problem, nonclustered_vps, method=self._tsp_clustering_method, forced_radius=keepout_radius + 0.2)
        for r in range(problem.number_of_robots):
            viewpoints[r].extend(clusters[r])

        # Re-balance the shared viewpoints between the robots to minimize the makespan
        if self._balance_viewpoints and problem.number_of_robots == 2:
            print('[BALANCING VIEWPOINTS]')

            # viewpoints in the close vicinity of a robot start must stay with that robot
            starts = [(problem.start_poses[r].position.x, problem.start_poses[r].position.y, problem.start_poses[r].position.z) for r in range(problem.number_of_robots)]
            movable = set()
            for vp in nonclustered_vps:
                p = vp.pose.point
                if all(np.linalg.norm([p.x - s[0], p.y - s[1], p.z - s[2]]) > keepout_radius + 0.2 for s in starts):
                    movable.add(vp.idx)

            viewpoints = tsp_solver.balanceViewpoints(problem, viewpoints, movable, self._path_planner, time_budget=self._balance_time_budget)

        # print out viewpoints
        for i in range(len(viewpoints)):
            print('viewpoints for UAV:', problem.robot_ids[i])
            for vp in viewpoints[i]:
                print('   [{:d}]:'.format(vp.idx), vp.pose)

        # add VPs to offline visualization
        plotter.addViewPoints(viewpoints, self._viewpoints_distance, self._viewpoints_distance)

        # # #}

        # Print out if the viewpoints collide with the environment
        for i in range(len(viewpoints)):
            for vp in viewpoints[i]:
                point = vp.pose.point
                if pointCollidesWithObstacles(point, [Point(o.x, o.y, o.z) for o in problem.obstacle_points], self._path_planner['safety_distance']):
                    rospy.logwarn('VP at %s collides with obstacles.', point)
        # plotter.show(legend=True)

        # # #{ Solve TSP to obtain waypoint path
        print('[PLANNING TSP TOUR]')

        waypoints = []
        for i in range(problem.number_of_robots):

            ## | -- keep out of the start locations of the other robots -- |
            self._path_planner['extra_keepout'] = []
            for j in range(problem.number_of_robots):
                if j != i:
                    sp = problem.start_poses[j]
                    self._path_planner['extra_keepout'].append((sp.position.x, sp.position.y, sp.position.z, keepout_radius))

            ## | --------------- Plan tour with a TSP solver -------------- |
            robot_waypoints = tsp_solver.plan_tour(problem, viewpoints[i], self._path_planner) # find decoupled TSP tour over viewpoints
            waypoints.append(robot_waypoints)

            print('[PLANNING TSP TOUR] robot {:d} done, elapsed time: {:.1f} s'.format(problem.robot_ids[i], time.time() - stopwatch_start))

            ## | ------------- add waypoints to visualization ------------- |
            plotter.addWaypoints(robot_waypoints, color=COLORS[i], lw=1.2, label='traj (id: ' + str(problem.robot_ids[i]) + ')')
        # # #}

        # # #{ Sample waypoints to trajectories
        trajectories     = []

        # create dynamic constraints
        constraints_velocity     = [self._max_velocity[0], self._max_velocity[1], self._max_velocity[2], self._max_heading_rate] # per axis velocity limits
        constraints_acceleration = [self._max_acceleration[0], self._max_acceleration[1], self._max_acceleration[2], self._max_heading_acceleration] # per axis acceleration limits

        ## | ------------------- Sample waypoints ------------------ |

        # for each robot
        for r in range(problem.number_of_robots):

            # generate trajectory for the robot's VPs
            print('[GENERATING TRAJECTORY] for robot with ID: {:d}'.format(problem.robot_ids[r]))
            trajectory = Trajectory(self._trajectory_dt, waypoints[r])

            # sample trajectory through its waypoints
            print("[SAMPLING TRAJECTORY]")
            trajectory = trajectory_utils.sampleTrajectoryThroughWaypoints(trajectory, with_stops=self._sample_with_stops,\
                                                                           smooth_path=True, smoothing_la_dist=self._smoothing_distance,\
                                                                           smoothing_sampling_step=self._smoothing_sampling_step,\
                                                                           velocity_limits=constraints_velocity,
                                                                           acceleration_limits=constraints_acceleration,
                                                                           corner_arcs=self._corner_arcs,
                                                                           obstacles_kdtree=self._path_planner.get('obstacles_kdtree'),
                                                                           obstacle_safety=self._path_planner['safety_distance'],
                                                                           corner_arc_radius=self._corner_arc_radius)

            # safety net: if the continuous sampling failed (e.g., TOPPRA could not
            # parametrize the path), retry with the always-feasible stop-at-waypoints
            # sampling instead of producing no trajectory at all
            if trajectory is None and not self._sample_with_stops:
                print('[SAMPLING TRAJECTORY] Continuous sampling failed, falling back to stop-at-waypoints sampling.')
                trajectory = Trajectory(self._trajectory_dt, waypoints[r])
                trajectory = trajectory_utils.sampleTrajectoryThroughWaypoints(trajectory, with_stops=True,\
                                                                               smooth_path=False, smoothing_la_dist=self._smoothing_distance,\
                                                                               smoothing_sampling_step=self._smoothing_sampling_step,\
                                                                               velocity_limits=constraints_velocity,
                                                                               acceleration_limits=constraints_acceleration)

            if trajectory is None:
                rospy.logerr('Unable to sample trajectory through waypoints. Read the log output to find out why.')
                rospy.signal_shutdown('Unable to sample trajectory through waypoints. Read the log output to find out why.');
                exit(-3)

            trajectories.append(trajectory)
        # # #}

        ## | ------------------- Resolve collisions ------------------- |
        delayed_robots, delays = [], []
        if self._collision_avoidance in self.ALLOWED_COLLISION_AVOIDANCE_METHODS:
            trajectories, delayed_robots, delays = trajectory_utils.resolveCollisions(self._collision_avoidance, problem, trajectories, self._safety_distance_mutual + self._mutual_collision_margin)
        else:
            print("[COLLISION AVOIDANCE] unknown method: {:s}".format(self._collision_avoidance))

        ## | ------ Add trajectories to the offline visualization ----- |
        for i in range(problem.number_of_robots):
            plotter.addTrajectoryPoses(trajectories[i].getPoses(), color=COLORS[i], label='traj. samples (id: ' + str(problem.robot_ids[i]) + ')')

        # # #{ Print trajectory infos
        print('###############################')
        print('##### Output trajectories #####')
        print('###############################')
        traj_t_max_idx, traj_d_max_idx = np.argmax([t.getTime() for t in trajectories]), np.argmax([t.getLength() for t in trajectories])
        for r in range(problem.number_of_robots):
            print('UAV ID: {:d}'.format(problem.robot_ids[r]))
            print('   Number of VPs:   {:d}'.format(len(viewpoints[r])-1))
            postfix  = ' (max)' if r == traj_t_max_idx else ''
            if r in delayed_robots:
                idx = delayed_robots.index(r)
                postfix += ' (incl. {:.2f} s delay)'.format(delays[idx])
            print('   Trajectory time: {:0.2f} s{:s}'.format(trajectories[r].getTime(), postfix))
            print('   Trajectory len:  {:0.2f} m{:s}'.format(trajectories[r].getLength(), ' (max)' if r == traj_d_max_idx else ''))
        print('###############################')
        # # #}

        ## | -------------- self-check of the solution -------------- |
        try:
            result = self.selfCheck(problem, viewpoints, trajectories)

            # SAFETY NET (stage 1): the inspection-tolerance slacks are the most
            # aggressive feature -- retry with zero slacks first, keeping the
            # shell-pose optimization and the relocation of unsafe viewpoints
            if not result['valid'] and self._shell_dp and \
                    (self._path_planner.get('shell_dp/radius_slack', 0.0) > 0.0 or self._path_planner.get('shell_dp/heading_slack', 0.0) > 0.0):
                print('[SELF-CHECK] solution invalid, replanning with zero inspection-tolerance slacks!')
                self._path_planner['shell_dp/radius_slack']  = 0.0
                self._path_planner['shell_dp/heading_slack'] = 0.0
                return self.planTrajectories(problem)

            # SAFETY NET (stage 2): still invalid -- replan with all the aggressive
            # features disabled (that configuration is extensively validated to
            # yield a full score)
            if not result['valid'] and (self._shell_dp or self._corner_arcs):
                print('[SELF-CHECK] solution invalid with enhancements enabled, replanning conservatively!')
                self._shell_dp     = False
                self._corner_arcs  = False
                return self.planTrajectories(problem)

            # LAST-RESORT NET: still invalid due to a too-small obstacle distance
            # (e.g., an inspection point whose whole tolerance sphere is unsafe).
            # Sacrifice the viewpoint nearest to the deepest violation and replan:
            # losing one inspection beats the zero score for the whole mission.
            if not result['valid'] and result['obstacle_violations'] and self._dropped_vp_count < 3:
                _, _, viol_pos = result['obstacle_violations'][0]
                nearest = None
                for rr in range(len(viewpoints)):
                    for vp in viewpoints[rr]:
                        if vp.idx == 0:
                            continue
                        d = np.linalg.norm(np.array(vp.pose.point.asList()) - viol_pos)
                        if nearest is None or d < nearest[0]:
                            nearest = (d, vp.idx)
                if nearest is not None:
                    print('[SELF-CHECK] dropping viewpoint of IP {:d} (nearest to the obstacle violation) and replanning!'.format(nearest[1]))
                    self._exclude_vp_idxs.add(nearest[1])
                    self._dropped_vp_count += 1
                    return self.planTrajectories(problem)

        except Exception as e:
            print('[SELF-CHECK] failed with exception: {:s}'.format(str(e)))

        print('[PLANNING FINISHED] total planning time: {:.1f} s'.format(time.time() - stopwatch_start))

        # # | --------------- plot velocity profiles --------------- |
        # plotter.plotDynamics(trajectories, self._max_velocity, self._max_acceleration, problem.robot_ids, dt=trajectory.dT)

        return trajectories, plotter
    # # #}

    # # #{ selfCheck()
    def selfCheck(self, problem, viewpoints, trajectories):
        '''
        Verifies the planned trajectories against the evaluation criteria of the mission
        (mirrors the checks done by the mrim_manager) and prints the results. Serves as
        an early warning: a FAIL here means the solution would be assigned a zero score.
        '''

        dt = self._trajectory_dt

        # evaluation limits (loaded from the mrim_manager config which is loaded into this node too)
        check_obst_dist   = rospy.get_param('~trajectories/check/obstacles', 1.5)
        check_mutual_dist = rospy.get_param('~trajectories/check/mutual', 2.5)
        insp_limit_dist   = rospy.get_param('~viewpoints/inspection_limits/distance', 0.3)
        insp_limit_hdg    = rospy.get_param('~viewpoints/inspection_limits/heading', 0.2)
        constraint_tol    = rospy.get_param('~dynamic_constraints/tolerance', 0.01)
        mission_timeout   = rospy.get_param('~mission/timeout', 200.0)

        # raw (unscaled) dynamic constraints
        sf      = self._dynamics_safety_factor
        vel_lim = [v / sf for v in self._max_velocity] + [self._max_heading_rate / sf]
        acc_lim = [a / sf for a in self._max_acceleration] + [self._max_heading_acceleration / sf]

        print('#############################################')
        print('########## SELF-CHECK OF THE SOLUTION #######')
        print('#############################################')

        all_ok = True

        xyzs = []
        hdgs = []
        for r in range(len(trajectories)):
            poses = trajectories[r].getPoses()
            xyzs.append(np.array([[p.point.x, p.point.y, p.point.z] for p in poses]))
            hdgs.append(np.array([p.heading for p in poses]))

        ## | ------------------ dynamic constraints ------------------ |
        for r in range(len(trajectories)):

            # velocities: first differences (heading wrapped), accelerations: differences of velocities
            dpos  = np.diff(xyzs[r], axis=0) / dt
            dhdg  = np.array([wrapAngle(hdgs[r][k] - hdgs[r][k - 1]) for k in range(1, len(hdgs[r]))]) / dt
            vels  = np.vstack([np.zeros((1, 3)), dpos])
            hvels = np.hstack([[0.0], dhdg])
            accs  = np.vstack([np.zeros((1, 3)), np.diff(vels, axis=0) / dt])
            haccs = np.hstack([[0.0], np.diff(hvels) / dt])

            for ax, name in enumerate(['x', 'y', 'z']):
                v_max, a_max = np.max(np.abs(vels[:, ax])), np.max(np.abs(accs[:, ax]))
                v_ok = v_max < vel_lim[ax] + constraint_tol
                a_ok = a_max < acc_lim[ax] + constraint_tol
                all_ok = all_ok and v_ok and a_ok
                print('[SELF-CHECK] [{:s}] UAV {:d} axis {:s}: max |vel| = {:.2f} m/s (limit {:.2f}), max |acc| = {:.2f} m/s^2 (limit {:.2f})'.format(
                    'OK' if (v_ok and a_ok) else 'FAIL', problem.robot_ids[r], name, v_max, vel_lim[ax], a_max, acc_lim[ax]))

            hv_max, ha_max = np.max(np.abs(hvels)), np.max(np.abs(haccs))
            hv_ok = hv_max < vel_lim[3] + constraint_tol
            ha_ok = ha_max < acc_lim[3] + constraint_tol
            all_ok = all_ok and hv_ok and ha_ok
            print('[SELF-CHECK] [{:s}] UAV {:d} heading: max |rate| = {:.2f} rad/s (limit {:.2f}), max |acc| = {:.2f} rad/s^2 (limit {:.2f})'.format(
                'OK' if (hv_ok and ha_ok) else 'FAIL', problem.robot_ids[r], hv_max, vel_lim[3], ha_max, acc_lim[3]))

        ## | ------------------- obstacle distances ------------------- |
        try:
            from scipy.spatial import cKDTree as CheckKDTree
        except ImportError:
            try:
                from scipy.spatial import KDTree as CheckKDTree
            except ImportError:
                CheckKDTree = None

        obstacle_violations = []
        if CheckKDTree is not None and problem.number_of_obstacle_points > 0:
            obst_tree = CheckKDTree(np.array([[o.x, o.y, o.z] for o in problem.obstacle_points]))
            for r in range(len(trajectories)):
                dists  = obst_tree.query(xyzs[r], k=1)[0]
                k_min  = int(np.argmin(dists))
                d_min  = float(dists[k_min])
                all_ok = all_ok and d_min > check_obst_dist
                if d_min <= check_obst_dist:
                    obstacle_violations.append((d_min, r, xyzs[r][k_min]))
                print('[SELF-CHECK] [{:s}] UAV {:d} min obstacle distance: {:.2f} m (limit {:.2f})'.format(
                    'OK' if d_min > check_obst_dist else 'FAIL', problem.robot_ids[r], d_min, check_obst_dist))
        obstacle_violations.sort(key=lambda v: v[0])

        ## | -------------------- mutual distances -------------------- |
        if len(trajectories) == 2:
            n_max   = max(len(xyzs[0]), len(xyzs[1]))
            idx     = np.arange(n_max)
            pos_a   = xyzs[0][np.minimum(idx, len(xyzs[0]) - 1)]
            pos_b   = xyzs[1][np.minimum(idx, len(xyzs[1]) - 1)]
            d_min   = float(np.min(np.linalg.norm(pos_a - pos_b, axis=1)))
            all_ok  = all_ok and d_min > check_mutual_dist
            print('[SELF-CHECK] [{:s}] mutual distance: min {:.2f} m (limit {:.2f})'.format(
                'OK' if d_min > check_mutual_dist else 'FAIL', d_min, check_mutual_dist))

        ## | --------------------- final positions -------------------- |
        for r in range(len(trajectories)):
            sp    = problem.start_poses[r]
            d_end = float(np.linalg.norm(xyzs[r][-1] - np.array([sp.position.x, sp.position.y, sp.position.z])))
            all_ok = all_ok and d_end <= 1.0
            print('[SELF-CHECK] [{:s}] UAV {:d} final position distance to start: {:.2f} m (limit 1.00)'.format(
                'OK' if d_end <= 1.0 else 'FAIL', problem.robot_ids[r], d_end))

        ## | -------------------- mission duration -------------------- |
        t_mission = max([t.getTime() for t in trajectories])
        all_ok    = all_ok and t_mission < mission_timeout
        print('[SELF-CHECK] [{:s}] mission duration: {:.1f} s (timeout {:.1f})'.format(
            'OK' if t_mission < mission_timeout else 'FAIL', t_mission, mission_timeout))

        ## | ------------------ inspection coverage ------------------- |
        ip_by_idx = {ip.idx: ip for ip in problem.inspection_points}
        n_ok      = 0
        n_vps     = 0
        for r in range(len(trajectories)):
            for vp in viewpoints[r]:
                if vp.idx == 0:
                    continue
                n_vps += 1
                ip     = ip_by_idx.get(vp.idx)
                if ip is None:
                    continue
                ip_pos    = np.array([ip.position.x, ip.position.y, ip.position.z])
                dist_devs = np.abs(np.linalg.norm(xyzs[r] - ip_pos, axis=1) - self._viewpoints_distance)
                hdg_devs  = np.abs(np.array([wrapAngle(h - ip.inspect_heading) for h in hdgs[r]]))
                inspected = bool(np.any((dist_devs <= insp_limit_dist) & (hdg_devs <= insp_limit_hdg)))
                if inspected:
                    n_ok += 1
                else:
                    print('[SELF-CHECK] [FAIL] UAV {:d} would NOT inspect point {:d} (min dist dev {:.2f} m, min hdg dev {:.2f} rad)'.format(
                        problem.robot_ids[r], vp.idx, float(np.min(dist_devs)), float(np.min(hdg_devs))))
        print('[SELF-CHECK] inspection coverage: {:d}/{:d} assigned viewpoints would be inspected'.format(n_ok, n_vps))
        print('#############################################')

        all_ok = all_ok and n_ok == n_vps

        return {'valid': all_ok, 'n_inspected': n_ok, 'n_assigned': n_vps, 'mission_time': t_mission,
                'obstacle_violations': obstacle_violations}
    # # #}

if __name__ == '__main__':
    try:
        mrim_planner = MrimPlanner()
    except rospy.ROSInterruptException:
        pass
