# MRS Summer School 2026 — Multi-Robot Inspection (Team Solution)

Two UAVs (🟥 red, 🟦 blue) must inspect a set of inspection points (IPs) in a 3D post-disaster environment as fast as possible: plan collision-free, dynamics-feasible trajectories that visit the viewpoint of every IP and return to the start. Score = number of inspected IPs; ties are broken by mission time + computation-time penalty. Any constraint violation (dynamics, 1.5 m obstacle distance, 2.5 m mutual distance, final position, hard compute limit) zeroes the score.

This repository is our team's solution built on the official [ctu-mrs/summer-school-2026](https://github.com/ctu-mrs/summer-school-2026) task (kept as the `upstream` remote — pull organizer fixes with `git pull upstream master`).

## Our solution

All changes live in [mrim_task/mrim_planner](mrim_task/mrim_planner) (the only folder used in the competition evaluation):

1. **Heading interpolation** — linear in traveled distance along each leg ([trajectory.py](mrim_task/mrim_planner/scripts/trajectory.py)).
2. **TSP over flight-time estimates** — per-axis trapezoidal time + heading-rotation time instead of Euclidean distance, obstructed straight lines penalized by a detour factor ([tsp_solvers.py](mrim_task/mrim_planner/scripts/solvers/tsp_solvers.py)).
3. **Viewpoint assignment** — k-means clustering of shared IPs + makespan balancing between the UAVs driven by a fast NN+2-opt tour estimator.
4. **Fast A\*** — heapq + numpy occupancy grid, endpoint free-cell snapping, exact metric endpoints, recursive straightening + greedy shortcut ([astar.py](mrim_task/mrim_planner/scripts/path_planners/grid_based/astar.py)); finished RRT/RRT\* as automatic fallback.
5. **Continuous trajectories** — viewpoint-exact pure-pursuit smoothing, heading held constant around viewpoints, TOPPRA time parametrization, no stops at waypoints.
6. **UAV↔UAV collision avoidance** — minimal start-delay search (either UAV, parked-UAV padding, mission-time optimal) + keepout spheres around the other UAV's start.
7. **Safety margins** — dynamics scaled to 97 % (virtual) / 90 % (real world), obstacle planning margin, mutual-distance margin.
8. **Self-check** — after planning, every zero-score condition is verified and printed as an OK/FAIL audit.
9. **Robustness fallbacks** — LKH failure → built-in 2-opt; TOPPRA failure → stop-at-waypoints sampling; A\* failure → RRT. No single failure can zero the score.
10. **Unseen-world validation** — [local_eval/generate_problem.py](local_eval/generate_problem.py) generates worlds similar to (and harder than) `apocalypse_large`; the solution is validated on 6 such worlds besides the 3 official ones.

11. **Shell-pose optimization (DP)** — for the fixed tour order, the actual inspection pose of every IP is chosen on its tolerance sphere by dynamic programming over flight-time estimates, so each UAV approaches every IP from the direction it is already flying (`tsp/shell_dp`). In the virtual challenge a controlled part of the inspection tolerances is spent as well (`radius_slack` 0.25 of the 0.3 m distance tolerance, `heading_slack` 0.15 of the 0.2 rad heading tolerance) — the reference is tracked exactly there; in the real-world config both slacks are 0 (tracking errors need the full tolerances).
12. **Unsafe-viewpoint relocation** — if a *nominal* viewpoint is closer to an obstacle than the evaluation limit (possible when an IP faces another structure — flying there would zero the whole score), the whole tolerance sphere is searched and the inspection is performed from a safe pose instead.
13. **Escalating safety net** — the planner self-checks every zero-score condition; on any failure it replans with zero slacks first (keeping the DP + relocation), then with all aggressive features disabled, and as a last resort sacrifices the viewpoint nearest to an obstacle violation (max 3) — one lost point always beats a zeroed mission.

### Results (evaluation logic identical to mrim_manager, all checks green)

| Problem | Score | Mission time | Planning time |
|---|:---:|:---:|:---:|
| apocalypse_small | 8/8 | 22.6 s | ~8 s |
| apocalypse_moderate | 16/16 | 38.8 s | ~24 s |
| apocalypse_large | 29/29 | 58.4 s | ~17 s |
| unseen_comp_a/b/c | 34+33+35 (all) | 63.2–72.8 s | ~34–78 s |
| unseen_dense | 34/34 | 72.4 s (+3.0 s penalty) | ~123 s |
| unseen_tight | 32/32 | 71.6 s | ~76 s |
| unseen_wide40 | 40/40 | 76.6 s | ~20 s |

(The config is tuned to the apocalypse_large regime — see [result.md](result.md) for the full parameter study. It is deliberately aggressive: on most unseen worlds the first planning attempt violates a tolerance and the escalating safety net replans with the slacks/shell-DP dialed back, which is where the long planning times come from. The net's fallback configurations are themselves validated on all 9 worlds; only unseen_dense exceeds the 120 s soft solution limit, paying 3.0 s of tie-break penalty.)

With the `real_world` config (3/3/1 m/s dynamics, 2.0/4.0 m distances, VP distance 5.0 m; cone 0.90, smoothing 1.2/2.0, zero slacks): apocalypse_large 29/29 @ 103.8 s, unseen_comp_a 34/34 @ 109.0 s, unseen_dense 34/34 @ 121.0 s (2 unsafe viewpoints auto-relocated — without relocation this world zero-scores), unseen_tight 32/32 @ 115.4 s, unseen_wide40 40/40 @ 132.0 s (1 viewpoint relocated). Min obstacle distances 2.46–2.71 m vs the 2.0 m limit, planning 18–25 s vs the 80 s soft limit, all first attempts clean.

(Official `mrim_manager` in the Apptainer container reproduces these numbers exactly.)

## Setup

Requirements: Linux (Ubuntu recommended; WSL2 works), ~6 GB disk, [Apptainer](https://apptainer.org/) (installed automatically on Ubuntu).

```bash
mkdir -p ${HOME}/git
cd ${HOME}/git && git clone https://github.com/thanhnguyencanh/summer-school-2026.git
cd summer-school-2026 && ./install.sh   # installs Apptainer, downloads MRS image, compiles the workspace
```

`install.sh` runs the three steps below — they can also be run individually from `simulation/` (useful for partial re-runs):

| Script | What it does | When to re-run |
|---|---|---|
| `simulation/01_install.sh` | installs Apptainer via apt (**needs sudo**, Ubuntu only) | once per machine |
| `simulation/02_download.sh` | downloads the ~3.5 GB MRS image to `simulation/images/` (resumable; **deletes the old image first**) | on image updates |
| `simulation/03_compile.sh` | `catkin build` of the workspace inside the container | after changing C++ code (`mrim_state_machine`); Python/config changes need no rebuild |

## Testing & evaluation

### 1. Official offline evaluation (recommended before any submission)

```bash
./simulation/run_offline.sh      # plans + evaluates + RViz playback (press the 'Send Topic' start button in RViz)
```

The planner prints a `SELF-CHECK OF THE SOLUTION` block (every line must be `[OK]`); the manager prints `FINAL SCORE: X/Y` after the playback. Switch problems in [mrim_task/mrim_planner/config/virtual.yaml](mrim_task/mrim_planner/config/virtual.yaml) (`problem/name`), options: `apocalypse_{small,moderate,large}.problem` and our generated `unseen_*.problem` worlds.

### 2. Full simulation (real-time dynamics, two simulated UAVs)

```bash
./simulation/run_simulation.sh   # tmux session; switch panes with Shift+←/→ (planner, state_machine, control)
./simulation/kill_simulation.sh  # stop it
```

### 3. Quick headless testing (no ROS/Apptainer — any machine, ~1 min setup)

```bash
./local_eval/setup.sh            # one-time: creates venv + builds the LKH solver (needs python3-venv, gcc, wget)
./local_eval/venv/bin/python local_eval/harness/run_planner.py --problem apocalypse_large.problem
./local_eval/venv/bin/python local_eval/harness/run_planner.py --problem unseen_dense.problem --config real_world
./local_eval/venv/bin/python local_eval/harness/run_planner.py --problem apocalypse_large.problem --set path_smoothing/lookahead_dist=1.2
```

Runs the real planner code with ROS stubbed out and evaluates it exactly like `mrim_manager` (score, dynamics, distances, final positions). The run must end with all `OK` and `INSPECTED` equal to the number of points. Generate more test worlds with `local_eval/generate_problem.py --name my_world --seed 7 --n-ips 34`.

## Parameter tuning guide

**Why this matters in the real world:** the virtual challenge evaluates the *reference* trajectory (our self-check reproduces it exactly, so nothing is left to chance), but the real-world challenge evaluates the **actual UAV pose**. Tracking error means the UAV can fly past a viewpoint outside the ±0.3 m / ±0.2 rad inspection window even though the reference is perfect — and the self-check cannot detect that. The knobs below (all in [config/virtual.yaml](mrim_task/mrim_planner/config/virtual.yaml) / [config/real_world.yaml](mrim_task/mrim_planner/config/real_world.yaml)) control the trade-off between speed and capture reliability.

| Parameter | Virtual / Real | Role |
|---|:---:|---|
| `trajectories/dynamics_safety_factor` | 0.97 / 0.90 | Scales every velocity/acceleration/heading-rate limit used for planning. **The main real-world reliability knob**: lower = slower but gentler reference the UAV can track accurately (small position/heading error near viewpoints, obstacle margins preserved). |
| `path_smoothing/heading_hold_dist` (m) | 0.6 / 0.8 | Distance along the path before/after every viewpoint in which the heading is frozen at the inspection heading. Increase if inspections are missed because the UAV is *already rotating* while passing the viewpoint. |
| `tsp/shell_dp/radius_slack` / `heading_slack` | 0.25, 0.15 / **0.0, 0.0** | Part of the inspection tolerances (±0.3 m, ±0.2 rad) the planner *spends* to shorten the tour; everything not spent remains as margin for tracking error. Must stay 0 in the real world. In virtual, the safety net resets them automatically if the self-check ever fails. |
| `path_smoothing/lookahead_dist` (m) | 2.0 / 2.0 | Pure-pursuit smoothing radius: larger = smoother and faster corners but larger deviation from the collision-free polyline; smaller = tighter and slower. Trajectories pass exactly through the viewpoints either way. Reduce towards 0.8 if obstacle margins get tight. |
| `path_smoothing/sampling_step` (m) | 1.2 / 1.2 | Spacing of the waypoints handed to TOPPRA. Smaller = the final spline follows the smoothed path more faithfully and decelerates more honestly in curves (safer, slower). Reduce towards 0.4 if constraints are violated or inspections are missed. |
| `trajectory_sampling/with_stops` | false / false | **Guaranteed-capture fallback**: stop at every waypoint — a standstill sample with exact position and heading at each viewpoint. Costs a lot of mission time; the last resort if real UAVs keep missing inspections. |
| `path_planner/obstacle_margin` (m) | 0.1 / 0.1 | Extra clearance added to the planning obstacle distance. Increase if the real UAV drifts closer to obstacles than planned (an obstacle violation zeroes the score). |
| `collision_avoidance/mutual_margin` (m) | 0.3 / 0.5 | Extra margin on the minimum UAV–UAV distance used when computing deconfliction delays. |
| `tsp/shell_dp/enabled`, `cone_angle` (rad) | true, 1.30 / true, 0.90 | Inspection-pose optimization on the tolerance spheres (incl. relocation of nominal viewpoints that violate the obstacle limit). `cone_angle` bounds how far poses may move from the nominal direction; smaller = more conservative. |

**Symptom → what to change (real world):**

- *Inspection missed, heading was off* → increase `heading_hold_dist` (0.8 → 1.2); verify `heading_slack: 0.0`.
- *Inspection missed, UAV flew past too fast / off-radius* → lower `dynamics_safety_factor` (0.90 → 0.85), reduce `sampling_step` (0.8 → 0.4) and `lookahead_dist` (1.5 → 0.8); verify `radius_slack: 0.0`.
- *Still missing inspections* → `with_stops: true` — guaranteed capture at every viewpoint, accept the slower mission.
- *Too close to obstacles / other UAV in real flight* → raise `obstacle_margin` / `mutual_margin`, lower `dynamics_safety_factor`.
- *Mission too slow, everything captured reliably* → cautiously raise `dynamics_safety_factor` and/or `lookahead_dist`.

## Competition submission

Deadline: **Monday, August 3, 11:59 p.m.** via the [Google form](https://forms.gle/RXf8DQzCJn8txecB8) (team name, members, zip archive — link updated by the organizers on Aug 1, the old `docs.google.com/...1jcoQr...` link is dead).

```bash
cd mrim_task && zip -r team_name.zip mrim_planner
```

Checklist before zipping:
- [ ] `run_offline.sh` passes with full score on `apocalypse_large` and several `unseen_*` worlds
- [ ] `config/virtual.yaml` and `config/real_world.yaml` are both tuned (the same zip is used for both challenges)
- [ ] planner self-check prints all `[OK]`

## Task reference

### Constraints

| Constraint | Virtual | Real-world |
|---|:---:|:---:|
| Max solution time (soft / hard) | 120 s / 200 s | 80 s / 150 s |
| Max mission time | 200 s | 240 s |
| Max velocity x,y / z | 5 / 2 m/s | 3 / 1 m/s |
| Max acceleration x,y / z | 5 / 2 m/s² | 3 / 1 m/s² |
| Max heading rate / acceleration | 1 rad/s / 2 rad/s² | 1 rad/s / 2 rad/s² |
| Min obstacle / mutual distance | 1.5 / 2.5 m | 2.0 / 4.0 m |
| Final point within start tolerance | 1.0 m | 1.0 m |

An IP is inspected when the UAV is within 0.3 m of the viewpoint distance sphere with heading within 0.2 rad (evaluated per 0.2 s sample; real-world uses the actual UAV pose, not the reference).

### Where the code lives

- `mrim_task/mrim_planner/scripts/planner.py` — entry point, parameters, self-check
- `scripts/trajectory.py` — smoothing, heading, TOPPRA sampling, collision avoidance
- `scripts/solvers/tsp_solvers.py` — clustering, balancing, TSP, path-planning fallback chain
- `scripts/path_planners/` — A\* (grid) and RRT/RRT\* (sampling)
- `config/virtual.yaml`, `config/real_world.yaml` — all tunables
- `mrim_task/mrim_manager/` — the evaluator (**do not modify**; its config defines dt, limits, viewpoint distance)

### Troubleshooting

- `open terminal failed: missing or unsuitable terminal` → run `./simulation/kill_simulation.sh`
- Apptainer on non-Ubuntu: install manually, see [official docs](https://apptainer.org/docs/user/main/quick_start.html#installation)
- Organizer contacts: Václav Riss `rissvacl@fel.cvut.cz`, Jindřich Třaskoš `traskjin@fel.cvut.cz`, Martin Zoula `zoulamar@fel.cvut.cz`

### References

- [1] Baca, T. et al., [The MRS UAV System](https://arxiv.org/pdf/2008.08050), *JINT 102(26), 2021* — https://github.com/ctu-mrs/mrs_uav_system
- [2] Pham, H., Pham, Q. C., [TOPPRA: time-optimal path parameterization](https://hungpham2511.github.io/toppra/)
