# Planner tuning results


## Baseline(Current)

| | small | moderate | large |
|---|---|---|---|
| final | 22.6 | 40.0 | 63.6 |
| solve | 7.9 | 11.6 | 16.9 |

---

## path_planner/method

| value | small | moderate | large |
|---|---|---|---|
| rrtstar | 39.4 ⚠ nondeterministic | 69.4 | 105.6 |
| **astar** | **24.2** | **44.6** | **68.0** |

RRT\* on small, 5 identical runs: 29.4 / 33.0 / 35.4 / 37.2 / 39.4. A\* is deterministic.
*(mission time; measured before the smoothing params changed)*

---

## tsp/shell_dp/cone_angle — base 0.55

| value | small | moderate | large |
|---|---|---|---|
| 0.35 | 23.8 | 42.0 | 65.4 |
| 0.45 | 22.8 | 41.2 | 64.2 |
| 0.55 | 22.6 | 40.0 | 63.6 |
| **0.70** | **22.2** | 38.8 | 63.0 |
| 0.90 | 22.4 | 38.0 | 61.4 |
| **1.10** | 22.6 | **37.4** | 61.4 |
| **1.30** | 23.4 | 39.8 | **60.4** |
| 1.50 | 24.0 | 39.4 | 62.0 |

Best differs per world: **0.70 / 1.10 / 1.30**.

---

## tsp/shell_dp/radius_slack — base 0.2 (tolerance 0.3)

| value | small | moderate | large |
|---|---|---|---|
| 0.0 | 22.6 | 40.4 | 64.6 |
| 0.1 | 22.6 | 40.2 | 64.2 |
| 0.2 | 22.6 | **40.0** | 63.6 |
| **0.25** | **22.4** | 40.0 | **63.2** |
| 0.30 | 22.8 `FAIL=1` | 40.4 `FAIL=2` | — |

---

## tsp/shell_dp/heading_slack — base 0.15 (tolerance 0.2)

| value | small | moderate | large |
|---|---|---|---|
| 0.0 | 22.8 | 40.2 | **63.6** |
| 0.10 | **22.6** | 40.2 | **63.6** |
| 0.15 | **22.6** | **40.0** | **63.6** |
| 0.20 | 22.8 `FAIL=1` | 40.4 `FAIL=2` | 64.6 `FAIL=3` |

No gain anywhere — keep base. 0.20 breaks the tolerance.

---

## path_smoothing/sampling_step — base 0.8

| value | small | moderate | large |
|---|---|---|---|
| 0.4 | 23.6 | 41.4 | 64.6 |
| 0.6 | 23.8 | 40.8 | 64.0 |
| 0.8 | 22.6 | 40.0 | 63.6 |
| 1.0 | 22.4 | **39.8** | 63.4 |
| **1.2** | **22.2** | **39.8** | **63.0** |

Only parameter that improves **all three** worlds at the same value.

---

## path_smoothing/lookahead_dist — base 1.5

| value | small | moderate | large |
|---|---|---|---|
| 1.0 | 22.8 | 42.0 | 65.0 |
| 1.5 | 22.6 | **40.0** | 63.6 |
| **2.0** | **22.2** | **40.0** | **63.2** |
| 2.5 | 22.2 `FAIL=1` | 45.2 `FAIL=2` | 72.4 `FAIL=3` (28/29) |

2.5 loses an inspection point on large.

---

## path_smoothing/heading_hold_dist — base 0.6

| value | small | moderate | large |
|---|---|---|---|
| 0.3 | **22.6** | 40.2 | **63.4** |
| 0.6 | **22.6** | **40.0** | 63.6 |
| 1.0 | 23.4 | 41.6 | 64.0 |

---

## tsp/detour_penalty — base 1.5

| value | small | moderate | large |
|---|---|---|---|
| **1.25** | **22.2** | 43.0 | 66.2 |
| 1.5 | 22.6 | **40.0** | **63.6** |
| 2.0 | 22.4 | 43.2 | 73.2 |

Helps small only; clearly harmful on large.

---

## trajectories/dynamics_safety_factor — base 0.97

| value | small | moderate | large |
|---|---|---|---|
| 0.95 | 23.0 | 43.8 | 64.8 |
| 0.97 | 22.6 | 40.0 | **63.6** |
| **0.99** | **22.0** | **39.8** | 67.2 |

Best single knob for small. Backfires on large.

---

## path_planner/obstacle_margin — base 0.1

| value | small | moderate | large |
|---|---|---|---|
| 0.0 | 23.0 | 41.6 | 66.6 |
| **0.1** | **22.6** | **40.0** | **63.6** |
| 0.2 | 24.4 | 40.8 | 65.8 |

Base is best everywhere — leave alone.

---

## path_planner/astar/grid_resolution — base 0.4

| value | small | moderate | large | solve (large) |
|---|---|---|---|---|
| 0.3 | 23.2 | 40.4 | 65.4 | 32.5 |
| **0.4** | **22.6** | **40.0** | **63.6** | 16.9 |
| 0.5 | **22.6** | 40.2 | 63.8 | **14.5** |

0.5 is ~15% faster to solve for +0.2 s on large — useful only if solve time becomes tight.

---

## tsp/clustering — base kmeans

| value | small | moderate | large |
|---|---|---|---|
| **kmeans** | **22.6** | 40.0 | **63.6** |
| random | 23.0 | **36.8** | 78.6 |

Biggest single gain on moderate (−3.2 s), biggest single loss on large (+15.0 s).

---

# Combinations

## Best per world — each combo measured on all three worlds

| Combo | config | small | moderate | large |
|---|---|---|---|---|
| baseline | — | 22.6 | 40.0 | 63.6 |
| **small-best** | `ss=1.2` `la=2.0` `dyn=0.99` `dp=1.25` | **21.4** | 41.2 ✗ | 63.2 |
| **moderate-best** | `cone=1.10` `ss=1.2` `dyn=0.99` | 22.6 = | **36.8** | 65.6 ✗ |
| **large-best** | `cone=1.30` `ss=1.2` `rs=0.25` `la=2.0` | 22.6 = | 38.8 `FAIL=1` | **58.4** |

Diagonal is each world's optimum: **21.4 / 36.8 / 58.4** vs baseline 22.6 / 40.0 / 63.6
(−5.3% / −8.0% / −8.2%). Off-diagonal shows the cost of using the wrong one:

- **small-best** is the mildest — it also beats baseline slightly on large (63.2) and only
  loses 1.2 s on moderate.
- **moderate-best** costs +2.0 s on large.
- **large-best** (currently committed) costs +2.8 s vs the moderate optimum and raises a
  self-check FAIL on moderate; it is neutral on small.

## Best single config for all three

| Config | small | moderate | large |
|---|---|---|---|
| baseline | 22.6 | 40.0 | 63.6 |
| `cone_angle=0.90` | 22.4 | 38.0 | 61.4 |
| **`cone_angle=0.90` + `sampling_step=1.2`** | **22.2** | **37.8** | **61.0** |

## Ablations — what each parameter actually contributed

| World | Drop from combo | Effect |
|---|---|---|
| small | − cone_angle | 21.6 → **21.4** (helps alone, hurts combined) |
| small | − sampling_step | 21.6 → **21.4** (same) |
| small | − dyn_safety | 21.6 → 22.0 (essential) |
| moderate | − clustering=random | 36.8 `F1` s23.8 → 36.8 `F0` **s11.9** (drop it) |
| moderate | − cone / − sampling_step / − dyn_safety | 41.0 / 42.6 / 42.8 (all essential) |
| large | − heading_hold | 58.4 → 58.4 (contributes nothing) |
| large | − cone / − sampling_step / − radius_slack / − lookahead | 62.2 / 59.4`F2` / 59.0`F1` / 59.4 (all essential) |


---

# Unseen-world validation & real-world tuning (Aug 3)

## Virtual: large-best on the 6 unseen worlds (final time = mission + compute penalty)

| Config | comp_a | comp_b | comp_c | dense | tight | wide40 | sum |
|---|---|---|---|---|---|---|---|
| large-best (cone 1.30, committed) | 72.8 | 63.2 | 65.0 | 75.4 ⚠ +3.0 pen | 71.6 | 76.6 | **424.6** |
| rs=0.2 variant | 72.8 | 63.2 | 65.0 | 119.7 ⚠ +47.3 pen | 71.6 | 77.0 | 469.3 ✗ |
| cone=1.10 variant | 69.6 | 63.0 | 63.2 | 72.8 | 72.4 | 78.8 | 419.8 |

All full score. large-best relies on the safety net on 5/6 worlds (first attempts overshoot
tolerances / dive to 0.18–0.37 m of obstacles; the net's fallback configs recover everything).
cone=1.10 is cleaner/faster on the unseen set but −0.6 s worse on apocalypse_large itself;
**decision: keep cone 1.30** (competition world expected large-like). rs=0.2 rejected.

## Real world (actual pose evaluated → slacks stay 0; reference-level measurements)

### cone_angle — base 0.45

| value | large | dense | wide40 | worst obst. margin |
|---|---|---|---|---|
| 0.45 | 115.0 | 137.4 | 142.4 | 2.27 m |
| 0.70 | 113.8 | 123.8 | 137.6 | 2.55 m |
| **0.90** | **106.0** | 126.4 | **135.4** | 2.55 m |

### path_smoothing (with cone 0.90) — base 0.8/1.5

| ss/la | large | dense | wide40 | worst obst. margin |
|---|---|---|---|---|
| 0.8/1.5 | 106.0 | 126.4 | 135.4 | 2.55 m |
| 1.2/1.5 | 104.8 | 124.6 | 133.4 | 2.52 m |
| **1.2/2.0** | **103.8** | **121.0** | **132.0** | 2.50 m |

### Inert knobs (measured/verified)

- `balance_time_budget` 12 → 30 s: identical results, cap never reached — keep 12.
- `path_planner/timeout`: no A* leg has ever timed out (~70 runs) — keep 10.

### Final real_world config (cone 0.90, ss 1.2, la 2.0, slacks 0, dyn 0.90)

| | large | comp_a | dense | tight | wide40 |
|---|---|---|---|---|---|
| final time | 103.8 | 109.0 | 121.0 | 115.4 | 132.0 |
| score | 29/29 | 34/34 | 34/34 | 32/32 | 40/40 |
| solve | 17.9 | 20.0 | 22.7 | 21.1 | 25.3 |

All first attempts clean (no self-check FAILs); margins 2.46–2.71 m (limit 2.0);
dense ×2 and wide40 ×1 unsafe viewpoints auto-relocated.
