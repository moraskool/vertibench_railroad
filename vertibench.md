# VertiBench data-collection brief (context handoff)

This file is a self-contained brief for an agent working **inside the VertiBench repo**
(`~/Documents/verti_bench`, run under the `chrono9` conda env via the `vpy` alias). It carries the
context from the parent planning repo (`RAIL_mrppddl_dev` / `resilient_mrp`) that you need to run the
data-collection task correctly. You do not have access to the parent repo, everything you need from it
is written out below.

---

## 0. The task, in three steps

**Goal: measure terrain/robot failure rates in VertiBench, to ground the planner's failure model.**

1. After the VertiBench install and a minimal scene run are confirmed, run simple **location A to location B**
   trials for **different robot configurations on different terrains**, **20-30 trials each**.
2. Write the code/script to **log the failure instances, time, and cost** of each run, producing our failure
   statistics.
3. **Verify these against our sim model** (the hand-authored table in section 2) and **adjust the model where
   necessary** (section 6 is how).

---

## 1. Why we are collecting this data

The parent project is a **failure-aware multi-robot planner** (ICRA target). Robots traverse an environment, represented as a graph (for the sake of focusing on the planning problem)
whose edges carry a terrain type and a hazard level. Crossing a risky edge can make a robot
**non-operational** (a terminal, mission-level failure, not a cost bump). The planner reasons about
these per-edge survival odds to trade route cost against the chance of losing a robot.

Right now the per-edge survival odds come from a **hand-authored** model. We want to **ground that model
in real-world framework**: measure how a real vehicle actually fares crossing each terrain, and feed those measured
numbers or use that to properly model of how failure looks like for ground vehicles on off-road terrains, back into the planner. VertiBench (Project Chrono vehicle physics engine sim) is where we get that ground truth.

So the concrete goal: **drive a vehicle across each terrain type many times and log, per run, whether it
made it, and if it failed, when and where it failed, and other necessary datapoints.**

---

## 2. The exact model this data calibrates

The planner scores each edge with a single survival probability:

```
p_success = 1 - hazard_severity * (1 - compatibility)
```

- `compatibility` in [0,1]: how well a given **vehicle** handles a given **terrain type**
  (1 = fully at home, 0 = totally unsuited).
- `hazard_severity` in [0,1]: how dangerous the **terrain** is in absolute terms.

Both are currently guessed. The four terrain types below are an **assumption and an abstraction**, we picked
four, but there could be more or fewer. **The count does not matter:** the failure problem (a vehicle may
become non-operational crossing risky ground) is general regardless of how many terrain types exist. So do
**not** treat "four" as a fixed taxonomy to force VertiBench into. We want to use VertiBench's real,
physics-based off-road terrain to **ground or check against these assumptions**: we do not want made-up
numbers, and the failure model itself may need adjustment. These feature axes (slope, roughness,
deformability) are how we connect VertiBench's elevation maps to whatever terrain description we end up with
(see section 6):

| terrain     | base_risk | slope | roughness | deformability |
|-------------|-----------|-------|-----------|---------------|
| clear       | 0.10      | 0.04  | 0.05      | 0.0           |
| rocky       | 0.30      | 0.08  | 0.40      | 0.1           |
| steep       | 0.40      | 0.25  | 0.15      | 0.1           |
| deformable  | 0.35      | 0.12  | 0.10      | 0.7           |

Hand-authored vehicle-vs-terrain compatibility (the numbers we most want to replace with measured ones):

| vehicle role          | clear | rocky | steep | deformable |
|-----------------------|-------|-------|-------|------------|
| steep specialist      | 0.95  | 0.88  | 0.92  | 0.30       |
| deformable specialist | 0.95  | 0.90  | 0.35  | 0.92       |
| rocky specialist      | 0.95  | 0.92  | 0.55  | 0.45       |
| balanced generalist   | 0.95  | 0.70  | 0.65  | 0.65       |

**What the data gives us:** for each (vehicle, terrain), an empirical success rate over many crossings.
That success rate is a direct estimate of `p_success` for that terrain at a fixed hazard level, from which
we can back out `compatibility`. The failure kinematics (when/where it fails) let us go beyond a single
probability toward a distance-to-failure distribution if we want a richer edge model later.

---

## 3. VertiBench inventory (from the official repo)

Facts to design the sweep around:

- **Vehicles (9):** `hmmwv`, `gator`, `feda`, `man5t`, `man7t`, `man10t`, `m113`, `art`, `vw`.
- **Controllers/systems (10, + manual):** `pid` (default), `eh`, `mppi`, `rl`, `mcl`, `acl`, `wmvct`,
  `mppi6`, `tal`, `tnt`, `manual`.
- **Worlds:** **100 elevation maps** (`world_id` 1..100, under `envs/data/BenchMaps/sampled_maps/Worlds`),
  each with **10 fixed start/goal pairs**, so **1000 built-in A-to-B tasks**. Use these built-in pairs as
  your "location A to location B", do not invent your own.
- **Scale:** `scale_factor` `1.0` (default), `1/6`, `1/10`.

**The maps are real-world, mixed-terrain** off-road elevation (rigid and deformable surfaces, boulders,
rocks, snow, in the same map). A single A-to-B run therefore crosses a *mixture* of conditions rather than
one clean terrain. That is fine and expected, section 6 is how we extract per-terrain signal from mixed maps.

---

## 4. What to collect (per-run schema)

One row per simulation run. Target the fields below. Some are already returned by the harness, others you
will need to instrument (see section 7).

| column                  | meaning                                                            | source |
|-------------------------|-------------------------------------------------------------------|--------|
| `vehicle`               | vehicle id (hmmwv, gator, ...)                                     | config |
| `world_id`              | elevation map (1..100)                                             | config |
| `start_goal_id`         | which of the world's 10 A-to-B pairs (0..9)                        | config |
| `controller`            | control system (pid to start)                                     | config |
| `speed`                 | target speed                                                      | config |
| `scale_factor`          | world scale (1.0 to start)                                        | config |
| `seed` / `experiment`   | run index / RNG seed, so any row is reproducible                  | config |
| `success`               | 1 if reached goal, 0 if failed                                    | run outcome |
| `failure_mode`          | rollover / stuck / timeout, if determinable                      | run outcome |
| `time_to_goal`          | sim time to reach goal (success runs)                            | run outcome |
| `time_at_failure`       | sim time when the vehicle failed (failure runs)                 | **instrument** |
| `distance_at_failure`   | path distance travelled before failing                          | **instrument** |
| `remaining_distance`    | distance from failure point to goal                             | **instrument** |
| `total_path_length`     | full intended A-to-B distance for that world/pair               | world map |
| `avg_roll`, `avg_pitch` | attitude stats over the run (stability proxy)                   | run outcome |
| `terrain_slope` / `terrain_roughness` / `terrain_deformability` | features of the crossed path (see §6), one value per column | **derive** |
| `terrain_bucket`        | which of our four types this path/failure maps to (see §6)      | **derive** |

A header stub with these exact columns lives at `results/csv/terrain_crossing.csv` in the parent repo, keep
the names identical so the CSV drops straight into our analysis.

`time`, `cost`, and `failure instance` from the task outline map to: `time_to_goal` / `time_at_failure`
(time), `total_path_length` and travelled distance (cost, our planner's cost unit is distance/time to goal),
and `success` / `failure_mode` / `*_at_failure` (failure instance).

Keep it **tidy/flat**: one CSV, one row per run, header on line 1. This mirrors how the parent repo logs its
planner benchmarks, so the two datasets analyse the same way.

---

## 5. Sweep design (the 20-30 trials each)

- **Axes:** `vehicle` × `terrain condition`. Fix `controller = pid` (deterministic, simplest baseline) and a
  single representative `speed` and `scale_factor = 1.0` for the first pass.
- **20-30 trials per (vehicle, terrain-condition) cell.** A proportion's Wilson interval tightens well by 30;
  go to 30 for cells near 50% success (max variance), 20 is fine for clearly easy/hard cells.
- **Where the independent trials come from (read this, PID is deterministic):** same vehicle, same world,
  same start/goal pair, same seed gives an **identical** result. Do **not** repeat one crossing 25 times. Get
  independent draws by varying across the **10 start/goal pairs of a world** and across **several worlds in
  the same terrain-condition bucket** (section 6), plus the seed.
- Run **headless** (`render=false use_gui=false`) for throughput. Render only a handful for a sanity video.

**Start tiny:** one vehicle, one world, one start/goal pair, 1 run, headless, confirm one fully-populated row
lands in the CSV. Then widen to the grid. Do not launch the full sweep before a single cell is proven.

---

## 6. Relating measured failures to terrain (this is step 3)

VertiBench maps are mixed-terrain; our model is written over abstract terrain types. Bridge the two through
the **terrain features**, not through a fixed terrain count. The failure signal we ultimately want is
`p_success` as a function of **(vehicle capability, terrain features)** — the number of named "types" is just
a convenient discretization of that.

1. **Characterize where the vehicle drove by geometry/material features** on the axes the model uses:
   **slope, roughness, deformability** (and rigid-vs-deformable material). The elevation maps and metadata
   live under `envs/data/BenchMaps/`, derive slope and roughness from the elevation grid along the route, and
   read the material/deformable flag from the world config.
2. **Attribute outcomes to terrain.** Two workable levels, pick per effort budget:
   - *Path-level (simpler):* summarise each A-to-B path by its feature profile and estimate a success rate
     per (vehicle, feature profile).
   - *Segment-level (richer, preferred):* since `distance_at_failure` tells you **where** on the path it
     failed, read the terrain features **at that failure location**. This attributes each failure to the
     local terrain it happened on, which is the cleanest signal from a mixed map.
3. **Relate measured to the model, then adjust.** The measured success rate at a given terrain-feature level
   is empirical `p_success`. Compare it to what our model predicts. Where they disagree, adjust the model:
   recalibrate `compatibility` / `hazard_severity`, **collapse or add terrain groupings** if the data clusters
   differently than our assumed four, or revise the model form itself if the mismatch is structural. Report
   before and after.

`terrain_bucket` in the schema is optional labelling for convenience, fill it if a clean grouping emerges from
the features, otherwise leave the raw feature columns to speak for themselves. Keeping `distance_at_failure`
and the per-path feature profile is what makes the segment-level attribution possible.

---

## 7. What the harness already gives you, and what you must instrument

Confirmed from `setup.py`:

- Entry point: `vpy setup.py vehicle=<> system=<> speed=<> world_id=<> scale_factor=<> max_time=<>
  num_experiments=<> render=false use_gui=false` (argparse/keyword style).
- `single_experiment(config)` builds a per-system sim object (`PIDSim`, `MPPISim`, `RLSim`, `EHSim`,
  `MCLSim`, `ACLSim`, `WMVCTSim`, `MPPI6Sim`, `TALSim`, `TNTSim`, `ManualSim`) and calls `sim.run()`.
- `sim.run()` currently returns **`(time_to_goal, success, avg_roll, avg_pitch)`**. So success and timing
  come for free, attitude stats too.

**Not yet exposed (you must add it):** `time_at_failure`, `distance_at_failure`, `remaining_distance`,
`total_path_length`, and `failure_mode`. These live inside the per-system sim loop (start with
`systems/PID/PID_sim.py`, method `run()`), where the vehicle pose is stepped each tick and the
success/failure check happens. The clean approach:

1. Read `PID_sim.run()` end to end. Find where it detects goal reached and where it detects failure
   (rollover/stuck/timeout), and where it has the current vehicle position and the goal position.
2. At the moment failure is declared, capture the current sim time, the accumulated travelled distance
   (sum of per-step position deltas, or an odometer the loop may already keep), and the distance from the
   current position to the goal. Record the failure reason.
3. Return these alongside the existing tuple (or better, return a dict) and have `setup.py`'s result
   assembly write them into the output row. Prefer a **dict return** to avoid breaking positional callers,
   or thread the extras through carefully.
4. Read the elevation grid + world config so you can attach the **per-path terrain features** (section 6)
   and thread the **start/goal pair index** through config into the row.

The install recipe (CUDA 11.8, OptiX, Chrono 9.0.1 source build, `chrono9` env) is already done on this
machine, that is why `vpy` and the 6 GB asset tree exist. Do not rebuild or re-clone.

---

## 8. Deliverable back to the parent project

- **One tidy CSV** with the schema in section 4, written somewhere stable inside the VertiBench repo
  (e.g. `results/terrain_crossing.csv`). We will copy it back into `resilient_mrp` for analysis.
- A **short note** on: how you mapped map features to our terrain types (section 6), any field you could not
  populate and why, the measured-vs-predicted comparison and any model adjustments, and the exact command(s)
  that produced the CSV so we can rerun.
- Do **not** commit the 6 GB assets or large sim artifacts. Just the CSV and any small scripts you add.

The end use on our side: turn the per-(vehicle, terrain) success rates into calibrated `compatibility` /
`hazard_severity` values (and optionally a distance-to-failure distribution) that ground or replace the
hand-authored table in section 2, and flag where the failure model itself needs revising. Keeping the schema
tidy and one-row-per-run is what makes that drop-in.

---

## 9. Constraints and gotchas

- This runs under **`chrono9` / `vpy`** (Python 3.9), **not** the parent repo's `uv` env. Never mix them.
- Headless throughput first, rendering is for spot-checks only.
- PID is deterministic: independence comes from varying (world, start/goal pair, seed), not from repeats.
- If an import segfaults, it is the known OpenMP clash: prefix with
  `LD_PRELOAD=/usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so.1`.
