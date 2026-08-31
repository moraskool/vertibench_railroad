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

**Where this stands (2026-08-29).** Steps 1 and 2 are done for a pilot batch: `collect_crossings.py`
and `analysis/terrain_features.py` exist and produced 270 rows covering 2 vehicles. Step 3 was never
done, and `terrain_bucket` is empty on every row. Round 2, specified in section 5, widens the pilot to
all 9 vehicles and a stratified sample of the worlds, at a fixed speed. Section 5b records what round 1
found and the design flaw round 2 corrects. Read section 5b before running anything.

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

- **Vehicles (9):** `hmmwv`, `gator`, `feda`, `man5t`, `man7t`, `man10t`, `m113`, `art`, `vw`. All nine are
  dispatched in `systems/PID/PID_sim.py:136-152`, so all nine run under `pid`.
- **Controllers/systems (10, + manual):** `pid` (default), `eh`, `mppi`, `rl`, `mcl`, `acl`, `wmvct`,
  `mppi6`, `tal`, `tnt`, `manual`.
- **Worlds:** **100 elevation maps** (`world_id` 1..100, configs under
  `envs/data/BenchMaps/sampled_maps/Configs/Final`), each with **exactly 10 fixed start/goal pairs**
  (verified: all 100 configs carry 10 `positions`), so **1000 built-in A-to-B tasks**. Use these built-in
  pairs as your "location A to location B", do not invent your own.
- **Scale:** `scale_factor` `1.0` (default), `1/6`, `1/10`.

**The worlds come pre-stratified two ways, and this is the bridge to our terrain model.** The obstacle
tier is the suffix on the config filename (`config3_mid.yaml`), and the surface class is the
`terrain_type` field inside the config. Counting all 100:

| tier | rigid | deformable | mixed | total |
|------|-------|------------|-------|-------|
| low  | 22    | 7          | 6     | 35    |
| mid  | 18    | 11         | 3     | 32    |
| high | 20    | 12         | 1     | 33    |
| **total** | **60** | **30** | **10** | **100** |

VertiBench labelling its own worlds is a cleaner archetype source than clustering slope and roughness after
the fact, so prefer this 3x3 grid when filling `terrain_bucket` (section 6). Note the corner: **only one
world in the entire population is mixed/high** (world 92), so that cell can never be deep. Report it as
thin rather than padding it.

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

## 5. Sweep design (round 2, the one to run)

**Principles:**

- **~15 trials per (vehicle, terrain class) cell.** SE on a failure rate is then about 0.12, enough to fit
  hazard and compatibility and far short of exhaustive. The precision curve is flat here: n=18 gives +/-21
  points at p=0.5 and n=24 gives +/-19, so buying larger cells is poor value.
- **The target is a continuous fit, not per-cell rates.** Per section 6, estimate `p_success` as a function of
  (vehicle, measured terrain features) across all rows. Cell counts only need to be large enough to support
  that fit, which is why balance matters more than depth.
- **Where independent draws come from:** vary across the **10 start/goal pairs of a world** and across
  **several worlds of the same terrain class**. Two pairs on one map are correlated, so prefer more worlds
  over more pairs, except where a class has few worlds in the population.
- Run **headless** (`render=false use_gui=false`) for throughput. Render only a handful for a sanity video.

**Fixed for round 2:** `controller = pid`, `speed = 8.0`, `scale_factor = 1.0`, `max_time = 60`, `seed = 0`,
all 9 vehicles. Speed is **fixed, not swept**. Round 1 swept it over five levels and that was a mistake
(section 5b). Our failure model has no speed term, so a single speed is what yields a clean `p_success`.

**Worlds: 16, chosen on two axes at once.**

| class | worlds | pairs | trials/vehicle | runs |
|-------|--------|-------|----------------|------|
| rigid (60 in pop) | 1, 3, 5, 15, 20, 42, 56, 60 | 0-1 | 8 x 2 = 16 | 144 |
| deformable (30) | 61, 62, 74, 79, 89 | 0-2 | 5 x 3 = 15 | 135 |
| mixed (10) | 92, 95, 97 | 0-4 | 3 x 5 = 15 | 135 |
| **total** | **16 worlds** | | | **414** |

**Why this shape:**

- **Rigid draws from more worlds, mixed from more pairs.** Rigid is plentiful so its trials come from eight
  uncorrelated maps. Only ten mixed worlds exist, so that class takes its trials from more pairs instead.
- **Mixed is kept, and it is not optional.** Measured `terrain_deformability` is 0.0 on every rigid world and
  1.0 on every deformable one. Every interior value in round 1 (0.08, 0.14, 0.16, 0.23, 0.31, 0.42, 0.44)
  came from a mixed world. Without them that predictor is binary and the continuous fit has no interior
  support. Mixed maps are also the deployment reality (section 3).
- **The tier spread is balanced deliberately.** Tier is effectively the slope axis: round 1 measured low at
  0.10-0.13, mid at 0.25-0.29, high at 0.43-0.54. The 16 worlds are **6 high, 4 mid, 6 low**. An earlier draw
  came out 6/7/3, which left the whole high-success end of the curve resting on three worlds. Keep the ends
  populated when substituting worlds.
- **Seven worlds overlap the pilot** (1, 3, 5, 61, 62, 74, 92), giving a free check: hmmwv and gator at speed
  8 should land close to round 1. Expect close, **not identical** (section 9), and treat the spread as a
  measurement of the noise floor, which is worth reporting.

**Cost: about 6.5 min/run blended across the nine vehicles**, measured, not inferred. The vehicles differ a
lot (m113 ~4m20s and vw ~5m33s on a 15 s run, worse at `max_time=60`), so do not size a sweep off hmmwv
alone. 414 runs on 2 threads is roughly **a full day**, not a single night.

**Parallelism and resume are built into the script.** `--parallel N` splits the task list by world across
N workers, each writing its own shard CSV which the parent merges at the end, so there is no shared file and
no lock. `--resume` skips any (vehicle, world, pair, speed) already in the CSV. Arms let one command mix the
three pair counts.

```bash
cd ~/Documents/verti_bench
export LD_PRELOAD=/usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so.1
V="hmmwv gator feda man5t man7t man10t m113 art vw"
ARMS="--arm 1,3,5,15,20,42,56,60:0,1 --arm 61,62,74,79,89:0,1,2 --arm 92,95,97:0,1,2,3,4"

# 0. see the plan without running anything
vpy collect_crossings.py --dry_run --parallel 2 --speeds 8 --vehicles $V $ARMS \
    --csv results/terrain_crossing_b2.csv

# 1. smoke test one cell, confirm one fully-populated row
vpy collect_crossings.py --vehicles hmmwv --worlds 5 --start_goal_ids 0 \
    --speeds 8 --max_time 60 --csv /tmp/smoke.csv

# 2. the run, survives the terminal closing
nohup vpy collect_crossings.py --parallel 2 --resume --speeds 8 --max_time 60 \
    --vehicles $V $ARMS --csv results/terrain_crossing_b2.csv \
    > sweep_b2.log 2>&1 &
```

`--dry_run` prints 414 runs and the per-worker split (216 and 198, worlds disjoint). Monitor with:

```bash
tail -f results/terrain_crossing_b2.csv.shard0.log   # per-worker progress
wc -l results/terrain_crossing_b2.csv*               # rows so far
ps aux | grep -c "[c]ollect_crossings"               # 3 while running: parent + 2 workers
```

When both workers exit the parent merges the shards into `results/terrain_crossing_b2.csv`, header once,
and deletes them. Expect 415 lines.

**If a worker dies partway**, rerun the exact same command. `--resume` reads the merged CSV, skips what is
already there, and runs only the remainder. The parent merges whatever the workers flushed before dying, so
nothing is lost and nothing is duplicated. This is why `--resume` exists: on a job this long, dying at hour
18 and restarting from zero is the real risk.

**Start tiny:** the smoke test above is step 0, not optional. Then watch RSS on both processes through a
deformable world before leaving the sweep unattended. The machine must stay awake for a full day, so disable
sleep, and note that WSL2 suspending will stall the run.

**Not swept, and why:** speed (fixed, round 1 covered it), the other 84 worlds, pairs 5-9 for the rigid and
deformable arms, `scale_factor` 1/6 and 1/10, and the ten non-`pid` controllers.

---

## 5b. What round 1 found, and the flaw round 2 fixes

Round 1 is on disk at `results/terrain_crossing.csv`, 270 rows, and was copied to the parent repo
unchanged. Design: 2 vehicles (`hmmwv`, `gator`) x 9 worlds x 3 pairs x **5 speeds** (4, 6, 8, 10, 12) x
1 seed. The nine worlds were one per cell of the section 3 grid, so the stratification was deliberate.

Outcome: 119 reached, 83 stuck, 55 rollover, 13 timeout.

**The flaw: speed was swept and then pooled.** Each per-world cell is quoted as 15 traversals, but those 15
are 3 paths x 5 speeds, and speed moves success from 35% to 54% overall. A design factor with a real effect
is currently sitting inside a number that gets read as a terrain rate. Round 2 fixes this by fixing speed.
Round 1 still stands on its own as the characterization of the speed effect, so nothing is wasted.

**Watch item, carried forward.** `hmmwv` beat `gator` in 8 of the 9 worlds, 0.59 against 0.30 overall, and
only world 5 inverted. Our model assumes two complementary specialists where the same ground is risky for one
and safe for the other. Round 1 shows one better vehicle plus one exception instead. Round 2 with nine
vehicles is the test: either more inversions appear across the wider roster, or the modelling assumption
softens from symmetric specialists to capability-dependent suitability. Report which.

**Two facts established while running round 1, both correcting this brief as originally written:**

- **Runs are not deterministic.** Section 9 used to claim PID gives an identical result for the same
  (vehicle, world, pair, seed). It does not: multithreaded collision handling makes repeats vary, which is
  why demo runs flip outcome. Repeats are therefore a legitimate source of variation, and the pilot-overlap
  check in section 5 measures the noise floor rather than proving reproducibility.
- **Per-run cost varies a lot by vehicle.** Probed directly: m113 about 4m20s and vw about 5m33s on a 15 s
  run, worse at `max_time=60`, blending to roughly 6.5 min/run across the nine. Timing a sweep off hmmwv
  alone underestimates it by several times over. Size budgets from the blended figure.

**Not yet done:** `terrain_bucket` is empty on all 270 rows, and the section 6 step 3 comparison against the
hand-authored model was never carried out. Those are the deliverables round 2 must actually close.

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

**For round 2 the continuous fit is the deliverable, and `terrain_bucket` is the readable summary of it.**
Fit `p_success` on the measured feature columns first, per the paragraph above. Then fill `terrain_bucket`
from the built-in strata (section 3), the tier in the config filename crossed with the config's
`terrain_type`, which is a 9-way label VertiBench itself assigns and beats clustering our feature columns
after the fact. Leaving it empty is what stalled round 1. Where the fitted surface and the 9 strata disagree,
report the disagreement and collapse the strata rather than forcing the data into them.

---

## 7. What the harness already gives you, and what you must instrument

**Round 1 already did this instrumentation.** `collect_crossings.py` (repo root) drives the sweep and writes
the 24-column CSV, and `analysis/terrain_features.py` measures the terrain features. The rest of this section
is the record of what was built and what is still missing. Two notes before the detail:

- **`world_cache` was removed (2026-08-29).** The loop is now world-outer and holds only the current world,
  dropping it before the next. Each world carries two 1291x1291 float64 grids, about 27 MB, and the old
  vehicle-outer loop cached every world it touched. At 32 worlds that is ~860 MB per process, which does not
  fit twice over on this machine. Keep the world-outer shape if you touch that loop.
- **Still missing: segment-level attribution.** `path_features()` samples 64 points along the *straight* A-to-B
  line, so the feature columns describe the intended route, not the driven one, and not the failure location.
  `distance_at_failure` is populated and `local_features()` already exists, so reading features at the failure
  point is a small addition and is the cleanest signal from a mixed map (section 6 step 2).

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

- **One tidy CSV** with the schema in section 4, written somewhere stable inside the VertiBench repo. For
  round 2 that is the two `results/terrain_crossing_b2_*.csv` shards concatenated into
  `results/terrain_crossing_b2.csv`, header once. Leave round 1's `terrain_crossing.csv` in place, do not
  append to it, the two batches have different speed designs and must not be pooled by accident.
- **`terrain_bucket` populated on every row.** This is the one field round 1 left empty and it is what blocks
  everything downstream.
- **The inversion**, per (vehicle, bucket): empirical `p_success`, then `hazard_severity` and `compatibility`
  backed out of `p_success = 1 - hazard_severity * (1 - compatibility)`, in a form we can paste over the
  section 2 tables.
- A **short note** on: how you mapped map features to our terrain types (section 6), any field you could not
  populate and why, the measured-vs-predicted comparison and any model adjustments, whether the wider vehicle
  roster produces the capability inversions the model assumes (section 5b watch item), and the exact
  command(s) that produced the CSV so we can rerun.
- Do **not** commit the 6 GB assets or large sim artifacts. Just the CSV and any small scripts you add.

The end use on our side: turn the per-(vehicle, terrain) success rates into calibrated `compatibility` /
`hazard_severity` values (and optionally a distance-to-failure distribution) that ground or replace the
hand-authored table in section 2, and flag where the failure model itself needs revising. Keeping the schema
tidy and one-row-per-run is what makes that drop-in.

---

## 9. Constraints and gotchas

- This runs under **`chrono9` / `vpy`** (Python 3.9), **not** the parent repo's `uv` env. Never mix them.
- Headless throughput first, rendering is for spot-checks only.
- **PID is not deterministic.** Multithreaded collision handling makes the same (vehicle, world, pair, seed)
  vary between runs. Independence still comes mainly from varying world and start/goal pair, since those
  change the terrain rather than resampling the same crossing, but repeats do carry information and a
  re-run of a pilot cell measures the noise floor rather than reproducing it.
- **Budget about 6.5 min/run blended across the nine vehicles**, not the ~1 min hmmwv suggests. m113 and vw
  are the slow ones, measured at 4m20s and 5m33s on a 15 s run.
- **Memory, not CPU, is the binding constraint** on this machine. Each world holds two 1291x1291 float64
  grids, about 27 MB, so `collect_crossings.py` keeps only the current world resident (section 7). Watch RSS
  through a deformable world before leaving a sweep unattended.
- If an import segfaults, it is the known OpenMP clash: prefix with
  `LD_PRELOAD=/usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so.1`.
