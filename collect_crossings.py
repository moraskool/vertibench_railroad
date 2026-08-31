"""Run VertiBench A-to-B crossings and append one row per run to the CSV
(results/terrain_crossing.csv). PID/headless baseline; see vertibench.md.

Example (single-cell smoke test):
  vpy collect_crossings.py --vehicles hmmwv --worlds 5 --start_goal_ids 0 \
      --speeds 4.0 --max_time 60

Example (one arm: worlds x pairs, all vehicles):
  vpy collect_crossings.py --vehicles hmmwv gator --worlds 1 5 20 \
      --start_goal_ids 0 1 2 3 4 --speeds 8 --max_time 60

Example (round 2: three arms with different pair counts, 2 workers, resumable):
  vpy collect_crossings.py --parallel 2 --resume --speeds 8 --max_time 60 \
      --vehicles hmmwv gator feda man5t man7t man10t m113 art vw \
      --arm 1,3,5,15,20,42,56,60:0,1 \
      --arm 61,62,74,79,89:0,1,2 \
      --arm 92,95,97:0,1,2,3,4 \
      --csv results/terrain_crossing_b2.csv

--resume skips (vehicle, world, pair, speed) rows already in the CSV, so a job
killed at hour 18 of 23 picks up where it stopped. Workers each write their own
shard file and the parent merges at the end, never a shared file plus a lock.
"""

import argparse
import collections
import csv
import json
import os
import subprocess
import sys

from verti_bench.envs.utils.utils import SetChronoDataDirectories
from verti_bench.setup import single_experiment
from verti_bench.analysis.terrain_features import load_world, route_for, path_features

# CSV schema. First 20 match the parent repo's terrain_crossing.csv exactly (keep
# names/order); sim_time + distance_traveled are appended as always-on fields
# (populated for every run, success or failure).
COLUMNS = [
    'vehicle', 'world_id', 'start_goal_id', 'controller', 'speed', 'scale_factor',
    'seed', 'success', 'failure_mode', 'time_to_goal', 'time_at_failure',
    'distance_at_failure', 'remaining_distance', 'total_path_length',
    'avg_roll', 'avg_pitch', 'terrain_slope', 'terrain_roughness',
    'terrain_deformability', 'terrain_bucket',
    'sim_time', 'distance_traveled',
    'fraction_completed', 'est_time_remaining',
]

DEFAULT_CSV = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                           "results", "terrain_crossing.csv")


def _derived(result):
    """Fraction of the intended path covered, and an estimate of the time that
    would have remained to the goal (remaining_distance / avg speed so far).
    Time-remaining is only an estimate: a failed run never reached the goal."""
    tpl = result.get('total_path_length')
    traveled = result.get('traveled_distance')
    frac = traveled / tpl if (tpl and traveled is not None and tpl > 0) else ''

    if result['success']:
        est_rem = 0.0  # already at the goal
    else:
        # Pace while actively progressing: use time_at_failure, not sim_time
        # (sim_time includes the idle stuck window before failure is declared).
        rem = result.get('remaining_distance')
        t_fail = result.get('time_at_failure')
        avg_speed = (traveled / t_fail) if (traveled and t_fail and t_fail > 0) else None
        est_rem = rem / avg_speed if (avg_speed and rem is not None) else ''
    return frac, est_rem


def _row(cfg, result, feats):
    """Map a run config + result + measured terrain features onto the CSV schema.

    terrain_slope/roughness/deformability are measured along the route;
    terrain_bucket is left blank (derived data-driven in the analysis step).
    """
    frac, est_rem = _derived(result)
    return {
        'vehicle': cfg['vehicle'], 'world_id': cfg['world_id'],
        'start_goal_id': result.get('start_goal_id', cfg.get('start_goal_id')),
        'controller': cfg['system'], 'speed': cfg['speed'],
        'scale_factor': cfg['scale_factor'], 'seed': cfg.get('seed'),
        'success': int(result['success']), 'failure_mode': result.get('failure_mode'),
        'time_to_goal': result.get('time_to_goal'),
        'time_at_failure': result.get('time_at_failure'),
        'distance_at_failure': result.get('distance_at_failure'),
        'remaining_distance': result.get('remaining_distance'),
        'total_path_length': result.get('total_path_length'),
        'avg_roll': result.get('avg_roll'), 'avg_pitch': result.get('avg_pitch'),
        'terrain_slope': feats['terrain_slope'],
        'terrain_roughness': feats['terrain_roughness'],
        'terrain_deformability': feats['terrain_deformability'],
        'terrain_bucket': '',
        'sim_time': result.get('sim_time'),
        'distance_traveled': result.get('traveled_distance'),
        'fraction_completed': frac,
        'est_time_remaining': est_rem,
    }


def _parse_arm(spec):
    """'1,3,5:0,1' -> ([1,3,5], [0,1]). One arm is a world set and its pair set."""
    worlds, pairs = spec.split(':')
    return ([int(w) for w in worlds.split(',') if w],
            [int(p) for p in pairs.split(',') if p])


def _tasks(arms, vehicles, speeds):
    """Every (world, vehicle, pair, speed) to run, world-outer so each world loads once."""
    return [(w, v, sg, sp)
            for worlds, pairs in arms for w in worlds
            for v in vehicles for sg in pairs for sp in speeds]


def _done_keys(csv_path):
    """(vehicle, world, pair, speed) already present, so --resume can skip them."""
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        return set()
    done = set()
    with open(csv_path, newline='') as fh:
        for r in csv.DictReader(fh):
            try:
                done.add((r['vehicle'], int(r['world_id']),
                          int(r['start_goal_id']), float(r['speed'])))
            except (ValueError, KeyError, TypeError):
                continue  # malformed/partial row, treat as not done and rerun it
    return done


def _shard(tasks, n):
    """Split by world across n workers, balanced by task count. A world lands in
    exactly one worker so no two processes hold the same map."""
    by_world = collections.OrderedDict()
    for t in tasks:
        by_world.setdefault(t[0], []).append(t)
    shards, loads = [[] for _ in range(n)], [0] * n
    for _, ts in sorted(by_world.items(), key=lambda kv: -len(kv[1])):
        i = loads.index(min(loads))
        shards[i].extend(ts)
        loads[i] += len(ts)
    return [s for s in shards if s]


def _merge(shard_csvs, out_csv):
    """Concatenate shard files into out_csv, header once, then delete the shards."""
    have = os.path.exists(out_csv) and os.path.getsize(out_csv) > 0
    with open(out_csv, 'a', newline='') as out:
        for sc in shard_csvs:
            if not os.path.exists(sc) or os.path.getsize(sc) == 0:
                continue
            with open(sc, newline='') as fh:
                lines = fh.readlines()
            out.writelines(lines if not have else lines[1:])
            have = True
    for sc in shard_csvs:
        if os.path.exists(sc):
            os.remove(sc)


def _run(tasks, args, csv_path):
    """Run tasks in order, one row per run, flushed as it goes."""
    write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    total, current, world = len(tasks), None, None

    with open(csv_path, 'a', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        if write_header:
            writer.writeheader()

        for done, (world_id, vehicle, sg, speed) in enumerate(tasks, 1):
            # Tasks are world-ordered, so load on change and keep only the current
            # world resident. Each holds two 1291x1291 float64 grids, about 27 MB.
            if world_id != current:
                world = load_world(world_id, args.scale_factor)
                current = world_id

            cfg = {
                'vehicle': vehicle, 'system': 'pid', 'speed': speed,
                'world_id': world_id, 'max_time': args.max_time,
                'scale_factor': args.scale_factor, 'render': False,
                'use_gui': False, 'start_goal_id': sg, 'seed': args.seed,
            }
            print(f"[{done}/{total}] vehicle={vehicle} world={world_id} "
                  f"pair={sg} speed={speed} seed={args.seed}", flush=True)
            result = single_experiment(cfg)

            # Measure terrain features along this world's start/goal route
            start, goal = route_for(world_id, sg, args.scale_factor)
            feats = path_features(world, start, goal)

            writer.writerow(_row(cfg, result, feats))
            fh.flush()
            print(f"    -> success={result['success']} "
                  f"mode={result.get('failure_mode')} "
                  f"t={result.get('sim_time'):.1f}s", flush=True)


def _spawn(tasks, args):
    """Fan tasks across args.parallel workers, each to its own shard CSV, then merge."""
    shards = _shard(tasks, args.parallel)
    procs, shard_csvs = [], []

    for i, chunk in enumerate(shards):
        task_file = f"{args.csv}.tasks{i}.json"
        shard_csv = f"{args.csv}.shard{i}"
        with open(task_file, 'w') as fh:
            json.dump(chunk, fh)
        shard_csvs.append(shard_csv)
        cmd = [sys.executable, os.path.realpath(__file__),
               '--_tasks', task_file, '--csv', shard_csv,
               '--scale_factor', str(args.scale_factor),
               '--max_time', str(args.max_time), '--seed', str(args.seed)]
        log = open(f"{args.csv}.shard{i}.log", 'w')
        print(f"worker {i}: {len(chunk)} runs over "
              f"{len({t[0] for t in chunk})} worlds -> {shard_csv}")
        procs.append((subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT),
                      log, task_file))

    codes = []
    for proc, log, task_file in procs:
        codes.append(proc.wait())
        log.close()
        os.remove(task_file)

    _merge(shard_csvs, args.csv)
    if any(codes):
        print(f"WARNING: worker exit codes {codes}. Rows written so far are merged; "
              f"rerun the same command with --resume to finish.", file=sys.stderr)
    return max(codes) if codes else 0


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--vehicles', nargs='+', default=['hmmwv'])
    p.add_argument('--worlds', nargs='+', type=int, default=[1])
    p.add_argument('--start_goal_ids', nargs='+', type=int, default=[0])
    p.add_argument('--arm', action='append', default=None, metavar='WORLDS:PAIRS',
                   help='repeatable arm, e.g. 61,62,74:0,1,2. Overrides '
                        '--worlds/--start_goal_ids and lets one run mix pair counts')
    p.add_argument('--speeds', nargs='+', type=float, default=[4.0])
    p.add_argument('--scale_factor', type=float, default=1.0)
    p.add_argument('--max_time', type=float, default=60.0)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--parallel', type=int, default=1, help='worker processes (default 1)')
    p.add_argument('--resume', action='store_true',
                   help='skip (vehicle, world, pair, speed) rows already in the CSV')
    p.add_argument('--dry_run', action='store_true', help='print the plan and exit')
    p.add_argument('--csv', default=DEFAULT_CSV)
    p.add_argument('--_tasks', default=None, help=argparse.SUPPRESS)  # worker only
    args = p.parse_args()

    SetChronoDataDirectories()

    # Worker: run exactly the task list the parent handed us.
    if args._tasks:
        with open(args._tasks) as fh:
            _run([tuple(t) for t in json.load(fh)], args, args.csv)
        return 0

    arms = ([_parse_arm(a) for a in args.arm] if args.arm
            else [(args.worlds, args.start_goal_ids)])
    tasks = _tasks(arms, args.vehicles, args.speeds)

    planned = len(tasks)
    if args.resume:
        done = _done_keys(args.csv)
        tasks = [t for t in tasks if (t[1], t[0], t[2], t[3]) not in done]
        print(f"resume: {planned} planned, {planned - len(tasks)} already in "
              f"{args.csv}, {len(tasks)} to run")
    else:
        print(f"{planned} runs planned")

    if not tasks:
        print("nothing to do")
        return 0
    if args.dry_run:
        print(f"worlds: {sorted({t[0] for t in tasks})}")
        for i, chunk in enumerate(_shard(tasks, max(1, args.parallel))):
            print(f"  worker {i}: {len(chunk)} runs, "
                  f"worlds {sorted({t[0] for t in chunk})}")
        return 0

    if args.parallel > 1:
        return _spawn(tasks, args)
    _run(tasks, args, args.csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())
