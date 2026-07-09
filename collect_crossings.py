"""Run VertiBench A-to-B crossings and append one row per run to the CSV
(results/terrain_crossing.csv). PID/headless baseline; see vertibench.md.

Example (single-cell smoke test):
  vpy collect_crossings.py --vehicles hmmwv --worlds 5 --start_goal_ids 0 \
      --speeds 4.0 --max_time 60

Example (sweep vehicles x worlds x pairs x speeds for independent trials):
  vpy collect_crossings.py --vehicles hmmwv gator --worlds 1 5 20 \
      --start_goal_ids 0 1 2 3 4 --speeds 4 6 8 --max_time 60
"""

import argparse
import csv
import os

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


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--vehicles', nargs='+', default=['hmmwv'])
    p.add_argument('--worlds', nargs='+', type=int, default=[1])
    p.add_argument('--start_goal_ids', nargs='+', type=int, default=[0])
    p.add_argument('--speeds', nargs='+', type=float, default=[4.0])
    p.add_argument('--scale_factor', type=float, default=1.0)
    p.add_argument('--max_time', type=float, default=60.0)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--csv', default=DEFAULT_CSV)
    args = p.parse_args()

    SetChronoDataDirectories()

    # Header only if the file is new/empty.
    write_header = not os.path.exists(args.csv) or os.path.getsize(args.csv) == 0
    total = (len(args.vehicles) * len(args.worlds)
             * len(args.start_goal_ids) * len(args.speeds))
    done = 0
    world_cache = {}  # world_id -> loaded map assets, reused across runs

    with open(args.csv, 'a', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        if write_header:
            writer.writeheader()

        for vehicle in args.vehicles:
            for world_id in args.worlds:
                for sg in args.start_goal_ids:
                    for speed in args.speeds:
                        done += 1
                        cfg = {
                            'vehicle': vehicle, 'system': 'pid', 'speed': speed,
                            'world_id': world_id, 'max_time': args.max_time,
                            'scale_factor': args.scale_factor, 'render': False,
                            'use_gui': False, 'start_goal_id': sg, 'seed': args.seed,
                        }
                        print(f"[{done}/{total}] vehicle={vehicle} world={world_id} "
                              f"pair={sg} speed={speed} seed={args.seed}")
                        result = single_experiment(cfg)

                        # Measure terrain features along this world's start/goal route
                        if world_id not in world_cache:
                            world_cache[world_id] = load_world(world_id, args.scale_factor)
                        start, goal = route_for(world_id, sg, args.scale_factor)
                        feats = path_features(world_cache[world_id], start, goal)

                        writer.writerow(_row(cfg, result, feats))
                        fh.flush()
                        print(f"    -> success={result['success']} "
                              f"mode={result.get('failure_mode')} "
                              f"t={result.get('sim_time'):.1f}s")


if __name__ == '__main__':
    main()
