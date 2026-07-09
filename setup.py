import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr 

import os
import sys
import glob
import multiprocessing
import random
import numpy as np
import logging
import yaml
import argparse
from PIL import Image

from verti_bench.envs.utils.utils import SetChronoDataDirectories

def _make_sim(config):
    """Lazily import and construct only the requested control system.

    Systems other than PID pull in heavy/optional deps (e.g. ROS grid_map_msgs
    for MPPI), so importing all of them eagerly breaks a PID-only environment.
    """
    system = config['system']
    if system == 'pid':
        from verti_bench.systems.PID.PID_sim import PIDSim
        return PIDSim(config)
    elif system == 'eh':
        from verti_bench.systems.EH.EH_sim import EHSim
        return EHSim(config)
    elif system == 'mppi':
        from verti_bench.systems.MPPI.MPPI_sim import MPPISim
        return MPPISim(config)
    elif system == 'rl':
        from verti_bench.systems.RL.RL_sim import RLSim
        return RLSim(config)
    elif system == 'mcl':
        from verti_bench.systems.MCL.MCL_sim import MCLSim
        return MCLSim(config)
    elif system == 'acl':
        from verti_bench.systems.ACL.ACL_sim import ACLSim
        return ACLSim(config)
    elif system == 'wmvct':
        from verti_bench.systems.WMVCT.WMVCT_sim import WMVCTSim
        return WMVCTSim(config)
    elif system == 'mppi6':
        from verti_bench.systems.MPPI6.MPPI6_sim import MPPI6Sim
        return MPPI6Sim(config)
    elif system == 'tal':
        from verti_bench.systems.TAL.TAL_sim import TALSim
        return TALSim(config)
    elif system == 'tnt':
        from verti_bench.systems.TNT.TNT_sim import TNTSim
        return TNTSim(config)
    elif system == 'manual':
        from verti_bench.systems.Manual.Manual_sim import ManualSim
        return ManualSim(config)
    raise ValueError(f"Unsupported system type: {system}")

def single_experiment(config):
    """Run a single simulation experiment.

    PID_sim.run() returns a rich result dict (with failure_mode and the
    distance/time-at-failure instrumentation); the other systems still return
    the legacy ``(time_to_goal, success, avg_roll, avg_pitch)`` tuple. Normalize
    both so downstream code keeps working unchanged.
    """
    sim = _make_sim(config)
    sim.initialize()
    result = sim.run()

    if isinstance(result, dict):
        return result

    # Legacy tuple shape -> original dict (unchanged behavior)
    time_to_goal, success, avg_roll, avg_pitch = result
    return {
        'time_to_goal': time_to_goal if success else None,
        'success': success,
        'avg_roll': avg_roll,
        'avg_pitch': avg_pitch
    }

def multiple_experiments(config, num_experiments=5):
    """Run multiple simulation experiments and aggregate results"""
    results = []
    
    for i in range(num_experiments):
        print(f"Running experiment {i + 1}/{num_experiments}")
        result = single_experiment(config)
        results.append(result)
        
    # Process results 
    success_count = sum(1 for r in results if r['success'])
    successful_times = [r['time_to_goal'] for r in results if r['time_to_goal'] is not None]
    avg_rolls = [r['avg_roll'] for r in results if r['success']]
    avg_pitches = [r['avg_pitch'] for r in results if r['success']]

    mean_traversal_time = np.mean(successful_times) if successful_times else None
    roll_mean = np.mean(avg_rolls) if avg_rolls else None
    roll_variance = np.var(avg_rolls) if avg_rolls else None
    pitch_mean = np.mean(avg_pitches) if avg_pitches else None
    pitch_variance = np.var(avg_pitches) if avg_pitches else None

    # Print results
    print("--------------------------------------------------------------")
    print(f"Success rate: {success_count}/{num_experiments}")
    if success_count > 0:
        print(f"Mean traversal time (successful trials): {mean_traversal_time:.2f} seconds")
        print(f"Average roll angle: {roll_mean:.2f} degrees, Variance: {roll_variance:.2f}")
        print(f"Average pitch angle: {pitch_mean:.2f} degrees, Variance: {pitch_variance:.2f}")
    else:
        print("No successful trials")
    print("--------------------------------------------------------------")
    
    return results

def parse_arguments():
    processed_args = []
    for arg in sys.argv[1:]: 
        if '=' in arg and not arg.startswith('-'):
            key, value = arg.split('=', 1)
            processed_args.append(f"--{key}")
            processed_args.append(value)
        else:
            processed_args.append(arg)
    
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description='Run vehicle simulation with various configurations')
    
    # Vehicle and system parameters
    parser.add_argument('--vehicle', type=str, default='hmmwv', help='Vehicle type (default: hmmwv)')
    parser.add_argument('--system', type=str, default='pid', help='Control system type (default: pid)')
    parser.add_argument('--speed', type=float, default=4.0, help='Target vehicle speed (default: 4.0)')
    
    # World parameters
    parser.add_argument('--world_id', type=int, default=1, help='World ID (1-100, default: 1)')
    parser.add_argument('--scale_factor', type=float, default=1.0,
                        help='Scale factor for terrain (default: 1.0, options: 1.0, 1/6, 1/10)')
    parser.add_argument('--start_goal_id', type=int, default=None,
                        help='Which of the world\'s 10 start/goal pairs (0-9). Default: random.')
    parser.add_argument('--seed', type=int, default=None,
                        help='RNG seed for reproducibility (default: None).')
    
    # Simulation parameters
    parser.add_argument('--max_time', type=float, default=60.0, help='Maximum simulation time in seconds (default: 60.0)')
    parser.add_argument('--num_experiments', type=int, default=1, 
                        help='Number of experiments to run (default: 1)')
    
    # Visualization parameters
    parser.add_argument('--render', type=lambda x: (str(x).lower() == 'true'), default=True, 
                        help='Enable rendering (default: True)')
    parser.add_argument('--use_gui', type=lambda x: (str(x).lower() == 'true'), default=False, 
                        help='Enable GUI control (default: False)')
    
    args = parser.parse_args(processed_args)
    return args

if __name__ == '__main__':
    # Load configuration file
    SetChronoDataDirectories()
    
    # Parse command-line arguments
    args = parse_arguments()
    
    # Create config dictionary from arguments
    config = {
        'vehicle': args.vehicle,
        'speed': args.speed,
        'system': args.system,
        'world_id': args.world_id,
        'max_time': args.max_time,
        'scale_factor': args.scale_factor,
        'render': args.render,
        'use_gui': args.use_gui,
        'start_goal_id': args.start_goal_id,
        'seed': args.seed
    }
    
    print("--------------------------------------------------------------")
    print("Verti-Bench Configs:")
    for key, value in config.items():
        print(f"  {key}: {value}")
    print("--------------------------------------------------------------")
    
    # Run simulation
    multiple_experiments(config, num_experiments=args.num_experiments)
    