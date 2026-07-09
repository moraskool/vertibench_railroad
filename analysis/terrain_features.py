"""Measure slope / roughness / deformability along a route from VertiBench map
assets, bridging mixed-terrain maps to the planner's terrain model (vertibench.md
sec 2, 6). No bucketing/thresholds - only the
three measured feature values; terrain_bucket is derived later from the data."""

import glob
import os

import numpy as np
import yaml

_MAPS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    "envs", "data", "BenchMaps", "sampled_maps",
)
_CONFIG_DIR = os.path.join(_MAPS_DIR, "Configs", "Final")

# Material label ints, matching envs/terrain.py::_load_texture_config.
_TYPE_TO_LABEL = {
    'clay': 0, 'concrete': 1, 'dirt': 2, 'grass': 3,
    'gravel': 4, 'rock': 5, 'wood': 6, 'mud': 7, 'sand': 8, 'snow': 9,
}
_LABEL_TO_TYPE = {v: k for k, v in _TYPE_TO_LABEL.items()}


def _find(pattern):
    matches = glob.glob(os.path.join(_CONFIG_DIR, pattern))
    if not matches:
        raise FileNotFoundError(f"No file matching {pattern} in {_CONFIG_DIR}")
    return matches[0]


def route_for(world_id, start_goal_id, scale_factor=1.0):
    """Return the (start, goal) (x, y) for one of a world's 10 pairs, scaled."""
    with open(_find(f"config{world_id}_*.yaml"), 'r') as f:
        cfg = yaml.safe_load(f)
    pair = cfg['positions'][start_goal_id]
    start = [c * scale_factor for c in pair['start'][:2]]
    goal = [c * scale_factor for c in pair['goal'][:2]]
    return start, goal


def load_world(world_id, scale_factor=1.0):
    """Load elevation grid, material labels, and geometry for one world."""
    with open(_find(f"config{world_id}_*.yaml"), 'r') as f:
        cfg = yaml.safe_load(f)

    # High-res heights (metres); reorient to match envs/terrain.py.
    heights = np.load(_find(f"height{world_id}_*.npy"))
    heights = np.rot90(np.flip(heights, axis=1), k=2, axes=(1, 0))
    labels = np.load(_find(f"labels{world_id}_*.npy"))

    deformable_labels = {
        _TYPE_TO_LABEL[t['terrain_type']] for t in cfg['textures'] if t.get('is_deformable')
    }

    length = cfg['terrain']['length'] * scale_factor
    width = cfg['terrain']['width'] * scale_factor
    hy, hx = heights.shape
    spacing_x, spacing_y = (2.0 * length) / hx, (2.0 * width) / hy
    # Slope field = gradient magnitude (rise/run) of the elevation grid.
    gy, gx = np.gradient(heights, spacing_y, spacing_x)

    return {
        'world_id': world_id, 'heights': heights,
        'slope_field': np.sqrt(gx ** 2 + gy ** 2),
        'labels': labels, 'deformable_labels': deformable_labels,
        'terrain_type': cfg['terrain_type'], 'length': length, 'width': width,
        'spacing_m': 0.5 * (spacing_x + spacing_y),
    }


def _to_pixel(x, y, length, width, dim_x, dim_y):
    """PyChrono (x, y) -> (row, col); mirrors transform_to_high_res (y negated)."""
    col = int(np.clip(round((dim_x / (2.0 * length)) * (x + length)), 0, dim_x - 1))
    row = int(np.clip(round((dim_y / (2.0 * width)) * (-y + width)), 0, dim_y - 1))
    return row, col


def local_features(world, x, y, window_m=2.0):
    """Slope / roughness / deformability / material at one (x, y) point."""
    heights, slope_field, labels = world['heights'], world['slope_field'], world['labels']
    hy, hx = heights.shape
    ly, lx = labels.shape

    row, col = _to_pixel(x, y, world['length'], world['width'], hx, hy)
    half = max(1, int(round((window_m / world['spacing_m']) / 2)))
    r0, r1 = max(0, row - half), min(hy, row + half + 1)
    c0, c1 = max(0, col - half), min(hx, col + half + 1)

    lrow, lcol = _to_pixel(x, y, world['length'], world['width'], lx, ly)
    label = int(labels[lrow, lcol])
    return {
        'slope': float(np.mean(slope_field[r0:r1, c0:c1])),
        'roughness': float(np.std(heights[r0:r1, c0:c1])),
        'deformability': 1.0 if label in world['deformable_labels'] else 0.0,
        'material': _LABEL_TO_TYPE.get(label, str(label)),
    }


def path_features(world, start, goal, n=64, window_m=2.0):
    """Measure mean features along the straight A->B route (scaled chrono coords)."""
    xs, ys = np.linspace(start[0], goal[0], n), np.linspace(start[1], goal[1], n)
    slopes, roughs, defs, materials = [], [], [], []
    for x, y in zip(xs, ys):
        f = local_features(world, x, y, window_m=window_m)
        slopes.append(f['slope']); roughs.append(f['roughness'])
        defs.append(f['deformability']); materials.append(f['material'])

    return {
        'terrain_slope': float(np.mean(slopes)),
        'terrain_roughness': float(np.mean(roughs)),
        'terrain_deformability': float(np.mean(defs)),
        'dominant_material': max(set(materials), key=materials.count),
    }