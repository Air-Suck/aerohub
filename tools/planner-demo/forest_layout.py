#!/usr/bin/env python3
"""Deterministic version of the official Swarm-Formation demo map."""

import math
import random


# Values from ego_planner/launch/normal_hexagon.launch.
SEED = 31
MAP_X_SIZE = 34.0
MAP_Y_SIZE = 15.0
MAP_Z_SIZE = 3.0
RESOLUTION = 0.1
PILLAR_COUNT = 60
PILLAR_WIDTH = 0.5
PILLAR_HEIGHT_RANGE = (2.0, 3.0)
PILLAR_INFLATION_RANGE = (0.5, 1.5)
MIN_PILLAR_DISTANCE = 0.8
RING_COUNT = 20
RING_RADIUS_Y = 1.0
RING_RADIUS_Z_RANGE = (1.0, 1.2)
RING_CENTER_Z_RANGE = (0.7, 3.0)
RING_YAW_RANGE = (-0.5, 0.5)
RING_TUBE_RADIUS = 0.06


def generate_layout(seed=SEED):
    """Use the official uniform distributions with a fixed reproducible seed."""
    rng = random.Random(seed)
    pillars = []
    while len(pillars) < PILLAR_COUNT:
        x = rng.uniform(-MAP_X_SIZE / 2.0, MAP_X_SIZE / 2.0)
        y = rng.uniform(-MAP_Y_SIZE / 2.0, MAP_Y_SIZE / 2.0)
        radius = PILLAR_WIDTH * rng.uniform(*PILLAR_INFLATION_RANGE) / 2.0
        height = rng.uniform(*PILLAR_HEIGHT_RANGE)
        if any(math.hypot(x - px, y - py) < MIN_PILLAR_DISTANCE
               for px, py, _, _ in pillars):
            continue
        pillars.append((round(x, 3), round(y, 3), round(radius, 3), round(height, 3)))

    rings = []
    for _ in range(RING_COUNT):
        rings.append((
            round(rng.uniform(-MAP_X_SIZE / 2.0, MAP_X_SIZE / 2.0), 3),
            round(rng.uniform(-MAP_Y_SIZE / 2.0, MAP_Y_SIZE / 2.0), 3),
            round(rng.uniform(*RING_CENTER_Z_RANGE), 3),
            RING_RADIUS_Y,
            round(rng.uniform(*RING_RADIUS_Z_RANGE), 3),
            round(rng.uniform(*RING_YAW_RANGE), 3),
        ))
    return pillars, rings


def ring_centerline(ring, segments=126):
    x, y, z, radius_y, radius_z, yaw = ring
    points = []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        local_y = radius_y * math.cos(angle)
        points.append((
            x - math.sin(yaw) * local_y,
            y + math.cos(yaw) * local_y,
            z + radius_z * math.sin(angle),
        ))
    return points


PILLARS, RINGS = generate_layout()
