#!/usr/bin/env python3
"""Use the existing 50 Hz clock bridge with a persistent AirSim RPC session."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'simulator/airsim/sensors'))
from uav_as2_sensor_adapter import rpc
rpc.Rpc = rpc.PersistentRpc
from uav_as2_sensor_adapter.clock import main

if __name__ == '__main__':
    main()
