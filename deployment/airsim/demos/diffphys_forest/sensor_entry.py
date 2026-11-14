"""Reuse the shared platform sensor driver and its physical extrinsic conversion."""
import sys
sys.path.insert(0,'/repo/simulator/airsim/sensors')
from uav_as2_sensor_adapter.node import main
if __name__=='__main__':main()
