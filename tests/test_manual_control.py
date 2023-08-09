"""
For connecting to the AirSim drone environment and testing API functionality
"""
import airsim

import os
import tempfile
import pprint

# connect to the AirSim simulator
client = airsim.MultirotorClient(ip="172.16.13.104")
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True,vehicle_name='UAV_1')

state = client.getMultirotorState(vehicle_name='UAV_1')
s = pprint.pformat(state)
print("state: %s" % s)

client.takeoffAsync(timeout_sec = 20, vehicle_name = 'UAV_1')
# client.moveByManualAsync(vx_max = 1E6, vy_max = 1E6, z_min = -1E6, duration = 1, vehicle_name='UAV_1')   # 控制杆量
# airsim.wait_key('Manual mode is setup. Press any key to send RC data to takeoff')


# 会持续控制需要覆盖
client.moveByRC(rcdata = airsim.RCData(pitch = 1, throttle = 0.5, is_initialized = True, is_valid = True), vehicle_name='UAV_1')

client.moveByRC(rcdata = airsim.RCData(pitch = 0, throttle = 0.1, is_initialized = True, is_valid = True), vehicle_name='UAV_1')
