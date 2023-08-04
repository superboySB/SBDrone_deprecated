import airsim
import time

# this script moves the drone to a location, then rests it thousands of time
# purpose of this script is to stress test reset API

# connect to the AirSim simulator 
client = airsim.MultirotorClient(ip="172.16.13.104",port=41451)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)


for idx in range(3000):
    # client.moveToPositionAsync(0, 0, -10, 5).join()
    # client.reset()
    # client.enableApiControl(True)
    print(client.getMultirotorState())
    print("%d" % idx)
    time.sleep(1)


# that's enough fun for now. let's quite cleanly
client.enableApiControl(False)
