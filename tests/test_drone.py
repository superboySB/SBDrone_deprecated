# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import os

# connect to the AirSim simulator
client = airsim.MultirotorClient(ip="172.16.13.104")
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
client.moveToPositionAsync(30, 100, 200, 5).join()


state = client.getMultirotorState(vehicle_name = '')


# take images
# responses = client.simGetImages([
#     airsim.ImageRequest("0", airsim.ImageType.DepthVis),
#     airsim.ImageRequest("1", airsim.ImageType.DepthPlanar, True)])
# print('Retrieved images: %d', len(responses))

# # do something with the images
# for response in responses:
#     if response.pixels_as_float:
#         print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
#         airsim.write_pfm('./py1.pfm', airsim.get_pfm_array(response))
#     else:
#         print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
#         airsim.write_file('./py1.png', response.image_data_uint8)