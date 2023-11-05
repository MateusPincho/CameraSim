# Make sure to have the add-on "ZMQ remote API" running in
# CoppeliaSim and have following scene loaded:
#
# scenes/messaging/synchronousImageTransmissionViaRemoteApi.ttt
#
# Do not launch simulation, but run this script
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time
import math
import numpy as np
import cv2

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

visionSensorHandle = sim.getObject('/Vision_sensor')
#passiveVisionSensorHandle = sim.getObject('/PassiveVisionSensor')

# When simulation is not running, ZMQ message handling could be a bit
# slow, since the idle loop runs at 8 Hz by default. So let's make
# sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)   
sim.setInt32Param(sim.intparam_idle_fps, 0)

# Run a simulation in stepping mode:
client.setStepping(True)
sim.startSimulation()

#Get Vision Sensor Transformation Matrix
transformation_matrix = sim.getObjectMatrix(visionSensorHandle, -1)
euler_angles = sim.getEulerAnglesFromMatrix(transformation_matrix)
print(euler_angles)

#Vision sensor internal parameters
#Run this before to adjust the perspective angle
def get_perspective_angle(focal_length, sensor_size):
    #for camera v2: 
    #focal lenght = 3.04mm
    #sensor size = 4.6 mm
    perspective_angle = 2*math.atan(sensor_size / (2*focal_length))
    return perspective_angle

print(get_perspective_angle(3.04,3.68))


while (t := sim.getSimulationTime()) < 10:
    img, resX, resY = sim.getVisionSensorCharImage(visionSensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)

    # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
    # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
    # and color format is RGB triplets, whereas OpenCV uses BGR:
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)

    cv2.imshow('', img)
    cv2.waitKey(1)
    client.step()  # triggers next simulation step

    cv2.imwrite('image.jpg',img)


sim.stopSimulation()

# Restore the original idle loop frequency:
sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

cv2.destroyAllWindows()

print('Program ended')
