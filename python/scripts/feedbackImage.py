from coppeliasim_zmqremoteapi_client import *
import cv2
import numpy as np

# create a client to connect to zmqRemoteApi server:
# (creation arguments can specify different host/port,
# defaults are host='localhost', port=23000)
client = RemoteAPIClient()

# get a remote object:
sim = client.getObject('sim')
print("program started")

visionSensorHandle = sim.getObject('/Vision_sensor')

sim.startSimulation()

while (t := sim.getSimulationTime()) < 100:
    img, resX, resY = sim.getVisionSensorCharImage(visionSensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(resY, resX, 3)

    # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
    # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
    # and color format is RGB triplets, whereas OpenCV uses BGR:
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)

    cv2.imshow('', img)
    cv2.waitKey(1)

sim.stopSimulation()
cv2.destroyAllWindows()


print("program ended")