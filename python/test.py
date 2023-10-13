from coppeliasim_zmqremoteapi_client import *

# create a client to connect to zmqRemoteApi server:
# (creation arguments can specify different host/port,
# defaults are host='localhost', port=23000)
client = RemoteAPIClient()

# get a remote object:
sim = client.getObject('sim')

# call API function:
h = sim.getObject('/Floor')
print(h)