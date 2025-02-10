import airsimneurips
from dataclasses import dataclass
import time

@dataclass
class GateObject:
    gateNumber: int
    x_pos: float
    y_pos: float
    z_pos: float

@dataclass
class GateSphere:
    gateNumber: int
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float



client = airsimneurips.MultirotorClient()
def setup():
    client.confirmConnection()
    client.simLoadLevel('Soccer_Field_Easy')
    client.enableApiControl(vehicle_name="drone_1")
    client.arm(vehicle_name="drone_1")
    client.simStartRace()

def moveToStart():
    start_position = airsimneurips.Vector3r(-4.25, -2.0, 1.8)
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)


def getGateData():
    # get all the gate object names
    scene_object_names = client.simListSceneObjects(name_regex='.*')
    gateNameList = []
    for name in scene_object_names:
        if ("Gate" in name):
            gateNameList.append(name)

    # create gate objects of desired data
    gateObjectList = []
    for gateName in gateNameList:
        gatePose = (client.simGetObjectPose(gateName)) # returns true pose 
        x_pos = gatePose.position.x_val
        y_pos = gatePose.position.y_val
        z_pos = gatePose.position.z_val
        gateNum = int( gateName[4:6] )
        currentGateObject = GateObject(gateNum, (x_pos-5.5), y_pos, z_pos) # adjust by 5.5 towards correct position
        gateObjectList.append(currentGateObject)

    # sort the gate objects from start gate to end gate
    sortedGateObjectList = sorted(gateObjectList, key=lambda i: i.gateNumber)

    return sortedGateObjectList


def getGateSphereData(gateDataObjs):
    RADIUS = 3.5
    spheres = []
    for gate in gateDataObjs:
        x_min = gate.x_pos-RADIUS
        x_max = gate.x_pos+RADIUS
        y_min = gate.y_pos-RADIUS
        y_max = gate.y_pos+RADIUS
        z_min = gate.z_pos-RADIUS
        z_max = gate.z_pos+RADIUS
        currentSphere = GateSphere(gate.gateNumber, x_min, x_max, y_min, y_max, z_min, z_max)
        spheres.append(currentSphere)
    return spheres
        

def moveToGate(gateData):
    print('going to gate ' +str(gateData.gateNumber))
    client.moveToPositionAsync(gateData.x_pos, gateData.y_pos, gateData.z_pos, 6.99)


def startMovement(gateDataObjs, gateSphereObjs):
    NUM_GATES = 12
    for i in range(NUM_GATES):
        # Initialize moveToPosition command for current gate
        moveToGate(gateDataObjs[i])
        
        # Check if reached sphere
        count = 0
        currentSphere = gateSphereObjs[i]
        while (reachedSphere(currentSphere, count) == False):
            count+=1
            pass
        #print('rechecks: '+str(count))

def reachedSphere(sphere, count):
    # get drone position
    dronePose = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
    x_pos = dronePose.x_val
    y_pos = dronePose.y_val
    z_pos = dronePose.z_val
    # x position check
    if (x_pos > sphere.x_min and x_pos < sphere.x_max):
        # y position check
        if (y_pos > sphere.y_min and y_pos < sphere.y_max):
            # z position check
            if (z_pos > sphere.z_min and z_pos < sphere.z_max):
                return True
    return False



if __name__ == '__main__':
    # client connection setup commands
    setup()

    # initalize start position for drone
    moveToStart()

    # get gate position data
    gateDataObjects = getGateData()

    # get gate sphere data
    gateSphereObjects = getGateSphereData(gateDataObjects)

    # move to a gate location
    startMovement(gateDataObjects, gateSphereObjects)

    print("move commands complete")