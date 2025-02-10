import airsimneurips
from dataclasses import dataclass
import time

@dataclass
class GateObject:
    gateNumber: int
    x_pos: float
    y_pos: float
    z_pos: float



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
        currentGateObject = GateObject(gateNum, x_pos, y_pos, z_pos)
        gateObjectList.append(currentGateObject)

    # sort the gate objects from start gate to end gate
    sortedGateObjectList = sorted(gateObjectList, key=lambda i: i.gateNumber)

    return sortedGateObjectList



def moveToGate(gateData):
    print('going to gate ' +str(gateData.gateNumber))
    client.moveToPositionAsync(gateData.x_pos, gateData.y_pos, gateData.z_pos, 5).join()


def getGateSphereData(gateObjects):
    pass


if __name__ == '__main__':
    # client connection setup commands
    setup()

    # initalize start position for drone
    moveToStart()

    # get gate data
    gateDataObjects = getGateData()

    # get gate sphere data
    gateSphereObjects = getGateSphereData(gateDataObjects)

    # move to a gate location
    for gateObj in gateDataObjects:
        gateObj.x_pos -= 5.5
        moveToGate(gateObj)

    print("move commands complete")