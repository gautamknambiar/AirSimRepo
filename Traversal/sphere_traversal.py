import airsimneurips
import math
import time
client = airsimneurips.MultirotorClient()

def init():
    client.confirmConnection()
    print('Connection confirmed')  # Confirms that connection to the simulation is successful
    client.enableApiControl(vehicle_name="drone_1")  # Enable API control for the drone
    client.arm(vehicle_name="drone_1")  # Arm the drone so it can take off
    client.simStartRace() # Start Race
    start_position = airsimneurips.Vector3r(-4.55, 0.5, 2.0) 
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)

def getGatePositions():
    objects = client.simListSceneObjects() 
    gates = [obj for obj in objects if 'Gate' in obj] # We only want objects that contain 'Gate' in their names
    gate_positions = {gate: client.simGetObjectPose(gate).position for gate in gates}
    print(gate_positions)
    return gate_positions

def inGateSphere(position: airsimneurips.Vector3r, radius = 4):
    dronePose = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position

    dx = dronePose.x_val - position.x_val
    dy = dronePose.y_val - position.y_val
    dz = dronePose.z_val - position.z_val
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    if distance <= radius:
        print(f"Reached the sphere for gate at position {position}, going to next gate")
        return True
    else:
        return False
def main():
    init()
    gate_positions = getGatePositions()
    for gate, position in gate_positions.items():
        print(f"Flying to {gate} at position {position}")
        client.moveToPositionAsync(position.x_val, position.y_val, position.z_val, 7, vehicle_name="drone_1")
        while not inGateSphere(position):
            time.sleep(0.01)
    print("Complete")

main()