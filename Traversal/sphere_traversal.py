import airsimneurips
import math
import time
client = airsimneurips.MultirotorClient()

def init():
    client.confirmConnection()
    print('Connection confirmed')  # Confirms that connection to the simulation is successful
    client.simLoadLevel('Soccer_Field_Easy')
    client.enableApiControl(vehicle_name="drone_1")  # Enable API control for the drone
    client.arm(vehicle_name="drone_1")  # Arm the drone so it can take off
    start_position = airsimneurips.Vector3r(-4.25, -2.0, 1.8)
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)


def getGatePositions():
    objects = client.simListSceneObjects()
    gates = [obj for obj in objects if 'Gate' in obj]
    gate_positions = {gate: client.simGetObjectPose(gate).position for gate in gates}
    def extract_gate_number(gate_name):
        remainder = gate_name.replace("Gate", "")
        number_str = remainder.split("_")[0]
        try:
            return int(number_str)
        except ValueError:
            return float('inf')
    sorted_gate_positions = {gate: gate_positions[gate] for gate in sorted(gate_positions, key=extract_gate_number)}
    print(sorted_gate_positions)
    return sorted_gate_positions

def inGateSphere(position: airsimneurips.Vector3r, radius = 3):
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
        client.moveToPositionAsync(position.x_val, position.y_val, position.z_val, 4, vehicle_name="drone_1")
        while not inGateSphere(position):
            pass
    print("Complete")

main()