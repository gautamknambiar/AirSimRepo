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


def inGateSphere(position: airsimneurips.Vector3r, radius=3):
    dronePose = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position

    dx = dronePose.x_val - position.x_val
    dy = dronePose.y_val - position.y_val
    dz = dronePose.z_val - position.z_val
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)
    
    if distance <= radius:
        print(f"Reached the sphere for gate at position {position}, going to next gate")
        return True
    else:
        return False
# 1d pid
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.previous_error = 0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

def main():
    init()
    gate_positions = getGatePositions()

    # Parameters definitely need tuning
    dt = 0.01  # 10 ms loop time
    pid_x = PIDController(kp=1.2, ki=0.1, kd=0.1, dt=dt)
    pid_y = PIDController(kp=1.2, ki=0.1, kd=0.1, dt=dt)
    pid_z = PIDController(kp=1.2, ki=0.1, kd=0.1, dt=dt)
    
    for gate, target in gate_positions.items():
        print(f"Going to {gate} at position {target}")
        pid_x.integral = pid_y.integral = pid_z.integral = 0
        pid_x.previous_error = pid_y.previous_error = pid_z.previous_error = 0
        
        while not inGateSphere(target):
            current = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
            error_x = target.x_val - current.x_val
            error_y = target.y_val - current.y_val
            error_z = target.z_val - current.z_val
            
            vx = pid_x.update(error_x)
            vy = pid_y.update(error_y)
            vz = pid_z.update(error_z)
            
            client.moveByVelocityAsync(vx, vy, vz, dt, vehicle_name="drone_1")
            
            time.sleep(dt)
    
    print("Race complete")

main()
