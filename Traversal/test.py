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
    gates = [obj for obj in objects if 'Gate' in obj]  # We only want objects that contain 'Gate' in their names
    gate_positions = {gate: client.simGetObjectPose(gate).position for gate in gates}
    print(gate_positions)
    return gate_positions

def inGateSphere(position: airsimneurips.Vector3r, radius=4):
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

# Basic PID controller class for one dimension
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp      # Proportional gain
        self.ki = ki      # Integral gain
        self.kd = kd      # Derivative gain
        self.dt = dt      # Time step
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

    # Define the time step and instantiate PID controllers for each axis.
    # These PID parameters are starting points and will likely need tuning.
    dt = 0.01  # 10 ms loop time
    pid_x = PIDController(kp=1.2, ki=0.0, kd=0.3, dt=dt)
    pid_y = PIDController(kp=1.2, ki=0.0, kd=0.3, dt=dt)
    pid_z = PIDController(kp=1.2, ki=0.0, kd=0.3, dt=dt)
    
    for gate, target in gate_positions.items():
        print(f"Racing to {gate} at position {target}")
        # Reset PID controllers for the new target (optional, but can help avoid accumulated error)
        pid_x.integral = pid_y.integral = pid_z.integral = 0
        pid_x.previous_error = pid_y.previous_error = pid_z.previous_error = 0
        
        # Control loop: run until the drone enters the sphere around the current gate.
        while not inGateSphere(target):
            current = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
            error_x = target.x_val - current.x_val
            error_y = target.y_val - current.y_val
            error_z = target.z_val - current.z_val
            
            # Compute velocity commands using the PID controllers.
            vx = pid_x.update(error_x)
            vy = pid_y.update(error_y)
            vz = pid_z.update(error_z)
            
            # Send velocity command to the drone.
            client.moveByVelocityAsync(vx, vy, vz, dt, vehicle_name="drone_1")
            
            time.sleep(dt)
    
    print("Race complete")

main()
