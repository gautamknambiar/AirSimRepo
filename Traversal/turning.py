import airsimneurips
import math
import time
import numpy as np
import matplotlib.pyplot as plt

client = airsimneurips.MultirotorClient()

def init():
    """
    Connect using API
    """
    client.confirmConnection()
    print('Connection confirmed')  # Confirms that connection to the simulation is successful
    client.simLoadLevel('Soccer_Field_Easy')
    client.enableApiControl(vehicle_name="drone_1")  # Enable API control for the drone
    client.arm(vehicle_name="drone_1")  # Arm the drone so it can take off
    start_position = airsimneurips.Vector3r(-3, -2.0, 1.8)
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)

def getGatePositions():
    """
    Using the Scene objects, grab the gate positions and names
    """
    objects = client.simListSceneObjects()
    print(objects)
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
    """
    Given a radius and position vector, calculate if the drone is in the gate sphere
    """
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
        """
        Updates PID values and returns an output value for 1d
        """
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

def get_forward_vector(q):
    """
    Compute the drone's forward unit vector from its orientation quaternion.
    Assumes the drone's local forward direction is along the +X axis.
    The quaternion q is assumed to have attributes: x_val, y_val, z_val, w_val.
    """

    #roll = math.atan2(2*(q.w_val*q.x_val + q.y_val*q.z_val), 1 - 2*(q.x_val**2 + q.y_val**2))
    pitch = math.asin(max(-1.0, min(1.0, 2*(q.w_val*q.y_val - q.z_val*q.x_val))))
    yaw = math.atan2(2*(q.w_val*q.z_val + q.x_val*q.y_val), 1 - 2*(q.y_val**2 + q.z_val**2))
    
    fx = math.cos(pitch) * math.cos(yaw)
    fy = math.cos(pitch) * math.sin(yaw)
    fz = math.sin(pitch)
    return (fx, fy, fz)

class FlightDataCollector:
    """
    A class to capture and store the drone's flight data at a specified interval.
    Includes a method to plot the captured flight path.
    """
    def __init__(self, capture_interval=0.1, vehicle_name="drone_1"):
        self.capture_interval = capture_interval
        self.vehicle_name = vehicle_name
        self.last_capture_time = time.time()
        self.flight_data = []

    def capture(self, client):
        """
        Captures the current state (position, orientation, velocity) of the drone if the capture
        interval has passed.
        """
        current_time = time.time()
        if current_time - self.last_capture_time >= self.capture_interval:
            state = client.getMultirotorState(vehicle_name=self.vehicle_name)
            pos = (state.kinematics_estimated.position.x_val,
                   state.kinematics_estimated.position.y_val,
                   state.kinematics_estimated.position.z_val)
            q = state.kinematics_estimated.orientation
            ori = get_forward_vector(q)
            vel = (state.kinematics_estimated.linear_velocity.x_val,
                   state.kinematics_estimated.linear_velocity.y_val,
                   state.kinematics_estimated.linear_velocity.z_val)
            self.flight_data.append({'pos': pos, 'ori': ori, 'vel': vel})
            self.last_capture_time = current_time

    def get_flight_data(self):
        return self.flight_data

    def plot_flight_path(self, gate_positions, orientation_color='green', velocity_color='red'):
        """
        Create a 3D plot of the flight path using the captured flight data.
        
        Parameters:
          - gate_positions: Dictionary of gate names and their positions.
          - orientation_color: Color for the orientation arrows.
          - velocity_color: Color for the velocity arrows.
          
        The plot shows:
          - Drone positions as blue dots.
          - Orientation vectors (arrows) at each captured point.
          - Velocity vectors (arrows) at each captured point.
          - Gate spheres (3 m radius default) as orange wireframes.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        xs = [data['pos'][0] for data in self.flight_data]
        ys = [data['pos'][1] for data in self.flight_data]
        zs = [data['pos'][2] for data in self.flight_data]
        ax.scatter(xs, ys, zs, c='blue', marker='o', label='Flight Path')
        
        first_ori = True
        first_vel = True
        for data in self.flight_data:
            x, y, z = data['pos']
            ori = data['ori']
            vel = data['vel']
            if first_ori:
                ax.quiver(x, y, z, ori[0], ori[1], ori[2],
                          length=1, color=orientation_color, normalize=True, label='Orientation')
                first_ori = False
            else:
                ax.quiver(x, y, z, ori[0], ori[1], ori[2],
                          length=1, color=orientation_color, normalize=True)
            if first_vel:
                ax.quiver(x, y, z, vel[0], vel[1], vel[2],
                          length=1, color=velocity_color, normalize=True, label='Velocity')
                first_vel = False
            else:
                ax.quiver(x, y, z, vel[0], vel[1], vel[2],
                          length=1, color=velocity_color, normalize=True)

        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        r = 3 # Needs to be dynamic?
        for gate, pos in gate_positions.items():
            cx, cy, cz = pos.x_val, pos.y_val, pos.z_val
            xsphere = cx + r * np.outer(np.cos(u), np.sin(v))
            ysphere = cy + r * np.outer(np.sin(u), np.sin(v))
            zsphere = cz + r * np.outer(np.ones_like(u), np.cos(v))
            ax.plot_wireframe(xsphere, ysphere, zsphere, color='orange', alpha=0.3)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title("Drone Flight Path with Orientation and Velocity")
        ax.legend()
        xlims = ax.get_xlim3d()
        ylims = ax.get_ylim3d()
        zlims = ax.get_zlim3d()

        xmean = np.mean(xlims)
        ymean = np.mean(ylims)
        zmean = np.mean(zlims)
        max_range = np.max([xlims[1] - xlims[0],
                            ylims[1] - ylims[0],
                            zlims[1] - zlims[0]])

        ax.set_xlim3d([xmean - max_range / 2, xmean + max_range / 2])
        ax.set_ylim3d([ymean - max_range / 2, ymean + max_range / 2])
        ax.set_zlim3d([zmean - max_range / 2, zmean + max_range / 2])
        ax.invert_zaxis()

        plt.show()

def capture_plot_reference(flight_data_collector: FlightDataCollector):
    end_position = airsimneurips.Vector3r(25, 10, -20)
    end_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(end_position, end_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)
    time.sleep(0.2)
    flight_data_collector.capture(client)
    end_position = airsimneurips.Vector3r(-3, -2.0, -20)
    end_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(end_position, end_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)
    time.sleep(0.2)
    flight_data_collector.capture(client)

def main():
    init()
    gate_positions = getGatePositions()

    dt = 0.01
    pid_x = PIDController(kp=1.2, ki=0.1, kd=0.1, dt=dt)
    pid_y = PIDController(kp=1.2, ki=0.1, kd=0.1, dt=dt)
    pid_z = PIDController(kp=1.2, ki=0.1, kd=0.1, dt=dt)
    
    flight_data_collector = FlightDataCollector(capture_interval=0.1, vehicle_name="drone_1")

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
            
            # Capture the drone state.
            flight_data_collector.capture(client)
            
            time.sleep(dt)
    
    print("Race complete")
    
    capture_plot_reference(flight_data_collector)
    flight_data_collector.plot_flight_path(gate_positions)

if __name__ == "__main__":
    main()
