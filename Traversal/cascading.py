import airsimneurips
import math
import time
import numpy as np
import matplotlib.pyplot as plt

client = airsimneurips.MultirotorClient()

def init():
    """
    Connect using API.
    """
    client.confirmConnection()
    print('Connection confirmed')
    client.simLoadLevel('Soccer_Field_Easy')
    client.enableApiControl(vehicle_name="drone_1")
    client.arm(vehicle_name="drone_1")
    time.sleep(1)
    start_position = airsimneurips.Vector3r(-1, -2.0, 1.8)
    start_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(start_position, start_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)

def getGatePositions(client=None):
    if client is None:
        try:
            client = globals()['client']
        except KeyError:
            raise ValueError("No client provided and no global client available.")
    
    objects = client.simListSceneObjects()
    print(f"{objects}\n\n")
    gates = [obj for obj in objects if 'Gate' in obj]
    print(f"{gates}\n\n")
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

def inGateSphere(position: airsimneurips.Vector3r, radius=1.5):
    """
    Check whether the drone is inside the sphere around the target gate.
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

# -------------------------------
# Original 1D PID controller:
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

# -------------------------------
# Cascaded PID: Outer loop for position and inner loop for velocity.
class CascadedPIDController:
    def __init__(self, outer_gains, inner_gains, dt):
        # Gains: (kp, ki, kd)
        self.outer = PIDController(outer_gains[0], outer_gains[1], outer_gains[2], dt)
        self.inner = PIDController(inner_gains[0], inner_gains[1], inner_gains[2], dt)
        self.dt = dt

    def update(self, setpoint, current, current_rate):
        """
        Outer loop:
          - setpoint: desired position for one axis.
          - current: current position of that axis.
          - current_rate: current velocity along that axis.
        
        The outer PID produces a desired velocity.
        The inner PID then computes the error between this desired velocity and the measured
        velocity, yielding a desired acceleration command.
        """
        position_error = setpoint - current
        desired_velocity = self.outer.update(position_error)
        velocity_error = desired_velocity - current_rate
        acceleration_command = self.inner.update(velocity_error)
        return acceleration_command, desired_velocity

# -------------------------------
# Flight data collection remains the same:
class FlightDataCollector:
    """
    A class to capture and store the drone's flight data.
    """
    def __init__(self, client: airsimneurips.MultirotorClient, capture_interval=0.1, vehicle_name="drone_1"):
        self.capture_interval = capture_interval
        self.client = client
        self.vehicle_name = vehicle_name
        self.last_capture_time = time.time()
        self.flight_data = []

    def capture(self, control=None):
        current_time = time.time()
        if current_time - self.last_capture_time >= self.capture_interval:
            state = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
            pos = (state.kinematics_estimated.position.x_val,
                   state.kinematics_estimated.position.y_val,
                   state.kinematics_estimated.position.z_val)
            # Compute the drone's forward vector from its orientation.
            q = state.kinematics_estimated.orientation
            ori = get_forward_vector(q)
            vel = (state.kinematics_estimated.linear_velocity.x_val,
                   state.kinematics_estimated.linear_velocity.y_val,
                   state.kinematics_estimated.linear_velocity.z_val)
            self.flight_data.append({'time': current_time, 'pos': pos, 'ori': ori, 'vel': vel, 'control': control})
            self.last_capture_time = current_time

    def get_flight_data(self):
        return self.flight_data

    def plot_flight_path(self, gate_positions=None, orientation_color='green', velocity_color='blue', control_color='red'):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        xs = [data['pos'][0] for data in self.flight_data]
        ys = [data['pos'][1] for data in self.flight_data]
        zs = [data['pos'][2] for data in self.flight_data]
        ax.scatter(xs, ys, zs, c='blue', marker='o', label='Flight Path')
        first_ori = True
        first_vel = True
        first_control = True
        for data in self.flight_data:
            x, y, z = data['pos']
            vel = data['vel']
            ori = data['ori']
            control = data['control']
            speed = np.linalg.norm(vel)

            if first_vel:
                ax.quiver(x, y, z, vel[0], vel[1], vel[2], length=1, color=velocity_color, normalize=True, label='Velocity')
                first_vel = False
            else:
                ax.quiver(x, y, z, vel[0], vel[1], vel[2], length=1, color=velocity_color, normalize=True)

            if first_ori:
                ax.quiver(x, y, z, ori[0], ori[1], ori[2], length=1, color=orientation_color, normalize=True, label='Orientation')
                first_ori = False
            else:
                ax.quiver(x, y, z, ori[0], ori[1], ori[2], length=1, color=orientation_color, normalize=True)
            
            if control is not None:
                if first_control:
                    ax.quiver(x, y, z, control[0], control[1], control[2], length=1, color=control_color, normalize=True, label='Control')
                    first_control = False
                else:
                    ax.quiver(x, y, z, control[0], control[1], control[2], length=1, color=control_color, normalize=True)

            if speed > 0:
                ax.text(x, y, z, f"{speed:.2f} m/s", color=velocity_color, fontsize=8)

        # Draw gate spheres
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        r = 1.5
        if gate_positions:
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
        ax.invert_yaxis()

        plt.show()

def get_forward_vector(q):
    """
    Compute the drone's forward unit vector from its orientation quaternion.
    Assumes the drone's local forward direction is along the +X axis.
    """
    pitch = math.asin(max(-1.0, min(1.0, 2*(q.w_val*q.y_val - q.z_val*q.x_val))))
    yaw = math.atan2(2*(q.w_val*q.z_val + q.x_val*q.y_val), 1 - 2*(q.y_val**2 + q.z_val**2))
    fx = math.cos(pitch) * math.cos(yaw)
    fy = math.cos(pitch) * math.sin(yaw)
    fz = math.sin(pitch)
    return (fx, fy, fz)

def capture_plot_reference(flight_data_collector: FlightDataCollector):
    end_position = airsimneurips.Vector3r(25, 10, -20)
    end_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(end_position, end_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)
    time.sleep(0.2)
    flight_data_collector.capture()
    end_position = airsimneurips.Vector3r(-3, -2.0, -20)
    end_rotation = airsimneurips.Quaternionr(0, 0, 0, 4.71)
    new_pose = airsimneurips.Pose(end_position, end_rotation)
    client.simSetVehiclePose(new_pose, ignore_collison=True)
    time.sleep(0.2)
    flight_data_collector.capture()

def main():
    init()
    gate_positions = getGatePositions()

    dt = 0.01  # Control loop time step
    g = 9.81  # gravitational acceleration in m/s^2

    # --- Cascaded PID gains for each axis.
    # Outer loop gains (position): these values determine the desired velocity response from the positional error.
    kp_outer = 0.5
    ki_outer = 0.0
    kd_outer = 0.0
    # Inner loop gains (velocity): these values determine the acceleration needed to follow the velocity setpoint.
    kp_inner = 1.2
    ki_inner = 0.1
    kd_inner = 0.1

    cascaded_x = CascadedPIDController((kp_outer, ki_outer, kd_outer), (kp_inner, ki_inner, kd_inner), dt)
    cascaded_y = CascadedPIDController((kp_outer, ki_outer, kd_outer), (kp_inner, ki_inner, kd_inner), dt)
    cascaded_z = CascadedPIDController((kp_outer, ki_outer, kd_outer), (kp_inner, ki_inner, kd_inner), dt)

    flight_data_collector = FlightDataCollector(client=client, capture_interval=0.05, vehicle_name="drone_1")
    
    # Metrics for race timing
    race_start_time = None
    race_end_time = None

    gates_list = list(gate_positions.items())
    total_gates = len(gates_list)

    # Hover throttle (assumed value at which the drone maintains altitude in hover)
    hover_throttle = 0.5
    throttle_scale = 10.0  # Scale factor to convert acceleration (m/s^2) to throttle adjustment
    
    for idx, (gate, target) in enumerate(gates_list):
        print(f"Going to {gate} at position {target}")
        # Reset cascaded controller internal states
        cascaded_x.outer.integral = cascaded_y.outer.integral = cascaded_z.outer.integral = 0
        cascaded_x.outer.previous_error = cascaded_y.outer.previous_error = cascaded_z.outer.previous_error = 0
        cascaded_x.inner.integral = cascaded_y.inner.integral = cascaded_z.inner.integral = 0
        cascaded_x.inner.previous_error = cascaded_y.inner.previous_error = cascaded_z.inner.previous_error = 0
        
        while not inGateSphere(target):
            state = client.getMultirotorState(vehicle_name="drone_1")
            current_pos = state.kinematics_estimated.position
            current_vel = state.kinematics_estimated.linear_velocity

            # For each axis, compute the desired acceleration command and the intermediate desired velocity.
            ax, vx_des = cascaded_x.update(target.x_val, current_pos.x_val, current_vel.x_val)
            ay, vy_des = cascaded_y.update(target.y_val, current_pos.y_val, current_vel.y_val)
            az, vz_des = cascaded_z.update(target.z_val, current_pos.z_val, current_vel.z_val)
            
            # Map horizontal acceleration commands into attitude (roll/pitch)
            # Note: a positive desired acceleration along x (forward) is achieved by pitching nose down (i.e. negative pitch)
            pitch_command = -math.atan2(ax, g)
            # For the y axis, positive acceleration is achieved by a positive roll command.
            roll_command = math.atan2(ay, g)
            # Altitude control: adjust throttle around hover_throttle based on desired vertical acceleration.
            throttle_adjustment = az / throttle_scale
            throttle_command = np.clip(hover_throttle + throttle_adjustment, 0.0, 1.0)

            # Desired yaw is computed from the horizontal position error (so the drone “points” toward the gate).
            error_x = target.x_val - current_pos.x_val
            error_y = target.y_val - current_pos.y_val
            desired_yaw = math.atan2(error_y, error_x)

            # Send command using the low-level control API.
            client.moveByRollPitchYawThrottleAsync(roll_command, pitch_command, desired_yaw, throttle_command, dt, vehicle_name="drone_1")
            
            # Capture flight data (include the control vector as [roll, pitch, yaw, throttle] for plotting)
            flight_data_collector.capture(control=[roll_command, pitch_command, desired_yaw, throttle_command])
            time.sleep(dt)

        # Record race start time at the first gate.
        if idx == 0:
            race_start_time = time.time()
        # Record race end time after the last gate.
        if idx == total_gates - 1:
            race_end_time = time.time()

    print("Race complete")
    # Capture additional data after finishing.
    for i in range(50):
        flight_data_collector.capture()
        time.sleep(dt)
    
    # --- Compute Performance Metrics ---
    if race_start_time is not None and race_end_time is not None:
        race_time = race_end_time - race_start_time
        print(f"Race completion time (first gate to last gate): {race_time:.2f} seconds")
    else:
        print("Race timing not recorded properly.")
    
    # Average velocity (using captured flight data between race start and end)
    flight_entries = flight_data_collector.get_flight_data()
    speeds = []
    for entry in flight_entries:
        t = entry['time']
        if race_start_time <= t <= race_end_time:
            vel = entry['vel']
            speed = np.linalg.norm(vel)
            speeds.append(speed)
    if speeds:
        avg_velocity = sum(speeds) / len(speeds)
        print(f"Average velocity between first and last gate: {avg_velocity:.2f} m/s")
    else:
        print("No velocity data recorded between first and last gate.")
    
    # Compute minimum distance to each gate (post race)
    gate_min_distances = []
    for gate, pos in gate_positions.items():
        min_distance = float('inf')
        for data in flight_entries:
            flight_pos = data['pos']
            dx = flight_pos[0] - pos.x_val
            dy = flight_pos[1] - pos.y_val
            dz = flight_pos[2] - pos.z_val
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            if distance < min_distance:
                min_distance = distance
        gate_min_distances.append(min_distance)
        print(f"Minimum distance to {gate}: {min_distance:.2f} meters")
    
    if gate_min_distances:
        avg_min_distance = sum(gate_min_distances) / len(gate_min_distances)
        print(f"Average minimum distance to each gate: {avg_min_distance:.2f} meters")
    else:
        print("No gate distance data recorded.")
    
    time.sleep(1)
    capture_plot_reference(flight_data_collector)
    flight_data_collector.plot_flight_path(gate_positions=gate_positions)

if __name__ == "__main__":
    main()
