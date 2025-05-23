import airsimneurips
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev          # new: for 3D spline interpolation
from mpl_toolkits.mplot3d import Axes3D              # new: enables 3D plotting support

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
    # If no client is provided, attempt to use the global variable 'client'
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
    
    sorted_gate_positions = {
        gate: gate_positions[gate] 
        for gate in sorted(gate_positions, key=extract_gate_number)
    }
    print(sorted_gate_positions)
    return sorted_gate_positions

def generate_spline_waypoints(gate_positions, points_per_segment=10):
    """
    Generates a smooth 3D spline through the gate centers,
    returning a list of airsimneurips.Vector3r waypoints.
    """
    coords = np.array([[p.x_val, p.y_val, p.z_val] for p in gate_positions.values()]).T
    # build spline that passes exactly through each gate center
    tck, u = splprep(coords, s=0)
    num_waypoints = points_per_segment * len(gate_positions)
    u_fine = np.linspace(0, 1, num_waypoints)
    x_fine, y_fine, z_fine = splev(u_fine, tck)
    return [airsimneurips.Vector3r(x, y, z) for x, y, z in zip(x_fine, y_fine, z_fine)]

def inGateSphere(position: airsimneurips.Vector3r, radius=1.5):
    """
    Given a radius and position vector, calculate if the drone is within that sphere.
    """
    dronePose = client.getMultirotorState(vehicle_name="drone_1").kinematics_estimated.position
    dx = dronePose.x_val - position.x_val
    dy = dronePose.y_val - position.y_val
    dz = dronePose.z_val - position.z_val
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    if distance <= radius:
        print(f"Reached sphere at position {position}, moving to next target")
        return True
    else:
        return False

# 1d PID
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
        Updates PID values and returns an output value for 1d.
        """
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class FlightDataCollector:
    """
    A class to capture and store the drone's flight data at a specified interval.
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
            pos = (
                state.kinematics_estimated.position.x_val,
                state.kinematics_estimated.position.y_val,
                state.kinematics_estimated.position.z_val
            )
            q = state.kinematics_estimated.orientation
            ori = get_forward_vector(q)
            vel = (
                state.kinematics_estimated.linear_velocity.x_val,
                state.kinematics_estimated.linear_velocity.y_val,
                state.kinematics_estimated.linear_velocity.z_val
            )
            self.flight_data.append({
                'time': current_time,
                'pos': pos,
                'ori': ori,
                'vel': vel,
                'control': control
            })
            self.last_capture_time = current_time

    def get_flight_data(self):
        return self.flight_data

    def plot_flight_path(self, gate_positions=None, waypoint_positions=None,
                         orientation_color='green', velocity_color='blue', control_color='red'):
        '''
        Plots flight path, orientation, velocity vectors, control vectors,
        gate spheres, and spline waypoints.
        '''
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        # plot actual flight path
        xs = [d['pos'][0] for d in self.flight_data]
        ys = [d['pos'][1] for d in self.flight_data]
        zs = [d['pos'][2] for d in self.flight_data]
        ax.scatter(xs, ys, zs, c='blue', marker='o', label='Flight Path')
        
        # orientation, velocity, control arrows
        first_ori = first_vel = first_ctrl = True
        for data in self.flight_data:
            x, y, z = data['pos']
            vel = data['vel']
            ori = data['ori']
            ctrl = data['control']
            if first_vel:
                ax.quiver(x, y, z, vel[0], vel[1], vel[2], length=1, normalize=True,
                          color=velocity_color, label='Velocity')
                first_vel = False
            else:
                ax.quiver(x, y, z, vel[0], vel[1], vel[2], length=1, normalize=True, color=velocity_color)
            if first_ori:
                ax.quiver(x, y, z, ori[0], ori[1], ori[2], length=1, normalize=True,
                          color=orientation_color, label='Orientation')
                first_ori = False
            else:
                ax.quiver(x, y, z, ori[0], ori[1], ori[2], length=1, normalize=True, color=orientation_color)
            if ctrl is not None:
                if first_ctrl:
                    ax.quiver(x, y, z, ctrl[0], ctrl[1], ctrl[2], length=1, normalize=True,
                              color=control_color, label='Control')
                    first_ctrl = False
                else:
                    ax.quiver(x, y, z, ctrl[0], ctrl[1], ctrl[2], length=1, normalize=True, color=control_color)
        
        # plot gate spheres
        u = np.linspace(0, 2*np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        r_gate = 1.5
        if gate_positions:
            for gate, pos in gate_positions.items():
                cx, cy, cz = pos.x_val, pos.y_val, pos.z_val
                xsph = cx + r_gate*np.outer(np.cos(u), np.sin(v))
                ysph = cy + r_gate*np.outer(np.sin(u), np.sin(v))
                zsph = cz + r_gate*np.outer(np.ones_like(u), np.cos(v))
                ax.plot_wireframe(xsph, ysph, zsph, color='orange', alpha=0.3)
        
        # plot spline waypoints
        if waypoint_positions:
            wx = [p.x_val for p in waypoint_positions]
            wy = [p.y_val for p in waypoint_positions]
            wz = [p.z_val for p in waypoint_positions]
            ax.scatter(wx, wy, wz, c='purple', marker='x', label='Spline Waypoints')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title("Drone Flight Path with Gates & Spline Waypoints")
        ax.legend()
        # adjust axes
        xlims = ax.get_xlim3d(); ylims = ax.get_ylim3d(); zlims = ax.get_zlim3d()
        xmid = np.mean(xlims); ymid = np.mean(ylims); zmid = np.mean(zlims)
        max_range = max(xlims[1]-xlims[0], ylims[1]-ylims[0], zlims[1]-zlims[0])
        ax.set_xlim3d([xmid-max_range/2, xmid+max_range/2])
        ax.set_ylim3d([ymid-max_range/2, ymid+max_range/2])
        ax.set_zlim3d([zmid-max_range/2, zmid+max_range/2])
        ax.invert_zaxis()
        ax.invert_yaxis()
        plt.show()

def get_forward_vector(q):
    """
    Compute the drone's forward unit vector from its orientation quaternion.
    Assumes the drone's local forward direction is along the +X axis.
    """
    pitch = math.asin(max(-1.0, min(1.0, 2*(q.w_val*q.y_val - q.z_val*q.x_val))))
    yaw = math.atan2(2*(q.w_val*q.z_val + q.x_val*q.y_val),
                     1 - 2*(q.y_val**2 + q.z_val**2))
    fx = math.cos(pitch)*math.cos(yaw)
    fy = math.cos(pitch)*math.sin(yaw)
    fz = math.sin(pitch)
    return (fx, fy, fz)

def capture_plot_reference(flight_data_collector: FlightDataCollector):
    # These calls to simSetVehiclePose are only for capturing reference states for plotting.
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

def finalize(flight_data_collector, gate_positions, spline_waypoints, race_start_time, race_end_time, dt):
    """
    Run all post-race data capture, metrics, and plotting.
    Safe to call even if race_end_time is None (e.g. on interrupt).
    """
    # extra capture after loop (matches your original 50-step capture)
    for _ in range(50):
        flight_data_collector.capture()
        time.sleep(dt)

    # --- Compute Performance Metrics ---
    entries = flight_data_collector.get_flight_data()

    # Race time
    if race_start_time is not None and race_end_time is not None:
        total_time = race_end_time - race_start_time
        print(f"Race completion time (first gate to last gate): {total_time:.2f} seconds")
    else:
        print("Race timing not recorded properly.")

    # Average velocity between first & last gate
    speeds = []
    if race_start_time is not None and race_end_time is not None:
        for e in entries:
            t = e['time']
            if race_start_time <= t <= race_end_time:
                v = e['vel']
                speeds.append(np.linalg.norm(v))
    if speeds:
        print(f"Average velocity between first and last gate: {sum(speeds)/len(speeds):.2f} m/s")
    else:
        print("No velocity data recorded between first and last gate.")

    # Average minimum distance to each gate
    gate_distances = []
    for gate, pos in gate_positions.items():
        min_d = float('inf')
        for e in entries:
            fp = e['pos']
            dx, dy, dz = fp[0]-pos.x_val, fp[1]-pos.y_val, fp[2]-pos.z_val
            d = math.sqrt(dx*dx + dy*dy + dz*dz)
            if d < min_d:
                min_d = d
        gate_distances.append(min_d)
        print(f"Minimum distance to {gate}: {min_d:.2f} meters")
    if gate_distances:
        print(f"Average minimum distance to each gate: {sum(gate_distances)/len(gate_distances):.2f} meters")
    else:
        print("No gate distance data recorded.")

    # small pause, then plotting
    time.sleep(1)
    capture_plot_reference(flight_data_collector)
    flight_data_collector.plot_flight_path(
        gate_positions=gate_positions,
        waypoint_positions=spline_waypoints
    )

def main():
    init()
    gate_positions = getGatePositions()
    spline_waypoints = generate_spline_waypoints(gate_positions, points_per_segment=10)

    dt = 0.01
    pid_x = PIDController(kp=1.2, ki=0.1, kd=0.8, dt=dt)
    pid_y = PIDController(kp=1.2, ki=0.1, kd=0.8, dt=dt)
    pid_z = PIDController(kp=1.2, ki=0.1, kd=0.8, dt=dt)

    flight_data_collector = FlightDataCollector(
        client=client, capture_interval=0.05, vehicle_name="drone_1"
    )

    race_start_time = None
    race_end_time = None
    total_waypoints = len(spline_waypoints)

    try:
        for idx, target in enumerate(spline_waypoints):
            print(f"Going to waypoint {idx+1}/{total_waypoints} at position {target}")
            pid_x.integral = pid_y.integral = pid_z.integral = 0
            pid_x.previous_error = pid_y.previous_error = pid_z.previous_error = 0

            while not inGateSphere(target, radius=1):
                state = client.getMultirotorState(vehicle_name="drone_1")
                pos = state.kinematics_estimated.position
                vel = state.kinematics_estimated.linear_velocity

                err_vec = np.array([
                    target.x_val - pos.x_val,
                    target.y_val - pos.y_val,
                    target.z_val - pos.z_val
                ])
                dist = np.linalg.norm(err_vec)
                direction = (err_vec / dist) if dist > 0 else np.zeros(3)

                base_speed = 10.0
                desired_vel = base_speed * direction
                curr_vel = np.array([vel.x_val, vel.y_val, vel.z_val])
                vel_err = desired_vel - curr_vel

                cx = pid_x.update(vel_err[0])
                cy = pid_y.update(vel_err[1])
                cz = pid_z.update(vel_err[2])
                command_vel = curr_vel + np.array([cx, cy, cz])

                yaw_deg = math.degrees(math.atan2(direction[1], direction[0]))
                yaw_mode = airsimneurips.YawMode(is_rate=False, yaw_or_rate=yaw_deg)

                client.moveByVelocityAsync(
                    command_vel[0], command_vel[1], command_vel[2],
                    dt, yaw_mode=yaw_mode, vehicle_name="drone_1"
                )

                flight_data_collector.capture(control=command_vel)
                time.sleep(dt)

            if idx == 0:
                race_start_time = time.time()
            if idx == total_waypoints - 1:
                race_end_time = time.time()

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Finalizing with data collected so far...")
        if race_start_time and not race_end_time:
            race_end_time = time.time()
        finalize(
            flight_data_collector,
            gate_positions,
            spline_waypoints,
            race_start_time,
            race_end_time,
            dt
        )
        return

    # normal completion
    print("Race complete")
    finalize(
        flight_data_collector,
        gate_positions,
        spline_waypoints,
        race_start_time,
        race_end_time,
        dt
    )

if __name__ == "__main__":
    main()