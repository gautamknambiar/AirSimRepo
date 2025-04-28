import airsimneurips
import math
import time
import numpy as np
from scipy.interpolate import splprep, splev       # ── new
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D            # ── new (3D plotting support)

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
    client.simSetVehiclePose(airsimneurips.Pose(start_position, start_rotation), ignore_collison=True)

def getGatePositions(client=None):
    if client is None:
        client = globals()['client']
    objects = client.simListSceneObjects()
    gates = [obj for obj in objects if 'Gate' in obj]
    poses = {g: client.simGetObjectPose(g).position for g in gates}
    def gate_idx(name):
        try:
            return int(name.replace("Gate","").split("_")[0])
        except:
            return float('inf')
    # sort by gate number
    return {g: poses[g] for g in sorted(poses, key=gate_idx)}

def generate_spline_waypoints(gate_positions, smoothing=0, num_points=200):
    """
    Take the sequence of gate positions and fit a 3D spline,
    returning a list of Vector3r waypoints along that spline.
    """
    pts = np.array([[p.x_val, p.y_val, p.z_val] for p in gate_positions.values()]).T
    tck, u = splprep(pts, s=smoothing)
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new, z_new = splev(u_new, tck)
    return [airsimneurips.Vector3r(x, y, z) for x, y, z in zip(x_new, y_new, z_new)]

def inSphere(position: airsimneurips.Vector3r, radius=0.5, vehicle_name="drone_1"):
    """
    Returns True if the drone is within `radius` of `position`.
    """
    p = client.getMultirotorState(vehicle_name=vehicle_name).kinematics_estimated.position
    dx, dy, dz = p.x_val - position.x_val, p.y_val - position.y_val, p.z_val - position.z_val
    return (dx*dx + dy*dy + dz*dz) <= radius**2

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp, self.ki, self.kd, self.dt = kp, ki, kd, dt
        self.integral = 0
        self.previous_error = 0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        out = self.kp*error + self.ki*self.integral + self.kd*derivative
        self.previous_error = error
        return out

class FlightDataCollector:
    def __init__(self, client, capture_interval=0.1, vehicle_name="drone_1"):
        self.client = client
        self.capture_interval = capture_interval
        self.vehicle_name = vehicle_name
        self.last_capture_time = time.time()
        self.flight_data = []

    def capture(self, control=None):
        now = time.time()
        if now - self.last_capture_time >= self.capture_interval:
            s = self.client.getMultirotorState(vehicle_name=self.vehicle_name)
            pos = (s.kinematics_estimated.position.x_val,
                   s.kinematics_estimated.position.y_val,
                   s.kinematics_estimated.position.z_val)
            ori = get_forward_vector(s.kinematics_estimated.orientation)
            vel = (s.kinematics_estimated.linear_velocity.x_val,
                   s.kinematics_estimated.linear_velocity.y_val,
                   s.kinematics_estimated.linear_velocity.z_val)
            self.flight_data.append({'time': now, 'pos': pos, 'ori': ori, 'vel': vel, 'control': control})
            self.last_capture_time = now

    def get_flight_data(self):
        return self.flight_data

    def plot_flight_path(self, gate_positions=None, waypoints=None,
                         orientation_color='green', velocity_color='blue', control_color='red'):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # flight trajectory
        xs = [d['pos'][0] for d in self.flight_data]
        ys = [d['pos'][1] for d in self.flight_data]
        zs = [d['pos'][2] for d in self.flight_data]
        ax.scatter(xs, ys, zs, c='blue', marker='o', label='Flight Path')

        # optionally scatter spline waypoints
        if waypoints:
            wx = [w.x_val for w in waypoints]
            wy = [w.y_val for w in waypoints]
            wz = [w.z_val for w in waypoints]
            ax.scatter(wx, wy, wz, c='magenta', marker='^', label='Spline Waypoints')

        # optionally draw gate spheres
        if gate_positions:
            u = np.linspace(0, 2*np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            r = 1.5
            for g, p in gate_positions.items():
                cx, cy, cz = p.x_val, p.y_val, p.z_val
                xsphere = cx + r * np.outer(np.cos(u), np.sin(v))
                ysphere = cy + r * np.outer(np.sin(u), np.sin(v))
                zsphere = cz + r * np.outer(np.ones_like(u), np.cos(v))
                ax.plot_wireframe(xsphere, ysphere, zsphere, color='orange', alpha=0.3)

        ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
        ax.set_title("Drone Flight Path, Gates, and Spline Waypoints")
        ax.legend()
        ax.invert_zaxis(); ax.invert_yaxis()
        plt.show()

def get_forward_vector(q):
    pitch = math.asin(max(-1, min(1, 2*(q.w_val*q.y_val - q.z_val*q.x_val))))
    yaw   = math.atan2(2*(q.w_val*q.z_val + q.x_val*q.y_val),
                       1 - 2*(q.y_val**2 + q.z_val**2))
    return (math.cos(pitch)*math.cos(yaw),
            math.cos(pitch)*math.sin(yaw),
            math.sin(pitch))

def finalize_flight(flight_data_collector, gate_positions, waypoints,
                    race_start_time, race_end_time):
    """
    Compute and print metrics, then plot whatever data we have.
    Safely handles missing race_end_time or empty flight data.
    """
    data = flight_data_collector.get_flight_data()
    if not data:
        print("No flight data was captured.")
        return

    # determine actual end time
    end_time = race_end_time or data[-1]['time']
    if race_start_time:
        print(f"Race time: {end_time - race_start_time:.2f} s")
    else:
        print("Race never officially started (no waypoints reached).")

    # average velocity over the race or entire dataset if no start
    speeds = [math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
              for d in data
              for v in ([d['vel']] if 'vel' in d else [])
              if (not race_start_time) or (race_start_time <= d['time'] <= end_time)]
    if speeds:
        print(f"Avg velocity: {sum(speeds)/len(speeds):.2f} m/s")
    else:
        print("No velocity data in the race interval.")

    # minimum distance to each gate
    for g, p in gate_positions.items():
        dists = [math.sqrt((d['pos'][0] - p.x_val)**2 +
                           (d['pos'][1] - p.y_val)**2 +
                           (d['pos'][2] - p.z_val)**2)
                 for d in data]
        if dists:
            print(f"Min dist to {g}: {min(dists):.2f} m")
        else:
            print(f"No position data to compute distance to {g}.")

    # finally show the plot
    time.sleep(1)
    flight_data_collector.plot_flight_path(
        gate_positions=gate_positions,
        waypoints=waypoints
    )

def main():
    init()
    gate_positions = getGatePositions()
    waypoints = generate_spline_waypoints(gate_positions, smoothing=0, num_points=200)

    # PID setup (unchanged) …
    dt = 0.01
    pid_x = PIDController(1.2, 0.1, 0.3, dt)
    pid_y = PIDController(1.2, 0.1, 0.3, dt)
    pid_z = PIDController(1.2, 0.1, 0.3, dt)
    pid_x.integral = pid_y.integral = pid_z.integral = 0
    pid_x.previous_error = pid_y.previous_error = pid_z.previous_error = 0

    flight_data_collector = FlightDataCollector(client, capture_interval=0.05)

    race_start_time = None
    race_end_time = None
    waypoint_radius = 0.5

    try:
        # --- the main follow-spline loop ---
        for i, wp in enumerate(waypoints):
            print(f"Heading to waypoint {i+1}/{len(waypoints)}")
            if i == 0:
                race_start_time = time.time()

            while not inSphere(wp, radius=waypoint_radius):
                state = client.getMultirotorState(vehicle_name="drone_1")
                pos = state.kinematics_estimated.position
                vel = state.kinematics_estimated.linear_velocity

                err = np.array([wp.x_val - pos.x_val,
                                wp.y_val - pos.y_val,
                                wp.z_val - pos.z_val])
                dist = np.linalg.norm(err)
                dir_vec = err/dist if dist>0 else np.zeros(3)
                desired_vel = 10.0 * dir_vec
                curr_vel = np.array([vel.x_val, vel.y_val, vel.z_val])

                # PID velocity control (unchanged) …
                if np.linalg.norm(curr_vel) < 0.1:
                    angle_err = 0
                else:
                    dp = np.dot(curr_vel, desired_vel)
                    angle_err = math.acos(np.clip(
                        dp/(np.linalg.norm(curr_vel)*np.linalg.norm(desired_vel)), -1,1))
                scale = math.cos(angle_err)
                adj_desired = desired_vel * scale
                vel_err = adj_desired - curr_vel

                cx = pid_x.update(vel_err[0])
                cy = pid_y.update(vel_err[1])
                cz = pid_z.update(vel_err[2])
                cmd = curr_vel + np.array([cx, cy, cz])

                yaw = math.degrees(math.atan2(dir_vec[1], dir_vec[0]))
                yaw_mode = airsimneurips.YawMode(is_rate=False, yaw_or_rate=yaw)

                client.moveByVelocityAsync(
                    cmd[0], cmd[1], cmd[2],
                    dt, yaw_mode=yaw_mode, vehicle_name="drone_1"
                )
                flight_data_collector.capture(control=cmd)
                time.sleep(dt)

            # last waypoint reached
            if i == len(waypoints) - 1:
                race_end_time = time.time()

    except KeyboardInterrupt:
        print("\n⛔ KeyboardInterrupt received — finalizing collected data…")

    # in either normal completion or interrupt, finalize
    finalize_flight(
        flight_data_collector,
        gate_positions,
        waypoints,
        race_start_time,
        race_end_time
    )

if __name__ == "__main__":
    main()