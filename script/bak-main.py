import asyncio
import time
import numpy as np
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError

import matplotlib.pyplot as plt

DEBUG = True
SIM = True
POLE_LATLON = False

# Global dictionary to store the latest YOLO target data
target_data = {
    'active': False,
    'dx': 0.0,  # -1.0 (left) to 1.0 (right)
    'dy': 0.0,  # -1.0 (top) to 1.0 (bottom)
    'last_seen': 0.0,
    'cooldown_until': 0.0 # Prevents re-triggering immediately after returning to mission
}

pole1 = (-35.36381716, 149.16499336)  #right side
pole2 = (-35.36311189, 149.16291509 ) #left side

pole1_rel = (-80, 20)
pole2_rel = (-230, 20)

clearance = 30
precision = 15
# precision = 50

# Mission Parameters
speed = 25.0
acceptance_radius = 10.0
alt_mission = 10.0

area_width, area_length = 100.0, 30.0   # meters (width across lanes, length along flight)
alt = 10.0
fov = 70.0
overlap = 0.30

lap = [
    #[pi, clearance, rotation_direction]
    [2 * np.pi, clearance, 1],
    # [2 * np.pi, clearance - 10, 1],
    [np.pi, clearance - 25, -1]
    ]

mission_num = 1
home_lat, home_lon = 0.0, 0.0

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculates the distance between two GPS coordinates in meters."""
    R = 6371000  # Radius of Earth in meters
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    delta_phi = np.radians(lat2 - lat1)
    delta_lambda = np.radians(lon2 - lon1)
    a = np.sin(delta_phi / 2.0)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda / 2.0)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return R * c

# --- UDP LISTENER SERVER ---
async def start_udp_listener(host="127.0.0.1", port=9000):
    """Listens for stream of YOLO target coordinates."""
    class TargetProtocol(asyncio.DatagramProtocol):
        def connection_made(self, transport):
            print(f"[*] UDP Listener active on {host}:{port} - Waiting for YOLO stream...")
            
        def datagram_received(self, data, addr):
            message = data.decode().strip()
            # Expecting format: "TARGET: 0.15, -0.05"
            if message.startswith("TARGET:") and time.time() > target_data['cooldown_until']:
                try:
                    coords = message.replace("TARGET:", "").strip()
                    dx, dy = map(float, coords.split(","))
                    target_data['dx'] = dx
                    target_data['dy'] = dy
                    target_data['last_seen'] = time.time()
                    target_data['active'] = True
                except ValueError:
                    pass

    loop = asyncio.get_running_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: TargetProtocol(),
        local_addr=(host, port)
    )
    return transport

def plot_waypoints(waypoints):
    xs = [p[0] for p in waypoints]
    ys = [p[1] for p in waypoints]

    plt.figure()
    plt.scatter(xs, ys)

    # Label each waypoint with its index
    for i, (x, y) in enumerate(waypoints):
        plt.text(x, y, str(i), fontsize=9)

    plt.title("Figure-8 Waypoints Sequence")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.axis('equal')
    plt.grid()
    plt.show()  

def generate_capsule_waypoints(A, B, r, tau):
    """
    tau ∈ [0,1] = global progress around capsule
    returns a point on the capsule boundary
    """

    A = np.array(A, dtype=float)
    B = np.array(B, dtype=float)

    v = B - A
    L = np.linalg.norm(v)

    u = v / L
    n = np.array([-u[1], u[0]])

    P = 2*L + 2*np.pi*r   # total perimeter
    t = tau * P           # convert normalized → arc length
    # print(f"tau = {tau},t = {t}")


    # ----- Region 1 : Top line -----
    
    if t < L:
        s = t / L
        p =  A + r*n + s*v
        segment = 0

    # ----- Region 2 : Semicircle at B -----
    elif t < L + np.pi*r:
        t2 = t - L
        theta = t2/r - np.pi/2
        p = B + r*(np.cos(theta)*u - np.sin(theta)*n)
        segment = 1

    # ----- Region 3 : Bottom line -----
    elif t < 2*L + np.pi*r:
        t3 = t - (L + np.pi*r)
        s = t3 / L
        p =  B - r*n - s*v
        segment = 2

    # ----- Region 4 : Semicircle at A -----
    else:
        t4 = t - (2*L + np.pi*r)
        theta = t4/r - np.pi/2
        p=  A + r*(-np.cos(theta)*u + np.sin(theta)*n)
        segment = 3
    return np.array([p[0], p[1], segment])

def perpendicular_offset(point_a, point_b, origin_point,side, clearance):
    """
    Compute point = point_b offset by `clearance` meters perpendicular to line (point_a->point_b).
    `origin_point` is used to choose which side of the line we want:
      - if the normal points toward origin_point, keep it; otherwise flip it.
    Returns numpy array [x, y].
    """
    a = np.asarray(point_a, dtype=float)
    b = np.asarray(point_b, dtype=float)
    origin = np.asarray(origin_point, dtype=float)

    # direction from a -> b
    v = b - a
    if np.allclose(v, 0):
        raise ValueError("pole points are identical")

    # 90° rotation (one perpendicular)
    n = np.array([-v[1], v[0]], dtype=float)

    # normalize to unit length
    n /= np.linalg.norm(n)

    return origin + clearance * side * n


def generate_figure8_around_poles(pole1_xy, pole2_xy, clearance_meters, precision):

    x1, y1 = pole1_xy
    x2, y2 = pole2_xy
    
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    
    dx = x1 - x2
    dy = y1 - y2
    distance = np.sqrt(dx**2 + dy**2)
    
    angle = np.arctan2(dy, dx)
    

    waypoints_xy = []
    for i in range(len(lap)):
        for t in range(precision):
            A = distance / 2.0 + lap[i][1]
            B = A

            t = lap[i][0] * (precision - t) / precision
            local_x = A * np.sin(t) * lap[i][2]
            local_y = B * np.sin(t) * np.cos(t)
            
            rot_x = local_x * np.cos(angle) - local_y * np.sin(angle)
            rot_y = local_x * np.sin(angle) + local_y * np.cos(angle)
            
            final_x = center_x + rot_x
            final_y = center_y + rot_y
            
            waypoints_xy.append((final_x, final_y))

    waypoints_xy.append((center_x, center_y))

    perp_center = perpendicular_offset(pole1_xy, pole2_xy, (center_x, center_y), -1, clearance_meters)
    waypoints_xy.append(perp_center)

    perp_home = perpendicular_offset(pole1_xy, pole2_xy, (home_lat, home_lon), -1, clearance_meters)
    waypoints_xy.append(perp_home)

    
    return waypoints_xy

def calc_clearance_point(pos_a, pos_b, clearance):
    ba = pos_b - pos_a
    pole_ar = np.array([pole1, pole2])


def transform_point(lx, ly, angle, cx, cy):
    """Helper to rotate and translate points"""
    rot_x = lx * np.cos(angle) - ly * np.sin(angle)
    rot_y = lx * np.sin(angle) + ly * np.cos(angle)
    return cx + rot_x, cy + rot_y

def xy_to_latlon(waypoints_xy, base_lat, base_lon):
    """
    Converts Gazebo Local X,Y meters back to Latitude/Longitude for GPS navigation.
    """
    lat_lon_waypoints = []
    meters_to_lat_deg = 1 / 111111.0
    meters_to_lon_deg = 1 / (111111.0 * np.cos(np.radians(base_lat)))
    
    for (x, y) in waypoints_xy:
        # ENU, X is Longitude (East/West), Y is Latitude (North/South)
        lat = base_lat + (y * meters_to_lat_deg)
        lon = base_lon + (x * meters_to_lon_deg)
        lat_lon_waypoints.append((lat, lon))
        
    return lat_lon_waypoints

def latlon_to_xy(lat, lon, home_lat, home_lon):
    """Convert a GPS coordinate to local XY meters relative to home."""
    meters_per_lat = 111111.0
    meters_per_lon = 111111.0 * np.cos(np.radians(home_lat))
    x = (lon - home_lon) * meters_per_lon
    y = (lat - home_lat) * meters_per_lat
    return (x, y)


def create_mission_items(waypoints, flight_alt_agl):
    mission_items = []

    for lat, lon in waypoints:
        item = MissionItem(
            lat, lon, flight_alt_agl, speed, True, 
            float('nan'), float('nan'), MissionItem.CameraAction.NONE,
            float('nan'), float('nan'), acceptance_radius, float('nan'), 
            float('nan'), MissionItem.VehicleAction.NONE
        )
        mission_items.append(item)

    return mission_items

def generate_scan(area_length, area_width, altitude, fov_deg, overlap,
                  N_line=10, N_turn=4):
    
    # --- camera footprint ---
    swath = 2 * altitude * np.tan(np.radians(fov_deg/2))
    lane_spacing = swath * (1 - overlap)

    # --- number of lanes ---
    n_lanes = int(np.ceil(area_width / lane_spacing))

    # --- center lanes ---
    total_span = (n_lanes - 1) * lane_spacing
    x0 = -total_span / 2
    lane_x = x0 + np.arange(n_lanes) * lane_spacing

    # --- turn radius ---
    r = lane_spacing / 2

    half_len = area_length / 2

    waypoints = []

    for i in range(n_lanes):

        x = lane_x[i]

        # alternate direction
        going_up = (i % 2 == 0)

        if going_up:
            y_start, y_end = -half_len, half_len
        else:
            y_start, y_end = half_len, -half_len

        # --- straight lane ---
        xs = np.linspace(y_start, y_end, N_line)
        ys = np.full_like(xs, x)
        waypoints.extend(zip(xs, ys))

        # --- turn ---
        if i < n_lanes - 1:

            x_next = lane_x[i+1]
            y_turn = y_end
            cx = y_turn
            cy = (x + x_next) / 2   

            if going_up:
                theta = np.linspace(np.pi, 0, N_turn)
            else:
                theta = np.linspace(0, np.pi, N_turn)
            arc_x = cx + r*np.sin(theta)
            if going_up:
                arc_y = cy + r*np.cos(theta)
            else:
                arc_y = cy - r*np.cos(theta)

            waypoints.extend(zip(arc_x, arc_y))

    return np.array(waypoints)


# INTERCEPTION - UNUSED
async def interception_task(drone):
    """Background task that waits for YOLO targets, centers the drone, and descends."""
    
    KP = 1.0          # Proportional Gain (Max speed in m/s when object is at the edge of the screen)
    THRESHOLD = 0.1   # How close to 0,0 is considered "centered" (10% from center)
    CENTER_TIME = 2.0 # How many seconds it must remain centered before descending
    
    while True:

        center_success = True
        await asyncio.sleep(0.1)
        
        # Wait until a target is detected
        if not target_data['active']:
            continue
            
        print("\n[INTERCEPT] Target Detected! Pausing mission...")
        await drone.mission.pause_mission()
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
  
        mission_lat, mission_lon, mission_abs_alt = 0, 0, 0
        yaw_deg = 0
        async for pos in drone.telemetry.position():
            mission_lat = pos.latitude_deg
            mission_lon = pos.longitude_deg
            mission_abs_alt = pos.absolute_altitude_m
            break
        async for heading in drone.telemetry.heading():
            yaw_deg = heading.heading_deg
            break
            
        print(f"[INTERCEPT] Saved Mission Location: LAT {mission_lat:.6f}, LON {mission_lon:.6f}")
        print("[INTERCEPT] Switching to Offboard Velocity Control...")

        try:
            await drone.offboard.start()
        except OffboardError as e:
            print(f"Starting offboard mode failed: {e}")
            target_data['active'] = False
            continue


        is_active = await drone.offboard.is_active()
        if not is_active:
            print("[INTERCEPT] Offboard mode failed to become active!")
            target_data['active'] = False
            continue # Go back to waiting for targets
            

# --- PRE-LOOP SETUP ---
        print("[INTERCEPT] Offboard mode started. Beginning visual servoing...")
        centered_duration = 0.0
        loop_rate = 0.1  # 10 Hz
        
        integral_x, integral_y = 0.0, 0.0
        prev_dx, prev_dy = 0.0, 0.0
        phase = "CENTERING" # Drone starts by centering. Changes to "DESCENDING" later.
        center_success = True
        KP, KI, KD = 1.0, 0.1, 0.5  # PID Tuning
        # 3. MAIN CONTROL LOOP
# --- PRE-LOOP SETUP ---
        print("[INTERCEPT] Offboard mode started. Beginning visual servoing...")
        centered_duration = 0.0
        loop_rate = 0.1  # 10 Hz
        
        integral_x, integral_y = 0.0, 0.0
        prev_dx, prev_dy = 0.0, 0.0
        KP, KI, KD = 1.0, 0.1, 0.5  # PID Tuning for XY
        
        TARGET_ALTITUDE = 2.0 # Meters AGL

        phase = "CENTERING" 
        center_success = True
        brake_counter = 0

        # 3. MAIN CONTROL LOOP
        try:
            while True:
                now = time.time()
                time_since_last = now - target_data['last_seen']

                # ==========================================
                # STEP 1: SAFETY & TIMEOUT CHECKS
                # ==========================================
                if time_since_last > 10.0:
                    print("[ERROR] Target lost for >10s! Aborting interception...")
                    center_success = False
                    break 

                if time_since_last > 1.0:
                    print(f"[WARNING] Target lost ({time_since_last:.1f}s ago)! Hovering...")
                    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    centered_duration = 0.0
                    await asyncio.sleep(loop_rate)
                    continue 

                # ==========================================
                # STEP 2: CALCULATE PID (XY Movement)
                # ==========================================
                dt = loop_rate
                dx, dy = target_data['dx'], target_data['dy']
                
                p_x, p_y = dy, dx 

                integral_x += p_x * dt
                integral_y += p_y * dt
                d_x = (p_x - prev_dx) / dt
                d_y = (p_y - prev_dy) / dt
                prev_dx, prev_dy = p_x, p_y

                v_x = -(KP * p_x + KI * integral_x + KD * d_x)
                v_y =  (KP * p_y + KI * integral_y + KD * d_y)
                v_z = 0.0 

                # ==========================================
                # STEP 3: EXECUTE CURRENT PHASE
                # ==========================================
                if phase == "CENTERING":
                    if abs(dx) < THRESHOLD and abs(dy) < THRESHOLD:
                        centered_duration += loop_rate
                        print(f"[PHASE 1] Centered... ({centered_duration:.1f}s / {CENTER_TIME}s)")
                    else:
                        centered_duration = 0.0
                        print(f"[PHASE 1] Adjusting: Vx={v_x:.2f} m/s, Vy={v_y:.2f} m/s")

                    if centered_duration >= CENTER_TIME:
                        print("\n[PHASE CHANGE] Target Locked! Starting smooth descent...\n")
                        phase = "DESCENDING"

                elif phase == "DESCENDING":
                    # Get current altitude quickly
                    current_alt = 0.0
                    async for pos in drone.telemetry.position():
                        current_alt = pos.relative_altitude_m
                        break 
                        
                    # Proportional Z-Descent (Slows down as it gets closer to 2.0m)
                    alt_error = current_alt - TARGET_ALTITUDE
                    v_z = alt_error * 0.5  # P-Controller for Altitude
                    
                    # Clamp descent speed between 0.2 m/s (minimum) and 1.0 m/s (maximum)
                    v_z = max(0.2, min(1.0, v_z))

                    print(f"[PHASE 2] Alt: {current_alt:.2f}m | Error: {alt_error:.2f}m | Vz: {v_z:.2f} m/s")

                    # If we are within 20cm of the target altitude, start braking
                    if alt_error <= 0.2:
                        print("\n[PHASE CHANGE] Reached Target Altitude! Initiating Braking Sequence...\n")
                        phase = "BRAKING"

                elif phase == "BRAKING":
                    # Override all PID outputs to strictly 0 to stop the drone
                    v_x, v_y, v_z = 0.0, 0.0, 0.0
                    brake_counter += 1
                    print(f"[PHASE 3] Braking motors... ({brake_counter}/10)")
                    
                    # Hold 0,0,0 for 10 loops (1.0 second) to kill all physical momentum
                    if brake_counter >= 10:
                        print("[SUCCESS] Drone stabilized. simulating drop.")
                        if brake_counter >= 50:
                            print("[INTERCEPT] Action complete. Exiting interception loop...")
                            async for pos in drone.telemetry.position():
                                distance_error = haversine_distance(pos.latitude_deg, pos.longitude_deg, mission_lat, mission_lon)
                                print(f"[DEBUG] distance to last mission point: {distance_error:.2f} m")
                                break
                            break
                

          

                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(v_x, v_y, v_z, 0.0))
                await asyncio.sleep(loop_rate)

        except Exception as e:
            print(f"[CRITICAL ERROR in Centering Loop]: {e}")
            center_success = False


        if center_success:
            try:
                await drone.offboard.stop()
                await drone.action.hold()
            except OffboardError:
                pass

           
        
        # 6. Return to the saved mission location
        print("[INTERCEPT] Returning to original mission location/altitude...")
        await drone.action.goto_location(mission_lat, mission_lon, mission_abs_alt, yaw_deg)

        async for pos in drone.telemetry.position():
            if haversine_distance(pos.latitude_deg, pos.longitude_deg, mission_lat, mission_lon) < 2.0:
                print("[INTERCEPT] Reached original mission location.")
                break
            await asyncio.sleep(0.5)

        # await asyncio.sleep(8) # Wait to fly back
        
        # 7. Resume the mission
        print("[INTERCEPT] Resuming mission...")
        target_data['active'] = False
        target_data['cooldown_until'] = time.time() + 15.0 # Ignore YOLO for 15s to prevent immediate re-trigger
        await drone.mission.start_mission()



async def run():
    drone = System()

    if SIM:
       await drone.connect(system_address="udpin://127.0.0.1:14551")
    else:
        await drone.connect(system_address="serial:///dev/ttyUSB0:115200")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    async for position in drone.telemetry.position():
        home_lat, home_lon = position.latitude_deg, position.longitude_deg
        print(f"Home position: LAT {home_lat:.6f}, LON {home_lon:.6f}")
        break

    await drone.mission.clear_mission()
    print("Generating and uploading new mission...")

    if POLE_LATLON:
        
        rel_pole1 = latlon_to_xy(pole1[0], pole1[1], home_lat, home_lon)
        rel_pole2 = latlon_to_xy(pole2[0], pole2[1], home_lat, home_lon)
    else:
        rel_pole1 = pole1_rel
        rel_pole2 = pole2_rel
    print(f"relative, pole1: {rel_pole1}, pole2: {rel_pole2}")

    v = np.array(rel_pole2) - np.array(rel_pole1)

    if mission_num == 1:
        print("Creating mission 1")
        waypoints = generate_figure8_around_poles(rel_pole1, rel_pole2, clearance, precision)

        waypoints.append((latlon_to_xy(home_lat, home_lon, home_lat, home_lon))) 
        plot_waypoints(waypoints)

        waypoints = xy_to_latlon(waypoints, home_lat, home_lon)
    elif mission_num == 2:
        print("Creating mission 2")
        wp = np.array([generate_capsule_waypoints(rel_pole1, rel_pole2, clearance, i/(precision-1)) for i in range(precision)])
        mask = wp[:,2] < 2

        waypoints = np.array([wp[i][:2] for i in range(len(wp))])  # Extract only x,y coordinates
        waypoints = waypoints[mask][:,:2] #remove last point

        pts = generate_scan(area_width, area_length, alt, fov, overlap, N_line=10, N_turn=4)
        x1, y1 = rel_pole1
        x2, y2 = rel_pole2
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0

        pts[:,0] *= -1
        pts = np.array(pts) - (np.array(center_x), np.array(center_y+clearance))

        vx = v[0]
        vy = v[1]
        v = np.array([vx, vy])



        theta = np.arctan2(v[1], v[0])
        R = np.array([
                    [np.cos(theta), -np.sin(theta)],
                    [np.sin(theta),  np.cos(theta)]
                ])
        pts  = (pts @ R.T)

        waypoints = np.append(waypoints, pts, axis=0)
        

        perp_home = (perpendicular_offset(rel_pole1, rel_pole2, (0, 0), -1, clearance))

        print(f"perp home {perp_home}")

        waypoints = np.vstack([waypoints, perp_home])

        plot_waypoints(waypoints)
        waypoints = xy_to_latlon(waypoints, home_lat, home_lon)


    mission_items = create_mission_items(waypoints, alt_mission)

    if DEBUG:
        print("")
        exit(0)

    mission_plan = MissionPlan(mission_items)
    await drone.mission.upload_mission(mission_plan)
    print("Mission uploaded.")


    print("Arming and Taking off...")
    try:
        await drone.action.set_takeoff_altitude(10.0)
        await drone.action.arm()
        await drone.action.takeoff()
        await asyncio.sleep(8)
    except ActionError as e:
        print(f"Takeoff failed, might already flying: {e}")
        
    # Start UDP Listener and Interception Task
    udp_transport = await start_udp_listener(host="127.0.0.1", port=9000)

    # intercept_task = asyncio.create_task(interception_task(drone))


    print("Starting pre-uploaded mission...")
    await drone.mission.start_mission()
    async for progress in drone.mission.mission_progress():
        print(f"Mission progress: {progress.current}/{progress.total}")
        if progress.current == progress.total:
            print("Mission complete!")
            break
            
    print("Returning to launch...")
    await drone.action.return_to_launch()

    async for pos in drone.telemetry.position():
        home_distance = haversine_distance(pos.latitude_deg, pos.longitude_deg, home_lat, home_lon)
        print(f"Distance to home: {home_distance:.2f} m")
        if home_distance < 2.0:
            print("Landed at home location.")
            # await drone.action.land()
            # await drone.action.
            break
        await asyncio.sleep(1)
    # intercept_task.cancel()
    udp_transport.close()




if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("Script interrupted by user.")