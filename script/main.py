import asyncio
import time
import numpy as np
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.offboard import VelocityBodyYawspeed, OffboardError

# Global dictionary to store the latest YOLO target data
target_data = {
    'active': False,
    'dx': 0.0,  # -1.0 (left) to 1.0 (right)
    'dy': 0.0,  # -1.0 (top) to 1.0 (bottom)
    'last_seen': 0.0,
    'cooldown_until': 0.0 # Prevents re-triggering immediately after returning to mission
}


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

def generate_figure8_waypoints(center_lat, center_lon, size_meters, precision):
    waypoints = []
    meters_to_lat_deg = 1 / 111111.0
    meters_to_lon_deg = 1 / (111111.0 * np.cos(np.radians(center_lat)))
    a = size_meters / 2.0
    for i in range(precision):
        t = 2 * np.pi * i / precision
        x_offset = a * np.sin(t)
        y_offset = a * np.sin(t) * np.cos(t)
        lat_offset = y_offset * meters_to_lat_deg
        lon_offset = x_offset * meters_to_lon_deg
        waypoints.append((center_lat + lat_offset, center_lon + lon_offset))
    return waypoints

def create_mission_items(waypoints, flight_alt_agl):
    mission_items = []
    speed = 10
    acceptance_radius = 10.0
    for lat, lon in waypoints:
        item = MissionItem(
            lat, lon, flight_alt_agl, speed, True, 
            float('nan'), float('nan'), MissionItem.CameraAction.NONE,
            float('nan'), float('nan'), acceptance_radius, float('nan'), 
            float('nan'), MissionItem.VehicleAction.NONE
        )
        mission_items.append(item)
    # mission_items.append(MissionItem(

    # ))
    return mission_items


# --- VISUAL SERVOING / INTERCEPTION LOGIC ---
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


# --- MAIN RUNNER ---
async def run():
    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14551")

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
        break

    
    await drone.mission.clear_mission()
    print("Generating and uploading new mission...")
    waypoints = generate_figure8_waypoints(home_lat, home_lon, 250, 8)
    mission_items = create_mission_items(waypoints, 10.0)
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
        # return

    # Start UDP Listener and Interception Task
    udp_transport = await start_udp_listener(host="127.0.0.1", port=9000)
    intercept_task = asyncio.create_task(interception_task(drone))

    print("Starting pre-uploaded mission...")
    await drone.mission.start_mission()

    async for progress in drone.mission.mission_progress():
        print(f"Mission progress: {progress.current}/{progress.total}")
        if progress.current == progress.total:
            print("Mission complete!")
            break
            
    print("Returning to launch...")
    await drone.action.return_to_launch()
    intercept_task.cancel()
    udp_transport.close()

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("Script interrupted by user.")