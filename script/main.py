import asyncio
import numpy as np
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.action import ActionError

# from ardupilot.Tools.autotest.sim_vehicle import progress

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculates the distance between two GPS coordinates in meters."""
    R = 6371000  # Radius of Earth in meters
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    delta_phi = np.radians(lat2 - lat1)
    delta_lambda = np.radians(lon2 - lon1)
    a = np.sin(delta_phi / 2.0)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(delta_lambda / 2.0)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    return R * c

def generate_figure8_waypoints(center_lat, center_lon, size_meters, precision):
    """
    Generates a list of (lat, lon) tuples for a figure-8 path.
    Note: The competition path is two semi-circles, but this lemniscate is good for testing smoothness.
    """
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
        new_lat = center_lat + lat_offset
        new_lon = center_lon + lon_offset
        waypoints.append((new_lat, new_lon))
    return waypoints

def create_mission_items(waypoints, flight_alt_agl):
    """
    Converts a list of (lat, lon) tuples into MAVSDK MissionItem objects.
    This is the corrected version for newer MAVSDK libraries.
    """
    mission_items = []
    for lat, lon in waypoints:
        item = MissionItem(
            # --- Original Arguments ---
            lat,
            lon,
            flight_alt_agl,  # Altitude is AGL for mission items
            10.0,            # Fly at 10 m/s
            True,            # is_fly_through = True for smooth corners
            float('nan'),    # gimbal_pitch_deg (not used)
            float('nan'),    # gimbal_yaw_deg (not used)
            MissionItem.CameraAction.NONE,
            float('nan'),    # loiter_time_s (not used)
            float('nan'),    # camera_photo_interval_s (not used)

            # --- NEW REQUIRED ARGUMENTS ---
            5.0,             # acceptance_radius_m: How close to get to the waypoint.
            float('nan'),    # yaw_deg: Let the drone control its yaw.
            float('nan'),    # camera_photo_distance_m (not used)
            MissionItem.VehicleAction.NONE # No special action at the waypoint
        )
        mission_items.append(item)
    return mission_items


async def upload_and_fly_mission(drone, mission_items):
    """
    Uploads a mission plan to the drone and executes it.
    """
    print("Creating mission plan...")
    mission_plan = MissionPlan(mission_items)

    print("Uploading mission...")
    try:
        await drone.mission.upload_mission(mission_plan)
        print("Mission uploaded successfully.")
    except Exception as e:
        print(f"Mission upload failed: {e}")
        return

    print("Arming drone...")
    try:
        await drone.action.arm()
    except ActionError as e:
        print(f"Arming failed: {e}")
        return

    print("Starting mission...")
    try:
        await drone.mission.start_mission()
    except Exception as e:
        print(f"Starting mission failed: {e}")
        return

    # Wait for the mission to complete
    async for progress in drone.mission.mission_progress():
        print(f"Mission progress: {progress.current}/{progress.total}")
        if progress.current == progress.total:
            print("Mission complete!")
            break

async def intercept_at_wp8(drone):
    """
    Wait until mission reaches waypoint 8,
    switch to guided control,
    move 10m right, hold, then return.
    """

    async for progress in drone.mission.mission_progress():
        print(f"Mission progress: {progress.current}/{progress.total}")

        if progress.current == 2:
            print("Waypoint 8 reached — intercepting mission")

            # pause mission
            await drone.mission.pause_mission()

            # switch to guided (Position control)
            await drone.action.hold()

            # get current position & heading
            async for pos in drone.telemetry.position():
                current_lat = pos.latitude_deg
                current_lon = pos.longitude_deg
                current_alt = pos.relative_altitude_m
                break

            async for heading in drone.telemetry.heading():
                yaw_deg = heading.heading_deg
                break

            print(f"Current heading: {yaw_deg:.1f}°")

            # compute 10m to the RIGHT of drone
            import math

            move_distance = 10  # meters
            earth_radius = 6378137.0

            # right = heading + 90 degrees
            move_bearing = math.radians(yaw_deg + 90)

            dlat = (move_distance * math.cos(move_bearing)) / earth_radius
            dlon = (move_distance * math.sin(move_bearing)) / (
                earth_radius * math.cos(math.radians(current_lat))
            )

            target_lat = current_lat + math.degrees(dlat)
            target_lon = current_lon + math.degrees(dlon)

            print("Moving 10m right of heading...")

            await drone.action.goto_location(
                target_lat,
                target_lon,
                current_alt,
                yaw_deg
            )

            # wait until position reached
            await asyncio.sleep(5)

            print("Holding position...")
            await drone.action.hold()
            await asyncio.sleep(5)

            print("Returning to mission...")
            await drone.mission.start_mission()

            break


async def run():
    """Main function to connect to the drone and run the mission."""
    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14551")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("Fetching home position...")
    async for position in drone.telemetry.position():
        home_lat = position.latitude_deg
        home_lon = position.longitude_deg
        break
    
    print(f"Home position set to: LAT {home_lat}, LON {home_lon}")

    # print(drone.telemetry.is_armed())

    await drone.action.return_to_launch()

    # Wait until landed
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed and disarmed.")
            break


    # Mission Parameters
    takeoff_altitude_agl = 10.0
    figure8_size_m = 250
    num_waypoints = 8 # More waypoints = smoother path for the figure-8

    print("Taking off...")
    try:
        await drone.action.set_takeoff_altitude(takeoff_altitude_agl)
        await drone.action.arm()
        await drone.action.takeoff()
    except ActionError as e:
        print(f"Takeoff failed: {e}")
        return

    # Wait until takeoff is complete
    await asyncio.sleep(5)

    await drone.mission.start_mission()

    print("mission drone...")


    # Wait for the mission to complete
    async for progress in drone.mission.mission_progress():
        print(f"Mission progress: {progress.current}/{progress.total}")
        if progress.current == progress.total:
            print("Mission complete!")
            break
    # await drone.action.arm()

    # await drone.mission.start_mission()

    # print("Mission started. Monitoring progress...")
    #     # async for progress in drone.mission.mission_progress():

    # async for progress in drone.mission.mission_progress():
    #     print(f"Mission progress: {progress.current}/{progress.total}")
    #     if progress.current == 2:
    #         await drone.mission.pause_mission()
    #         await drone.action.hold()
    #         async for pos in drone.telemetry.position():
    #             # print(f"Current position: LAT {pos.latitude_deg}, LON {pos.longitude_deg}, ALT {pos.relative_altitude_m}")
    #             # break
    #             lon = pos.longitude_deg
    #             lat = pos.latitude_deg
    #             alt = pos.relative_altitude_m
    #             print(f"Current position: LAT {lat}, LON {lon}, ALT {alt}")
    #             break
            
    #         await drone.action.goto_location(
    #             lat, lon, alt - 10, 3.14 / 2
    #         )

    #         await asyncio.sleep(5)
    #         for pos in drone.telemetry.position():
    #             # print(f"Current position: LAT {pos.latitude_deg}, LON {pos.longitude_deg}, ALT {pos.relative_altitude_m}")
    #             # break
    #             lon = pos.longitude_deg
    #             lat = pos.latitude_deg
    #             alt = pos.relative_altitude_m
    #             print(f"Current position: LAT {lat}, LON {lon}, ALT {alt}")
    #         # break
           
    #         await drone.action.land()

    #         print("Landed.")

            # long = await drone.telemetry.position

    
    # await intercept_at_wp8(drone)
    



    # # Generate the waypoints for the mission
    # print("Generating mission waypoints...")
    # waypoints = generate_figure8_waypoints(home_lat, home_lon, figure8_size_m, num_waypoints)
    # mission_items = create_mission_items(waypoints, takeoff_altitude_agl)

    # # Upload and fly the mission
    # await upload_and_fly_mission(drone, mission_items)

    # print("Mission finished. Returning to launch...")
    # await drone.action.return_to_launch()

    # # Wait until landed
    # async for in_air in drone.telemetry.in_air():
    #     if not in_air:
    #         print("-- Landed and disarmed.")
    #         break

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        
        print("Mission interrupted by user.")