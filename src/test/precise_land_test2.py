import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from drone.drone_controller import DroneController
from control.coordinates import Coordinates
from config.config_manager import ConfigManager
import logger.flight_log as flight_log

async def sequence(drone: DroneController, speed, target_coordinates: Coordinates, goal_radius, offboard_acceptable_distance):
    logger = drone.get_logger_instance()
    logger.write("taking off")
    await drone.flight_controller.takeoff(5)
    logger.write("finished taking off")
    await drone.flight_controller.hovering(5)
    logger.write("going to the target position")
    await drone.flight_controller.go_to_location(speed, target_coordinates, goal_radius, margin_to_target=5)
    logger.write("start precise landing")

    try:
        result = await drone.flight_controller.offboard_precise_land()
    except Exception as e:
        import traceback
        import re
        error_class = type(e)
        error_description = str(e)
        err_msg = '%s: %s' % (error_class, error_description)
        logger.write(err_msg)
        tb = traceback.extract_tb(sys.exc_info()[2])
        trace = traceback.format_list(tb)
        logger.write('---- traceback ----')
        for line in trace:
            if '~^~' in line:
                logger.write(line.rstrip())
            else:
                text = re.sub(r'\n\s*', ' ', line.rstrip())
                logger.write(text)
        logger.write('-------------------')
        result = False

    logger.write("finished precise landing")
    if result:
        await asyncio.sleep(5)
        await drone.flight_controller.disarm()

        lat = 0
        lon = 0
        AVE_NUMBER = 10
        for i in range(AVE_NUMBER):
            lat += drone.position_manager.adjusted_coordinates_lat()
            lon += drone.position_manager.adjusted_coordinates_lon()
            await asyncio.sleep(1)
        lat /= AVE_NUMBER
        lon /= AVE_NUMBER
        logger.write(f"now, position is (lat: {lat}, lon: {lon})")
        lat_dif = abs(target_coordinates.latitude() - lat) * drone.flight_controller.lat_unit 
        lon_dif = abs(target_coordinates.longitude() - lon) * drone.flight_controller.lon_unit
        if lat_dif ** 2 + lon_dif ** 2 > offboard_acceptable_distance ** 2:
            logger.write("too far from the target position, trying to re-takeoff")
            await drone.arm()
            await drone.flight_controller.takeoff(5)
            result = False

    if not result:
        logger.write("failed precise landing, going above the target.")
        await drone.flight_controller.go_to_location(speed, target_coordinates, 0.5)
        logger.write("starting vertical land")
        await drone.flight_controller.land()
    logger.write("landed")

async def main():
    config = ConfigManager()
    section = "DEBUG"
    
    SPEED = config.read_float(section, "Speed")
    longitude = config.read_float(section, "TargetLon")
    latitude = config.read_float(section, "targetlat")
    hov_alt = config.read_float(section, "HovAlt")
    TARGET = Coordinates(longitude=longitude, latitude=latitude, altitude=hov_alt)
    GOAL_RADIUS = config.read_float(section, "GoalRadius")
    OFFBOARD_ACCEPTABLE_DISTANCE = config.read_float(section, "OffboardAcceptableDistance")

    drone = DroneController()

    await drone.connect()
    await drone.arm()
    asyncio.create_task(drone.invoke_sensor())

    flight_log.start(drone.get_logger_instance(), drone.get_position_manager_instance())

    # 1秒待機してから新しいタスクを追加
    await asyncio.sleep(1)
    await drone.add_sequence_task(sequence(drone, SPEED, TARGET, GOAL_RADIUS, OFFBOARD_ACCEPTABLE_DISTANCE))

    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        print("Main loop cancelled")

if __name__ == "__main__":
    asyncio.run(main())