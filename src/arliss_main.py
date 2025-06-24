import asyncio
from drone.drone_controller import DroneController
from control.coordinates import Coordinates
from config.config_manager import ConfigManager
from case.case_handler import CaseHandler
import logger.sensor_log as sensor_log
import logger.flight_log as flight_log
import time

async def sequence(drone: DroneController, speed: float, target_coordinates: Coordinates, goal_radius: float, offboard_acceptable_distance, config: ConfigManager):
    logger = drone.get_logger_instance()
    config_section = "ARLISS"
    status = config.read(config_section, "Status")

    if not status == "flight":
        logger.write("arming")
        await drone.arm()
        logger.write("taking off")
        await drone.flight_controller.takeoff(target_coordinates.altitude())

        logger.write("finished taking off")

        await drone.flight_controller.hovering(5)
        config.write(config_section, "Status", "flight")

    logger.write("going to the target position")
    
    while True:
        if await drone.flight_controller.go_to_location(speed, target_coordinates, goal_radius, margin_to_target=5):
            break
        else:
            logger.write("arming")
            await drone.arm()
            logger.write("taking off")
            await drone.flight_controller.takeoff(target_coordinates.altitude())
            logger.write("finished taking off")
            await drone.flight_controller.hovering(5)
            config.write(config_section, "Status", "flight")

    logger.write("got target, landing temporarily")
    await drone.flight_controller.land()
    await asyncio.sleep(10)
    await drone.flight_controller.disarm()
    logger.write("disarmed, re-taking off")
    await drone.arm()
    await drone.flight_controller.takeoff(5)
    logger.write("starting precise land")

    try:
        result = await drone.flight_controller.offboard_precise_land()
        logger.write("finished precise landing")
    except Exception as error:
        logger.write("something went wrong during precise landing, landing forcibly")
        await drone.flight_controller.land()
        await asyncio.sleep(20)
        logger.write("disarming...")
        await drone.flight_controller.disarm()
        await asyncio.sleep(10)
        logger.write("trying to re-takeoff")
        await drone.arm()
        await drone.flight_controller.takeoff(target_coordinates.altitude())
        result = False

    if result:
        await asyncio.sleep(10)
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
            await drone.flight_controller.takeoff(target_coordinates.altitude())
            result = False

    if not result:
        logger.write("failed precise landing, going above the target.")
        await drone.flight_controller.go_to_location(speed, target_coordinates, goal_radius)
        logger.write("starting vertical land")
        await drone.flight_controller.land()
    logger.write("landed")

async def main():
    drone: DroneController = DroneController()
    case: CaseHandler = CaseHandler(drone)
    
    logger = drone.get_logger_instance()
    lora = drone.get_lora_instance()

    config: ConfigManager = ConfigManager()
    config_section = "ARLISS"
    nichrome_pin_no = config.read_int(config_section, "NichromePin")

    lora_sync = config.read_int(config_section, "LoraSync")
    lora_freq = config.read_int(config_section, "LoraFreq")
    lora_sf = config.read_int(config_section, "LoraSf")
    lora_bw = config.read_int(config_section, "LoraBw")
    lora_pwr = config.read_int(config_section, "LoraPwr")
    
    logger.write("Starting Lora...")
    await lora.lora_start()
    logger.write("Lora started.")
    await asyncio.sleep(5)
    logger.write("Setting sync...")
    await lora.lora_set_sync(lora_sync)
    logger.write("Sync set.")
    await asyncio.sleep(5)
    logger.write("Setting frequency...")
    await lora.lora_set_freq(lora_freq)
    logger.write("Frequency set.")
    await asyncio.sleep(5)
    logger.write("Setting spreading factor...")
    await lora.lora_set_sf(lora_sf)
    logger.write("Spreading factor set.")
    await asyncio.sleep(5)
    logger.write("Setting bandwidth...")
    await lora.lora_set_bw(lora_bw)
    logger.write("Bandwidth set.")
    await asyncio.sleep(5)
    logger.write("Setting power...")
    await lora.lora_set_pwr(lora_pwr)
    logger.write('Power set.')
    await asyncio.sleep(5)
    logger.write("Saving settings...")
    await lora.lora_save()
    logger.write("Settings saved.")
    await asyncio.sleep(5)
    await lora.lora_send('LoRa OK')

    sensor_log.start(logger, lora, case.light, case.pressure)

    status = config.read(config_section, "Status")
    if status == "outside":
       case.judge_storage()
       config.write(config_section, "Status", "storage")
       await lora.lora_send('Storage Succeeded')

    status = config.read(config_section, "Status")
    if status == "storage":
        case.phase = "Storage"
        case.judge_release()
        config.write(config_section, "Status", "release")
        await lora.lora_send('Release Succeeded')

    status = config.read(config_section, "Status")
    if status == "release":
        countdown(30, logger)
        case.phase = "Released"
        await case.judge_landing()
        config.write(config_section, "Status", "land")
        await lora.lora_send('Landind Succeded')

    sensor_log.stop()

    await drone.connect()

    countdown(10, logger)

    logger.write(f"1para and case nichrome cut start")
    case.para_case_stand_nichrome(nichrome_pin_no)
    logger.write(f"1para and case nichrome cut end")

    countdown(10, logger)

    logger.write(f"1para and case nichrome cut start")
    case.para_case_stand_nichrome(nichrome_pin_no)
    logger.write(f"1para and case nichrome cut end")

    case.nichrome_cleanup()
    await lora.lora_send('Nichromecut End')

    countdown(10, logger)
    asyncio.create_task(drone.invoke_sensor())
    countdown(110, logger)

    #config_path: str = Path(__file__).resolve().parent.parent.joinpath("assets/config/config.ini")
    #config = ConfigManager(config_path)
    
    #await drone.arm()
    await asyncio.sleep(5)

    position_manager = drone.get_position_manager_instance()
    flight_log.start(logger, position_manager)
    
    speed = config.read_float(config_section, "Speed")
    hov_alt = config.read_float(config_section, "HovAlt")
    target_lon = config.read_float(config_section, "TargetLon")
    target_lat = config.read_float(config_section, "TargetLat")
    goal_radius = config.read_float(config_section, "GoalRadius")
    offboard_acceptable_distance = config.read_float(config_section, "OffboardAcceptableDistance")
    target_coordinates_2 = Coordinates(target_lon, target_lat, hov_alt)
    
    await drone.add_sequence_task(sequence(drone, speed, target_coordinates_2, goal_radius, offboard_acceptable_distance, config))
    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        logger.write("Main loop cancelled")

    # try:
    #     await drone.add_sequence_task(drone.sequence_test_mission(speed,target_coordinates_1,target_coordinates_2))
    # except asyncio.CancelledError:
    #     logger.write("Main loop cancelled")
    
    flight_log.stop()

    logger.write("sequence ended")

def countdown(seconds, logger):
    while seconds > 0:
        logger.write(f"{seconds} seconds")
        time.sleep(1)
        seconds -= 1


if __name__ == "__main__":
    asyncio.run(main())