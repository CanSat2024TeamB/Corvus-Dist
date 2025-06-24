import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from control.coordinates import Coordinates
from pathlib import Path
from config.config_manager import ConfigManager
from drone.drone_controller import DroneController

async def sequence_test_mission(drone: DroneController, speed, *target_coordinates: Coordinates):
    logger = drone.get_logger_instance()
    await drone.flight_controller.go_to(speed, *target_coordinates)
    logger.write('mission started')
    while True:
        await asyncio.sleep(0.1)
        mission_completed = await drone.flight_controller.if_mission_finished()
        if mission_completed:
            logger.write('mission finished start hovering')
            await drone.flight_controller.hovering(5)
            logger.write('hovering finished start landing')
            await drone.flight_controller.land()
            logger.write("landed")
            break

async def main():
    #config_path: str = Path(__file__).resolve().parent.parent.joinpath("assets/config/config.ini")
    #config = ConfigManager(config_path)
    drone = DroneController()
    await drone.connect()
    asyncio.create_task(drone.invoke_sensor())
    #await drone.arm()
    await asyncio.sleep(5)

    speed = 1.0
    first_lon = drone.position_manager.adjusted_coordinates_lon()
    first_lat = drone.position_manager.adjusted_coordinates_lat()
    hov_alt = 1
    hov_alt = 3
    target_coordinates_1 = Coordinates(first_lon,first_lat,hov_alt)
    target_coordinates_2 = Coordinates(140.1080417,35.7702389,3)
    
    await drone.add_sequence_task(sequence_test_mission(drone, speed, target_coordinates_1, target_coordinates_2))
    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        print("Main loop cancelled")

if __name__ == "__main__":
    asyncio.run(main())