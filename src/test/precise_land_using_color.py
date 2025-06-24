import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from control.coordinates import Coordinates
from pathlib import Path
from config.config_manager import ConfigManager
from drone.drone_controller import DroneController

async def sequence_test_precise_land_using_color(drone: DroneController, speed, target_coordinates: Coordinates):
    logger = drone.get_logger_instance()

    logger.write("arming")
    await drone.arm()
    logger.write("taking off...")
    await drone.flight_controller.takeoff(target_coordinates.altitude())
    logger.write("finished taking off")
    await drone.flight_controller.hovering(5)
    logger.write('goto started')
    await drone.flight_controller.go_to_location(speed, target_coordinates, 0.1)
    logger.write("start precise landing")
    await drone.flight_controller.offboard_land_using_color()
    logger.write('end')

async def main():
    # config_path: str = Path(__file__).resolve().parent.parent.joinpath("assets/config/config.ini")
    # config = ConfigManager(config_path)
    drone = DroneController()
    await drone.connect()

    lora = drone.get_lora_instance()
    await lora.lora_start()
    await asyncio.sleep(5)
    
    # `invoke_sensor`の呼び出し
    asyncio.create_task(drone.invoke_sensor())

    # すぐに`invoke_sensor`タスクを開始する
    await asyncio.sleep(5)

    speed = 3.0
    target_coordinates = Coordinates(140.108073773, 35.770522881999995, 5)

    # `add_sequence_task`の呼び出し
    await drone.add_sequence_task(sequence_test_precise_land_using_color(drone, speed, target_coordinates))

    try:
        # 無限ループを維持するためのFutureを作成
        await asyncio.Future()
    except asyncio.CancelledError:
        print("Main loop cancelled")

if __name__ == "__main__":
    asyncio.run(main())