import sys
from pathlib import Path
import datetime

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from control.coordinates import Coordinates
from pathlib import Path
from config.config_manager import ConfigManager
from drone.drone_controller import DroneController
from sensor.camera_handler import CameraHandler

async def capture_video_during_flight(drone: DroneController, speed, target_coordinates, output_path, video_length):
    logger = drone.get_logger_instance()

    logger.write("arming")
    await drone.arm()
    logger.write("taking off...")
    await drone.flight_controller.takeoff(5)
    logger.write("finished taking off")
    await drone.flight_controller.go_to_location(speed, target_coordinates, 0.1)
    camera_handler = CameraHandler.get_instance()
    if camera_handler.is_connected():
        logger.write(f"start capturing {video_length} s video")
        camera_handler.capture_video(output_path, video_length)
    else:
        logger.write("Camera is not connected.")
    await drone.flight_controller.hovering(video_length)
    await drone.flight_controller.land()

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
    target_coordinates = Coordinates(139.887311364, 35.766587931, 5)
    output_path = str(Path(__file__).parent.parent.parent.joinpath(f"assets/log/mov_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.mp4"))

    # `add_sequence_task`の呼び出し
    await drone.add_sequence_task(capture_video_during_flight(drone, speed, target_coordinates, output_path, 30))

    try:
        # 無限ループを維持するためのFutureを作成
        await asyncio.Future()
    except asyncio.CancelledError:
        print("Main loop cancelled")

if __name__ == "__main__":
    asyncio.run(main())