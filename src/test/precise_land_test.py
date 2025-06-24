import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from drone.drone_controller import DroneController
import logger.flight_log as flight_log

async def sequence_test_precise_land(drone: DroneController):
    print("taking off")
    await drone.flight_controller.takeoff(5)
    print("finished taking off")
    await drone.flight_controller.hovering(5)
    print("start landing")
    try:
        await drone.flight_controller.offboard_precise_land()
    except RuntimeError as e:
        print(e)
        await drone.flight_controller.land()
    print("landed")

async def main():
    drone = DroneController()

    await drone.connect()

    lora = drone.get_lora_instance()
    await lora.lora_start()
    await asyncio.sleep(5)
    
    await drone.arm()
    asyncio.create_task(drone.invoke_sensor())

    flight_log.start(drone.get_logger_instance(), drone.get_position_manager_instance())

    # 5秒待機してから新しいタスクを追加
    await asyncio.sleep(5)
    await drone.add_sequence_task(sequence_test_precise_land(drone))

    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        print("Main loop cancelled")

if __name__ == "__main__":
    asyncio.run(main())