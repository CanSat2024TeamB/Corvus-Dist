import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from drone.drone_controller import DroneController

async def sequence_test_hovering(drone: DroneController):
    logger = drone.get_logger_instance()

    await drone.flight_controller.takeoff(5)
    logger.write('reached start hovering')
    await drone.flight_controller.hovering(10)
    logger.write('finish hovering start landing')
    await drone.flight_controller.land()

async def main():
    drone = DroneController()

    await drone.connect()
    await drone.arm()
    asyncio.create_task(drone.invoke_sensor())

    # 5秒待機してから新しいタスクを追加
    await asyncio.sleep(5)
    await drone.add_sequence_task(sequence_test_hovering(drone))

    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        print("Main loop cancelled")

if __name__ == "__main__":
    asyncio.run(main())