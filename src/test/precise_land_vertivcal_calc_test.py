import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from drone.drone_controller import DroneController
import asyncio

async def precise_land_vertical_calc_test(drone: DroneController):
    await drone.flight_controller.precise_land_vertical_calc_test()

async def camera_calc_test_loop1(drone: DroneController) -> None:
    # すべてのタスクをリストに追加
    drone.tasks.extend([
        asyncio.create_task(drone.lidar_handler.invoke_loop()),
        asyncio.create_task(drone.compass_handler.invoke_loop())
    ])

    # 全てのタスクが完了するのを待つ
    await asyncio.sleep(float('inf'))

async def main():
    dronecontroller = DroneController()
    await dronecontroller.connect()
    asyncio.create_task(camera_calc_test_loop1(dronecontroller))
    await asyncio.sleep(5)
    await dronecontroller.add_sequence_task(precise_land_vertical_calc_test(dronecontroller))
    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        print("Main loop cancelled")


if __name__ == "__main__":
    asyncio.run(main())