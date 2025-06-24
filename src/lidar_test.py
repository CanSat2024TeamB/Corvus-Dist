import sys
from pathlib import Path
from drone.drone_controller import DroneController
import asyncio

async def lidar_test(drone: DroneController):
    while True:
        print(drone.position_manager.adjusted_altitude())
        await asyncio.sleep(0.5)

async def lidar_test_loop(drone: DroneController) -> None:
        # すべてのタスクをリストに追加
        drone.tasks.extend([
            asyncio.create_task(drone.lidar_handler.invoke_loop()),
            asyncio.create_task(drone.compass_handler.invoke_loop()),
            asyncio.create_task(lidar_test(drone))
        ])
        # 全てのタスクが完了するのを待つ
        await asyncio.sleep(float('inf'))

async def main():
    dronecontroller = DroneController()
    await dronecontroller.connect()
    asyncio.create_task(lidar_test_loop(dronecontroller))
    await asyncio.sleep(float('inf'))

if __name__ == "__main__":
    asyncio.run(main())
