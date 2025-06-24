import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from drone.drone_controller import DroneController
from control.coordinates import Coordinates

async def run():
    drone: DroneController = DroneController()
    await drone.connect()
    asyncio.create_task(drone.invoke_sensor())
    await drone.arm()
    print("taking off...")
    await drone.flight_controller.takeoff(5)
    await drone.flight_controller.hovering(3)
    speed = 3
    target = Coordinates(-119.116452099, 40.8638488, 5)
    print("goint to the target")
    await drone.flight_controller.go_to_location(speed, target, 0.5, margin_to_target = 5)
    print("landing...")
    await drone.flight_controller.land()

if __name__ == "__main__":
    asyncio.run(run())
