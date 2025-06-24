import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from drone.drone_controller import DroneController
from mavsdk.offboard import (Attitude, PositionNedYaw, OffboardError)

async def test1():
    drone = DroneController()

    await drone.connect()
    await drone.arm()
    #asyncio.create_task(drone.invoke_sensor())

    drone_instance = drone.get_drone_instance()
    
    try:
        await drone_instance.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone_instance.action.disarm()
        return

    print("-- Go up at 70% thrust")
    await drone_instance.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.7))
    await asyncio.sleep(2)

    print("-- Roll 30 at 60% thrust")
    await drone_instance.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(2)

    print("-- Roll -30 at 60% thrust")
    await drone_instance.offboard.set_attitude(Attitude(-30.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(2)

    print("-- Hover at 60% thrust")
    await drone_instance.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(2)

    print("-- Stopping offboard")
    try:
        await drone_instance.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")

    await drone_instance.action.land()

async def ned_test():
    drone = DroneController()

    await drone.connect()
    await drone.arm()

    drone_instance = drone.get_drone_instance()

    print("-- Setting initial setpoint")
    await drone_instance.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone_instance.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone_instance.action.disarm()
        return

    print("-- Go 0m North, 0m East, -5m Down \
            within local coordinate system")
    await drone_instance.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(10)

    print("-- Go 5m North, 0m East, -5m Down \
            within local coordinate system, turn to face East")
    await drone_instance.offboard.set_position_ned(PositionNedYaw(5.0, 0.0, -5.0, 90.0))
    await asyncio.sleep(10)

    print("-- Go 5m North, 10m East, -5m Down \
            within local coordinate system")
    await drone_instance.offboard.set_position_ned(PositionNedYaw(5.0, 10.0, -5.0, 90.0))
    await asyncio.sleep(15)

    print("-- Go 0m North, 10m East, 0m Down \
            within local coordinate system, turn to face South")
    await drone_instance.offboard.set_position_ned(PositionNedYaw(0.0, 10.0, 0.0, 180.0))
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone_instance.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")
    
    await drone_instance.action.land()

if __name__ == "__main__":
    asyncio.run(test1())