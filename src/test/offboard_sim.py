from mavsdk import System
import asyncio

from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw, VelocityBodyYawspeed)

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14550")

    # Wait for the drone to connect
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Connected to drone!")
            break
    
    print("Waiting for drone to be armable...")
    async for is_armable in drone.telemetry.health():
        if is_armable:
            print("Drone is armable")
            break
        await asyncio.sleep(1)

    # Arm the drone
    print("Arming the drone...")
    await drone.action.arm()

    async for is_armed in drone.telemetry.armed():
        if is_armed:
            print("drone is armed")
            break

    await drone.action.set_takeoff_altitude(5)
    await drone.action.takeoff()

    await asyncio.sleep(15)

    print("initializing offbord mode")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0))

    print("starting offboard controll")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(error._result.result)

    print("ascending 3 m")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -3.0, 0))
    await asyncio.sleep(5)
        
    print("start_turning")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 20))
    await asyncio.sleep(10)
    print("reached target")

    await drone.offboard.stop()
    print("stopped offboard controll")

    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())