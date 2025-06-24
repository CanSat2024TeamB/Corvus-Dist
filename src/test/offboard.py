import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from drone.drone_controller import DroneController
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw, VelocityBodyYawspeed)
import asyncio

import time
import datetime

from threading import Thread

drone_controller = DroneController()

def record_alt():
    global drone_controller
    logger = drone_controller.get_logger_instance()
    while True:
        altitude = drone_controller.position_manager.raw_altitude()
        yaw_deg = drone_controller.position_manager.yaw_deg()
        
        logger.write(f"{datetime.datetime.now().strftime('%f')}", f"alt: {altitude}, yaw: {yaw_deg}", no_print = True)

        time.sleep(0.1)

async def run():
    global drone_controller
    logger = drone_controller.get_logger_instance()

    logger.write("start sequence")

    await drone_controller.connect()
    asyncio.create_task(drone_controller.invoke_sensor())

    record_alt_thread = Thread(target = record_alt, daemon = True)
    record_alt_thread.start()

    await drone_controller.arm()

    await asyncio.sleep(1)

    logger.write("drone taking off")
    
    await drone_controller.flight_controller.takeoff(3)
    logger.write("hovering...")
    await drone_controller.flight_controller.hovering(3)
    logger.write("end hovering")

    drone = drone_controller.get_drone_instance()

    logger.write("initializing offbord mode")

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0))

    logger.write("starting offboard controll")

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(error._result.result)

    logger.write("ascending 1.5 m")

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.5, 0))
    await asyncio.sleep(5)

    logger.write("hovering")

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(2)

    logger.write("start turning")

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 20))
    await asyncio.sleep(10)

    logger.write("reached target")
    
    # yaw_deg = drone_controller.position_manager.yaw_deg()
    # yaw_rad = yaw_deg * math.pi / 180
    # await drone.offboard.set_velocity_body(VelocityBodyYawspeed(math.cos(yaw_rad) * 0.1, math.sin(yaw_rad) * 0.1, 1 * 0.1, 0))
    # await asyncio.sleep(20)

    await drone.offboard.stop()

    logger.write("stopped offboard controll")

    await drone.action.land()
    await asyncio.sleep(10)

    logger.write("landed")

if __name__ == "__main__":
    asyncio.run(run())