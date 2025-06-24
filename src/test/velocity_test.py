import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
import time
from drone.drone_controller import DroneController
from case.case_handler import CaseHandler
import numpy as np
from logger.logger import Logger


async def main():
    #config_path: str = Path(__file__).resolve().parent.parent.joinpath("assets/config/config.ini")
    #config = ConfigManager(config_path)
    drone_controller = DroneController()
    case = CaseHandler(drone_controller)
    logger_ = drone_controller.logger
    
    await drone_controller.connect()
    while True:
        a = await case.ac_vel.get_velocity()
        b = await case.ac_vel.get_acceleration()
        logger_.write(str(a))
        logger_.write(str(b))
        print(str(b))
        await asyncio.sleep(0.01)
        


if __name__ == "__main__":
    asyncio.run(main())