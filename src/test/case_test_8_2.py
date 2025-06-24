import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from drone.drone_controller import DroneController
from case.case_handler import CaseHandler

async def main():
    dronecontroller = DroneController()
    case = CaseHandler(dronecontroller)

    global status 
    status = "outside"

    if status == "outside":
        case.judge_storage()
        status = "storage"
    if status == "storage":
        case.judge_release()
        status = "release"
    if status == "release":
        await case.judge_landing()
        status = "land"



if __name__ == "__main__":
    asyncio.run(main())