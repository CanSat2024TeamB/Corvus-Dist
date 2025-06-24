import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
from drone.drone_controller import DroneController
from case.case_handler import CaseHandler
import time

async def main():
    dronecontroller = DroneController()
    case = CaseHandler(dronecontroller)
    nichrome_pin_no = 25

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
    countdown(10)
    print(f"1para and case nichrome cut start")
    case.para_case_stand_nichrome(nichrome_pin_no)
    print(f"1para and case nichrome cut end")
    countdown(10)
    print(f"1para and case nichrome cut start")
    case.para_case_stand_nichrome(nichrome_pin_no)
    print(f"1para and case nichrome cut end")
    case.nichrome_cleanup()
    #dronecontroller.arm()
        

def countdown(seconds):
    while seconds > 0:
        print(f"{seconds}秒")
        time.sleep(1)
        seconds -= 1


if __name__ == "__main__":
    asyncio.run(main())