import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
import time
from pathlib import Path
from config.config_manager import ConfigManager
from drone.drone_controller import DroneController
from case.case_handler import CaseHandler

async def main():

    nichrome_pin_1 = 7
    nichrome_pin_2 = 25

    #config_path: str = Path(__file__).resolve().parent.parent.joinpath("assets/config/config.ini")
    #config = ConfigManager(config_path)
    dronecontroller = DroneController()
    drone = dronecontroller.get_drone_instance()
    case = CaseHandler(drone)
    #countdown(20)
    #await dronecontroller.connect()
    #while True:
        #print(f"Velocity stability confirmation start")
        #if await case.judge_velocity_stable(1):
            #print(f"Velocity stability cinfirmed")
            #break
        #else:
            #print(f"Velocity not stable.restart")

    countdown(10)

    print(f"1para and case nichrome cut start")
    case.para_case_stand_nichrome(nichrome_pin_1)
    print(f"1para and case nichrome cut end")
    countdown(10)
    print(f"1para and case nichrome cut start")
    case.para_case_stand_nichrome(nichrome_pin_1)
    print(f"1para and case nichrome cut end")
    countdown(30)
    print(f"2arm nichrome cut start")
    case.para_case_stand_nichrome(nichrome_pin_2)
    print(f"2arm nichrome cut end")
    countdown(10)
    print(f"2arm nichrome cut start")
    case.para_case_stand_nichrome(nichrome_pin_2)
    print(f"2arm nichrome cut end")
    case.nichrome_cleanup()
    #dronecontroller.arm()

def countdown(seconds):
    while seconds > 0:
        print(f"{seconds}秒")
        time.sleep(1)
        seconds -= 1


if __name__ == "__main__":
    asyncio.run(main())

