import asyncio
from drone.drone_controller import DroneController
from control.coordinates import Coordinates
from pathlib import Path
from config.config_manager import ConfigManager
from case.case_handler import CaseHandler
import time

async def main():
    drone = DroneController()
    case = CaseHandler(drone)
    
    config = ConfigManager()
    config_section = "NOSHIRO"
    nichrome_pin_no = config.read_int(config_section, "NichromePin")

    while True:
        light_value = case.light.get_light_value()
        pressure_value = case.pressure.get_pressure()
        print("Light:",light_value,"Pressure:",pressure_value)
        time.sleep(1)


if __name__ == "__main__":
    asyncio.run(main())