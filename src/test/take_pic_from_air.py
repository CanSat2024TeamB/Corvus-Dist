import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
import cv2
import datetime

from picamera2 import Picamera2

from drone.drone_controller import DroneController

async def main():
    camera = Picamera2()
    config = camera.create_preview_configuration({ "format": "RGB888", "size": (1920, 1080) })
    camera.configure(config)
    camera.start()

    drone = DroneController()
    await drone.connect()
    asyncio.create_task(drone.invoke_sensor())
    await drone.arm()
    await drone.flight_controller.takeoff(5)
    await drone.flight_controller.hovering(5)

    time = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')

    for i in range(10):
        image = camera.capture_array()
        image = cv2.rotate(image, cv2.ROTATE_180)
        cv2.imwrite(f"/home/admin/corvus/assets/log/pic_{time}_{i}.png", image)
        await asyncio.sleep(1)
    
    await drone.flight_controller.land()

if __name__ == "__main__":
    asyncio.run(main())
    
