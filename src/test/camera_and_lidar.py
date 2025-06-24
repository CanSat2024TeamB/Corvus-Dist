import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
import datetime
import multiprocessing

from drone.drone_controller import DroneController
from sensor.camera_handler import (CameraHandler, ConeDetector)

def invoke_detection(conf, finished, arr):
    camera_handler = CameraHandler.get_instance()
    cone_detector = ConeDetector(camera_handler)

    print("checking camera connection...")
    if not camera_handler.is_connected():
        raise RuntimeError("Camera is not connected. Stopped the precies land sequence.")
    print("camaera connection checked")

    while finished.value == 0:
        pos = cone_detector.capture_cone_position_and_save(f"/home/admin/corvus/assets/log/detect_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.png", conf, use_color_assist = True)
        if pos[0] is None:
            arr[0] = -2
            arr[1] = -2
        else:
            arr[0] = pos[0]
            arr[1] = pos[1]

async def sequence(drone: DroneController):
    PROB_THRESHOLD = 0.2 #画像認識probabilityの閾値
    arr = multiprocessing.Array("f", 2)
    arr[0] = -2
    arr[1] = -2
    finished = multiprocessing.Value("b", 0)

    logger = drone.get_logger_instance()
    
    def get_pos():
        nonlocal arr
        pos = [None, None]
        if arr[0] >= -1:
            pos[0] = arr[0]
            pos[1] = arr[1]
        return pos

    logger.write("sleeping...")
    await asyncio.sleep(10)
    logger.write("woke up, starting camera...")

    try:
        process = multiprocessing.Process(target = invoke_detection, args = (PROB_THRESHOLD, finished, arr), daemon = True)
        process.start()
    except RuntimeError as error:
        logger.write(f"Camera does not connected, {error._result.result}")
        finished.value = 1
        return False
    
    while True:
        logger.write(f"pos: {get_pos()}")
        await asyncio.sleep(0.1)
        if drone.get_position_manager_instance().adjusted_altitude() < 0.1:
            logger.write("altitude < 0.1m")
            logger.write("landing...")
            break

async def main():
    drone = DroneController()
    await drone.connect()

    asyncio.create_task(drone.invoke_sensor())

    await drone.add_sequence_task(sequence(drone))

    try:
        await asyncio.Future()
    except asyncio.CancelledError:
        print("Main loop cancelled")

if __name__ == "__main__":
    asyncio.run(main())