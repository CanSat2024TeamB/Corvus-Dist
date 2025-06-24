import sys
from pathlib import Path
import datetime

sys.path.append(str(Path(__file__).parent.parent))

import asyncio
import time
from sensor.camera_handler import CameraHandler

camera_handler = CameraHandler.get_instance()
output_path = str(Path(__file__).parent.parent.parent.joinpath(f"assets/log/mov_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.mp4"))


print("start capturing")
camera_handler.capture_video(output_path, 10)
time.sleep(15)
print("finish")