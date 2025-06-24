import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent))

from sensor.light_handler import LightSensor
import time

if __name__ == "__main__":
    sensor = LightSensor()
    try:
        while True:
            value = sensor.read_light_value()
            print(f"Light Sensor Value: {value}")
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()