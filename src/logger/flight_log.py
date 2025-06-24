import time
from threading import Thread

from logger.logger import Logger
from control.position_manager import PositionManager

finished = False

def flight_logger(logger: Logger, position_manager: PositionManager):
    while not finished:
        altitude = position_manager.adjusted_altitude()
        longitude = position_manager.adjusted_coordinates_lon()
        latitude = position_manager.adjusted_coordinates_lat()
        yaw_deg = position_manager.yaw_deg()
        logger.write(f"alt: {altitude}, long: {longitude}, lat: {latitude}, yaw: {yaw_deg}", no_print = True)

        time.sleep(0.1)

def start(logger: Logger, position_manager: PositionManager):
    global finished
    finished = False
    logging_thread = Thread(target = flight_logger, args = (logger, position_manager,), daemon = True)
    logging_thread.start()

def stop():
    global finished
    finished = True
