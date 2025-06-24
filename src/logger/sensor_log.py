import time
import asyncio
from threading import Thread

from logger.logger import Logger
from sensor.light_handler import LightSensor
from sensor.pressure_handler import PressureHandler
from lora.lora import Lora

finished = False

def sensor_logger(logger: Logger, light_handler: LightSensor, pressure_handler: PressureHandler):
    while not finished:
        light = light_handler.get_light_value()
        pressure = pressure_handler.get_pressure()
        logger.write(f"light: {light}, pressure: {pressure}", no_print = True)

        time.sleep(0.5)

def transmit(lora: Lora, pressure_handler: PressureHandler):
    async def _transmit(lora: Lora, pressure_handler: PressureHandler):
        while not finished:
            pressure = pressure_handler.get_pressure()
            await lora.lora_send(f"p:{pressure}")
            await asyncio.sleep(30)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    loop.run_until_complete(_transmit(lora, pressure_handler))
    loop.close()

def start(logger: Logger, lora: Lora, light_handler: LightSensor, pressure_handler: PressureHandler):
    global finished
    finished = False
    logging_thread = Thread(target = sensor_logger, args = (logger, light_handler, pressure_handler), daemon = True)
    trasnmitting_thread = Thread(target = transmit, args = (lora, pressure_handler), daemon = True)
    logging_thread.start()
    trasnmitting_thread.start()

def stop():
    global finished
    finished = True