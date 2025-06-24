import asyncio
import serial.tools.list_ports
from mavsdk import System

from sensor.lidar_handler import LiDARHandler
from sensor.battery import Battery_watch
from sensor.gps_handler import GPSHandler
from control.position_manager import PositionManager
from sensor.compass_handler import CompassHandler
from flight.flight_controller import FlightController
from logger.logger import Logger
from sensor.acceleration_velocity import Acceleration_Velocity
from lora.lora import Lora


class DroneController:
    pixhawk_address: str = "serial:///dev/ttyACM0:115200"
    #pixhawk_address: str = "udp://:14540"

    default_log_dir = "/home/admin/corvus/assets/log"

    def __init__(self, log_dir = default_log_dir):
        self.drone_instance: System = System()
        #self.drone = System(mavsdk_server_address='localhost', port=50051)
        self.lidar_handler: LiDARHandler = LiDARHandler(self.drone_instance)
        self.gps_handler: GPSHandler = GPSHandler(self.drone_instance)
        self.battery_watch: Battery_watch = Battery_watch(self.drone_instance)
        self.compass_handler: CompassHandler = CompassHandler(self.drone_instance)
        self.position_manager: PositionManager = PositionManager(self.drone_instance, self.gps_handler, self.compass_handler, self.lidar_handler)
        self.logger: Logger = Logger(log_dir)
        self.flight_controller: FlightController = FlightController(self.drone_instance, self.position_manager, self.logger)
        self.ac_vel: Acceleration_Velocity = Acceleration_Velocity(self.drone_instance)
        self.lora: Lora = Lora(self.drone_instance)

        self.tasks = []
       

    def get_drone_instance(self):
        return self.drone_instance
    
    def get_position_manager_instance(self):
        return self.position_manager

    def get_logger_instance(self):
        return self.logger
    
    def get_lora_instance(self):
        return self.lora

    async def connect(self) -> bool:
        ports = serial.tools.list_ports.comports()
        dev_lst = []
        for port in ports:
            dev_lst.append(port.device)
        
        if '/dev/ttyACM0' in dev_lst:
            self.logger.write("system_address: /dev/ttyACMO")
            DroneController.pixhawk_address = "serial:///dev/ttyACM0:115200"
        if '/dev/ttyACM1' in dev_lst:
            self.logger.write("system_address: /dev/ttyACM1")
            DroneController.pixhawk_address = "serial:///dev/ttyACM1:115200"

        self.logger.write('Connecting...')
        await self.drone_instance.connect(system_address = DroneController.pixhawk_address)
        self.logger.write("Waiting for drone to connect...")

        async for state in self.drone_instance.core.connection_state():
            if state.is_connected:
                self.logger.write("Connected to drone!")
                break
            await asyncio.sleep(0.1)

    async def gps_ok(self):
        async for health in self.drone_instance.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    self.logger.write("Global position estimate OK")
                    break

        return True
    
    async def arm(self) -> bool:
        self.logger.write('gps check start')
        await self.gps_handler.catch_gps()
        self.logger.write('global and local position ok')

        self.logger.write("Waiting for drone to be armable...")
        async for is_armable in self.drone_instance.telemetry.health():
            if is_armable:
                self.logger.write("Drone is armable")
                break
            await asyncio.sleep(0.11)

        self.logger.write("Arming the drone...")
        await self.drone_instance.action.arm()

        async for is_armed in self.drone_instance.telemetry.armed():
            if is_armed:
                self.logger.write("drone is armed")
                break
            await asyncio.sleep(0.1)
        
        return True

    async def invoke_sensor(self) -> None:
        # すべてのタスクをリストに追加
        self.tasks.extend([
            asyncio.create_task(self.lidar_handler.invoke_loop()),
            asyncio.create_task(self.gps_handler.invoke_loop()),
            # asyncio.create_task(self.battery_watch.invoke_loop()),  # バッテリーハンドラのコルーチンが必要なら追加
            asyncio.create_task(self.compass_handler.invoke_loop()),
            # asyncio.create_task(self.flight_controller.invoke_loop()),  # フライトコントローラのコルーチンが必要なら追加
            asyncio.create_task(self.logger_write()),
            asyncio.create_task(self.lora_write())
        ])

        # 全てのタスクが完了するのを待つ
        await asyncio.sleep(float('inf'))

    async def add_sequence_task(self, coro):
        if not hasattr(self, 'tasks'):
            self.tasks = []

        # 新しいタスクを追加
        new_task = asyncio.create_task(coro)
        self.tasks.append(new_task)
        self.logger.write('added task')
    
    async def exe_sequence_task(self, coro):
        if not hasattr(self, 'tasks'):
            self.tasks = []

        # 新しいタスクを追加
        new_task = asyncio.create_task(coro)
        self.tasks.append(new_task)
        self.logger.write('added task')

        for task in self.tasks:
            await task

    async def logger_write(self):
        while True:
            await asyncio.sleep(1)
            message_1 = str(self.position_manager.adjusted_altitude())
            message_2 = str(self.position_manager.adjusted_coordinates_lon())
            message_3 = str(self.position_manager.adjusted_coordinates_lat())
            #message_4 = str(self.ac_vel.get_velocity())
            #message_5 = str(self.battery_watch.remaining_percent())
            #message_6 = str(self.battery_watch.voltage_v())
            #message_7 = str(self.battery_watch.temperature_degc())
            
            self.logger.write(message_1,message_2,message_3)

    async def lora_write(self):
        while True:
            await asyncio.sleep(5)
            #message_4 = str(self.position_manager.adjusted_altitude())
            message_5 = str(round(self.position_manager.adjusted_coordinates_lon(), 4))
            message_6 = str(round(self.position_manager.adjusted_coordinates_lat(), 4))

            #message_4 = str(self.ac_vel.get_velocity())
            #message_5 = str(self.battery_watch.remaining_percent())
            #message_6 = str(self.battery_watch.voltage_v())
            #message_7 = str(self.battery_watch.temperature_degc())
            
            message = ' '.join([message_6, message_5])
            await self.lora.lora_send(message)
            await asyncio.sleep(25)
        
####################################################################################################

    # async def sequence_test_endurance(self,speed, *target_coordinates: Coordinates): #要書き換え
    #     await self.flight_controller.takeoff(5)
    #     await self.flight_controller.hovering(10)
    #     await self.flight_controller.go_to(speed, *target_coordinates)
    #     while True:
    #         await asyncio.sleep(1)
    #         if self.battery_watch.remaining_percent()<35:
    #             await self.flight_controller.land() 
