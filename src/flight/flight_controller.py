import asyncio
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.offboard import (Attitude, PositionNedYaw, VelocityBodyYawspeed, OffboardError)
from pathlib import Path
import math
import numpy as np

import time
import datetime
import multiprocessing

from control.position_manager import PositionManager
from control.coordinates import Coordinates
from sensor.camera_handler import CameraHandler, ConeDetector
from logger.logger import Logger

class FlightController:

    def __init__(self, drone: System, position_manager: PositionManager, logger: Logger):
        self.drone: System = drone
        self.position_manager: PositionManager = position_manager
        self.logger = logger
        self.target_latitude = 0
        self.target_longitude = 0
        self.target_altitude = 0
        self.current_ASML = 0 
        self.target_final_altitude = 0 
        self.current_lidar_alt = 0
        self.yaw_deg = 0
        self.detected_pos = [None,None]
        self.nondetected_counter = 0
        self.nondetected_counter_max = 10
        self.alp = 45 ## カメラ取り付け角
        self.theta = [54.2993633956, 42.0750220508] ##カメラ視野角
        self.lat_unit = 111051.665 #m 緯度一度の長さ　ARLISS
        self.lon_unit = 84282.462 #m　経度一度の長さ　ARLISS

        
        self.detected_flag = False
        self.is_in_air: bool = False
        
    async def takeoff(self, takeoff_altitude) -> bool:
        take_off_max_time = 0
        await self.drone.action.set_takeoff_altitude(takeoff_altitude*2)
        await self.drone.action.takeoff()
        self.logger.write('sent take off command')
        await asyncio.sleep(3)
    
        while self.position_manager.adjusted_altitude() <= takeoff_altitude:
            await asyncio.sleep(0.1)
            take_off_max_time += 0.1
            if take_off_max_time > 20:
                self.logger.write('take off max time')
                break
        return True
        # await asyncio.sleep(0.1)
        # if self.position_manager.adjusted_altitude() >= 3:    
        #     return True
        # else:
        #     await self.land()
        #     await asyncio.sleep(5)
        #     return await self.takeoff(takeoff_altitude)

    async def hovering(self, time: float) -> bool:
        await self.drone.action.hold()
        await asyncio.sleep(time)
        return True
    
    async def stop_here(self):
        await self.drone.action.hold()
    
    async def land(self) -> bool:
        await self.drone.action.land()
        while True:
            if self.position_manager.adjusted_altitude() < 0.1:
                return True
            await asyncio.sleep(1)
    
    async def disarm(self) -> bool:
        await self.drone.action.disarm()
        return True
    
    async def kill(self) -> bool:
        await self.drone.action.kill()
        return True
    
    ############################################################################################
    async def go_to(self, speed, *target_coordinates: Coordinates) -> bool:
        mission_items = []
        await self.drone.mission.clear_mission()

        self.logger.write('mission cleared')

        for coordinates in target_coordinates:
            mission_items.append(MissionItem(coordinates.latitude(), coordinates.longitude(), coordinates.altitude(), speed, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE))
            #mission_items.append(MissionItem(coordinates.latitude(), coordinates.longitude(), coordinates.altitude(), speed, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), MissionItem.VehicleAction.NONE))
        mission_plan = MissionPlan(mission_items)

        self.logger.write('mission plan made')
        self.logger.write('do not return setting')

        await self.drone.mission.set_return_to_launch_after_mission(False)

        self.logger.write('do not return start upload')

        await self.drone.mission.upload_mission(mission_plan)

        self.logger.write('mission plan uploading.move to hold mode')

        await self.stop_here() #####先輩のをみるとholdに入れてる
        await asyncio.sleep(10) #####アップロードにかかる時間？

        self.logger.write("Waiting for drone to be armable...")

        async for is_armable in self.drone.telemetry.health():
            if is_armable:
                self.logger.write("Drone is armable")

                break
            await asyncio.sleep(0.11)

        self.logger.write("Arming the drone...")

        await self.drone.action.arm()

        async for is_armed in self.drone.telemetry.armed():
            if is_armed:
                self.logger.write("drone is armed")

                break
            await asyncio.sleep(0.1)
        await self.drone.mission.start_mission()
        return True
    
    async def if_mission_finished(self) -> bool:
        return await self.drone.mission.is_mission_finished()

    ###################################################################################################
    async def go_to_location(self, speed, target_coordinates: Coordinates, circle_radious, margin_to_target=0):
        logger = self.logger
        self.target_latitude = target_coordinates.latitude()
        self.target_longitude = target_coordinates.longitude()
        #self.target_altitude = target_coordinates.altitude()
        self.current_AMSL = self.position_manager.adjusted_coordinates_AMSL()
        self.current_lidar_alt = self.position_manager.adjusted_altitude()

        self.set_interval_waypoint(margin_to_target) ##ゴール10m手前に目標地点を設置 不要ならコメントアウト

        logger.write('current AMSL', self.current_AMSL)
        logger.write('current lidar', self.current_lidar_alt)
        
        self.target_final_altitude = self.current_AMSL

        # if YAW_NORTH == True:
        #     self.yaw_deg = 0
        # else:
        #     self.yaw_deg = self.calculate_yaw_angle()
        self.yaw_deg = self.calculate_yaw_angle()

        await self.drone.action.goto_location(self.target_latitude, self.target_longitude, self.target_final_altitude, self.yaw_deg)
        logger.write('goto started')
        await self.drone.action.set_current_speed(speed)
        
        while not self.if_goto_location_finished(self.target_latitude, self.target_longitude, circle_radious):
            await asyncio.sleep(1)
            self.current_lidar_alt = self.position_manager.adjusted_altitude()

            if self.current_lidar_alt < 3:
                logger.write('dangerous altitude, landing forcibly')
                await self.land()
                logger.write("landed")
                await asyncio.sleep(10)
                logger.write("disarming...")
                await self.disarm()
                logger.write("disarmed")
                return False
            
            elif self.current_lidar_alt < 5:
                logger.write('altitude too low')
                self.target_final_altitude += 0.1
                logger.write('target AMSL alt', self.target_final_altitude)

                # if YAW_NORTH == True:
                #     self.yaw_deg = 0
                # else:
                #     self.yaw_deg = self.calculate_yaw_angle()
                self.yaw_deg = self.calculate_yaw_angle()

                await self.drone.action.goto_location(self.target_latitude, self.target_longitude,  self.target_final_altitude, self.yaw_deg)
                
            elif self.current_lidar_alt > 10:
                logger.write('altitude too high')
                self.target_final_altitude -= 0.1
                logger.write('target AMSL alt', self.target_final_altitude)

                # if YAW_NORTH == True:
                #     self.yaw_deg = 0
                # else:
                #     self.yaw_deg = self.calculate_yaw_angle()
                self.yaw_deg = self.calculate_yaw_angle()

                await self.drone.action.goto_location(self.target_latitude, self.target_longitude, self.target_final_altitude, self.yaw_deg)
        logger.write('goto finished')
        await self.drone.action.set_current_speed(0.001)
        return True
    
    def if_goto_location_finished(self, target_latitude, target_longitude, circle_radious):
            lat_dif = abs(target_latitude - self.position_manager.adjusted_coordinates_lat()) *  self.lat_unit 
            lon_dif = abs(target_longitude - self.position_manager.adjusted_coordinates_lon()) * self.lon_unit
            return lat_dif**2 + lon_dif**2 < circle_radious**2
    
    def calculate_yaw_angle(self):
            return 0
            lat_dist = (self.target_latitude - self.position_manager.adjusted_coordinates_lat()) *  self.lat_unit
            lon_dist = (self.target_longitude - self.position_manager.adjusted_coordinates_lon()) * self.lon_unit
            yaw_deg =  90.0 - math.degrees(math.atan2(lon_dist, lat_dist))
            self.logger.write(yaw_deg)
            return yaw_deg
    
    def set_interval_waypoint(self, radious_m):
            phi = self.calculate_yaw_angle()
            self.target_latitude -= radious_m * math.cos(math.radians(phi))/self.lat_unit
            self.target_longitude -= radious_m * math.sin(math.radians(phi))/self.lon_unit
            return

##############################################################################################################

    async def precise_land_slope(self) -> bool:
        camera_handler = CameraHandler.get_instance()
        cone_detector = ConeDetector(camera_handler)
        if not camera_handler.is_connected():
            await self.go_to_location(1.0, Coordinates(self.target_longitude,self.target_latitude,self.target_final_altitude), 1.0)
            await self.land()
            raise RuntimeError("Camera is not connected. Stopped the precies land sequence.")
     
        while self.detected_pos == [None,None]:
                await asyncio.sleep(1)
                self.detected_pos = cone_detector.capture_cone_position_and_save(str(Path(__file__).parent.parent.parent.joinpath(f"assets/log/img_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.jpg")), 0.3)
                #self.detected_pos = cone_detector.get_pos(use_color_assist = True)
                self.logger.write(self.detected_pos)
                self.nondetected_counter += 1
                if self.nondetected_counter == self.nondetected_counter_max:
                    await self.go_to_location(1.0, Coordinates(self.target_longitude,self.target_latitude,self.target_final_altitude), 0.5)
                    await self.land()
                    return 
        
        self.nondetected_counter = 0
        cone_x = self.detected_pos[0]
        cone_y = self.detected_pos[1]
        beta =  self.theta[0] * cone_x * 0.5
        gamma = self.theta[1] * cone_y * 0.5
        delta = self.calculate_delta_angle(self.target_latitude,self.target_longitude)
        self.logger.write('delta') 
        self.current_lidar_alt = self.position_manager.adjusted_altitude()
        east_len_m = self.current_lidar_alt * math.tan(math.radians(self.alp + gamma)) * math.cos(math.radians(delta - beta))
        self.logger.write('east_len_m')
        north_len_m = self.current_lidar_alt * math.tan(math.radians(self.alp + gamma)) * math.sin(math.radians(delta - beta))
        self.logger.write('north_len_m')
        error_lon = east_len_m / self.lon_unit
        error_lat = north_len_m / self.lat_unit
        self.current_AMSL = self.position_manager.adjusted_coordinates_AMSL()
        self.target_final_altitude = self.current_AMSL-self.current_lidar_alt

        await self.drone.action.goto_location(self.target_latitude + error_lat,
                                              self.target_longitude + error_lon, 
                                              self.target_final_altitude, self.calculate_yaw_angle())
        await self.drone.action.set_current_speed(0.5)
        self.logger.write('last descending')

        while self.position_manager.adjusted_altitude() > 0.25:
            self.logger.write('still')
            await asyncio.sleep(0.2)
        await self.land()

        return
################################################################################################################################################
    async def precise_land_right_angle(self) -> bool:
        camera_handler = CameraHandler.get_instance()
        cone_detector = ConeDetector(camera_handler)
        if not camera_handler.is_connected():
            await self.go_to_location(1.0, Coordinates(self.target_longitude,self.target_latitude,self.target_final_altitude), 0.1)
            await self.land()
            raise RuntimeError("Camera is not connected. Stopped the precies land sequence.")
     
        while self.detected_pos == [None,None]:
                await asyncio.sleep(1)
                self.detected_pos = cone_detector.capture_cone_position_and_save(str(Path(__file__).parent.parent.parent.joinpath(f"assets/log/img_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.jpg")), 0.3)
                self.logger.write(self.detected_pos)
                #self.detected_pos = cone_detector.get_pos(use_color_assist = True)
                #self.logger.write(self.detected_pos)
                self.nondetected_counter += 1
                if self.nondetected_counter == self.nondetected_counter_max:
                    await self.go_to_location(1.0, Coordinates(self.target_longitude,self.target_latitude,self.target_final_altitude), 0.1)
                    await self.land()
                    return 
        
        self.nondetected_counter = 0
        cone_x = self.detected_pos[0]
        cone_y = self.detected_pos[1]
        beta =  self.theta[0] * cone_x * 0.5
        gamma = self.theta[1] * cone_y * 0.5
        delta = self.calculate_delta_angle(self.target_latitude,self.target_longitude) 
        self.logger.write('delta') 
        self.current_lidar_alt = self.position_manager.adjusted_altitude()
        east_len_m = self.current_lidar_alt * math.tan(math.radians(self.alp + gamma)) * math.cos(math.radians(delta - beta))
        self.logger.write(east_len_m)
        north_len_m = self.current_lidar_alt * math.tan(math.radians(self.alp + gamma)) * math.sin(math.radians(delta - beta))
        self.logger.write(north_len_m)
        error_lon = east_len_m / self.lon_unit
        error_lat = north_len_m / self.lat_unit
        self.current_AMSL = self.position_manager.adjusted_coordinates_AMSL()

        await self.go_to_location(0.5, Coordinates(self.target_longitude + error_lon, self.target_latitude + error_lat, self.current_AMSL), 0.1)
        await self.land()

        return
    
    def calculate_delta_angle(self,target_latitude, target_longitude):
        current_lat = self.position_manager.adjusted_coordinates_lat()
        current_lon = self.position_manager.adjusted_coordinates_lon()

        d_lat = target_latitude - current_lat
        d_lon = target_longitude - current_lon

        x = self.lon_unit * d_lon
        y = self.lat_unit * d_lat
        return math.degrees(math.atan2(x, y)) ##-180~180
        
    async def precise_land_right_angle_calc_confirm_test(self,delta):
        camera_handler = CameraHandler.get_instance()
        cone_detector = ConeDetector(camera_handler)
        if not camera_handler.is_connected():
            self.logger.write('camera cannot use')
            return
     
        while self.detected_pos == [None,None]:
                await asyncio.sleep(1)
                self.detected_pos = cone_detector.capture_cone_position_and_save(str(Path(__file__).parent.parent.parent.joinpath(f"assets/log/img_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.jpg")), 0.3)
                self.logger.write(self.detected_pos)
                #self.detected_pos = cone_detector.get_pos(use_color_assist = True)
                #self.logger.write(self.detected_pos)
                self.nondetected_counter += 1
                if self.nondetected_counter == self.nondetected_counter_max:
                    self.logger.write('cannot detect 10 times')
                    return 
        
        self.nondetected_counter = 0
        cone_x = self.detected_pos[0]
        cone_y = self.detected_pos[1]
        beta =  self.theta[0] * cone_x * 0.5
        gamma = self.theta[1] * cone_y * 0.5 
        self.current_lidar_alt = self.position_manager.adjusted_altitude()
        east_len_m = self.current_lidar_alt * math.tan(math.radians(self.alp + gamma)) * math.cos(math.radians(delta - beta))
        self.logger.write(east_len_m)
        north_len_m = self.current_lidar_alt * math.tan(math.radians(self.alp + gamma)) * math.sin(math.radians(delta - beta))
        self.logger.write(north_len_m)
        return 
######################################################################################################################################################    
    async def precise_land_vertical(self):
        camera_handler = CameraHandler.get_instance()
        cone_detector = ConeDetector(camera_handler)
        if not camera_handler.is_connected():
            self.logger.write('camera cannot use')
            await self.land()
            return
     
        while self.detected_pos == [None,None]:
            await asyncio.sleep(1)
            image = camera_handler.capture_bgr()
            self.detected_pos = cone_detector.calc_color_center(image)
            self.logger.write(self.detected_pos)
            cone_detector.draw_circle_and_save(
                image, 
                self.detected_pos[0], 
                self.detected_pos[1], 
                f"/home/admin/corvus/assets/log/color_detect_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.png"
            )
            #self.detected_pos = cone_detector.get_pos(use_color_assist = True)
            #self.logger.write(self.detected_pos)
            self.nondetected_counter += 1
            if self.nondetected_counter == self.nondetected_counter_max:
                self.logger.write('cannot detect 10 times')
                await self.descend(-1.0)
                self.nondetected_counter = 0
                continue

            if self.position_manager.adjusted_altitude() > 12.0:
                self.logger.write('cannot found')
                await self.land()
                return
                
        self.nondetected_counter = 0
        cone_x = self.detected_pos[0]
        cone_y = self.detected_pos[1]
        self.current_lidar_alt = self.position_manager.adjusted_altitude()
        yaw_deg = self.position_manager.yaw_deg()
        r = np.array([[cone_x*self.current_lidar_alt*math.tan(math.radians(self.theta[0]))],
                          [cone_y*self.current_lidar_alt*math.tan(math.radians(self.theta[1]))]]) #機体軸における、目標地点との差(ｍ)
        rotate = np.array([[math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))],
                               [-math.sin(math.radians(yaw_deg)), math.cos(math.radians(yaw_deg))]])
        r_e = np.dot(rotate,r).flatten() #地面固定座標系における、目標地点との差(ｍ)
        self.logger.write(f"east:{r_e[0]}, north:{r_e[1]}")
        error_lon = r_e[0] / self.lon_unit
        error_lat = r_e[1] / self.lat_unit
        self.current_AMSL = self.position_manager.adjusted_coordinates_AMSL()

        await self.go_to_location(0.5, Coordinates(self.target_longitude + error_lon, self.target_latitude + error_lat, self.current_AMSL), 0.1)
        await self.land()
    
    
    async def precise_land_vertical_calc_test(self):
        camera_handler = CameraHandler.get_instance()
        cone_detector = ConeDetector(camera_handler)
        if not camera_handler.is_connected():
            self.logger.write('camera cannot use')
            return
     
        while self.detected_pos == [None,None]:
            await asyncio.sleep(1)
            image = camera_handler.capture_bgr()
            self.detected_pos = cone_detector.calc_color_center(image)
            self.logger.write(self.detected_pos)
            cone_detector.draw_circle_and_save(
                image, 
                self.detected_pos[0], 
                self.detected_pos[1], 
                f"/home/admin/corvus/assets/log/color_detect_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.png"
            )
            #self.detected_pos = cone_detector.get_pos(use_color_assist = True)
            #self.logger.write(self.detected_pos)
            self.nondetected_counter += 1
            if self.nondetected_counter == self.nondetected_counter_max:
                self.logger.write('cannot detect 10 times')
                return
                
        self.nondetected_counter = 0
        cone_x = self.detected_pos[0]
        cone_y = self.detected_pos[1]
        self.current_lidar_alt = self.position_manager.adjusted_altitude()
        self.logger.write(self.current_lidar_alt)
        yaw_deg = self.position_manager.yaw_deg()
        r = np.array([[cone_x*self.current_lidar_alt*math.tan(math.radians(self.theta[0])*0.5)],
                          [cone_y*self.current_lidar_alt*math.tan(math.radians(self.theta[1])*0.5)]]) #機体軸における、目標地点との差(ｍ)
        self.logger.write(f"x:{r[0]}, y:{r[1]}")
        self.logger.write(f"Yaw:{yaw_deg}")
        rotate = np.array([[math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))],
                               [-math.sin(math.radians(yaw_deg)), math.cos(math.radians(yaw_deg))]])
        r_e = np.dot(rotate,r).flatten() #地面固定座標系における、目標地点との差(ｍ)
        self.logger.write(f"east:{r_e[0]}, north:{r_e[1]}")
        return


    #########################################################################################################

    def invoke_detection(self, conf, finished, arr):
        camera_handler = CameraHandler.get_instance()
        cone_detector = ConeDetector(camera_handler)

        self.logger.write("checking camera connection...")
        if not camera_handler.is_connected():
            raise RuntimeError("Camera is not connected. Stopped the precies land sequence.")
        self.logger.write("camaera connection checked")

        while finished.value == 0:
            pos = cone_detector.capture_cone_position_and_save(f"/home/admin/corvus/assets/log/detect_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S.%f')}.png", conf, use_color_assist = True)
            if pos[0] is None:
                arr[0] = -2
                arr[1] = -2
            else:
                arr[0] = pos[0]
                arr[1] = pos[1]

    async def offboard_precise_land(self) -> bool:        
        CAMERA_YAW_DEG = 0 #pixhawk正面からはかったカメラの指向方向 (deg, 右回り正)
        LAND_ALTITUDE = 0.2 #コーンに接近していってlandに移行する高度
        PROB_THRESHOLD = 0.3 #画像認識probabilityの閾値
        DECENDING_SPEED = 0.5 #降下速度
        ADJUST_FACTOR = 1.5 #上下左右方向の補正係数(1.0が無調整)

        arr = multiprocessing.Array("f", 2)
        arr[0] = -2
        arr[1] = -2
        finished = multiprocessing.Value("b", 0)

        position = PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        velocity_body = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
        
        async def set_position(self, new_position: PositionNedYaw):
            nonlocal position
            position = new_position
            await self.drone.offboard.set_position_ned(position)

        async def set_altitude(self, altitude: float):
            nonlocal position
            await set_position(self, PositionNedYaw(position.north_m, position.east_m, -1 * altitude, self.position_manager.yaw_deg()))
            
            alt_threshold = 0.5
            while True:
                if abs(self.position_manager.adjusted_altitude() - altitude) < alt_threshold:
                    return

        async def set_velocity_body(self, new_velocity_body: VelocityBodyYawspeed):
            nonlocal velocity_body
            velocity_body = new_velocity_body
            await self.drone.offboard.set_velocity_body(velocity_body)

        async def add_velocity_body(self, delta_velocity: VelocityBodyYawspeed):
            nonlocal velocity_body
            velocity_body = VelocityBodyYawspeed(velocity_body.forward_m_s + delta_velocity.forward_m_s, velocity_body.right_m_s + delta_velocity.right_m_s, velocity_body.down_m_s + delta_velocity.down_m_s, velocity_body.yawspeed_deg_s + delta_velocity.yawspeed_deg_s)
            await self.drone.offboard.set_velocity_body(velocity_body)

        async def turn_clock_wise(self, speed: float):
            """
            set yaw angular veocity of the drone
            Parameter
            ---------
            speed : float
                clock-wise angular speed rate (degree / s), negative param cause anti-clock-wise rotation.
            """
            await add_velocity_body(self, VelocityBodyYawspeed(0.0, 0.0, 0.0, speed))

        async def stop_rotation(self):
            nonlocal velocity_body
            await set_velocity_body(self, VelocityBodyYawspeed(velocity_body.forward_m_s, velocity_body.right_m_s, velocity_body.down_m_s, 0.0))
        
        def multiply_velocity_body(velocity_body: VelocityBodyYawspeed, f: float) -> VelocityBodyYawspeed:
            return VelocityBodyYawspeed(velocity_body.forward_m_s * f, velocity_body.right_m_s * f, velocity_body.down_m_s * f, velocity_body.yawspeed_deg_s)

        async def adjust_velocity(self, yaw_deg, pos):
            nonlocal ADJUST_FACTOR
            yaw_rad = yaw_deg * math.pi / 180
            normalized_delta_velocity = VelocityBodyYawspeed(pos[0] * (-1 * math.sin(yaw_rad)), pos[0] * math.cos(yaw_rad), pos[1], 0.0)
            await add_velocity_body(self, multiply_velocity_body(normalized_delta_velocity, ADJUST_FACTOR))

        def calc_velocity_body_to_target(self, pos) -> VelocityBodyYawspeed:
            nonlocal CAMERA_YAW_DEG
            nonlocal ADJUST_FACTOR
            front_vec = math.sin(math.radians(45 + pos[1] * self.theta[1] / 2 * ADJUST_FACTOR))
            right_vec = math.sin(math.radians(pos[0] * self.theta[0] / 2 * ADJUST_FACTOR))
            down_vec = math.cos(math.radians(pos[0] * self.theta[0] / 2 * ADJUST_FACTOR)) * math.cos(math.radians(45 + pos[1] * self.theta[1] / 2 * ADJUST_FACTOR))
            #front_vec = math.sin(math.radians(45 + pos[1] * self.theta[1] / 2)) * math.cos(math.radians(CAMERA_YAW_DEG + pos[0] * self.theta[0] / 2))
            #right_vec = math.sin(math.radians(45 + pos[1] * self.theta[1] / 2)) * math.sin(math.radians(CAMERA_YAW_DEG + pos[0] * self.theta[0] / 2))
            #down_vec = math.cos(math.radians(45 + pos[1] * self.theta[1] / 2))
            return VelocityBodyYawspeed(front_vec, right_vec, down_vec, 0.0)
        
        def get_pos(self):
            nonlocal arr
            pos = [None, None]
            if arr[0] >= -1:
                pos[0] = arr[0]
                pos[1] = arr[1]
            return pos

        async def rotate_and_search_cone(self: FlightController, rotate_rate: float) -> bool:
            search_time = 60 #この秒数見つからなかったら強制的に着陸
            await turn_clock_wise(self, rotate_rate)
            
            time_start = time.perf_counter()
            while True: ####### コーンがみつからなかったときに近くを徘徊するコードがまだない
                pos = get_pos(self)

                if pos[0] is not None:
                    self.logger.write("cone detected")
                    self.logger.write(f"cone pos: {pos}")
                    await stop_rotation(self)
                    await asyncio.sleep(1)

                    pos = get_pos(self)
                    
                    if pos[0] is not None:
                        self.logger.write("cone position confirmed")
                        self.logger.write(f"confirmed cone pos: {pos}")
                        return True
                    else:
                        return await rotate_and_search_cone(self, rotate_rate / 2) # コーンを認識して止まった後、静止状態でもう一回とって認識できなかったらゆっくり回ってもう一回（推定のラグを考慮）

                await asyncio.sleep(0.1)
                time_now = time.perf_counter()
                if time_now - time_start > search_time:
                    return False
                
        async def approach_cone(self: FlightController):
            found_cone = await rotate_and_search_cone(self, 30)
            if found_cone:
                nonlocal CAMERA_YAW_DEG
                nonlocal LAND_ALTITUDE
                nonlocal DECENDING_SPEED

                # await set_velocity_body(self, multiply_velocity_body(calc_velocity_body_to_target(self), DECENDING_SPEED))

                # while True:
                #     #pos = cone_detector.capture_cone_position(PROB_THRESHOLD, use_color_assist = True)
                #     pos = cone_detector.capture_cone_position_and_save(f"/home/admin/corvus/assets/log/detect_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.png", PROB_THRESHOLD, use_color_assist = True)
                #
                #     if pos[0] is not None:
                #         self.logger.write("cone detected while approaching cone")
                #         self.logger.write(f"pos: {pos}")
                #         await adjust_velocity(self, CAMERA_YAW_DEG, pos)
                #     else:
                #         self.logger.write("lost cone")
                #         await set_velocity_body(self, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                #         self.logger.write("restarting searching cone")
                #         return await approach_cone(self)
                    
                #     if self.position_manager.adjusted_altitude() < LAND_ALTITUDE:
                #         self.logger.write("got ready to land")
                #         # await set_velocity_body(self, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                #         return

                while True:
                    #pos = cone_detector.capture_cone_position(PROB_THRESHOLD, use_color_assist = True)
                    pos = get_pos(self)

                    if pos[0] is not None:
                        self.logger.write("cone detected while approaching cone")
                        self.logger.write(f"pos: {pos}")
                        await set_velocity_body(self, multiply_velocity_body(calc_velocity_body_to_target(self, pos), DECENDING_SPEED))
                    else:
                        MISS_TOLERANCE = 1
                        for i in range(MISS_TOLERANCE):
                            pos = get_pos(self)
                            await asyncio.sleep(0.5)
                            if pos[0] is not None:
                                break
                        
                        if pos[0] is None:
                            self.logger.write("lost cone")
                            await set_velocity_body(self, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                            self.logger.write("restarting searching cone")
                            return await approach_cone(self)
                    
                    await asyncio.sleep(0.1)
                    self.logger.write(f"lidar value: {self.position_manager.adjusted_altitude()}")
                    if self.position_manager.adjusted_altitude() < LAND_ALTITUDE:
                        self.logger.write("got ready to land")
                        # await set_velocity_body(self, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                        return True
                    
            else:
                self.logger.write("Could not find cone in the searching process.")
                # await set_velocity_body(self, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                return False

        self.logger.write("start precise landing")

        await set_position(self, position)
        await set_velocity_body(self, velocity_body)
        
        self.logger.write("starting offboard landing...")
        try:
            await self.drone.offboard.start()
            #self.logger.write("setting altitude 3 m")
            #await set_altitude(3)
            
            try:
                process = multiprocessing.Process(target = self.invoke_detection, args = (PROB_THRESHOLD, finished, arr), daemon = True)
                process.start()
            except RuntimeError as error:
                self.logger.write(f"Camera does not connected, {error._result.result}")
                finished.value = 1
                return False

            result = await approach_cone(self)
            
            finished.value = 1
            await self.drone.offboard.stop()
            self.logger.write("finished drone offboard control")

            if result:
                self.logger.write("got to the target. Landing...")
                await self.land()
                return True
            else:
                self.logger.write("Could not get to the target")
                return False
        except OffboardError as error:
            self.logger.write(f"Starting offboard controll failed, {error._result.result}")
            return False
##########################################################################################################################        
    async def offboard_land_using_color(self):
        CAMERA_YAW_DEG = 0 #pixhawk正面からはかったカメラの指向方向 (deg, 右回り正)
        PROB_THRESHOLD = 0.2
        LAND_ALTITUDE = 1 #コーンに接近していってlandに移行する高度
        DECENDING_SPEED = 0.5 #降下速度
        ADJUST_FACTOR = 2.0 #上下左右方向の補正係数(1.0が無補正、値を大きくすると左右方向の補正が強くなる)

        camera_handler = CameraHandler.get_instance()
        cone_detector = ConeDetector(camera_handler)
        self.logger.write("checking camera connection...")
        if not camera_handler.is_connected():
            raise RuntimeError("Camera is not connected. Stopped the precies land sequence.")
        self.logger.write("camaera connection checked")

        velocity_body = VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)

        async def set_velocity_body(self, new_velocity_body: VelocityBodyYawspeed):
            nonlocal velocity_body
            velocity_body = new_velocity_body
            await self.drone.offboard.set_velocity_body(velocity_body)

        async def add_velocity_body(self, delta_velocity: VelocityBodyYawspeed):
            nonlocal velocity_body
            velocity_body = VelocityBodyYawspeed(velocity_body.forward_m_s + delta_velocity.forward_m_s, velocity_body.right_m_s + delta_velocity.right_m_s, velocity_body.down_m_s + delta_velocity.down_m_s, velocity_body.yawspeed_deg_s + delta_velocity.yawspeed_deg_s)
            await self.drone.offboard.set_velocity_body(velocity_body)
        
        def multiply_velocity_body(velocity_body: VelocityBodyYawspeed, f: float) -> VelocityBodyYawspeed:
            return VelocityBodyYawspeed(velocity_body.forward_m_s * f, velocity_body.right_m_s * f, velocity_body.down_m_s * f, velocity_body.yawspeed_deg_s)

        def calc_velocity_body_to_target(self, pos) -> VelocityBodyYawspeed:
            nonlocal CAMERA_YAW_DEG
            nonlocal ADJUST_FACTOR
            front_vec = math.sin(math.radians(pos[1] * self.theta[1] / 2))
            right_vec = math.sin(math.radians(pos[0] * self.theta[0] / 2))
            down_vec = math.cos(math.radians(pos[0] * self.theta[0] / 2)) * math.cos(math.radians(pos[1] * self.theta[1] / 2))
            return VelocityBodyYawspeed(front_vec * ADJUST_FACTOR, right_vec * ADJUST_FACTOR, down_vec, 0.0)
        
        async def adjust_velocity(self, pos):
            nonlocal ADJUST_FACTOR
            nonlocal CAMERA_YAW_DEG
            camera_yaw_rad = CAMERA_YAW_DEG * math.pi / 180
            normalized_delta_velocity = VelocityBodyYawspeed(pos[0] * math.cos(camera_yaw_rad) - pos[1] * math.sin(camera_yaw_rad), pos[0] * math.sin(camera_yaw_rad) + pos[1] * math.cos(camera_yaw_rad), 0.0, 0.0)
            await add_velocity_body(self, multiply_velocity_body(normalized_delta_velocity, ADJUST_FACTOR))

        self.logger.write("start precise landing")
        await set_velocity_body(self, velocity_body)
    
        self.logger.write("starting offboard landing...")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            self.logger.write(f"Starting offboard controll failed, {error._result.result}")
            return False

        pos = [None, None]
        for i in range(10):
            image = camera_handler.capture_bgr()
            pos = cone_detector.calc_color_center(image)
            cone_detector.draw_circle_and_save(image, pos[0], pos[1], f"/home/admin/corvus/assets/log/color_detect_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.png")
            if pos[0] is not None:
                break
            await asyncio.sleep(1)

        if pos[0] is not None:
            while True:
                image = camera_handler.capture_bgr()
                pos = cone_detector.calc_color_center(image)
                cone_detector.draw_circle_and_save(image, pos[0], pos[1], f"/home/admin/corvus/assets/log/color_detect_{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}.png")

                await set_velocity_body(self, multiply_velocity_body(calc_velocity_body_to_target(self, pos), DECENDING_SPEED))

                if pos[0] is not None:
                    self.logger.write("cone detected while approaching cone")
                    self.logger.write(f"pos: {pos}")
                    await adjust_velocity(self, pos)
                else:
                    self.logger.write("lost cone")
                    await set_velocity_body(self, VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
                    self.logger.write("landing forcibly")
                    break
                
                if self.position_manager.adjusted_altitude() < LAND_ALTITUDE:
                    self.logger.write("got ready to land")
                    break
        else:
            self.logger.write("Could not find cone in the searching process.")
            self.logger.write("landing forcibly")

        await self.drone.offboard.stop()
        self.logger.write("finished drone offboard control")

        await self.land()
        return True
##########################################################################################################################################
    async def rotate_yaw(self, yaw):
        await self.drone.action.set_current_speed(0.1)
        self.target_latitude = self.position_manager.adjusted_coordinates_lat()
        self.target_longitude = self.position_manager.adjusted_coordinates_lon()
        self.target_altitude = self.position_manager.adjusted_altitude()
        self.current_AMSL = self.position_manager.adjusted_coordinates_AMSL()
        self.target_final_altitude = self.current_AMSL
        self.yaw_deg = yaw
        
        self.logger.write('rotate')
        await self.drone.action.goto_location(self.target_latitude, self.target_longitude, self.target_final_altitude, self.yaw_deg)

    async def descend(self,descend_m):
        await self.drone.action.set_current_speed(0.5)
        self.target_latitude = self.position_manager.adjusted_coordinates_lat()
        self.target_longitude = self.position_manager.adjusted_coordinates_lon()
        self.current_AMSL = self.position_manager.adjusted_coordinates_AMSL()
        self.current_lidar_alt = self.position_manager.adjusted_altitude()
        self.target_final_altitude = self.current_AMSL-2*descend_m

        self.logger.write('descend')
        await self.drone.action.goto_location(self.target_latitude, self.target_longitude, self.target_final_altitude, self.yaw_deg)
        while abs(self.current_lidar_alt-self.position_manager.adjusted_altitude()) < abs(descend_m):
            await asyncio.sleep(0.2)
        return


    async def fly_orbit(self, radius, velocity, yaw, latitude, longitude, altitude):
        await self.drone.action.do_orbit(radius_m=radius,
                                   velocity_ms=velocity,
                                   yaw_behavior=yaw,
                                   latitude_deg=latitude,
                                   longitude_deg=longitude,
                                   absolute_altitude_m=altitude)

    
    def update_is_in_air(self, is_in_air: bool) -> None:
        self.is_in_air = is_in_air
        return
    
    async def invoke_loop(self) -> None:
        async for is_in_air in self.drone.telemetry.in_air():
            self.update_is_in_air(is_in_air)
            await asyncio.sleep(1)