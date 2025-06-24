from sensor.pressure_handler import PressureHandler
from sensor.acceleration_velocity import Acceleration_Velocity
from sensor.light_handler import LightSensor
from wire.wirehandler import WireHandler
from config.config_manager import ConfigManager
import time
import numpy as np

class CaseHandler:
    def __init__(self,dronecontroller):
        self.dronecontroller = dronecontroller
        self.drone = dronecontroller.get_drone_instance()
        self.logger = dronecontroller.get_logger_instance()

        self.light = LightSensor()
        self.pressure = PressureHandler()
        self.wirehandler = WireHandler()
        self.ac_vel = Acceleration_Velocity(self.drone)

        config = ConfigManager()
        config_section = "ARLISS"
        
        self.stable_pre_val = 1 ##1mで大体0.1hpaの差 2.5秒に一回の判定なので、2m/sでも0.5くらい変わる。
        self.stable_vel_val = 0.5
        
        self.nichrome_duration = 10
        
        #収納判定用定数
        self.judge_storage_border_light = config.read_int(config_section, "JudgeStorageBorderLight")
        self.judge_storage_countmax = config.read_int(config_section, "JudgeStorageCountmax")
        self.judge_storage_maxtime = config.read_int(config_section, "JudgeStorageMaxtime")
        self.judge_storage_sleep_time = config.read_float(config_section, "JudgeStorageSleepTime")

        #放出判定用定数 
        self.judge_release_maxtime = config.read_int(config_section, "JudgeReleaseMaxtime")
        self.judge_release_lig_countmax = config.read_int(config_section, "JudgeReleaseLigCountmax")
        self.judge_release_pre_countmax = config.read_int(config_section, "JudgeReleasePreCountmax")
        self.judge_release_border_light = config.read_int(config_section, "JudgeReleaseBorderLight")
        self.judge_release_sleep_time = config.read_float(config_section, "JudgeReleaseSleepTime")
        self.stable_judge_count_release = config.read_int(config_section, "StableJudgeCountRelease") 
        
        #着地判定用定数
        self.judge_landing_maxtime = config.read_int(config_section, "JudgeLandingMaxtime")
        self.stable_judge_count_vel = config.read_int(config_section, "StableJudgeCountVel")
        self.stable_judge_count_land = config.read_int(config_section, "StableJudgeCountLand")
        self.stable_judge_max_failure = config.read_int(config_section, "StableJudgeMaxFailure")

    def judge_pressure_stable(self,interval_def_ave_pressure):
        stable_count = 0
        if self.phase == "Storage":
            for i in range(self.stable_judge_count_release):
                def_pre = self.pressure.dif_ave_pressure(interval_def_ave_pressure)
                # print(def_pre)####消す
                self.logger.write(f"Pressure_def: {def_pre}")
                if def_pre <= self.stable_pre_val:
                    stable_count += 1

                else:
                    break    
            
            if stable_count == self.stable_judge_count_release:
                return True
            else:
                return False
        if self.phase == "Released":
            for i in range(self.stable_judge_count_land):
                def_pre = self.pressure.dif_ave_pressure(interval_def_ave_pressure)
                # print(def_pre)####消す
                self.logger.write(f"Pressure_def: {def_pre}")
                if abs(def_pre) <= self.stable_pre_val:
                    stable_count += 1

                else:
                    break    
            
            if stable_count == self.stable_judge_count_land:
                return True
            else:
                return False


        
    async def judge_velocity_stable(self, interval_def_ave_velocity):
        stable_count = 0
        for i in range(self.stable_judge_count_vel):
            def_vel = await self.ac_vel.dif_ave_velocity(interval_def_ave_velocity)
            # print(def_vel)  ## 値の確認のための出力、必要ない場合はコメントアウトする
            self.logger.write(f"velocity_def: {def_vel}")

            if np.all(np.abs(def_vel) <= self.stable_vel_val):
                stable_count += 1
            else:
                break

        if stable_count == self.stable_judge_count_vel:
            return True
        else:
            return False
        
    def read_timer(self,time_sta): #時間計測用の関数
        time_end = time.perf_counter()# 時間計測終了
        self.tim = time_end - time_sta
        # print(tim)
        return self.tim
    
    def judge_storage(self):
        ##############################
        self.phase = "Outside"
        self.light_counter = 0 #counterを設定
        time_sta = time.perf_counter()
        ##############################
        while (self.read_timer(time_sta) <= self.judge_storage_maxtime):#300秒間実行

            if self.light.CANUSELIGHT == True:
                if self.light_counter < self.judge_storage_countmax:

                    light_value = self.light.get_light_value()

                    if light_value < self.judge_storage_border_light:#300以下であればcounterにプラス1
                        self.light_counter +=1
                        
                    else:
                        self.light_counter = 0#一回でも500以上であるならば外にいる判定
                        self.logger.write("Still Outside")
                        # print("Still Outside") #あとで消す
                        
                else:#100回連続で暗い判定ができたら中であると判定
                    break
            else:
                self.logger.write("CAN NOT USE LIGHT")
                # print("CAN NOT USE LIGHT")
            
            # print(light_value)
            # print(self.light_counter) #あとで消す
            time.sleep(self.judge_storage_sleep_time)
            self.logger.write(f"LIGHT: {light_value} {self.light_counter}")

        # print("Storage Succeeded")
        self.logger.write("Storage Succeeded")
        self.phase = "Storage"
        self.message = "Storage"
        self.STORAGE = True
        return True
    
    def judge_release(self):
        ##############################
        self.light_counter = 0
        self.pressure_counter = 0
        time_sta = time.perf_counter()
        ##############################
        while (self.read_timer(time_sta) <= self.judge_release_maxtime):
            
            if self.light.CANUSELIGHT == False:
                light_value = float('nan')
                self.light_counter = self.judge_release_lig_countmax
                self.logger.write("CAN NOT USE LIGHT")
                time.sleep(5)

            if self.pressure.CANUSEPRESSURE == False:
                pressure_value = float('nan')
                self.pressure_counter = self.judge_release_pre_countmax
                self.logger.write("CAN NOT USE PRESSURE")
                time.sleep(5)

            if (self.light_counter < self.judge_release_lig_countmax) and self.light.CANUSELIGHT == True:
                light_value = self.light.get_light_value()
                if light_value > self.judge_release_border_light:#明るい判定が出たらcounterに+1
                    self.light_counter +=1
            
                else:#10回たまらないうちに暗い判定が出たらリセット
                
                    self.light_counter = 0
                    self.logger.write("Light still low")
                    # print("Light still low")

                self.logger.write(f"LIGHT: {light_value} {self.light_counter}")


            if (self.pressure_counter < self.judge_release_pre_countmax) and self.pressure.CANUSEPRESSURE == True:
                Judge = self.judge_pressure_stable(1) 
                pressure_value = self.pressure.get_pressure()

                if Judge == False:#pressureが変化していたら
                    self.pressure_counter +=1
                
            
                else:#10連続で変化を観測できなかったらリセット
                    self.pressure_counter = 0
                    self.logger.write("Disp pressure too stable or minus")
                    # print("Disp pressure too stable or minus")
                
                self.logger.write(f"Pressure_counter: {self.pressure_counter}")
            
            if (self.light_counter >= self.judge_release_lig_countmax) and (self.pressure_counter >= self.judge_release_pre_countmax):
                break

            time.sleep(self.judge_release_sleep_time)

        self.logger.write("Release Succeeded")
        # print("Release Succeeded")
        self.phase = "Released"
        self.message = "Released"
    

    async def judge_landing(self):
        ##############################
        time_sta = time.perf_counter()
        ##############################
        
        self.logger.write("Pressure stability confirmation start")
        # print(f"Pressure stability confirmation start")

        while (self.read_timer(time_sta) <= self.judge_landing_maxtime):
            if self.pressure.CANUSEPRESSURE == True:
                if self.judge_pressure_stable(1): #5秒の測定の平均値を1秒ごとに計算
                    break
        
        self.logger.write("Pressure stability confirmed")
        # print(f"Pressure stability confirmed")

        await self.dronecontroller.connect()

        self.logger.write("Velocity stability confirmation start")
        # print(f"Velocity stability confirmation start")

        failure_count = 0
        while True:
            if await self.judge_velocity_stable(1):
                self.logger.write("Velocity stability cinfirmed")
                # print(f"Velocity stability cinfirmed")
                break
            else:
                self.logger.write("Velocity not stable.restart")
                # print(f"Velocity not stable.restart")
                failure_count += 1
            
            if failure_count >= self.stable_judge_max_failure:
                self.logger.write("Velocity stability forcibly confirmed")
                break
        
        self.logger.write("Landing Succeeded")
        # print("Landing Succeeded")
        self.phase = "Land"
        self.message = "Land"


    def para_case_stand_nichrome(self,nichrome_pin_no):
        self.wirehandler.nichrome_cut(nichrome_pin_no, self.nichrome_duration)


    def nichrome_cleanup(self):
        self.wirehandler.cleanup()    