import bme680
import time

class PressureHandler:
    def __init__(self):
        try:
            self.sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
            self.CANUSEPRESSURE = True
        except (RuntimeError, IOError):
            try:
                self.sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)
                self.CANUSEPRESSURE = True
            except (RuntimeError, IOError):
                print("Error: Pressure sensor not found.")
                self.CANUSEPRESSURE = False
        if self.CANUSEPRESSURE:
            self.sensor.set_pressure_oversample(bme680.OS_4X)
            self.sensor.set_temperature_oversample(bme680.OS_8X)
            self.sensor.set_filter(bme680.FILTER_SIZE_3)

        self.error_count = 0
        self.max_errors = 3
        self.interval = 1
    
    def get_temperature(self):
        if self.sensor.get_sensor_data():
            return self.sensor.data.temperature
        else:
            return None
        
    
    def get_pressure(self): #このif文で例外処理ができているか??
        if self.sensor.get_sensor_data():
            return self.sensor.data.pressure
        else:
            self.error_count += 1
            if self.error_count >= self.max_errors:
                self.CANUSEPRESSURE = False
            return 0
        
        
    def ave_pressure(self):
        pressure_lst = []
        for i in range(5):
            pre = self.get_pressure()
            print(pre)#####消す
            pressure_lst.append(pre)
            time.sleep(self.interval)

            if i == 4:
                ave_pre = sum(pressure_lst)/len(pressure_lst)

        return ave_pre
    
    def dif_ave_pressure(self,interval_def_ave_pressure):
        ave_pre_before = self.ave_pressure()
        time.sleep(interval_def_ave_pressure)
        ave_pre_after = self.ave_pressure()
        return ave_pre_after - ave_pre_before