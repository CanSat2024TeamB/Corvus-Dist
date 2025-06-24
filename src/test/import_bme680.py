import bme680
import time

# BME680センサーの初期化
try:
    sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)
except IOError:
    sensor = bme680.BME680(bme680.I2C_ADDR_SECONDARY)

# センサーの設定
sensor.set_humidity_oversample(bme680.OS_2X)
sensor.set_pressure_oversample(bme680.OS_4X)
sensor.set_temperature_oversample(bme680.OS_8X)
sensor.set_filter(bme680.FILTER_SIZE_3)

# データ取得ループ
try:
    while True:
        if sensor.get_sensor_data():
            temperature = sensor.data.temperature
            pressure = sensor.data.pressure
            humidity = sensor.data.humidity

            print(f'Temperature: {temperature:.2f} C')
            print(f'Pressure: {pressure:.2f} hPa')
            print(f'Humidity: {humidity:.2f} %')

            time.sleep(1)

except KeyboardInterrupt:
    pass