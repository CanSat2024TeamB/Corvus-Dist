import spidev

class LightSensor:
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 1000000
        self.error_count = 0
        self.max_errors = 10
        self.CANUSELIGHT = True

    def get_light_value(self):
        try:
            # SPI通信で値を読み込む
            resp = self.spi.xfer2([0x68, 0x00])
            light_value = ((resp[0] << 8) + resp[1]) & 0x3FF
            # 値が0の場合でもエラーチェックを行う
            if light_value == 0:
                self.error_count += 1
                if self.error_count >= self.max_errors:
                    self.CANUSELIGHT = False
                print("Error reading light intensity: {e}")
                return float('nan')
            return light_value
        except Exception as e:
            self.error_count += 1
            if self.error_count >= self.max_errors:
                self.CANUSELIGHT = False
            print("Error readting light intensity: {e}")
            return float('nan')

    def close(self):
        # SPI通信を終了する
        self.spi.close()

# # 使用例
# if __name__ == "__main__":
#     sensor = LightSensor()
#     try:
#         while True:
#             value = sensor.read_value()
#             print(f"Light Sensor Value: {value}")
#             time.sleep(1)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         sensor.close()
