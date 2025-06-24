import RPi.GPIO as GPIO
import time

class WireHandler():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)

    def nichrome_cut(self, pin_no, duration):
        GPIO.setup(pin_no, GPIO.OUT)  
        GPIO.output(pin_no, GPIO.HIGH)  # ニクロム線に電流を流す（スイッチオン）
        time.sleep(duration)  # duration秒間継続
        GPIO.output(pin_no, GPIO.LOW)  # ニクロム線の電流を止める（スイッチオフ）
        time.sleep(0.5)

    def cleanup(self):
        GPIO.cleanup()
