import sys
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent.parent.joinpath("assets/module")))

from pathlib import Path
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FfmpegOutput
import cv2
import numpy as np
import threading
import cone_detector
import time

import multiprocessing

class CameraHandler:
    _unique_instance = None

    def __new__(self):
        raise NotImplementedError("Cannot generate instance by constructor. Call get_instance() method instead.")
    
    @classmethod
    def __internal_new__(self):
        self._is_connected = False

        try:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration({ "format": "RGB888" })
            self.camera.configure(config)
            self.camera.start()
        except Exception as e:
            print(f"Cannot connect the camera.")
        else:
            self._is_connected = True
            camera_config = self.camera.create_preview_configuration()
            self.width = camera_config["main"]["size"][0]
            self.height = camera_config["main"]["size"][1]
        finally:
            return super().__new__(self)

    @classmethod
    def get_instance(self):
        if self._unique_instance is None:
            self._unique_instance = self.__internal_new__()
        return self._unique_instance
    
    def get_width(self):
        return self.width
    
    def get_height(self):
        return self.height

    def is_connected(self):
        return self._is_connected
        
    def capture_bgr(self):
        image = self.camera.capture_array()
        return cv2.rotate(image, cv2.ROTATE_180) # カメラの取り付け上下が逆になってることを考慮

    def capture_rgb(self):
        frame = self.camera.capture_array()
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return cv2.rotate(image, cv2.ROTATE_180) # カメラの取り付け上下が逆になってることを考慮
    
    def capture_and_save(self, path = "image.jpg"):
        image = self.capture_bgr()
        cv2.imwrite(path, image)
    
    def capture_video(self, output_path, length = 10):
        def video_capturer(self, output_path, length):
            video_config = self.camera.create_video_configuration()
            self.camera.stop()
            self.camera.configure(video_config)

            encoder = H264Encoder(10000000)
            output = FfmpegOutput(output_path)

            try:
                self.camera.start_recording(encoder, output)
                time.sleep(length)
                self.camera.stop_recording()
                self.camera.start()
            except Exception as e:
                print("Error has occured. Stopped capturing video")
                print(e)
        
        video_thread = threading.Thread(target = video_capturer, args = (self, output_path, length,))
        video_thread.start()

class ConeDetector:
    IOU_THRESHOLD = 0.1

    def __init__(self, camera_handler: CameraHandler, model_path: str = Path(__file__).parent.parent.parent.joinpath("assets/model/cone_ncnn_model_v9_320_opt"), imgsz = 320):
        cone_detector.load_model(str(model_path), imgsz)
        self.camera_handler = camera_handler
        self.frame = None
        self.pos = multiprocessing.Array("f", 2)
        self.pos[0] = -2
        self.pos[1] = -2
        self.started = False
        self.finished = multiprocessing.Value("b", 0)
    
    def get_pos(self):
        pos = [None, None]
        pos[0] = self.pos[0]
        pos[1] = self.pos[1]
        if pos[0] < -1:
            pos = [None, None]
        
        return pos
    
    def calc_color_center(self, frame):
        if frame is None:
            return [None, None]
        
        LOW_COLOR_1 = np.array([0, 100, 100])
        HIGH_COLOR_1 = np.array([20, 255, 255])

        LOW_COLOR_2 = np.array([160, 100, 100])
        HIGH_COLOR_2 = np.array([180, 255, 255])

        AREA_RATIO_THRESHOLD = 0.0002  ####最初0.002

        height = frame.shape[0]
        width = frame.shape[1]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ex_img = cv2.inRange(hsv, LOW_COLOR_1, HIGH_COLOR_1) + cv2.inRange(hsv, LOW_COLOR_2, HIGH_COLOR_2)
        contours, hierarchy = cv2.findContours(ex_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        areas = np.array(list(map(cv2.contourArea, contours)))
        if len(areas) == 0 or np.max(areas) / (height * width) < AREA_RATIO_THRESHOLD:
            return [None, None]
        else:
            max_index = np.argmax(areas)
            result = cv2.moments(contours[max_index])
            x = result["m10"] / result["m00"]
            y = result["m01"] / result["m00"]
            return [x / width * 2 - 1, 1 - y / height * 2]
    
    def detector(self, conf = IOU_THRESHOLD, use_color_assist = False):
        while self.finished.value == 0:
            print("capturing...")
            frame = self.camera_handler.capture_bgr()
            print("captured")

            pos = cone_detector.get_pos(frame, conf)
            print("hello")

            if pos[0] is None and use_color_assist: # 機械学習による推論で物体推定ができなかった場合色情報をもとに推定を行う
                color_pos = self.calc_color_center(frame)
                if color_pos[0] is not None:
                    print("found cone using color assist")
                pos = color_pos

            self.pos[0] = pos[0]
            self.pos[1] = pos[1]

        self.finished.value = 0

    def start(self, conf = IOU_THRESHOLD, use_color_assist = False):
        if self.started:
            return
        
        process = multiprocessing.Process(target = self.detector, args = (conf, use_color_assist,), daemon = True)
        process.start()
        self.started = True
    
    def stop(self):
        self.finished.value = 1
        self.started = False

    def capture_cone_position(self, conf = IOU_THRESHOLD, use_color_assist = False):
        image = self.camera_handler.capture_bgr()

        if not image is None:
            result = cone_detector.get_pos(image, conf)
            if result[0] >= -1:
                return [result[0], result[1]]
            elif use_color_assist:
                return self.calc_color_center(image)
            else:
                return [None, None]
        else:
            return [None, None]

    def capture_cone_position_and_save(self, output_path, conf = IOU_THRESHOLD, use_color_assist = False):
        image = self.camera_handler.capture_bgr()

        if not image is None:
            result = cone_detector.get_pos(image, conf)
            color = (255, 255, 255)
            
            if result[0] >= -1:
                pass
            elif use_color_assist:
                result = self.calc_color_center(image)
                color = (0, 255, 255)
            else:
                result = [None, None]

            self.draw_circle_and_save(image, result[0], result[1], output_path, color)
            return result
        else:
            return [None, None]
    
    def draw_circle_and_save(self, image, normalized_x, normalized_y, output_path, color = (255, 255, 255)):
        height = image.shape[0]
        width = image.shape[1]
        result = image
        if not (normalized_x is None or normalized_y is None):
            x = int((normalized_x + 1) / 2 * width)
            y = int((-normalized_y + 1) / 2 * height)
            result = cv2.circle(image, (x, y), 25, color, thickness = 5)
        cv2.imwrite(output_path, result)
        return result


# def test1():
#     camera_handler = CameraHandler()
#     cone_detector = ConeDetector(camera_handler, Path(__file__).parent.parent.parent.joinpath("assets/model/cone_ncnn_model_v9_320"), 320)
#     cone_detector.start()
#     start = time.perf_counter()
#     while True:
#         time.sleep(0.1)
#         pos = cone_detector.get_pos()
#         now = time.perf_counter()
#         print(f"{now - start}s {pos}\n")
#         if now - start > 15:
#             break
#     cone_detector.stop()

# def test2():
#     camera_handler = CameraHandler()
#     image = camera_handler.capture_rgb()
#     cv2.imwrite("capture.png", image)

# def test3():
#     camera_handler = CameraHandler()
#     camera_handler.capture_video("test.mp4")

#test3()