import time
import cv2
import numpy as np
from picamera2 import Picamera2

class Camera:
    def __init__(self):
        try:
            self.picam2 = Picamera2()
            self.picam2.configure(self.picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
        except Exception as e:
            print(f"Error initializing camera: {e}")
            self.picam2 = None
        self.hsv_min1 = np.array([0, 180, 180])
        self.hsv_max1 = np.array([10, 255, 255])
        self.hsv_min2 = np.array([170, 180, 180])
        self.hsv_max2 = np.array([179, 255, 255])
    def start(self):
        if self.picam2 is not None:
            self.picam2.start()
        else:
            print("Camera not initialized. Cannot start.")
    def capture_image(self):
        if self.picam2 is not None:
            return self.picam2.capture_array()
        else:
            print("Camera not initialized. Cannot capture image.")
            return None

    def release(self):
        if self.picam2 is not None:
            self.picam2.stop()
    def detect_cone(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(image, self.hsv_min1, self.hsv_max1)
        mask2 = cv2.inRange(image, self.hsv_min2, self.hsv_max2)
        image = cv2.bitwise_or(mask1, mask2)
        return image
    def histogram_equalization(self, image):
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        h, s, v = cv2.split(image)
        v = clahe.apply(v)
        return cv2.merge((h, s, v))
    def get_cone_position(self, image):
        camera_order = 0
        cx = None
        cy = None
        #モルフォロジー変換
        kernel = np.ones((5,5), np.uint8)
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)
        #輪郭抽出
        counts, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if counts:
            largest_contour = max(counts, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            cap_cx = image.shape[1] // 2
            print(f"Largest contour area: {area}")  # デバッグ用 - 面積を表示
            M = cv2.moments(largest_contour)
            if M["m00"] > 500:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if area > 150000:
                    print("Close enough to Corn, stopping")
                    camera_order = 4
                elif cx < cap_cx - 100:
                    print("Corn is on the left")
                    camera_order = 3
                elif cx > cap_cx + 100:
                    print("Corn is on the right")
                    camera_order = 2
                else:
                    print("Corn is in the center")
                    camera_order = 1
        else:
            cx, cy = None, None
            print("No Corn detected, Searching for Corn")
            camera_order = 0
        return cx, cy, image, camera_order
if __name__ == "__main__":
    camera = Camera()
    camera.start()
    while True:
        try:
            cap = camera.capture_image()
            if cap is None:
                print("Failed to capture image. Exiting.")
                break
            cap = camera.histogram_equalization(cap)
            cap = camera.detect_cone(cap)
            cx, cy, _, camera_order = camera.get_cone_position(cap)
            cv2.waitKey(1)  # Adjust the wait time as needed
            print(cx, cy, camera_order)
        except Exception as e:
            print(f"Error during processing: {e}")
    camera.release()
    cv2.destroyAllWindows()