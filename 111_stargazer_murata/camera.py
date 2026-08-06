from logging import getLogger, StreamHandler
import cv2
import numpy as np
from picamera2 import Picamera2


class Camera:
    def __init__(self, logger=None):
        if logger is None:
            logger = getLogger(__name__)
            logger.addHandler(StreamHandler())
            logger.setLevel(10)  # DEBUGレベル
        self._logger = logger

        self._picam2 = Picamera2()

        # フレームをBGR順で取得するように明示的に設定する。
        # これを指定しないとPicamera2はデフォルトでRGB系フォーマットを
        # 返すことがあり、cv2.cvtColor(..., cv2.COLOR_BGR2HSV) と
        # 組み合わせるとR/Bチャンネルが入れ替わり、赤と青を
        # 取り違えて検出してしまう原因になる。
        config = self._picam2.create_preview_configuration(main={"format": "RGB888"})
        self._picam2.configure(config)

    def start(self):
        """カメラを起動"""
        self._picam2.start()
        self._logger.info("Camera started")

    def red_detect(self, frame):
        # frameはBGR888設定により既にBGR順であることが保証されている
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 赤色のHSVの値域1
        hsv_min = np.array([0, 117, 104])
        hsv_max = np.array([11, 255, 255])
        mask1 = cv2.inRange(hsv, hsv_min, hsv_max)

        # 赤色のHSVの値域2
        hsv_min = np.array([169, 117, 104])
        hsv_max = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, hsv_min, hsv_max)

        return mask1 + mask2

    def analyze_red(self, frame, mask):
        camera_order = 0
        # 画像の中にある領域を検出する
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 画像の中に赤の領域があるときにループ
        if 0 < len(contours):

            # 輪郭群の中の最大の輪郭を取得する
            biggest_contour = max(contours, key=cv2.contourArea)

            # 最大の領域の外接矩形を取得する
            rect = cv2.boundingRect(biggest_contour)

            # 最大の領域の中心座標を取得する
            center_x = (rect[0] + rect[2] // 2)
            center_y = (rect[1] + rect[3] // 2)

            # 最大の領域の面積を取得する
            area = cv2.contourArea(biggest_contour)

            # 最大の領域の長方形を表示する
            cv2.rectangle(frame, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 0, 255), 2)

            # 最大の領域の中心座標を表示する
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

            # 最大の領域の面積を表示する
            cv2.putText(frame, str(area), (rect[0], rect[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)

            frame_center_x = frame.shape[1] // 2

            # 中心座標のx座標が画像の中心より大きいか小さいか判定
            if area > 7000:
                print("Close enough to red")
                camera_order = 4

            elif area > 10:
                if frame_center_x - 50 <= center_x <= frame_center_x + 50:
                    print("The red object is in the center")  # 直進
                    camera_order = 1
                elif center_x > frame_center_x + 50:
                    print("The red object is in the right")  # 右へ
                    camera_order = 2
                elif center_x < frame_center_x - 50:
                    print("The red object is in the left")  # 左へ
                    camera_order = 3

            else:
                print("The red object is too minimum")

        else:
            print("The red object is None")

        return frame, camera_order

    def judge_cone(self, show=False):
        """コーンの検出と判定"""
        frame = self._picam2.capture_array()
        mask = self.red_detect(frame)
        frame, camera_order = self.analyze_red(frame, mask)

        if show:
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)

        return camera_order

    def get_forever(self, camera_order, show=False):
        """カメラを起動し、コーンの検出を継続的に行う"""
        self.start()
        while True:
            camera_order.value = self.judge_cone(show)