import cv2
import numpy as np
import csv
import time
import cv2
import numpy as np
import time
import csv
import warnings

from utils.utils import *
from utils.Vis3D import ClassVis3D

# 基本設定(2025-12-17)
IP = "192.168.1.120"
PORT = "8000"
###


def read_csv(filename="software/config.csv"):
    rows = []
    with open(filename, "r") as csvfile:
        csvreader = csv.reader(csvfile)
        _ = next(csvreader)
        for row in csvreader:
            rows.append((int(row[1]), int(row[2])))
    return rows

def IP_to_url(IP, PORT):
    return f"http://{IP}:{PORT}/cam_feed/0"

class GETTactileStream():
    def __init__(self, config_csv="software/config.csv", IP=None, PORT=None):
        self.corners = read_csv(config_csv) 
        self.cropped_size = (640, 480)

    def start_stream(self, plot=False, plot_diff=False):
        # ===== ユーザ設定 =====
        url = IP_to_url(IP, PORT)
        out_size = (640, 480)  # (w,h) 表示/処理解像度。config.csv の座標もこの座標系に合わせる前提です。
        # =====================

    # 1) config.csv からマスク作成
        corners = self.corners

        # 2) URL からフレーム取得
        cap = cv2.VideoCapture(url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 効かない環境もありますが入れておく

        if not cap.isOpened():
            raise RuntimeError(f"Cannot open stream: {url}")

        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            # 3) 解像度を out_size に統一（マスク座標系と一致させる）
            frame = cv2.resize(frame, out_size)  # out_size は (w,h) で渡せます

            cv2.imshow("raw_RGB", frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break

        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    GET_Stream = GETTactileStream()
    GET_Stream.start_stream(plot=True, plot_diff=False)