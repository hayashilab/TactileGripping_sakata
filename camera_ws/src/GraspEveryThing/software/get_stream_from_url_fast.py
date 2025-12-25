import cv2
import csv
import time
import threading
import numpy as np
import urllib.request
from utils.utils import *  # poisson_reconstruct, find_marker, interpolate_grad など

# カメラのIPアドレスとポート番号を設定
IP = "192.168.1.120"
PORT = "8000"

DEPTH_TO_MM = 13.75


def IP_to_url(IP, PORT):
    return f"http://{IP}:{PORT}/cam_feed/0"


class FrameReader:
    def __init__(self, cap, sleep_on_fail=0.005):
        self.cap = cap
        self.sleep_on_fail = sleep_on_fail
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._frame = None
        self._ret = False
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(self.sleep_on_fail)
                continue
            with self._lock:
                self._ret = ret
                self._frame = frame

    def read(self):
        with self._lock:
            return self._ret, self._frame

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)


class GETTactileStream:
    def __init__(self, markers=False):
        self._ref_frame = None
        self.markers = markers  # Trueでマーカー補間（重い）

        # === REMOVED: corners / tactile_mask / mask生成関連 ===

    def _reset(self):
        self._ref_frame = None

    def _open_capture(self, url):
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    # === CHANGED: マスク処理を完全に削除 ===
    def _calc_diff_gray(self, ref_bgr, img_bgr, ksize=11, sigma=0):
        if ref_bgr is None or img_bgr is None:
            return None

        ref_g = cv2.cvtColor(ref_bgr, cv2.COLOR_BGR2GRAY)
        img_g = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        ref_blur = cv2.GaussianBlur(ref_g, (ksize, ksize), sigma)
        diff = (img_g.astype(np.float32) - ref_blur.astype(np.float32)) / 255.0 + 0.5
        return diff

    def _calc_diff_rgb(self, ref_bgr, img_bgr, ksize=11, sigma=0):
        if ref_bgr is None or img_bgr is None:
            return None

        ref_blur = cv2.GaussianBlur(ref_bgr, (ksize, ksize), sigma)
        diff = (img_bgr.astype(np.float32) - ref_blur.astype(np.float32)) / 255.0 + 0.5
        return diff

    # === CHANGED: マスク外ゼロ化を削除（全画素で勾配） ===
    def _img2grad(self, diff_img):
        if diff_img is None:
            return None, None

        dx = cv2.Sobel(diff_img, cv2.CV_32F, 1, 0, ksize=3)
        dy = cv2.Sobel(diff_img, cv2.CV_32F, 0, 1, ksize=3)
        return dx, dy

    # （任意）markers=Trueのときに勾配補間
    def demark_grad(self, diff_img_gray, dx, dy):
        # find_markerは3ch前提の実装なので、gray→BGR化して渡す
        diff_3ch = cv2.cvtColor(
            (np.clip(diff_img_gray, 0, 1) * 255).astype(np.uint8),
            cv2.COLOR_GRAY2BGR
        )
        mask = find_marker(diff_3ch)
        dx = interpolate_grad(dx, mask)
        dy = interpolate_grad(dy, mask)
        return dx, dy

    def grad2depth(self, diff_img_gray, dx, dy):
        if dx is None or dy is None:
            return None

        if self.markers:
            dx, dy = self.demark_grad(diff_img_gray, dx, dy)

        zeros = np.zeros_like(dx, dtype=np.float32)
        unitless_depth = poisson_reconstruct(dy, dx, zeros)
        depth_mm = DEPTH_TO_MM * unitless_depth
        return depth_mm.astype(np.float32)

    def img2depth(self, diff_img_gray):
        dx, dy = self._img2grad(diff_img_gray)
        return self.grad2depth(diff_img_gray, dx, dy)

    def start_stream(self, plot=True, plot_diff=True, plot_depth=True):
        url = IP_to_url(IP, PORT)
        cap = self._open_capture(url)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open stream: {url}")

        reader = FrameReader(cap)
        reader.start()

        self._reset()

        while True:
            ret, frame = reader.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            # 初回参照フレーム
            if self._ref_frame is None:
                self._ref_frame = frame.copy()

            if plot:
                cv2.imshow("raw_RGB", frame)

            diff_gray = None
            diff_rgb = None
            if plot_diff:
                diff_rgb = self._calc_diff_rgb(self._ref_frame, frame, ksize=11, sigma=0)
            if plot_depth:
                diff_gray = self._calc_diff_gray(self._ref_frame, frame, ksize=11, sigma=0)

            if plot_diff and diff_rgb is not None:
                diff_vis = np.clip(diff_rgb * 255.0, 0, 255).astype(np.uint8)
                cv2.imshow("diff_rgb", diff_vis)

            if plot_depth and diff_gray is not None:
                depth = self.img2depth(diff_gray)
                if depth is not None:
                    depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    cv2.imshow("depth", depth_vis)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                # 参照フレーム更新（照明変動対策）
                self._ref_frame = frame.copy()

        reader.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    GET_Stream = GETTactileStream(markers=False)  # markers=Trueで補間ON（重い）
    GET_Stream.start_stream(plot=True, plot_diff=True, plot_depth=True)
