import cv2
import csv
import time
import threading
import numpy as np
import warnings
import time 
import queue

from utils.utils import *  # poisson_reconstruct, find_marker, interpolate_grad など
from utils.Vis3D import ClassVis3D

# カメラのIPアドレスとポート番号を設定
IP = "192.168.1.120"
PORT = "8000"

# Constants
DEPTH_TO_MM = 13.75
PX_TO_MM = 16.6
DEPTH_THRESHOLD = 0.1
AUTO_CLIP_OFFSET = 10 # [frames]

def IP_to_url(IP, PORT):
    return f"http://{IP}:{PORT}/cam_feed/0"

# Threaded frame reader
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

# Thread save frames
class AsyncAviSaver:
    """
    OpenCV VideoWriter を別スレッドで回して .avi 保存するためのクラス。
    - ループ側は write(frame) するだけ
    - キューが満杯ならフレームを捨ててリアルタイム性を維持（drop_if_full=True）
    """

    def __init__(
        self,
        out_path: str,
        fps: float = 30.0,
        fourcc: str = "MJPG",
        max_queue: int = 128,
        drop_if_full: bool = True,
        resize_to: tuple[int, int] | None = None,  # (w,h)
        is_color: bool = True,
    ):
        self.out_path = out_path
        self.fps = float(fps)
        self.fourcc = cv2.VideoWriter_fourcc(*fourcc)
        self.max_queue = int(max_queue)
        self.drop_if_full = bool(drop_if_full)
        self.resize_to = resize_to
        self.is_color = is_color

        self._q = queue.Queue(maxsize=self.max_queue)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker, daemon=True)

        self._writer = None
        self._opened = False
        self._dropped = 0

    @property
    def dropped_frames(self) -> int:
        return self._dropped

    def start(self):
        self._thread.start()
        return self

    def _ensure_writer(self, frame: np.ndarray):
        # フレームサイズ確定（必要ならリサイズ後のサイズで保存）
        if self.resize_to is not None:
            w, h = self.resize_to
        else:
            h, w = frame.shape[:2]

        self._writer = cv2.VideoWriter(self.out_path, self.fourcc, self.fps, (w, h), self.is_color)
        if not self._writer.isOpened():
            raise RuntimeError(f"VideoWriter open failed: path={self.out_path}")

        self._opened = True

    def write(self, frame: np.ndarray):
        """メインループ側から呼ぶ。キューに積むだけ。"""
        if frame is None:
            return

        # グレースケールが来た場合、VideoWriterがcolor前提ならBGR化
        if frame.ndim == 2 and self.is_color:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # 保存時のサイズに合わせる（CPU節約したい場合はresize_toを指定）
        if self.resize_to is not None:
            w, h = self.resize_to
            if frame.shape[1] != w or frame.shape[0] != h:
                frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)

        try:
            if self.drop_if_full:
                self._q.put_nowait(frame)
            else:
                self._q.put(frame)  # 満杯だと待つ（＝メインが詰まりやすい）
        except queue.Full:
            self._dropped += 1

    def _worker(self):
        while not self._stop.is_set() or not self._q.empty():
            try:
                frame = self._q.get(timeout=0.1)
            except queue.Empty:
                continue

            if not self._opened:
                self._ensure_writer(frame)

            # 実際の書き込み
            self._writer.write(frame)
            self._q.task_done()

        # 終了処理
        if self._writer is not None:
            self._writer.release()

    def close(self):
        """終了時に呼ぶ。キューを吐き切ってから閉じる。"""
        self._stop.set()
        self._thread.join(timeout=2.0)
        # 念のため
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception:
                pass
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
        diff_img = (img_g.astype(np.float32) - ref_blur.astype(np.float32)) / 255.0 + 0.5
        diff_img = np.clip(diff_img, 0.0, 1.0)
        return diff_img

    def _calc_diff_rgb(self, ref_bgr, img_bgr, ksize=11, sigma=0):
        if ref_bgr is None or img_bgr is None:
            return None

        ref_blur = cv2.GaussianBlur(ref_bgr, (ksize, ksize), sigma)
        diff = (img_bgr.astype(np.float32) - ref_blur.astype(np.float32)) / 255.0 + 0.5
        return diff

    # === CHANGED: マスク外ゼロ化を削除（全画素で勾配） ===
    def _img2grad(self, diff_img, tactile_mask=None, eps=1e-6):
        """
        diff_img: float32 (H,W,3), 値域 [0,1], 中立 0.5
        tactile_mask: (H,W) 0/1 or bool. Noneならマスクなし。
        return: dx, dy float32 (H,W)
        """
        if diff_img is None:
            return None, None

        # mask外を0.5に固定（中立）
        if tactile_mask is not None:
            d = diff_img.copy()
            d[tactile_mask == 0] = 0.5
        else:
            d = diff_img

        # OpenCV(BGR)としてチャンネルを取り出し
        B = d[:, :, 0]
        G = d[:, :, 1]
        R = d[:, :, 2]

        # ---- Aの定義（色差→傾き）----
        dx = (B - (R + G) * 0.5)     # 元コードの index をBGRに合わせて解釈した例
        dy = (R - G)

        # 数値安定化：sqrt(1 - dx^2) の中が負にならないようクリップ
        dx = np.clip(dx, -0.999, 0.999)
        dy = np.clip(dy, -0.999, 0.999)

        dx = dx / (np.sqrt(1.0 - dx * dx) + eps) / 128.0
        dy = dy / (np.sqrt(1.0 - dy * dy) + eps) / 128.0

        return dx.astype(np.float32), dy.astype(np.float32)

    # （任意）markers=Trueのときに勾配補間
    def _demark_grad(self, diff_img, dx, dy):
        if diff_img is None:
            return dx, dy

        if diff_img.ndim == 2:
            diff_3ch = cv2.cvtColor(diff_img.astype(np.float32), cv2.COLOR_GRAY2BGR)
        else:
            diff_3ch = diff_img.astype(np.float32)

        mask = find_marker(diff_3ch)
        dx = interpolate_grad(dx, mask)
        dy = interpolate_grad(dy, mask)
        return dx, dy

    def _grad2depth(self, diff_img, dx, dy):
        if dx is None or dy is None:
            return None

        if self.markers:
            dx, dy = self._demark_grad(diff_img, dx, dy)

        zeros = np.zeros_like(dx, dtype=np.float32)
        unitless_depth = poisson_reconstruct(dy, dx, zeros)
        depth_mm = DEPTH_TO_MM * unitless_depth
        return depth_mm.astype(np.float32)

    def _img2depth(self, diff_img):
        dx, dy = self._img2grad(diff_img)
        return self._grad2depth(diff_img, dx, dy)
            
    def start_stream(self, plot=True, plot_diff=True, plot_depth=True):
        url = IP_to_url(IP, PORT)
        cap = self._open_capture(url)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open stream: {url}")

        reader = FrameReader(cap)
        reader.start()
        
        saver = AsyncAviSaver(out_path="./videos/tactile_stream.avi", fps=30.0, fourcc = "MJPG", max_queue = 128, drop_if_full=True, resize_to = None).start()

        self._reset()
        vis3d = None

        print("Streaming...")
        while True:
            ret, frame = reader.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue
            
            saver.write(frame)

            # 初回参照フレーム
            if self._ref_frame is None:
                self._ref_frame = frame.copy()

            if plot:
                cv2.imshow("raw_RGB", frame)

            diff_rgb = None
            if plot_diff or plot_depth:
                diff_rgb = self._calc_diff_rgb(self._ref_frame, frame, ksize=11, sigma=0)

            if plot_diff and diff_rgb is not None:
                diff_vis = np.clip(diff_rgb * 255.0, 0, 255).astype(np.uint8)
                cv2.imshow("diff_rgb", diff_vis)

            if plot_depth and diff_rgb is not None:
                depth = self._img2depth(diff_rgb)
                if depth is not None:
                    if vis3d is None:
                        vis3d = ClassVis3D(n=depth.shape[1], m=depth.shape[0])
                    vis3d.update(depth / PX_TO_MM)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                # 参照フレーム更新（照明変動対策）
                self._ref_frame = frame.copy()
                
        saver.close()
        reader.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    GET_Stream = GETTactileStream(markers=False)  # markers=Trueで補間ON（重い）
    GET_Stream.start_stream(plot=True, plot_diff=True, plot_depth=True)
