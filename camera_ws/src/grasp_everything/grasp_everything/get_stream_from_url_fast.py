import cv2
import csv
import time
import threading
import numpy as np
import warnings
import time 
import queue

# カメラのIPアドレスとポート番号を設定
IP = "192.168.1.120"
PORT = "8000"

# Constants
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

### ストリーミングクラス
class GETTactileStream:
    def __init__(self):
        self._ref_frame = None

    def _reset(self):
        self._ref_frame = None

    #--- 動画キャプチャオープン ---
    #--- 2026-01-16 loreで640x480で受け取るようにimaging_driver側を修正 ---
    def _open_capture(self, url):
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(url)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap
    
    # --- diff計算 ---
    def _calc_diff_gray(self, ref_bgr, img_bgr, ksize=11, sigma=0):
        if ref_bgr is None or img_bgr is None:
            return None

        ref_g = cv2.cvtColor(ref_bgr, cv2.COLOR_BGR2GRAY)
        img_g = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        ref_blur = cv2.GaussianBlur(ref_g, (ksize, ksize), sigma)
        diff_img = (img_g.astype(np.float32) - ref_blur.astype(np.float32)) / 255.0 + 0.5
        diff_img = np.clip(diff_img, 0.0, 1.0)
        return diff_img
    
    # --- diff計算 RGB版 ---    
    def _calc_diff_rgb(self, ref_bgr, img_bgr, ksize=11, sigma=0):
        if ref_bgr is None or img_bgr is None:
            return None

        ref_blur = cv2.GaussianBlur(ref_bgr, (ksize, ksize), sigma)
        diff = (img_bgr.astype(np.float32) - ref_blur.astype(np.float32)) / 255.0 + 0.5
        return diff

    def start_stream(self, plot=True, plot_diff=True):
        url = IP_to_url(IP, PORT)
        cap = self._open_capture(url)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open stream: {url}")

        reader = FrameReader(cap)
        reader.start()
        
        saver = AsyncAviSaver(out_path="./videos/tactile_stream.avi", fps=30.0, fourcc = "MJPG", max_queue = 128, drop_if_full=True, resize_to = None).start()

        self._reset()
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
            if plot_diff:
                diff_rgb = self._calc_diff_rgb(self._ref_frame, frame, ksize=11, sigma=0)

            if plot_diff and diff_rgb is not None:
                diff_vis = np.clip(diff_rgb * 255.0, 0, 255).astype(np.uint8)
                cv2.imshow("diff_rgb", diff_vis)

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
    GET_Stream = GETTactileStream()
    GET_Stream.start_stream(plot=True, plot_diff=True)
