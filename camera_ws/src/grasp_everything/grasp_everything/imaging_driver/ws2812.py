import time
import signal
import sys
from rpi_ws281x import PixelStrip, Color

# 各セグメントのLED数
LED_COUNT_G = 18 # 0-17
LED_COUNT_W = 6   # 18-23
LED_COUNT_R = 18 # 24-41
LED_COUNT_B = 6   # 42-47

LED_COUNT = LED_COUNT_G + LED_COUNT_W + LED_COUNT_R + LED_COUNT_B  # 48

LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 20
LED_INVERT = False

strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
strip.begin()

def all_off():
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, Color(0, 0, 0))
    strip.show()

def handle_stop(signum, frame):
    all_off()
    print("ws2812.py stopped. LEDs off.")
    sys.exit(0)

# strip が初期化された後に signal を登録
signal.signal(signal.SIGINT, handle_stop)
signal.signal(signal.SIGTERM, handle_stop)

def set_range(start, end, color):
    """start〜end を含む範囲に color を設定"""
    for i in range(start, end + 1):
        strip.setPixelColor(i, color)

def show_segments():
    # 0-17: 緑
    set_range(0, 17, Color(0, 255, 0))
    # 18-23: 白
    set_range(18, 23, Color(255, 255, 255))
    # 24-41: 赤
    set_range(24, 41, Color(255, 0, 0))
    # 42-47: 青
    set_range(42, 47, Color(0, 0, 255))
    strip.show()

try:
    show_segments()
    print("Segments set. Press Ctrl+C to turn off and exit.")
    while True:
        time.sleep(1)

finally:
    all_off()