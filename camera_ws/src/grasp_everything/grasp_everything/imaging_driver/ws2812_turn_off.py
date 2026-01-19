#!/usr/bin/env python3
"""LEDを強制消灯するスクリプト"""
from rpi_ws281x import PixelStrip, Color

LED_COUNT = 48
LED_PIN = 18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 20
LED_INVERT = False

strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
strip.begin()

for i in range(strip.numPixels()):
    strip.setPixelColor(i, Color(0, 0, 0))
strip.show()

print("LEDs turned off.")
