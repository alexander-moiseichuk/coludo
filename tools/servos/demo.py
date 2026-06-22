# Copyright (c) 2025 Alex Yeryomin
# The demo program for ESP32-С3 with OLED display 72x40 pixels.

from machine import Pin, I2C
from time import sleep
import ssd1306
from random import randint

i2c = I2C(0, sda=Pin(5), scl=Pin(6), freq=400000)
#display = SH1106_I2C(128, 64, i2c_display)
display = ssd1306.SSD1306_I2C(128, 64, i2c)
display.contrast(255)

# SSD1306 controller has 132x64 pixel buffer
BufferWidth, BufferHeight = 132, 64
ScreenWidth, ScreenHeight = 72, 40
xOffset, yOffset = (BufferWidth - ScreenWidth) // 2, (BufferHeight - ScreenHeight) // 2

while True:
    x, y = randint(0, ScreenWidth - 1), randint(0, ScreenHeight - 1)
    dx, dy = randint(-3, 3), randint(-3, 3)

    display.invert(0)
    display.fill(0)

    for _ in range(200):
        display.fill(0)
        display.text('x = ' + str(x) + ' y = ' + str(y), xOffset+10,yOffset + 10, 1)
        display.ellipse(xOffset + x, yOffset + y, 3, 3, 1)
        display.show()
        x += dx
        if x < 0 or x >= ScreenWidth:
            dx = -dx
        y += dy
        if y < 0 or y >= ScreenHeight:
            dy = -dy
        
    for i in range(6):
        display.invert(i % 2)
        display.show()
        sleep(0.2)
 
