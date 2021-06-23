## M5ATOM Matrix ESP-IDF sample

This is a sample application of M5ATOM Matrix using IMU(MPU6886),
LEDs(WS2812C) and button. MPU6886 driver is taken and modified
from the [original arduino M5ATOM library](https://github.com/m5stack/M5Atom).
For LEDs, [NeoPixel LED driver for ESP-IDF](https://github.com/ogochan/neopixel-idf) is used. AHRS is a geometric algebra based complementary filter.

It displays AHRS result with setting pure quaternion components to RGB values.
When button pushed, led displays "heart beat".

### ChangeLog

[2021 Jun 22] Adjust for ESP-IDF 4.4.

[2020 Jun 12] Initial commit.
