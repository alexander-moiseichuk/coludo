# Main hardware

Preliminary list before checking weight, as it should fit to 100 gramms limitation for F6 engine.
Another important limitation is power consumption which nice to keep under 5V 0.7A as per LM7805 stabilizer.

## Platform

To have good enough performance (as micropython is running on single core) the esp32-p4 selected as potential working
solution: [FireBeetle 2 ESP32-P4 AI Development Kit MIPI CSI DSI Wi-Fi 6 and IO Expansion Board](https://www.dfrobot.com/product-2950.html).
WiFi is a nice option to have remote (telnet) console before and for some period of time during launch.

If this unit will be not enough [low-end ARM](https://www.cnx-software.com/2025/10/31/sakura-pi-rk3308b-sbc-offers-rgb-lcd-interface-supports-mainline-linux/) 
could be an option but may lead too much battery pack weigh.

**Required, weight 30.5g**

## Accelerometer and Gyro
Something on [MPU6050](https://www.amazon.com/dp/B0BMY15TC4?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_13) could be acceptable.

**Required, weight 3.2g**

## Altimeter (pressure)
As stated in docs the [Gravity: ICP-10111 Pressure Sensor](https://www.dfrobot.com/product-2525.html) looks promising for accuracy (8.5cm), 
sampling rate and power consumption (2mA).

**Required, weight 7.1g**

## Altimeter (laser)
Ultrasonic sensors are not suitable for 4m+ distance, but lasers are power hungry, so should be turned on on landing.
For now will rely on athmo altimeter but [50m TOF Laser Ranging Sensor, 100Hz](https://www.dfrobot.com/product-2923.html)
looks like a promising option taking into account price and [technical specification](https://wiki.dfrobot.com/SKU_SEN0648_TOF_laser_ranging_sensor_50m)

**Very optional as power consumption needs another battery, weight ~20g**

## Battery
Not many options for [5V USB-C low-weight](https://www.amazon.com/dp/B07SZKNST4) power delivery are avaialbe.
Need to clarify after checking weight for [3.7V batteries](https://www.amazon.com/dp/B0F7QJ4BVK)

Alternative is to connect e.g. from [6F22 9V using plug](https://www.amazon.com/dp/B083QFFH66) and [some LM7805 voltage stabilizer](https://www.amazon.com/dp/B00LTQTZYQ)

**Required, weight 42.1g for 6F22 and LM7805 board**

## Button 
Button is required for detection stage separation, when active part (engine work) completed and booster throws away
glider and open parashute. As a button seems [Gravity: Digital Crash Sensor](https://www.dfrobot.com/product-763.html) feasible,
it will be in pressed state during start and opened after separation, so required to be set in Pull-Up mode as waiting
on start will be much longer then flight, so when button will be connected to ground (LOW) when pressed (no separation),
and get HIGH when not pressed (separated). In theory it will save some milliwats.

**Required, weight 6.2g**

## Camera
Some simple camera under 4K nice to have, ideally with autofocus. Used [Camera for Raspberry Pi](https://www.dfrobot.com/product-1179.html)
just because it was in shop to fit free delivery.

**Optional, weight 4.5g**

## Navigation
As accelerometer might be not enough for landing into proper zone and glissade, the 
[Teyleten Robot ATGM336H GPS+BDS Dual-Mode Module Flight Control Satellite Positioning Navigator](https://www.amazon.com/dp/B09LQDG1HY)
will be helpful as has [low weight, sufficient accuracy, <30 mA power consumption and up to 10 Hz update rate](https://docs.cirkitdesigner.com/component/ab5c0c19-2fd9-4121-964e-1009970a950a/gps-atgm336h)

**Required, weight 7.4g**

## Servos for fins
[SG90 is a popular variation](https://www.amazon.com/Micro-Servos-Helicopter-Airplane-Controls/dp/B07MLR1498) for such needs. But any other options seems doable.
In case of too high power consumption peaks the sequential control might be required i.e. not turning all engines in the same time.

**Required, weight 10.6g per each engine and wires, at least 2 are required**

## SD card
Any suitable by size and throughput as code, videos and logs will be written here. 

**Optional (only for camera), weight 1g**

# Auxillary hardware

## Logic analyser
Just a popular and sufficient unit to check what is happening [HiLetgo USB Logic Analyzer Device with EMI Ferrite Ring USB Cable 24MHz 8CH 24MHz 8 Channel UART IIC SPI Debug](https://www.amazon.com/dp/B077LSG5P2)
Some set of [Goupchn SMD IC Test Hook Clips 10PCS 10 Colors for Logic Analyzer](https://www.amazon.com/dp/B0D3ZWTCW4) will speedup process.

## Power meter 
For periodic checks during development how much power consumed something like [USB C Tester Power Meter](https://www.amazon.com/dp/B0DFBSFL38).
If device allow to pass commands over USB it will be much better as will allow to control situation when wifi console is off.


# Potential configurations

There are a number of composition options possible

| Engines | Components configuration         | Weight [g] | Notes                                                                            |
| ------- | -------------------------------- | ---------- | -------------------------------------------------------------------------------- |
|    2    | required only components         |   117.7    | very minimalistic version without video recording                                |
|    2    | required and optional components |   123.2    | this one is plan B if glider will go out of weight or power consumption targets  |
|    3    | required only components         |   128.3    | more control but no video, not much difference to plan A                         |
|    3    | required and optional components |   133.8    | this one working target configuration as plan A                                  |
|    4    | required only components         |   138.9    | no idea how it will be useful                                                    |
|    4    | required and optional components |   144.4    | rich case, controllable on top level, but could be issues with power consumption |

**Note:** sticky pads for attaching boards and engines furniture not added
