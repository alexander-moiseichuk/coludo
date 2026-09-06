# External Pinout for Main Board
For context all of the wiring will be interpreted with a reference frame of looking at the board from 
the top and all of the letters are facing the correct side
ie. they are able to be read normally. 

> **This lane is `i2c:1`, not `i2c:0`.** GPIO 31/30 is the SECOND I2C controller; `i2c:0` is GPIO 7/8
> and stays on the board (BNO055, BMP280, INA226). A config naming `id: 0` for these devices scans
> clean and never binds, which is a slow failure to find.

## Frontal Grid of Main Board
The frontal grid of the main board is used to connect the following items:
- Recorder RX Pin
- Laser Connection
- Pito Tube/Air Pressure

| Pin number | Signal | GPIO | Bus / role |
|---|---|---|---|
| 1 | GND | - | Ground from MCU | 
| 2 | 3v3 | - | Power from MCU |
| 3 | **I2C1** SDA | 31 | External forward sensor cluster (VL53L4CX, SDP810 airspeed) |
| 4 | **I2C1** SCL | 30 | External forward sensor cluster (VL53L4CX, SDP810 airspeed) |
| 5 | UART1 TX | 20 | Recorder (Luckfox) RX Pin | 

