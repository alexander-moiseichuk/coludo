# External Pinout for Main Board
For context all of the wiring will be interpreted with a reference frame of looking at the board from 
the top and all of the letters are facing the correct side
ie. they are able to be read normally. 

## Frontal Grid of Main Board
The frontal grid of the main board is used to connect the following items:
- Recorder RX Pin
- GPS RX and TX
- Laser Connection
- Pito Tube/Air Pressure

| Pin number | Signal | GPIO | Bus / role |
|---|---|---|---|
| 1 | GND | - | Ground from MCU | 
| 2 | 3v3 | - | Power from MCU |
| 3 | I2C0 SDA | 7 | External forward sensor cluster (VL53L4CX, SDP810 airspeed) |
| 4 | I2C0 SCL | 8 | External forward sensor cluster (VL53L4CX, SDP810 airspeed) |
| 5 | VL53L4CX INT | 3 | laser data-ready | 
| 6 | UART2 RX | 23 | GNSS (ATGM336H) TX Pin |
| 7 | UART2 TX | 22 | GNSS (ATGM336H) RX Pin |
| 8 | UART1 TX | 20 | Recorder (Luckfox) RX Pin | 


## Back Grid of Main Board
The backward grid of the main board is used to connect the following items:
- Servo control Pins
- Power delivery controls pin
- USB power delivered separately
- Servo power delivered separately

| Pin number | Signal | GPIO | Bus / role |
|---|---|---|---|
| 1 | GND | - | Ground from MCU | 
| 2 | 3v3 | - | Power from MCU |
| 3 | I2C1 SCL | 30 | External Backward sensor power cluster (INA226) |
| 4 | I2C1 SDA | 31 | External Backward sensor power cluster (INA226) |
| 5 | INA226 ALERT | 29 | hardware over-current trip (rides the I2C1 cable) | 
| 6 | Right Servo | 32 | Right Elevon Servo Control |
| 7 | Yaw Servo | 26 | Yaw Servo Control |
| 8 | Left Servo | 27 | Left Elevon Servo Control | 
| — | *(unlabelled 9th pad)* | — | Present in the Gerbers at (38.499, −4.387) mm — offset from the row and ~4.6 mm inboard. Not a signal: a mounting hole or test-point via. Recorded so a future reader does not go looking for a net that is not there. |
