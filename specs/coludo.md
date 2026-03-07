# Introduction

This document intendent to provide top-view information about `Coludo` project as well become input for [Spec Kit](https://github.com/github/spec-kit) constitution, specification, plan and task.

The idea of the `Coludo` comes from fact that normal active control rocket launches are boring, and it is much more interesting that during apogee a glider is deployed and it gently lands back down.
I have already test flew the bottom booster stage with the E16 and F15 engine, and all I have to do is to develop a glider that can be placed on top of the rocket instead of just a nosecone [Spiral space plane](https://russianspaceweb.com/spiral_development.html) or [X-37B glider](https://en.wikipedia.org/wiki/Boeing_X-37)
![4rth glider prototype composition](https://github.com/alexander-moiseichuk/coludo/blob/main/doc/photos/TMS-4%20with%20electronics.jpg)

## Limitations

To make it lighter in future the `Coludo` can be transformed into a single-stage glider, since currently limited to lifting weight of only 100-150 grams
as it was discovered in [hardware components](../doc/hardware.md). This weight limitation also limited the size of the power supply so for now only 6F22 battery with 9V-5V convertor fits,
in return the target power consumption must be shrunk down to under 3.5W (or 5V and ~700mA) as [LM7805 voltage stabilizer](https://www.amazon.com/dp/B00LTQTZYQ) cannot reliably handle more.
The last wish to be enlisted is to use Micropython as available for this limited HW with utilizing asyncio as much as possible: (I don't know  what this means)
- hardware weight and power restrictions limits main controller to [eSBC esp32-p4](https://wiki.dfrobot.com/FireBeetle_2_ESP32_P4_Development_Board_IO_Expansion_Kit)
- event C/C++ is more effective but Micropython is prefferable as known solution
- to compensate Micropython single-thread affinity the [asyncio](https://github.com/peterhinch/micropython-async) must be used with using interrupts and potential Threads usage to utilize 2nd core in Python code if necessary

# Lifecycle

The glider lifecycle is quite short and can be simplified to the following stages:
- **setting** (up) - turning on all devices and testing them before placing vertically on launch pad rod vertically until engine ignition 
- **boosting** (active stage) - acceleration booster and glider system (almost) vertically
- **gliding** (passive stage) - in 4-6 second after engine stopped, the extraction system separates glider from booster and glider continue to fly to landing zone
- **landing** - gliding assumed gentle landing, speed will become zero and altitude not changed which means the flight finished.

## Setting (does setting mean preflight setup?)

The Setting stage started with powering up and finished with engine activation (Boosting stage). Expected duration will be about 15 minutes.
As electronics started, the following operations must be performed:
- device position: horizontal, oriented to North
- blinking rate of power indication LED is set to 2 Hz (250ms on and 250ms off)
- creation of all required Python objects
- initial calibration and zeroing all components as altimeter, compass, accelerometer and gyro, fins engines check
- wifi network with ssid **coludo_MAC** is activated and remote text console available becomes available
- if camera used - the video streaming must be available.
- if Storage (SD card) is installed, the video stream and telemetry must be recorded to the components respective flies
- if microphone enabled - it should start recording
- GPS must get a fix on satelites on and landing zone must be available in settings, direction vector must be calculated with distance to land zone under 200 meters
  - fix polling rate can be 1Hz 
  - system time syncronized to GPS time
- Flight Controller will handle these preparation steps and validate the device's state based on component feedback
- when all done, LED blinking should be changed to 100ms on and 900ms off to indicate readiness
- now `Coludo` can be placed on the launch pad and installed vertically

## Boosting

The Boosting stage happens next, it involves engine activation and finishes with booster separation, overall lasting no more than 10 seconds:
- device position: vertical, initially on launching rod, then Flight Controller must keep it perpendicular to the ground
- on the vertical rod, the GPS must be switched to high rate mode (5 or 10 Hz), maximizing positional accuracy and elevation 
- with engine acceleration Setting->Boosting happens and  the Flight Controller must actively tweak fin position
- when separation happens wings will open automatically, throug mechanical release and a pressure switch positioned at the bottom
  of the rocket will produce a HIGH signal, which will indicate the end of the first stage. (boosting)

## Gliding

After of the Boosting stage the separation happens and Gliding stage starts which is expected to finish in about 40 seconds with landing as close as possible to target:
- the glider must reorient itself from vertical to horizontal flight and keep top fin up according to gyro positioning
- the most basic and safest approaching and landing procedure must be used by Flight Controller:
  - direction determined to landing point for zone entrance short side
  - turn is started to landing point using vertical fin, horizontal fins meanwhile assist in the roll axis by keeping the pitch and roll as horizontal as possible 
  - after completion of turn, the direction must be checked and adjusted while flying over the landing zone 
  - if no land contact happened after exiting from landing zone new nearest entrance point should be immediately determined
  - as the direction is determined, it will turn to the new entrance point and if anything the cycle is repeated
- these glissade maneuvers must be continued until the altimeter shows low altitude the Landing mode will be turned on

## Landing

Pre-Landing and landing should be enabled when glider will be closer enough to contact with surface (e.g. 3m or about) and must lead as safe as possible data delivery for future investigation:
- the direction of flight must be as straight, without any roll and pitch or any other dangerous evolutions as contact with land may happened any time
- flushing telemetry and video/audio must increase rate from 1Hz to 10Hz to save as much data as possible
- as soon as contact happened, the speed will be close to zero and altitude stop changing
- after some silence time (e.g. 5s) when speed and altitude change closer to minimal the landing will be recognized as completed, all data streams closed
- flight accomplished, tasks stopped, objects cleaned from memory and storage must be unmounted as part of preparation to power turning off using pyb.stop() or pyb.standby()

# Flight control

The common principles should be used across all software components including `Flight Controller` which is the main component which controls the rest of stack.
It allows the stack to be tweakable based on settings e.g. Camera could be turned off from Console before flight.
As stated above to archive required latency targets (1ms for component reaction, 5ms for Flight Conroller decision) the cooperative multitasking (asyncio) should be used.

## Flight Controller

Controller is the main component which:
- creates all required Components 
- keep track of the rocket's current State { Setting, Boosting, Gliding, Landing }
- get Landing Zone coordinates, understand TargetPoint and landing parameters e.g. distance from start to TargetPoint must be nearby to LaunchPoint e.g. 200m
- controls Components states and gets async feedback for course correction
- if the Flight Controller crashes the async loop should be restarted

Main maneuvers of the Controller depends on the current stage. The procedure of directional change compensation needs to be clarified but the main goal will be to prevent overcorrection and react with
feedback multiplier i.e. if error is angle A into some direction the fins will be twisted by A * feedback(A) angle. For example, feedback(A) can be always 0.5 or 1 for
simplest case. In the same time maximal twist angle must be limited by -+ 45 degrees.

Examples: 
- yaw shows direction 30 degrees right, in this case vertical fin needs to be turned 15 degrees left
- pitch shows nose down 10 pitch (or -10) degrees off, in this case horizontal fins must be turned down by 5 degrees (or -5) to push for resulting zero
- roll shows 25 degrees right, so fins should be twisted for 13 degrees but left down and right up.
- this is done to prevent overcorrection caused by potential communicational latency between components and sensors.

This feedback() function could be smarter depending on air speed, sensor data quality and delivery delays, if software works fast then factor over 1 will allow to stabilize faster but with some extra G-factor. 


### Controller Setting maneuvers

After pre-start internal things and testing fins rotations Controller should fix all fins into a "zeroed position" (angle=0) and check sensors for engine ignition.

### Controller Boosting maneuvers

The main point of the software is to keep glider and booster strictly vertical:
- detect inclination and turn all fins to some direction to make vertical fin orthogonal to incline surface
- as the vertical fin points to incline on angle A (glider collapses on top) turns horizontal fins down to opposite direction
- OR when vertical fin points to opposite inclining on angle -A (glider falling on bottom) turn horizontal fins up to opposite direction

During Boosting phase as speed is not high probably it is probably best to keep the feedback factor > 1

### Controller Gliding maneuvers

Not many different evolutions are required:
- if glider after separation happened to be upside-down the left or right half-roll is required using all 3 fins pointing into opposite direction to roll
- direct flight when vertical fin set to 0 and left and right fins keep pitch horizontal
- left turn when vertical fin turned right to target direction and left/right fins assists (or just keep horizon level)
- right turn when vertical fin turned left for up to target direction and left/right fins assists (or just keep horizon)

### Controller Landing manoeuvres

Final step when coming to LandingZone or nearby on low altitude, no time for manoeuvres except keep going with minimal corrections:
- direct flight when vertical fin set to 0 and left and right fins keep pitch horizontal
- small corrections to left turn when vertical fin turned right to target direction and left/right fins keep horizont
- right turn when vertical fin turned left for up to target direction and left/right fins keep horizont

## Settings

Everything that operates throughout the flight should start with data from [esp32.NVS](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/nvs_flash.html) 
class which could be wrapped as Settings, e.g. some Task could be enabled, or disabled as stated in Settings as well LandingZone should be specified here.

## Landing Zone and Target Point

The LandingZone is a rectangular area with two specification points: top-left point (TL) and bottom-right (BR).
The TargetPoint for landing will be determined as average in between TL and BR latitutude and longitude. 
The altitude of TargetPoint will be the same as LaunchingPoint.

The LandingZone will automatically produce 2 more entrance points in the middle of the shortest side to compute if the glissade the glider is performing is
on safest approach and not coming from longest side. Quadratic landing zones or zones 500+ meters must be rejected.

Horizontally (longitude) stretched landing zone

            |TL------------------------------------------------------------------------|
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
       ---> Enterance                    targetPoint                            Entrance <----
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |------------------------------------------------------------------------BR|


Vertically (latitutude) stretched landing zone

                       Enterance
                           |
            |TL------------_--------------|
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |         targetPoint         |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |                             |
            |--------------^------------BR|
                          |
                      Entrance

## Sensors and Interrupts
If hardware components can be linked to interrupt, that interrupt must be utilized to reduce reaction time.
Of course, interrupts must be properly [connected to asyncio as specified in guides](https://github.com/peterhinch/micropython-async/tree/master/v3/threadsafe)

## Sensors Fusion/Backup

To have reliable control over parameters the sensor data fusion must be implemented e.g. based on priorities and timeouts: if several sensors are available and they produce the same kind of data 
the best should be used until not the data isn't outdated, otherwise backup sensor(s) should be selected. If more [advanced technique is available](https://github.com/micropython-IMU/micropython-fusion) it could be applied as well.

Example: for altitude the queue of selection could be the following
- main sensor ICP-10111 timeout 100 ms (e.g. for drop speed 10m/s and granularity 8.5cm => ~10 samples per meter)
- backup sensor 1 could be BMP280 timeout 200 ms
- backup sensor 2 could be accelerometer
- backup sensor 3 could be navigation - it has rate 10 Hz/100ms but real elevation data update ~10m which leads to 1 second to timeout

Proper cross-analysis for initial fusion (backing) should be performed by documentation and can be tweaked later after trials.
These sequences and constants will be hardcoded as in flight no time to tweak decisions.

## Tasks

One task must be created explicitly - the Controller, it creates the rest of the tasks which are located in the `tasks` folder and support some common API e.g.:
- setup() - async call to make initial task activation (or reset) and returns True or False
- run() - async and other methods to proceed usual activities
- notify() - to subscribe owner object for specific tasks for some callback to send async notification about change/update (what is owner object?)
- report() - produce report about task status
- finish() - to shutdown task
- validate() - to evaluate current task status and return True if everything is fine or false otherwise
- testing() - async call performs basic functionality testing e.g. as a part of setup()

For the Task's common scope, more calls might be required, for example:
- directory() - build list of names for all tasks by scanning tasks/ subfolder. Controller will use it for activation auxiliary tasks e.g. 
- create() - to create specified tasks by name if Settings allows for task class name and returns task reference or None
- close() - deactivate some task and cleanup resources, might be require after landing
- active() - query another active task or tasks (if None passed) by name e.g. camera may query Storage (SD card)
```
# here is the activation command of important tasks 
.....
# here is the activation command of non-important tasks

for name in Task.directory():
    if not Task.active(name):
        task = Task.create()
        if task.setup():
            logger.info(f'task {name} is up and running')
        else:
            logger.warning(f'task {name} failed to setup')
            task.close()
```
As set of tasks is not changed over flight the creation order and dependencies can be hardcoded in Controller e.g. enabling Wifi enables 
- Console to control and check parameters
- if Storage available - backing Logging and Telemetry to storage
- if Camera available - translation of video stream begins

But to make code less complex one task can ask object of another task by class i.e. if Console created it will auto connect during setup() to Wifi or UART.

## Logging

Logging will be done in milliseconds since the system is on for some component and free-style string:
```
  111 Controller :: setup started
  ...
 2222 Controller :: boosting detected
  ...
 5555 Controller :: landing completed
``` 

The centralized logging should be used with 3 possible destinations:
- UART if it available
- Network logging to port **1235** if WiFi is available, so using netcat or telnet possible to track logging until `Coludo` is in network range
- /sd/logging/date_time.log if Storage is available

## Telemetry

Similar to Logging approach but resulted in fixed-contents CSV files (;-separated) per each reporting case e.g. `/sd/telemetry/date_time.cpu.csv`:

```
uptime;utilization;temperature
111:40:51
2222:45:52
5555:48:55
``` 

Later telemetry can be used e.g. in converting position and elevation of GPS to GPX format to create a trace of the glider's flight in 3D.


## Console

Console allows:
- check current status remotely
- query active tasks and status
- enable/disable tasks
- restarts processing
- check report/statistics by calling task.report()
- all other debugging/profiling kind of activities

By default Console must operate on UART, if WiFi is available, Console must be accessible on port **1234**

## Pins distribution

Just for better organization has sense to collect pin mapping in single class PinMap for each physical devices. 

## System status

To speedup debugging and avoid any issues the system status probing must be implemented as [esp32 board allows](https://docs.micropython.org/en/v1.14/esp32/quickref.html)
to report in logs/telemetry [basic paramaters](https://randomnerdtutorials.com/micropython-esp32-esp8266-device-info):
- CPU frequency, utilization, temperature and task account, maybe top 5 consumers profiling
- memory heap information
- storage information as used/available in kilobytes

## Accelerometer

The [bno055 accelerometer](https://wiki.dfrobot.com/Gravity_BNO055_%2B_BMP280%20intelligent_10DOF_AHRS_SKU_SEN0253) is used to get acceleration
and inform Controller about big positional updates to provide additional information for decision making.
The [API of bno055](https://github.com/micropython-IMU/micropython-bno055) required calibration and provide all required data.
In-house calibration data can be stored in Settings to avoid having to calibrate before right launch.

Main source of stage switch from Settings stage (on Launch pad) to Boosting (engine ignited and whole journey started). 

## Gyroscope

The [bno055 gyroscope](https://wiki.dfrobot.com/Gravity_BNO055_%2B_BMP280%20intelligent_10DOF_AHRS_SKU_SEN0253) is used to collect yaw, pitch and roll data
and update status for Controller through async callback if change is noticeable.
The [API of bno055](https://github.com/micropython-IMU/micropython-bno055) requires calibration and provides all required data.
In-house calibration data can be stored in Settings to avoid this operation before launch.

To reduce error it must be installed closer to physical center of glider.

## Geomagnetic

The [bno055 geomagnetic](https://wiki.dfrobot.com/Gravity_BNO055_%2B_BMP280%20intelligent_10DOF_AHRS_SKU_SEN0253) is used to get directions
and inform Controller about big updates to provide additional information for decision making.
The [API of bno055](https://github.com/micropython-IMU/micropython-bno055) required calibration and provide all required data and could be
used for backing Navigation sensor. In-house calibration data can be stored in Settings to avoid this operation before launch.

## Navigation
Main source of direction control is [ATGM336H-5N-31](https://docs.cirkitdesigner.com/component/ab5c0c19-2fd9-4121-964e-1009970a950a/gps-atgm336h)
and it should operate in 1Hz mode during setup() and after position fix and vertical pre-boost stage switched to 10Hz mode.
The API, probably, [needs to be improved](https://hirokun.jp/av/ATGM336H-5N.pdf) to use ISR for serial and allow [rate control for main AT6558 chip](https://www.espruino.com/datasheets/AT6558.pdf), as all examples produce just basic reads:
- https://github.com/PermatechCA/ATGM336H
- https://github.com/Liuyufanlyf/GPS-GNSS_Module_for_MaixPy
- https://github.com/Albresky/GPS-ATGM336H-Library

To switch to [10Hz the following steps recommended](https://forum.arduino.cc/t/gps-module-change-update-rate-to-10hz/665212) :
- increase baud rate from 9600 to 11500 by pushing "$PCAS01,5*19\r\n"  (What is Baud rate?)
- at 10Hz, the module may struggle to output every NMEA sentence. Use the "$PCAS03\r\n" command to disable unnecessary sentences (like GSV or GSA) to reduce serial traffic.
    ```
    $PCAS10,3*1F<cr><lf>    factory reset, cold restart (loses fix memory)
    $PCAS01,5*19<cr><lf> 	 115200 baud
    $PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02<cr><lf>  GNGGA and GNRMC sentences only
    ```
- change update rates with PSAS command "$PCAS02,100*1E\r\n"

Example [which looks working](https://pastebin.com/2qfX5fpJ)
```
gpsPort.begin(9600, SERIAL_8N1, 35, 33);
 
while (gpsPort.available()) {
    delay(1); // ms
}
gpsPort.println("$PCAS01,5*19");
delay(200); // ms
gpsPort.flush();
gpsPort.end();

delay(1000);
gpsPort.begin(115200, SERIAL_8N1, 35, 33);
gpsPort.println("$PCAS02,100*1E");
```

Controller should perform fusion of data of the Accelerometer and Navigation just in case navigation is lost during flight

## Altimeter
As stated in docs the [Gravity: ICP-10111 Pressure Sensor](https://wiki.dfrobot.com/SKU_SEN0517_Gravity_ICP_10111_Pressure_Sensor) looks promising for accuracy (8.5cm), 
sampling rate and power consumption (2mA). Numbers could be backed up with:
- [BMP280 Digital Pressure Sensor](https://wiki.dfrobot.com/Gravity_BNO055_%2B_BMP280%20intelligent_10DOF_AHRS_SKU_SEN0253) which has 1m accuracy but
  there are several APIs available e.g. [a bit more fresh](https://github.com/PaszaVonPomiot/micropython-driver-bmp280) or [minimalistic](https://github.com/flrrth/pico-bmp280)
- Navigation system which can report elevation

The main source of information to switch from Gliding stage to Landing, when delta of altitude from minimal to current 3m or less. (did you mean the altitude reaches 3 meters or less?)

## Button 
Button is required for detection stage separation, when active part (engine work) completed and the ejection charge pushed out glider and opened the parachute. 
In this moment, the button aka [Gravity Digital Crash Sensor](https://wiki.dfrobot.com/Crash_Sensor__SKU__SEN0138_) turns from pressed state to opened and 
informs Controller about this change, and changes the state from Boosting to Gliding. This button must be connected to ground (LOW) when pressed (Setting or Boosting stages), and get HIGH when not pressed (Gliding or Landing).

## Servos
[Beffkkip SG90](https://www.amazon.com/Micro-Servos-Helicopter-Airplane-Controls/dp/B07MLR1498) is selected as candidate to contol 3 fins: vertical, left and right.
Expected torque is 1.2-1.4 g/cm, speed 0.11 seconds/60 degrees, but probably precise calibration is required as it very producer-depending. (what is producer depending?)
The various descriptions have [a bit different specs](https://friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf) but overall
use is basically simple in [native code](https://docs.cirkitdesigner.com/component/8aba6e12-58e6-4433-b2d9-9fd7544a2371/micro-servo-9g) as well in [Micropython](https://www.upesy.com/blogs/tutorials/esp32-servo-motor-sg90-on-micropython)
The power minimization must be taken into account to avoid delivery overload as it might consume up to 1A:
- angle should be updated only if not set already, default angle is 0 degrees by hardware assembly
- proper calibration should be used [during setup](https://www.upesy.com/blogs/tutorials/esp32-servo-motor-sg90-on-micropython)
- control each servo sequentially from the start to the stop of the cycle, not all at the same time
- the limiting angle for rotation will be from -45 to +45 degrees which makes [reaction time under 1ms](https://peppe8o.com/sg90-servo-motor-with-raspberry-pi-pico-and-micropython/)
  probably has sense to take into account [range correction](https://github.com/kuzned/servo-correction)
- test function can implement timing for changing angle from 0 to N degrees and back to 0, when N changed to -+5, 10, 15, ..., 45

## Storage
The auxiliary large storage expected to be SD card which is nicely supported by the built-in libraries and only detection, mounting and unmounting are important.
Test function could do some read/write sequential testing. 

## WiFi
The Wifi (2.4GHz only for distance and power saving) must be automatically enabled during boot up with SSID **coludo_MAC** and default secret password ;)
The specified address scope should be used and `Coludo` should be **.1** node e.g. 192.168.10.1, occupying unprivileged ports e.g. 1234, 1235, 1236 etc.

## Camera
The optional [Camera for Raspberry Pi](https://www.dfrobot.com/product-1179.html) can produce 30 FPS FHD.
If sufficient Storage is installed then Camera can write data to /sd/video/ folder.
If the WiFi is enabled it probably could be populated over web e.g. tcp port **1236**

## Audio
The very optional functionality to record sounds from board built-in mic as .WAV or any other simple format to replay later.
If Storage is installed then Audio can write data to /sd/audio/ folder. 

# Miscellaneous

Placeholder for additional ideas
