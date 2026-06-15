# Introduction

This document is intended to provide a top-level architectural overview of the `Coludo` project.

> **Note:** hardware composition, pin mapping, Wi-Fi role, storage, and the configuration
> lifecycle are governed by [`board-config.md`](board-config.md), which is authoritative
> wherever it conflicts with statements below. In particular: the board joins the Control
> Center's Wi-Fi network as a **station** (it does not host an access point), the controller
> has **no SD card** (logs/telemetry/video go to the Recorder over UART), and the **camera
> lives on the Recorder**, not the controller.

The core concept of `Coludo` stems from the idea that traditional active-control rocket launches can be made significantly more engaging by introducing a secondary phase: at apogee, a glider deploys from the booster stage to achieve a controlled, gentle recovery back to earth. 

The lower booster stage has already been successfully test-flown using E16 and F15 model rocket engines. The current phase of development focuses on replacing the standard nosecone with an autonomous glider capable of piggyback deployment, drawing inspiration from vehicles like the [Spiral space plane](https://russianspaceweb.com/spiral_development.html) and the [X-37B glider](https://en.wikipedia.org/wiki/Boeing_X-37).

![4th glider prototype composition](https://github.com/alexander-moiseichuk/coludo/blob/main/doc/photos/TMS-4%20with%20electronics.jpg)

## Limitations

To optimize weight distribution in future iterations, `Coludo` may eventually transition into a single-stage glider. However, the current airframe is strictly limited to a payload capacity of 100–150 grams, as documented in the [hardware components specifications](../doc/hardware.md) where everything fits under 100 grams. The video recording and telemetry recording module was implemented separately with its own power supply. 

This rigid weight constraint severely limits the onboard power supply. The current physical envelope can accommodate a 800 mAh single cell LiPo battery with a power booster or a LiPo 6F22 9V battery paired with a power down regulator. To avoid any power shocks controller and engine regulators must be separated. Consequently, the maximum target power consumption for the electronics suite must remain under 3.5W (approximately 5V @ 700mA). To achieve this safely, a high-efficiency 5V switching regulator (such as a UBEC or buck converter) must be used; a traditional linear [LM7805 voltage stabilizer](https://www.amazon.com/dp/B00LTQTZYQ) cannot reliably handle the transient current spikes drawn by the control servos without inducing thermal shutdown.

The software architecture relies on MicroPython to manage this hardware stack, prioritizing cooperative multitasking via `asyncio`:
* **Hardware Constraints:** The weight and power limitations restrict the primary flight controller to the [eSBC esp32-p4](https://wiki.dfrobot.com/FireBeetle_2_ESP32_P4_Development_Board_IO_Expansion_Kit) development platform.
* **Language Choice:** While a compiled C/C++ codebase offers raw execution efficiency, MicroPython is preferred due to rapid prototyping familiarity.
* **Concurrency:** To circumvent MicroPython's single-thread affinity, [asyncio](https://github.com/peterhinch/micropython-async) is heavily utilized. This ensures non-blocking cooperative multitasking, supplemented by hardware interrupts and selective multi-threading to leverage the ESP32's second core where necessary.
* **Phased Rollout:** Phase one focuses entirely on validating sensor fusion and telemetry acquisition to guarantee data integrity. Active control surfaces and motorized actuation loops will only be enabled after these telemetry baselines are proven in flight trials. 

There is some problems with micropython in general (measured on the target board — see the
[benchmark findings](../doc/benches/esp32p4-micropython-findings.md)):
- GC pauses scale with live-object count on the PSRAM heap: ~0.3 ms on a clean heap but
  ~67 ms with only 10k small live objects — far beyond the control-loop budget.
- `asyncio.sleep_ms()` quantises to the ~10 ms FreeRTOS tick, so cooperative scheduling tops
  out near 100 Hz; a 5 ms / 200 Hz loop is **not** achievable via `asyncio.sleep`.
- The whole GC heap is in PSRAM (~12 MB/s memcpy) and `bytearray` slice-assignment is
  O(buffer length), so large preallocated buffers must be written with `struct.pack_into`.
- Asyncio is cooperative — if one task yields late, the whole system lags.
- Servo updates, GNSS parsing, and IMU callbacks all compete for time.

There are several improvements planned to mitigate those problems:
1. The 32 MB PSRAM lets the heap grow so GC runs less often — but each collection is slower
   (PSRAM is slow), so allocations on hot paths are minimised regardless.
2. gc.collect() is called before critical moments and GC is disabled during the flight.
3. The sub-10 ms control loop is paced by a hardware timer (+ `ThreadSafeFlag`) or a busy-wait
   on `ticks_us()`, not `asyncio.sleep`; if needed it moves to its own core thread.
4. If that is not enough, native code is used for the flight controller and servo control.

# Lifecycle

The operational lifecycle of the glider is brief and divided into four distinct phases:
* **Setting:** Ground initialization, sensor calibration, and vertical pre-launch staging.
* **Boosting (Active Stage):** Powered ascent of the combined booster-glider stack along a near-vertical trajectory.
* **Gliding (Passive Stage):** Triggered at apogee when the rocket motor's built-in black powder ejection charge fires (following a designated 4–6 second delay after burnout). The resulting internal pressure forces the glider clear of the booster, initiating autonomous wing deployment and navigation back to the designated landing zone.
* **Landing:** The final approach matrix where the glider flares, stabilizes horizontal velocity, and touches down.

## Setting 

The Setting phase begins at system power-on and terminates immediately upon engine ignition (transition to Boosting). The expected ground pad duration is approximately 15 minutes. 

Upon electronic initialization, the following sequential operations are executed:
* **Physical Orientation:** The airframe must be kept horizontal and oriented toward true North for baseline indexing.
* **Status Indication:** The main power LED is set to flash at a slow 2 Hz cycle (250ms ON / 250ms OFF).
* **Object Instantiation:** The MicroPython environment initializes all core software components and drivers.
* **Calibration:** The system zeroes out the altimeter, digital compass, accelerometer, and gyroscope while performing a full deflection check of the fin servos.
* **Network Connectivity:** The board joins the Control Center's Wi-Fi network as a **station** (see [`board-config.md`](board-config.md)) and establishes a connection with the ground control station (PC) to facilitate remote diagnostics and real-time monitoring.
* **Recorder Link:** If the Recorder module is present, the UART telemetry/log sink is opened (the controller has no local SD card; the Recorder owns video and storage).
* **GNSS Lock:** The GPS module begins polling at 1 Hz to acquire a multi-satellite 3D fix. The coordinates of the target landing zone must fall within a 200-meter threshold vector relative to the launch point. System time is automatically synchronized to the GPS atomic clock.
* **Validation:** The Flight Controller polls all subsystems. If all validation gates pass, the LED status changes to a "Ready" heartbeat pattern (100ms ON / 900ms OFF).
* **Staging:** The vehicle is cleared to be mounted vertically on the launch rail.

Potential problems:
- GNSS accuracy during dynamic flight is often ±5–15 m.
- At low altitude, multipath error increases.
- A 200 m zone is fine, but your boundary‑tracking logic assumes much better accuracy.

What will happen - The glider will:
- “hunt” for the boundary
- overshoot repeatedly
- oscillate between entry points
- possibly exit the zone unintentionally

To fix these issues there are some possible workarounds:
1. deadband logic (don’t correct unless error > X meters)
2. heading‑only navigation (ignore lateral error when close)
3. wind compensation (IMU + GNSS drift vector)

## Flight Controller

Controller is the main component which:

- creates all required Components
- keeps track of the rocket's current State { Setting, Boosting, Gliding, Landing }
- get Landing Zone coordinates, understand TargetPoint and landing parameters e.g. distance from start to TargetPoint must be nearby to LaunchPoint e.g. 200m
- controls Components' states and gets async feedback for course correction
- if the Flight Controller crashes the async loop should be restarted

Main maneuvers of the Controller depend on the current stage. The main goal will be to prevent overcorrection which will be achieved with the use of a Proportional-Integral-Derivative gains control algorithm (PIDgca). The idea previous about the Feedback Multiplier (Proportional Control) won't work since the glider will wildly overcorrect causing severe rocking or even the risk of stall. Thus it would be nice to have an IMU which gives position, speed, and acceleration.

Examples:
- yaw shows direction 30 degrees right, in this case vertical fin needs to be turned 15 degrees left or however much the PIDgca states
- pitch shows nose down 10 pitch (or -10) degrees off, in this case horizontal fins must be turned down by 5 degrees (or -5) to push for resulting zero
- roll shows 25 degrees right, so fins should be twisted for 13 degrees but left down and right up.
- this is done to prevent overcorrection caused by potential communicational latency between components and sensors.
- This feedback() function could be smarter depending on air speed, sensor data quality and delivery delays, if software works fast then factor over 1 will allow to stabilize faster but with some extra G-factor.

## Controller Setting maneuvers

After pre-start internal things and testing fins rotations Controller should fix all fins into a "zeroed position" (angle=0) and check sensors for engine ignition.

## Controller Boosting maneuvers

The main point of the software is to keep glider and booster strictly vertical:

detect inclination and turn all fins to some direction to make vertical fin orthogonal to incline surface
as the vertical fin points to incline on angle A (glider collapses on top) turns horizontal fins down to opposite direction
OR when vertical fin points to opposite inclining on angle -A (glider falling on bottom) turn horizontal fins up to opposite direction
During Boosting phase as speed is not high probably it is probably best to keep the feedback factor > 1

## Controller Gliding maneuvers

Not many different evolutions are required:

if glider after separation happened to be upside-down the left or right half-roll is required using all 3 fins pointing into opposite direction to roll
direct flight when vertical fin set to 0 and left and right fins keep pitch horizontal
left turn when vertical fin turned right to target direction and left/right fins assists (or just keep horizon level)
right turn when vertical fin turned left for up to target direction and left/right fins assists (or just keep horizontal)
Aggressive turns will be permitted only over minimal controllable speed.

## Controller Landing manoeuvres

Final step when coming to LandingZone or nearby on low altitude, no time for manoeuvres except keep going with minimal corrections:

direct flight when vertical fin set to 0 and left and right fins keep pitch horizontal
small corrections to left turn when vertical fin turned right to target direction and left/right fins keep horizont
right turn when vertical fin turned left for up to target direction and left/right fins keep horizont

## Boosting

The Boosting phase spans engine ignition through booster separation. While a zero-delay motor (like an F15-0) would trigger instantly, the operational profile utilizes motors featuring a built-in 4–6 second delay tracking element to coast cleanly to apogee:
* **Attitude Maintenance:** The airframe occupies a vertical stance on the launching rail. The Flight Controller dynamically monitors the pitch and roll axes to maintain a trajectory perpendicular to the local horizon.
* **GNSS Acceleration:** Upon detecting launch rail departure, the GPS module is programmatically escalated to a high-speed update mode (5 Hz or 10 Hz) to maximize spatial resolution during high-velocity ascent.
* **Dynamic Stabilization:** The Flight Controller actively manipulates the control surfaces to counteract wind shear and aerodynamic instability.
* **Separation Matrix:** At peak altitude, the motor's integrated black powder ejection charge fires, pressurizing the interior of the booster body tube. This pressure forces the glider upward and out of the booster. During the boosting phase, the glider’s wingtips are nested inside the booster's main body tube to hold them securely folded against aerodynamic drag. As the glider is pushed clear of the airframe, tension from rubber bands anchored at the front of the airplane automatically pulls the wings outward into their locked, deployed flight configuration. Concurrently, a dedicated separation loop—monitored via a physical pressure switch or a breakaway wire pulled from a flight computer socket—flags the physical separation event, outputting a digital logic change to instantly transition the software into Gliding mode.

## Gliding

Following booster separation, the Gliding phase executes for approximately 40 seconds, maneuvering the aircraft toward the target coordinates:
* **Attitude Recovery:** The glider must immediately execute an pitch/roll correction to transition from a vertical posture to a stable, horizontal gliding envelope, maintaining a "top-fin-up" orientation using real-time gyroscope vectors.
* **Navigation Architecture:** A streamlined gliding approach minimizes processing overhead:
  * The system computes a vector pointing to the shortest boundary entrance of the rectangular landing zone.
  * A heading adjustment is initiated via the vertical stabilizer (yaw axis), while the horizontal stabilizers actively damp out unwanted roll and pitch variations to maintain a flat slip-angle.
  * Upon crossing into the designated airspace, the system constantly samples its track over the landing zone.
  * If the glider overshoots or exits the boundaries without ground contact, it recalculates a vector to the nearest alternative entry point and loops the logic pattern.
* **Glissade Descent:** This iterative correction loop continues until the barometric altimeter registers a low-altitude threshold, shifting execution into Pre-Landing mode.

## Landing

The Pre-Landing sequence triggers when the glider drops to 4-12 meters AGL (Above Ground Level) relative to the launch pad elevation and speed is vertical speed < −1.5 m/s and roll < 10°. The priority shifts from destination tracking to structural preservation:
* **Attitude Lock:** The flight surfaces lock into a straight-and-level attitude glide. All aggressive rolling, pitching, or yawing maneuvers are suppressed to ensure clean underbelly contact with the ground.
* **Data Logging Surge:** To capture maximum high-resolution structural and aerodynamic impact data, the telemetry and multimedia flush rates are boosted from 1 Hz to 10 Hz.
* **Touchdown Detection:** Ground impact is verified when horizontal/vertical velocities decay to near-zero margins and barometric altitude output stabilizes completely.
* **De-initialization:** Following a 5-second confirmation window of absolute silence, the flight is officially flagged as completed. All open data streams are flushed to the Recorder over UART (the controller has no local filesystem to unmount), and the controller puts the hardware into a low-power state via the ESP32 `machine.deepsleep()` API (the earlier `pyb.stop()`/`pyb.standby()` calls are pyboard-only and do not apply to the ESP32 port).

Horizontally (longitude) stretched landing zone
```
            |TL------------------------------------------------------------------------|
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
       ---> Entrance                    targetPoint                            Entrance <----
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |                                                                          |
            |------------------------------------------------------------------------BR|
```

Vertically (latitutude) stretched landing zone
```
                       Entrance
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
```



Task creation orders and internal dependencies are explicitly hardcoded within the controller to keep execution logic simple and predictable.

## Degraded Mode

Incase of complete sensor faliure or other critical errors the degraded mode will be enabled: 

- When IMU degraded/produced invalid data the glider must fly straight or minimize turns
- If GNSS is lost - glide in current heading, prioritize gentle descent

## Sensors and Interrupts

If a hardware component can be linked to an interrupt, that interrupt must be utilized to reduce reaction time. Of course, interrupts must be properly connected to asyncio as specified in guides

## Sensors Fusion/Backup

To have reliable control over parameters the sensor data fusion must be implemented e.g. based on priorities and timeouts: if several sensors are available and they produce the same kind of data the best should be used until not the data isn't outdated, otherwise backup sensor(s) should be selected. If more advanced technique is available it could be applied as well.

Example: for altitude the queue of selection could be the following

- main sensor ICP-10111 timeout 100 ms (e.g. for drop speed 10m/s and granularity 8.5cm => ~10 samples per meter)
- backup sensor 1 could be BMP280 timeout 200 ms
- backup sensor 2 could be accelerometer
- backup sensor 3 could be navigation - it has rate 10 Hz/100ms but real elevation data update ~10m which leads to 1 second to timeout

Proper cross-analysis for initial fusion (backing) should be performed by documentation and can be tweaked later after trials. Sensor disagreements will be handled during timeouts and limits per each individually and switching to backup sensor. For example, the controller expects GPS data every 100 ms and if there is no data or repetative data for at least 200 ms then it will switch to the IMU.  

## Tasks

One task must be created explicitly - the Controller, it creates the rest of the tasks which are located in the tasks folder and support some common API e.g.:

- setup() - async call to make initial task activation (or reset) and returns True or False
- run() - async and other methods to proceed usual activities
- notify() - to subscribe owner object for specific tasks for some callback to send async notification about change/update (what is owner object?)
- report() - produce report about task status
- finish() - to shutdown task
- validate() - to evaluate current task status and return True if everything is fine or false otherwise

The testing part per each task can be implemented in test/ subfolder separately from the main code: 
- testing() - async call performs basic functionality testing e.g. as a part of setup() 

For the Task's common scope, more calls might be required, for example:

- directory() - build list of names for all tasks by scanning tasks/ subfolder. Controller will use it for activation auxiliary tasks e.g.
- create() - to create specified tasks by name if Settings allows for task class name and returns task reference or None
- close() - deactivate some task and cleanup resources, might be require after landing
- active() - query another active task or tasks (if None passed) by name e.g. camera may query Storage (SD card)
 here is the activation command of important tasks 
.....
 here is the activation command of non-important tasks

for name in Task.directory():
    if not Task.active(name):
        task = Task.create()
        if task.setup():
            logger.info(f'task {name} is up and running')
        else:
            logger.warning(f'task {name} failed to setup')
            task.close()
As set of tasks is not changed over flight the creation order and dependencies can be hardcoded in Controller e.g. enabling Wifi enables

Console to control and check parameters
if Storage available - backing Logging and Telemetry to storage
if Camera available - translation of video stream begins
But to make code less complex one task can ask object of another task by class i.e. if Console created it will auto connect during setup() to Wifi or UART.

## Task Data-Flow and Message Propagation

Data does not flow through a single generic message bus. A bus that allocates a message
object per sample at IMU rates (100–200 Hz) would fragment the heap and trigger GC pauses,
violating the `<10 ms` control-loop budget; and under cooperative `asyncio` a single slow
subscriber would stall the publisher inline. Instead the mechanism is chosen per data class:

* **Hot sensor data (direct, latest-value "blackboard").** High-rate readings (IMU, baro) are
  written in place into preallocated per-quantity slots — each holding `value + timestamp +
  source` — with latest-wins semantics and no per-sample allocation. The control loop and the
  sensor-fusion layer read the freshest *valid* slot directly (staleness is the fusion
  priority/timeout logic). The control loop is therefore self-contained: it reads the
  blackboard and writes servos without pulling per-cycle data through any queue, so it keeps
  running even if other tasks stall. It is paced by a hardware timer, not `asyncio.sleep`,
  which floors at ~10 ms on this port (see the
  [benchmark findings](../doc/benches/esp32p4-micropython-findings.md)).

* **Everything else goes through one Recorder.** For simplicity there is a single non-hot path.
  Every task reports logs and telemetry **directly to the Recorder** (`Recorder.log()`,
  `Recorder.tlm()` — a global singleton), and each record is stamped with `time.time_ns()//1000`
  (microseconds, monotonic, no wrap). The Recorder enqueues complete UART-ready text lines into
  two PSRAM ring buffers by priority:
  * **Telemetry — 1st priority queue.**
  * **Logs — 2nd priority queue.**
  An async drain loop empties these to the Recorder module (Luckfox) over UART, telemetry before
  logs. The UART push happens **first** (it is the authoritative flight-data sink); any other
  subscribers — notably the Control Center live view — receive the same records **only after**
  they have been pushed to UART. This guarantees recorder durability first and treats CC as a
  best-effort secondary consumer. Records are written into the rings with `struct.pack_into`
  rather than slice-assignment, which is O(buffer length) on this port (see the
  [benchmark findings](../doc/benches/esp32p4-micropython-findings.md)). Telemetry streams are
  created via a `Telemetry(file, fields)` helper that emits a CSV header first and then
  timestamped rows; all streams in a boot share one session prefix (`YYYYMMDD_HHMMSS`, produced
  from the RTC the first time telemetry is emitted) so each flight's files are distinct.

This collapses what would otherwise be a separate event-bus plus ring buffers into the Recorder:
discrete events are just log records, and the priority queues are the decoupling buffers
between fast producers and the slow UART/CC drains.

## Logging

Log strings append system uptime values in milliseconds alongside a standard descriptor layout:

111 Controller :: setup started
 2222 Controller :: boosting detected
 5555 Controller :: landing completed

 The centralized logging manager multiplexes data across these potential sinks depending on system state:
- Hardwired UART serial interface (console).
- Raw network sockets to the Control Center over TCP (active only when the Wi-Fi connection is maintained, i.e. prestart).
- The Recorder module over the dedicated `uart_recorder` link, which persists logs to its own SD card (the controller has no local SD). See [recorder module](../src/camera).

## Telemetry

Telemetry mirrors the logging architecture but outputs data in structured, semicolon-separated CSV profiles streamed to the Recorder (e.g., the Recorder stores `telemetry/date_time.cpu.csv` on its SD card):
uptime;utilization;temperature
111;40;51
2222;45;52
5555;48;55

Post-flight parsing arrays can extract these files to compile automated 3D spatial flight path models in standard GPX formatting.

## Storage Write Constraints

Raw write evaluations show that direct synchronous block-writing to local flash or SD arrays introduces SPI bus-locking delays lasting up to 80ms. This latency is unacceptable for a tight flight control loop. The controller therefore carries **no SD card at all**; data is offloaded over UART. Two defensive measures decouple data offload from the control loop:

- A high-rate circular FIFO buffer inside the ESP32's PSRAM absorbs bursts so producing tasks never block on I/O.

- The buffer drains over the dedicated `uart_recorder` line to the Recorder module, which owns the SD card and persists logs, telemetry, and video.

See [recorder module in sources](../src/camera) and [`board-config.md`](board-config.md).

## Console

The interactive system console provides terminal access to:
- Remote status monitoring.
- Active task auditing and dynamic run-state toggling.
- Global software reset commands.
- Profiling dumps using the localized task.report() method.
A local UART line is always available for direct on-bench debugging. Over Wi-Fi the board does **not** host a console; instead it connects to the Control Center as a client and answers the line protocol, which CC exposes to operators (telnet on TCP 1235) and to the browser. See [`cc-protocol.md`](cc-protocol.md).

## Pins Distribution

To ensure hardware modularity, physical microcontroller pins are **not** hardcoded; they are defined by the board configuration (`buses` and `pins` sections of `board.json`, with firmware defaults in `config_default.py`). The controller reads this config at boot to build the pin map and instantiate the declared components. See [`board-config.md`](board-config.md) for the schema and activation lifecycle.

## System Status

The telemetry loop tracks and records standard internal diagnostic variables exposed by the MicroPython ESP32 port architecture to simplify hardware debugging:
- Core CPU operational frequency, execution load factors, core temperatures, and thread tracking.
- Heap memory allocation pools, free block footprints, and fragment boundaries.
- Persistent storage availability benchmarks listed in kilobytes.

## Accelerometer

An onboard bno055 accelerometer registers linear forces, passing high-G vectors to the master process via asynchronous exception handling. The sensor abstracts data collection using standard bno055 MicroPython drivers. To eliminate the need for field calibration on the launch pad, static calibration vectors are loaded directly from NVS memory blocks during ground initialization.

This sensor serves as the primary trigger source for transitioning from the Setting phase to the Boosting phase.

## Gyroscope

The rotational tracking loops rely on the integrated bno055 gyroscope to sample yaw, pitch, and roll rates. The driver issues asynchronous callbacks to the flight controller whenever angular rates cross a defined deadband threshold. To eliminate sensor drift errors, the module must be physically aligned as closely as possible to the physical center of gravity of the airframe. Due to the low G tolerance of the BNO055, the ADXL375 is better to use. 

## Geomagnetic

The bno055 geomagnetic sensor extracts absolute magnetic heading vectors. It serves as a direct drift-correction mechanism to validate and backup the primary GNSS tracking coordinates.

## Navigation

Horizontal position tracking uses an ATGM336H-5N-31 high-sensitivity GNSS array. The module operates in a low-power 1 Hz mode during ground staging. Upon detecting vertical launch acceleration, the controller forces a command down the serial line to escalate the update frequency to a high-speed 10 Hz rate.

The standard serial driver structure must be modified to use Interrupt Service Routines (ISR) to handle the higher data rates supported by the core AT6558 chip architecture, replacing standard polling examples found in open-source references:

- PermatechCA ATGM336H Library
- Liuyufanlyf MaixPy GNSS Driver
- Albresky ATGM336H Driver Repository

To scale the data processing up to the 10 Hz threshold without overflowing the serial buffers, the system follows standard NMEA high-rate command structures:

- The serial interface speed (Baud Rate) escalates from 9600 to 115200 bits per second via a $PCAS01,5*19\r\n control string.

- Unnecessary NMEA sentences (such as GSV or GSA) are suppressed using the $PCAS03 mask to minimize data packet sizes, leaving only GNGGA and GNRMC strings active. 

$PCAS10,3*1F<cr><lf>    # Enforces factory cold restart
$PCAS01,5*19<cr><lf>    # Escalates interface speed to 115200 baud
$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02<cr><lf>  # Filters out all sentences except GNGGA and GNRMC

- The update rate is shifted to 100ms intervals using the tracking string $PCAS02,100*1E\r\n.

A verified MicroPython initialization snippet handles this handshake sequence.

The Flight Controller continually correlates accelerometer vectors alongside GNSS strings to maintain dead-reckoning positioning if the satellite signal drops out mid-flight.

## Altimeter

High-resolution altitude tracking uses a Gravity: ICP-10111 Pressure Sensor, selected for its 8.5cm operational accuracy and low 2mA current consumption. Barometric calculations are cross-checked against a secondary onboard BMP280 Digital Pressure Sensor and incoming GNSS elevation metrics.A verified vertical delta $\le 3\text{ meters}$ AGL acts as the absolute trigger to drop the master state machine from Gliding to Landing mode. Due to low altitude mode not working very well on the barometer the laser range finder is mandatory for safety. 

## Separation Sensor (Switch or Breakaway Wire)

The physical booster separation event is handled via an explicit electrical disconnect or micro-switch configured as a hardware interrupt. While nested on the booster airframe, the glider holds the circuit closed. Two implementation pathways are supported:

- Pressure Micro-Switch: A Gravity Digital Crash Sensor mounted to the airframe that springs open immediately as the glider leaves the booster body tube.
- Breakaway Pin/Socket: A physical wire loop plugged into a dedicated port on the flight computer. When the motor's black powder ejection charge pops the glider out of the body tube, the tethered wire pulls free from the socket.

The resulting state transition instantly alters the input pin logic to HIGH, invoking an unblock event via a hardware interrupt. This forces the master Flight Controller to transition immediately from Boosting to Gliding state. For separation detection, sensor or termination wire and IMU can be used simultaneously to ensure proper separation detection: 
1. Separation sensor triggered
2. IMU detects sudden pitch/roll change
3. Altimeter shows positive vertical deceleration
4. Wire is disconnected and pin gets 0

## Servos

Three independent Beffkkip SG90 micro-servos drive the vertical stabilizer and dual elevon surfaces. These servos provide a nominal stall torque of 1.2–1.4 kg·cm and an actuation speed of 0.11 seconds per 60 degrees. Due to significant manufacturer variability among component clones, custom hardware pulse-width modulation (PWM) calibration maps must be verified during system setup.To mitigate severe voltage drops on the primary 5V power line (as individual micro-servos can draw up to 1A under stall loads), the flight software enforces strict electrical safety protocols:
- Position update commands are suppressed if the target angle matches the current surface deflection state.
- Target positioning parameters are checked against baseline calibration maps loaded during system setup.
- The Flight Controller triggers servo updates sequentially rather than simultaneously to prevent additive current spikes.
- Angular deflections are structurally limited to an operational envelope of -45° to +45°.
- This small throw keeps surface travel times well under 1ms, utilizing range correction tracking profiles where applicable.
- The integrated diagnostic task handles sequential verification by sweeping the surfaces through steps and measuring return latencies.

## Storage

High-capacity storage does **not** live on the controller. The Recorder module (Luckfox Pico) owns the SD card and persists logs, telemetry, and video received over the `uart_recorder` link. See [recorder module](../src/camera) and [`board-config.md`](board-config.md). This keeps the SPI bus off the controller's critical path entirely.

## Wi-Fi

The integrated 2.4GHz Wi-Fi subsystem is optimized for extended range. During ground staging the board joins the **Control Center's** network as a **station** (SSID, credentials, CC host/port and tunable TX power come from the `wifi` section of `board.json`; Bluetooth is disabled to improve the link). Once a network socket connection to the Control Center is established, the flight controller unlocks remote parameter tuning, health monitoring, and live telemetry streaming. The link exists only in prestart; it is expected to be lost from ignition onward. See [`board-config.md`](board-config.md).

## Camera

Video capture is **not** a controller responsibility. It is handled by the independent Recorder module (Luckfox Pico + sc3336b), which records 2304×1296 30 FPS video to its own SD card on its own power supply. Isolating it on a separate board prevents video encoding overhead and storage I/O from impacting the primary flight control tasks. See [recorder module](../src/camera).

## Audio

Audio is captured (if at all) by the Recorder module alongside its video, not by the controller. The controller has no microphone or local storage. This subsystem is optional and out of scope for the controller firmware.

# Overall Design Risks and Mitigations

- First flights will be with telemetry ONLY collection without active control to understand potential locking and sensor problems which will be mitigated later by adding more functional components like watchdog, heartbeat, or runtime health monitor service. 
- Assume by aerodynamics that the minimum effective airspeed to control the glider is about 10 meters per second.
- Having GPS accuracy target as 10 meters, I will asign the landing zone's sides at atleast 50 meters.
- To not overload the system with current and keep the battery safe, at least 800 mAh battary will be used with seperate voltage boosters for the controller and engines. Additionally, the servos' positioning / adjusting will be done sequentially. 
- Definition of "no control" will be clarified through trials with a telemtry only glider, preliminary

| Subsystem / Loop            | Target Frequency | Max Allowed Latency | Notes / Rationale |
|-----------------------------|------------------|----------------------|-------------------|
| **Primary PID Control Loop** | 50–100 Hz        | < 10 ms     | Core stabilization loop; must run even if other tasks stall. Should ideally run on its own core or native module. |
| **IMU Sampling (BNO055)**   | 100–200 Hz       | < 5 ms       | Gyro/accel data must be fresh for stable control. Interrupt-driven preferred. |
| **Servo Output Update**     | 40–60 Hz         | < 20 ms      | Standard RC servo frame rate; sequential updates to avoid current spikes. |
| **GNSS Parsing**            | 10 Hz            | < 150 ms     | Only needed for navigation; not safety‑critical for attitude control. |
| **Altimeter (ICP‑10111)**   | 20–50 Hz         | < 50 ms      | Needed for landing detection and vertical rate estimation. |
| **Telemetry Logging**       | 1 Hz (normal)    | < 500 ms     | Low priority; should never block control loops. |
| **Telemetry Burst (Landing)** | 10 Hz          | < 100 ms     | Only after control authority is no longer critical. |
| **Task Scheduler / Asyncio** | 20–50 Hz        | < 20 ms      | Supervisory logic; must not interfere with PID loop timing. |
| **Watchdog Reset Window**   | —                | 100–200 ms   | If PID loop or IMU updates stall beyond this, system must reset or enter degraded mode. |

**Measured reality check** (see [benchmark findings](../doc/benches/esp32p4-micropython-findings.md)):
`asyncio.sleep_ms()` floors at ~10 ms (FreeRTOS 100 Hz tick), so the 50–100 Hz / 200 Hz rows
above cannot be met with `asyncio.sleep` — those loops must be paced by a hardware timer or a
`ticks_us()` busy-wait. A fragmented-heap `gc.collect()` was measured at ~67 ms, which is why
GC is controlled explicitly and disabled during the flight.
