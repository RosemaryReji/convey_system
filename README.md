# Robust STM32-Based Conveyor Belt Colour Sorting System

An industrial-grade, non-blocking, two-way colour-sorting firmware implementation for the **STM32F103C8T6 ("Blue Pill")** running the official **STM32duino** core.

---

## 1. Project Overview & Sorting Logic

The system automates part sorting along a miniature conveyor belt into two deterministic categories:
1. **RED Category:**
   * Detected colour: **RED** (Normalised $r$ ratio beats $g$ and $b$ by $\ge 25\%$).
   * Diverter Action: Sweeps to **45°** (Diverts object into **LEFT Bin**).
   * Dwell: Holds for `SERVO_HOLD_MS` (400 ms), then returns to center (**90°**).
2. **OTHER Category:**
   * Detected colour: **ANY OTHER COLOUR** (Green, Blue, Yellow, White, Black, or low-confidence unrecognised items).
   * Diverter Action: Sweeps to **135°** (Diverts object into **RIGHT Bin**).
   * Dwell: Holds for `SERVO_HOLD_MS` (400 ms), then returns to center (**90°**).

Every valid object triggers a distinct, visible physical sweep away from the 90° center position.

---

## 2. Complete Wiring Table & Pin Connections

| Blue Pill Pin | Device / Signal | Connection & Constraints |
| :--- | :--- | :--- |
| **PA9** | USB-TTL RX | USART1 TX (115200 baud debug logging) |
| **PA10** | USB-TTL TX | USART1 RX (115200 baud) |
| **PC13** | Onboard LED | System State & Sort Blink Codes (Active LOW) |
| **PB9** | Push Button | Run/Pause Toggle to GND (`INPUT_PULLUP`, 200 ms debounce) |
| **PB6** | TCS34725 SCL | I2C1 Clock (3.3V Logic Rail) |
| **PB7** | TCS34725 SDA | I2C1 Data (3.3V Logic Rail) |
| **PA1** | SG90 Servo Signal | Hardware PWM via Timer `TIM2_CH2` (500 µs – 2400 µs) |
| **PA8** | L298N ENA | Hardware PWM via Timer `TIM1_CH1` (Remove 5V ENA Jumper!) |
| **PB5** | L298N IN1 | Direction Control Bit 1 (Push-Pull GPIO OUT) |
| **PB10** | L298N IN2 | Direction Control Bit 2 (Push-Pull GPIO OUT) |
| **PB12** | FC-51 IR OUT | Object Detect (Active LOW; **Power sensor from 3.3V!**) |
| **PA13 / PA14**| ST-Link SWD | **RESERVED**: SWDIO / SWCLK (Never touch in firmware) |
| **GND** | Ground Rail | **MANDATORY**: Common Ground for STM32, L298N, Servo, Sensors |

---

## 3. Critical Electrical & Safety Rules

1. **Common Ground Connection (Mandatory):**
   The STM32 GND, L298N GND, External 5V Servo Supply GND, and Motor Battery negative (-) terminal **must all be firmly tied together**. If the servo ground is isolated from the STM32 ground, PWM pulses float and the servo will twitch or fail to move.
2. **Servo Power Isolation:**
   The SG90 servo motor pulls instantaneous start/stall current spikes up to **800 mA**. It **must** be powered from a dedicated external regulated **5V ( $\ge 1.5\text{ A}$)** power source. Never power the servo from the Blue Pill's 3.3V rail or the Blue Pill's 5V pin.
3. **Sensor Supply & Pin Voltage Ratings:**
   * The **TCS34725** and **FC-51 IR sensor** must be powered from the Blue Pill's **3.3V** pin.
   * **Caution:** STM32F103 pin **PB12 is NOT 5V-tolerant**. Connecting an FC-51 powered from 5V directly to PB12 will degrade or destroy the microcontroller pin.
4. **Conveyor Motor Voltage & L298N Drop:**
   The yellow TT DC gear motor is rated for **3 V to 6 V**.
   * The L298N dual H-bridge has an internal bipolar transistor voltage drop of approximately **$1.8\text{ V} - 2.0\text{ V}$**.
   * Power the L298N motor rail from a 6V battery or buck converter (never direct from an unregulated 12V supply).
   * Remove the onboard black jumper on **ENA** and plug PA8 directly to the ENA header pin.
5. **Software Pause vs. Hardware E-Stop:**
   The tactile push-button on PB9 is a **software pause control**, not a certified emergency stop. In high-risk systems, true emergency stopping requires a physical emergency disconnect switch in series with the motor power rail.

---

## 4. Required Libraries & Toolchain Setup

### Arduino IDE Setup
* **IDE:** Arduino IDE 2.3.8
* **Board Package:** STM32duino (`STMicroelectronics:stm32` v3.0.0 or later)
  * URL: `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
* **Board Configuration:**
  * **Board:** Generic STM32F1 series
  * **Board part number:** BluePill F103C8 (or Generic F103C8Tx)
  * **U(S)ART support:** Enabled (generic 'Serial')
  * **Upload method:** STM32CubeProgrammer (SWD)
  * **USB support:** None

### Required Libraries (Install via Library Manager)
1. **Adafruit TCS34725** by *Adafruit* (v1.4.4+)
2. **Adafruit BusIO** by *Adafruit* (v1.17.4+)
3. **Servo** (Standard STM32-compatible Servo library included in core)

---

## 5. System Calibration Guide

### Step 1: Color Classifier Calibration (`MIN_CLEAR` & `DOMINANCE`)
1. Set `#define TEST_MODE 1` at the top of `conveyor_sorter.ino` and upload.
2. Open Serial Monitor at **115200 baud**.
3. Place sample items (Red targets, Other colored targets, and the bare conveyor belt) beneath the TCS34725:
   * **`MIN_CLEAR_THRESHOLD` (default 50):** Ensure that the empty conveyor belt reads below this number (classified as OTHER without false triggering), while colored objects read $> 100$.
   * **`RED_DOMINANCE_RATIO` (default 1.25):** If red targets have a glossy or faded surface, inspect the normalized `r`, `g`, `b` values. Adjust between `1.15` and `1.35` as needed.

### Step 2: Servo Arm Throw Angles (`TEST_MODE 2`)
1. Set `#define TEST_MODE 2` and upload.
2. The servo will continuously cycle: `45° (RED) -> 90° (CENTER) -> 135° (OTHER) -> 90° (CENTER)`.
3. Verify that the arm sweeps cleanly into both diversion chutes without binding or hitting mechanical stops.

### Step 3: IR Obstacle Sensor Trimpot Tuning (`TEST_MODE 3`)
1. Set `#define TEST_MODE 3` and upload.
2. Turn the potentiometer on the FC-51 IR sensor until the onboard sensor LED turns **ON** only when an object passes the detection beam, and turns **OFF** when the beam is clear.

### Step 4: Conveyor Motor Direction & Speed (`TEST_MODE 4`)
1. Set `#define TEST_MODE 4` and upload.
2. The motor runs Forward (2s) -> Dynamic Brake (2s) -> Reverse (2s) -> Dynamic Brake (2s).
3. If "Forward" moves the belt backwards, swap the physical motor wires at the L298N OUT1/OUT2 screw terminals.

### Step 5: Settle & Transit Timing Calibration (`TEST_MODE 0`)
* **`SENSOR_SETTLE_MS` (default 120 ms):** Non-blocking virtual delay between the IR trigger and the optical colour reading. Adjust so the object is centered squarely under the TCS34725 when the reading occurs.
* **`OBJECT_TRAVEL_MS` (default 1500 ms):** Transit time required for an object to travel from the IR sensor to the diverter flap. Increase or decrease based on conveyor speed.

---

## 6. Circular FIFO Queue & Overflow Policy

The firmware uses a statically allocated 4-slot circular FIFO queue:
* **In-Order Delivery:** Guarantees that items are diverted in the exact order they were detected.
* **Overflow Protection:** If 4 items are in transit simultaneously and a 5th arrives before the first is diverted, the system rejects the 5th object, logs `[OVERFLOW]` to Serial, and increments `g_queueOverflowCount`. It **never overwrites** existing in-flight records.
* **Virtual Run Clock (`runMs`):** When the conveyor is stopped via the PB9 button, the virtual clock freezes. Queued objects retain their remaining travel times without distortion. When resumed, transit timers continue seamlessly.

---

## 7. Troubleshooting Checklist

| Symptom | Probable Cause | Action |
| :--- | :--- | :--- |
| **Compiler Error: `fatal error: Adafruit_TCS34725.h`** | Missing library | Install `Adafruit TCS34725` and `Adafruit BusIO` via Library Manager. |
| **Servo twitches, stutters, or does not move** | Missing Common GND or current sag | 1. Connect external supply GND to Blue Pill GND.<br>2. Add a $220\,\mu\text{F}$ capacitor across servo 5V and GND. |
| **Conveyor motor does not run** | ENA jumper still connected or speed low | 1. Remove 5V ENA jumper; connect PA8 to ENA.<br>2. Increase `DEFAULT_MOTOR_SPEED` if motor stalls under load. |
| **Objects always classified as OTHER** | Ambient light too low or threshold too high | 1. Turn on white LED on TCS34725.<br>2. Check `MIN_CLEAR_THRESHOLD` in `TEST_MODE 1`. |
| **TCS34725 Init Error on startup** | I2C pull-ups or wiring loose | Check PB6 (SCL) and PB7 (SDA) wiring and 3.3V power. |

