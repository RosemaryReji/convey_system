/**
 * ============================================================================
 * PROJECT: Robust STM32-Based Conveyor Belt Colour Sorting System
 * TARGET MCU: STM32F103C8T6 ("Blue Pill")
 * CORE: STM32duino (STMicroelectronics:stm32 v3.0.0+)
 * FILE: conveyor_sorter.ino
 *
 * ----------------------------------------------------------------------------
 * PIN ASSIGNMENTS & HARDWARE CONNECTIONS
 * ----------------------------------------------------------------------------
 * Blue Pill Pin | Device / Signal       | Notes & Electrical Constraints
 * --------------+-----------------------+-------------------------------------
 * PA9           | USART1 TX             | 115200 baud debug output to PC
 * PA10          | USART1 RX             | 115200 baud debug input from PC
 * PC13          | Onboard LED           | Active LOW (LOW = ON, HIGH = OFF)
 * PB9           | Run/Stop Push Button  | Active LOW (INPUT_PULLUP to GND)
 * PB6           | TCS34725 SCL (I2C1)   | 3.3V logic (External pull-up to 3.3V)
 * PB7           | TCS34725 SDA (I2C1)   | 3.3V logic (External pull-up to 3.3V)
 * PA1           | SG90 Servo Signal     | PWM Timer TIM2_CH2 (500us - 2400us)
 * PA8           | L298N ENA (Motor PWM) | PWM Timer TIM1_CH1 (0 - 255)
 * PB5           | L298N IN1             | Direction bit 1 (Push-Pull Output)
 * PB10          | L298N IN2             | Direction bit 2 (Push-Pull Output)
 * PB12          | FC-51 IR Sensor OUT   | Active LOW (Must power sensor with 3.3V!)
 *
 * RESERVED SWD DEBUGGER PINS (DO NOT TOUCH IN CODE):
 * PA13 / PA14 : SWD TMS / TCK for ST-Link V2
 * PA11 / PA12 : USB D- / D+
 *
 * ----------------------------------------------------------------------------
 * CRITICAL ELECTRICAL & SAFETY RULES:
 * 1. COMMON GROUND: The external 5V supply GND, L298N GND, Motor power GND,
 *    and STM32 Blue Pill GND MUST be tied together on a central ground bus.
 * 2. SERVO POWER: SG90 peak stall current reaches ~800mA. It MUST be powered
 *    from an external regulated 5V (>= 1.5A) source. NEVER power from Blue Pill 3.3V!
 * 3. SENSOR POWER: Power the FC-51 and TCS34725 from the Blue Pill 3.3V rail.
 *    PB12 is NOT 5V tolerant on STM32F103C8T6.
 * 4. MOTOR POWER: TT gear motor rated 3-6V. Power L298N motor rail from a regulated
 *    5-6V battery or buck converter (never direct 12V adapter).
 * 5. L298N ENA JUMPER: Remove the onboard 5V jumper from ENA and connect PA8 to ENA.
 * ============================================================================
 */

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>

// ============================================================================
// COMPILE-TIME TEST MODE SELECTOR
// 0 = Normal conveyor sorter operation (Default)
// 1 = Continuous color sensor raw readings
// 2 = Servo calibration sweep (45 -> 90 -> 135 -> 90)
// 3 = IR obstacle sensor monitor
// 4 = Motor test (Forward -> Brake -> Reverse -> Brake)
// ============================================================================
#define TEST_MODE 0

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================
constexpr uint8_t PIN_USART1_TX   = PA9;   // Hardware USART1 TX
constexpr uint8_t PIN_USART1_RX   = PA10;  // Hardware USART1 RX
constexpr uint8_t PIN_STATUS_LED  = PC13;  // Onboard LED (Active LOW)
constexpr uint8_t PIN_BUTTON      = PB9;   // Run/Stop toggle button
constexpr uint8_t PIN_I2C_SCL     = PB6;   // I2C1 Clock
constexpr uint8_t PIN_I2C_SDA     = PB7;   // I2C1 Data
constexpr uint8_t PIN_SERVO       = PA1;   // SG90 PWM signal (TIM2)
constexpr uint8_t PIN_MOTOR_ENA   = PA8;   // Motor PWM speed (TIM1)
constexpr uint8_t PIN_MOTOR_IN1   = PB5;   // Motor Direction IN1
constexpr uint8_t PIN_MOTOR_IN2   = PB10;  // Motor Direction IN2
constexpr uint8_t PIN_IR_OUT      = PB12;  // FC-51 IR digital output

// ============================================================================
// CONFIGURABLE TUNING PARAMETERS (SYSTEM TIMINGS & ANGLES)
// ============================================================================
constexpr int      SERVO_RED_ANGLE     = 45;    // Divert LEFT for RED
constexpr int      SERVO_CENTER_ANGLE  = 90;    // Neutral idle / pass-through
constexpr int      SERVO_OTHER_ANGLE   = 135;   // Divert RIGHT for ANY OTHER colour
constexpr uint32_t SERVO_HOLD_MS       = 400;   // Arm hold duration at divert angle
constexpr uint32_t SERVO_SETTLE_MS     = 180;   // Mechanical settling time back to 90 deg

constexpr uint32_t SENSOR_SETTLE_MS    = 120;   // Virtual-time delay before colour read
constexpr uint32_t OBJECT_TRAVEL_MS    = 1500;  // Conveyor transit time from IR to diverter
constexpr uint8_t  DEFAULT_MOTOR_SPEED = 180;   // Motor PWM duty cycle (0-255)
constexpr uint32_t BUTTON_DEBOUNCE_MS  = 200;   // Push-button debounce threshold

// Colour Classification Parameters
constexpr uint16_t MIN_CLEAR_THRESHOLD = 50;    // Minimum clear channel threshold
constexpr float    RED_DOMINANCE_RATIO = 1.25f; // R must beat G and B by >= 25%

// System Dimensions & Bounds
constexpr uint8_t  QUEUE_CAPACITY      = 4;     // Static circular FIFO capacity
constexpr uint16_t SERVO_PULSE_MIN_US  = 500;   // Standard SG90 lower pulse bound
constexpr uint16_t SERVO_PULSE_MAX_US  = 2400;  // Standard SG90 upper pulse bound

// LED Indicator Timing
constexpr uint32_t LED_BLINK_PULSE_MS  = 120;   // Active blink pulse duration
constexpr uint32_t LED_BLINK_PAUSE_MS  = 120;   // Interval between multiple blinks

// ============================================================================
// DATA TYPES & SYSTEM STATES
// ============================================================================
enum SystemState : uint8_t {
  STATE_STOPPED = 0,
  STATE_RUNNING = 1,
  STATE_SENSOR_FAULT = 2
};

enum ColorCategory : uint8_t {
  COLOR_OTHER = 0,
  COLOR_RED   = 1
};

enum AcquisitionState : uint8_t {
  ACQ_IDLE,                 // Waiting for IR falling edge
  ACQ_SETTLING,             // Non-blocking wait for object to settle under TCS34725
  ACQ_WAIT_FOR_IR_RELEASE   // Prevent re-trigger until object clears sensor
};

enum ServoState : uint8_t {
  SERVO_STATE_IDLE,         // Arm resting at 90 deg, available
  SERVO_STATE_DIVERTING,    // Arm held at 45 or 135 deg
  SERVO_STATE_RETURNING     // Arm commanded back to 90 deg, waiting to settle
};

struct QueueEntry {
  ColorCategory color;
  uint32_t      detectRunMs; // Virtual run-clock timestamp at IR detection
};

// ============================================================================
// STATIC CIRCULAR FIFO QUEUE CLASS
// ============================================================================
class CircularQueue {
private:
  QueueEntry m_entries[QUEUE_CAPACITY];
  uint8_t    m_head;
  uint8_t    m_tail;
  uint8_t    m_count;

public:
  CircularQueue() : m_head(0), m_tail(0), m_count(0) {}

  bool isFull() const  { return m_count >= QUEUE_CAPACITY; }
  bool isEmpty() const { return m_count == 0; }
  uint8_t getCount() const { return m_count; }

  bool enqueue(ColorCategory c, uint32_t runMs) {
    if (isFull()) return false;
    m_entries[m_tail].color       = c;
    m_entries[m_tail].detectRunMs = runMs;
    m_tail = (m_tail + 1) % QUEUE_CAPACITY;
    m_count++;
    return true;
  }

  bool peek(QueueEntry &outItem) const {
    if (isEmpty()) return false;
    outItem = m_entries[m_head];
    return true;
  }

  bool dequeue(QueueEntry &outItem) {
    if (isEmpty()) return false;
    outItem = m_entries[m_head];
    m_head = (m_head + 1) % QUEUE_CAPACITY;
    m_count--;
    return true;
  }

  void clear() {
    m_head  = 0;
    m_tail  = 0;
    m_count = 0;
  }
};

// ============================================================================
// GLOBAL PERIPHERAL INSTANCES & STATE VARIABLES
// ============================================================================
Servo diverterServo;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
CircularQueue g_queue;

// Virtual Clock & Runtime
SystemState      g_systemState       = STATE_STOPPED;
uint32_t         g_runMs             = 0;        // Advances strictly when RUNNING
uint32_t         g_lastWallMs        = 0;        // Physical millis() tracking

// Object Acquisition Engine
AcquisitionState g_acqState          = ACQ_IDLE;
uint32_t         g_acqTriggerRunMs   = 0;

// Servo Mechanism Engine
ServoState       g_servoState        = SERVO_STATE_IDLE;
uint32_t         g_servoTimerRunMs   = 0;
ColorCategory    g_currentDivertColor= COLOR_OTHER;

// Button Filtering
bool             g_btnLastRaw        = HIGH;
bool             g_btnStable         = HIGH;
uint32_t         g_btnLastDebounceMs = 0;

// Statistics & Counters
uint32_t         g_totalSorted       = 0;
uint32_t         g_totalRed          = 0;
uint32_t         g_totalOther        = 0;
uint32_t         g_queueOverflowCount= 0;
uint32_t         g_sensorErrorCount  = 0;

// Non-blocking Status LED Blink Machine
uint8_t          g_ledBlinksRemaining= 0;
bool             g_ledBlinkPhaseOn   = false;
uint32_t         g_ledBlinkTimerMs   = 0;

// Motor Running Configuration
uint8_t          g_activeMotorSpeed  = DEFAULT_MOTOR_SPEED;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================
void initializeHardware();
void updateRunClock();
void updateButton();
void updateIRAndAcquisition();
void updateSortingAndServo();
void updateMotor();
void updateStatusLED();
void motorForward(uint8_t speed);
void motorReverse(uint8_t speed);
void motorBrake();
void motorCoast();
void setLedHardware(bool turnOn);
void queueLedBlinks(uint8_t count);
bool readColourSensor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c);
ColorCategory classifyColour(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
const char* getColorName(ColorCategory c);

// ============================================================================
// ARDUINO SETUP
// ============================================================================
void setup() {
  initializeHardware();
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================
void loop() {
#if TEST_MODE == 0
  updateRunClock();
  updateButton();
  updateIRAndAcquisition();
  updateSortingAndServo();
  updateMotor();
  updateStatusLED();
#else
  // Dedicated compile-time test routines
  runSelectedTestMode();
#endif
}

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================
void initializeHardware() {
  // Start USART1 debug console (PA9 TX, PA10 RX)
  Serial.begin(115200);

  // Pin Modes
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_IR_OUT, INPUT);
  pinMode(PIN_MOTOR_ENA, OUTPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);

  // Initial Safe Outputs: Motor completely halted, LED off
  motorBrake();
  setLedHardware(false);

  // Attach Diverter Servo with standard explicit pulse widths
  diverterServo.attach(PIN_SERVO, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  diverterServo.write(SERVO_CENTER_ANGLE);
  g_servoState = SERVO_STATE_IDLE;

  // Initialize I2C1 (PB6 SCL, PB7 SDA)
  Wire.begin();

  Serial.println(F("\n=================================================="));
  Serial.println(F("STM32 CONVEYOR COLOUR SORTER FIRMWARE v2.0"));
  Serial.println(F("STMicroelectronics:stm32 Core (Generic BluePill)"));
  Serial.println(F("=================================================="));

  // Initialize TCS34725
  if (!tcs.begin()) {
    g_systemState = STATE_SENSOR_FAULT;
    g_sensorErrorCount++;
    Serial.println(F("FATAL ERROR: TCS34725 not detected! Check 3.3V, GND, PB6(SCL), PB7(SDA)."));
  } else {
    Serial.println(F("TCS34725 colour sensor initialized successfully."));
    g_systemState = STATE_STOPPED;
  }

  g_lastWallMs = millis();
  Serial.println(F("System initialized in STOPPED state."));
  Serial.println(F("Press push button on PB9 to start conveyor.\n"));
}

// ============================================================================
// VIRTUAL RUN CLOCK IMPLEMENTATION
// Advances strictly while in STATE_RUNNING using rollover-safe delta math.
// ============================================================================
void updateRunClock() {
  uint32_t currentWallMs = millis();
  uint32_t deltaMs = currentWallMs - g_lastWallMs;
  g_lastWallMs = currentWallMs;

  if (g_systemState == STATE_RUNNING) {
    g_runMs += deltaMs;
  }
}

// ============================================================================
// BUTTON PROCESSING (PB9) - 200ms SOFTWARE DEBOUNCE
// ============================================================================
void updateButton() {
  bool rawReading = digitalRead(PIN_BUTTON);
  uint32_t now = millis();

  if (rawReading != g_btnLastRaw) {
    g_btnLastDebounceMs = now;
    g_btnLastRaw = rawReading;
  }

  if ((now - g_btnLastDebounceMs) >= BUTTON_DEBOUNCE_MS) {
    if (rawReading != g_btnStable) {
      g_btnStable = rawReading;

      // Falling edge: Button pressed down to GND
      if (g_btnStable == LOW) {
        if (g_systemState == STATE_STOPPED) {
          // Resume / Start
          g_systemState = STATE_RUNNING;
          Serial.println(F("\n>>> EVENT: Conveyor STARTED (RUNNING) <<<"));
        } else if (g_systemState == STATE_RUNNING) {
          // Pause / Stop
          g_systemState = STATE_STOPPED;
          motorBrake();
          diverterServo.write(SERVO_CENTER_ANGLE);
          g_servoState = SERVO_STATE_IDLE;
          Serial.println(F("\n>>> EVENT: Conveyor PAUSED (STOPPED) <<<"));
          Serial.print(F("Queued items retained: "));
          Serial.println(g_queue.getCount());
        } else if (g_systemState == STATE_SENSOR_FAULT) {
          // Attempt recovery on button press
          Serial.println(F("Attempting TCS34725 sensor fault recovery..."));
          if (tcs.begin()) {
            g_systemState = STATE_STOPPED;
            Serial.println(F("Sensor recovered. System in STOPPED state."));
          } else {
            Serial.println(F("Recovery failed. Sensor still unreachable."));
          }
        }
      }
    }
  }
}

// ============================================================================
// OBJECT ACQUISITION & NON-BLOCKING SENSOR SETTLING
// ============================================================================
void updateIRAndAcquisition() {
  if (g_systemState != STATE_RUNNING) {
    return;
  }

  bool irPinState = digitalRead(PIN_IR_OUT);

  switch (g_acqState) {
    case ACQ_IDLE:
      // Object detected on falling edge (Active LOW)
      if (irPinState == LOW) {
        g_acqTriggerRunMs = g_runMs;
        g_acqState        = ACQ_SETTLING;
      }
      break;

    case ACQ_SETTLING:
      // Wait for object to travel and settle beneath TCS34725 in virtual time
      if ((g_runMs - g_acqTriggerRunMs) >= SENSOR_SETTLE_MS) {
        uint16_t r = 0, g = 0, b = 0, c = 0;
        if (!readColourSensor(r, g, b, c)) {
          g_sensorErrorCount++;
          Serial.println(F("[ERROR] TCS34725 optical read failed. Transaction aborted."));
        } else {
          ColorCategory classified = classifyColour(r, g, b, c);

          // Push into queue using the trigger timestamp as transit reference
          if (!g_queue.enqueue(classified, g_acqTriggerRunMs)) {
            g_queueOverflowCount++;
            Serial.print(F("[OVERFLOW] FIFO queue full (capacity "));
            Serial.print(QUEUE_CAPACITY);
            Serial.println(F(")! Object dropped."));
          } else {
            Serial.print(F("[DETECTED] Object: "));
            Serial.print(getColorName(classified));
            Serial.print(F(" | Enqueued at runMs: "));
            Serial.print(g_acqTriggerRunMs);
            Serial.print(F(" | Queue depth: "));
            Serial.println(g_queue.getCount());
          }
        }
        g_acqState = ACQ_WAIT_FOR_IR_RELEASE;
      }
      break;

    case ACQ_WAIT_FOR_IR_RELEASE:
      // Must return HIGH before arming for the next physical object
      if (irPinState == HIGH) {
        g_acqState = ACQ_IDLE;
      }
      break;
  }
}

// ============================================================================
// SORTING ENGINE & SERVO ARBITRATION
// ============================================================================
void updateSortingAndServo() {
  if (g_systemState != STATE_RUNNING) {
    return;
  }

  switch (g_servoState) {
    case SERVO_STATE_IDLE: {
      // Check oldest object in FIFO queue
      QueueEntry item;
      if (g_queue.peek(item)) {
        uint32_t elapsedRun = g_runMs - item.detectRunMs;
        if (elapsedRun >= OBJECT_TRAVEL_MS) {
          // Object has reached diverter
          g_currentDivertColor = item.color;
          int targetAngle = (item.color == COLOR_RED) ? SERVO_RED_ANGLE : SERVO_OTHER_ANGLE;

          diverterServo.write(targetAngle);
          g_servoTimerRunMs = g_runMs;
          g_servoState      = SERVO_STATE_DIVERTING;

          Serial.print(F("[SORTING] Diverting "));
          Serial.print(getColorName(item.color));
          Serial.print(F(" -> Angle: "));
          Serial.print(targetAngle);
          Serial.print(F(" deg (elapsed "));
          Serial.print(elapsedRun);
          Serial.println(F(" ms)"));
        }
      }
      break;
    }

    case SERVO_STATE_DIVERTING:
      // Hold diverter position for configured duration
      if ((g_runMs - g_servoTimerRunMs) >= SERVO_HOLD_MS) {
        diverterServo.write(SERVO_CENTER_ANGLE);
        g_servoTimerRunMs = g_runMs;
        g_servoState      = SERVO_STATE_RETURNING;
      }
      break;

    case SERVO_STATE_RETURNING:
      // Allow mechanical return to 90 deg before accepting next diversion
      if ((g_runMs - g_servoTimerRunMs) >= SERVO_SETTLE_MS) {
        // Diversion cycle officially completed: dequeue item and update statistics
        QueueEntry completedItem;
        g_queue.dequeue(completedItem);

        g_totalSorted++;
        if (completedItem.color == COLOR_RED) {
          g_totalRed++;
          queueLedBlinks(1); // 1 blink for RED
        } else {
          g_totalOther++;
          queueLedBlinks(2); // 2 blinks for OTHER
        }

        Serial.print(F("[SORT COMPLETED] "));
        Serial.print(getColorName(completedItem.color));
        Serial.print(F(" | Totals -> Sorted: "));
        Serial.print(g_totalSorted);
        Serial.print(F(" (Red: "));
        Serial.print(g_totalRed);
        Serial.print(F(", Other: "));
        Serial.print(g_totalOther);
        Serial.print(F(") | Overflows: "));
        Serial.println(g_queueOverflowCount);

        g_servoState = SERVO_STATE_IDLE;
      }
      break;
  }
}

// ============================================================================
// MOTOR SPEED & STATE DISPATCHER
// ============================================================================
void updateMotor() {
  if (g_systemState == STATE_RUNNING) {
    motorForward(g_activeMotorSpeed);
  } else {
    motorBrake();
  }
}

// ============================================================================
// L298N LOW-LEVEL MOTOR CONTROL FUNCTIONS
// Truth Table:
// Forward: IN1 = HIGH, IN2 = LOW,  ENA = PWM
// Reverse: IN1 = LOW,  IN2 = HIGH, ENA = PWM
// Brake:   IN1 = LOW,  IN2 = LOW,  ENA = 255 (Dynamic regenerative short)
// Coast:   IN1 = LOW,  IN2 = LOW,  ENA = 0   (High-impedance floating)
// ============================================================================
void motorForward(uint8_t speed) {
  digitalWrite(PIN_MOTOR_IN1, HIGH);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  analogWrite(PIN_MOTOR_ENA, speed);
}

void motorReverse(uint8_t speed) {
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, HIGH);
  analogWrite(PIN_MOTOR_ENA, speed);
}

void motorBrake() {
  // Low-side switches sink motor windings to ground for active braking
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  analogWrite(PIN_MOTOR_ENA, 255);
}

void motorCoast() {
  // Free-wheeling / high-impedance stop
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  analogWrite(PIN_MOTOR_ENA, 0);
}

// ============================================================================
// COLOUR READING & NORMALIZED CLASSIFIER
// ============================================================================
bool readColourSensor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  tcs.getRawData(&r, &g, &b, &c);
  // Verify transaction integrity (if bus hangs or disconnects, returns 0 or 0xFFFF)
  if (c == 0 && r == 0 && g == 0 && b == 0) {
    return false;
  }
  return true;
}

ColorCategory classifyColour(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  Serial.print(F("[TCS34725 RAW] R: "));
  Serial.print(r);
  Serial.print(F(" | G: "));
  Serial.print(g);
  Serial.print(F(" | B: "));
  Serial.print(b);
  Serial.print(F(" | C: "));
  Serial.print(c);

  // Insufficient ambient/reflected clear light
  if (c < MIN_CLEAR_THRESHOLD) {
    Serial.println(F(" -> Result: OTHER (Clear intensity below threshold)"));
    return COLOR_OTHER;
  }

  // Normalized color ratios
  float rf = (float)r / (float)c;
  float gf = (float)g / (float)c;
  float bf = (float)b / (float)c;

  Serial.print(F(" | Norm: r="));
  Serial.print(rf, 3);
  Serial.print(F(" g="));
  Serial.print(gf, 3);
  Serial.print(F(" b="));
  Serial.print(bf, 3);

  ColorCategory result = COLOR_OTHER;
  if ((rf >= gf * RED_DOMINANCE_RATIO) && (rf >= bf * RED_DOMINANCE_RATIO)) {
    result = COLOR_RED;
  } else {
    result = COLOR_OTHER;
  }

  Serial.print(F(" -> Result: "));
  Serial.println(getColorName(result));

  return result;
}

const char* getColorName(ColorCategory c) {
  return (c == COLOR_RED) ? "RED" : "OTHER";
}

// ============================================================================
// STATUS LED MACHINE (PC13 - ACTIVE LOW)
// ============================================================================
inline void setLedHardware(bool turnOn) {
  digitalWrite(PIN_STATUS_LED, turnOn ? LOW : HIGH);
}

void queueLedBlinks(uint8_t count) {
  g_ledBlinksRemaining = count;
  g_ledBlinkPhaseOn    = false;
  g_ledBlinkTimerMs    = millis();
}

void updateStatusLED() {
  uint32_t now = millis();

  if (g_systemState == STATE_SENSOR_FAULT) {
    // Rapid continuous blink on fault (50ms toggle)
    static uint32_t faultBlinkMs = 0;
    static bool faultState = false;
    if (now - faultBlinkMs >= 80) {
      faultBlinkMs = now;
      faultState = !faultState;
      setLedHardware(faultState);
    }
    return;
  }

  // Handle post-sorting blink sequences
  if (g_ledBlinksRemaining > 0) {
    if (g_ledBlinkPhaseOn) {
      if (now - g_ledBlinkTimerMs >= LED_BLINK_PULSE_MS) {
        g_ledBlinkPhaseOn = false;
        g_ledBlinkTimerMs = now;
        setLedHardware(false);
        g_ledBlinksRemaining--;
      }
    } else {
      if (now - g_ledBlinkTimerMs >= LED_BLINK_PAUSE_MS) {
        g_ledBlinkPhaseOn = true;
        g_ledBlinkTimerMs = now;
        setLedHardware(true);
      }
    }
  } else {
    // Normal state indication: Steady ON when RUNNING, OFF when STOPPED
    setLedHardware(g_systemState == STATE_RUNNING);
  }
}

// ============================================================================
// COMPILE-TIME HARDWARE TEST ROUTINES (TEST_MODE > 0)
// ============================================================================
#if TEST_MODE != 0
void runSelectedTestMode() {
  static uint32_t testTimerMs = 0;
  uint32_t now = millis();

#if TEST_MODE == 1
  // Continuous Color Reading
  if (now - testTimerMs >= 300) {
    testTimerMs = now;
    uint16_t r, g, b, c;
    if (readColourSensor(r, g, b, c)) {
      classifyColour(r, g, b, c);
    } else {
      Serial.println(F("[TEST 1 ERROR] TCS34725 read failure!"));
    }
  }

#elif TEST_MODE == 2
  // Servo Calibration Sweep: 45 -> 90 -> 135 -> 90
  static uint8_t sweepStep = 0;
  if (now - testTimerMs >= 1200) {
    testTimerMs = now;
    switch (sweepStep) {
      case 0:
        diverterServo.write(SERVO_RED_ANGLE);
        Serial.println(F("[TEST 2] Servo -> 45 deg (RED / LEFT)"));
        sweepStep = 1;
        break;
      case 1:
        diverterServo.write(SERVO_CENTER_ANGLE);
        Serial.println(F("[TEST 2] Servo -> 90 deg (CENTER)"));
        sweepStep = 2;
        break;
      case 2:
        diverterServo.write(SERVO_OTHER_ANGLE);
        Serial.println(F("[TEST 2] Servo -> 135 deg (OTHER / RIGHT)"));
        sweepStep = 3;
        break;
      case 3:
      default:
        diverterServo.write(SERVO_CENTER_ANGLE);
        Serial.println(F("[TEST 2] Servo -> 90 deg (CENTER)"));
        sweepStep = 0;
        break;
    }
  }

#elif TEST_MODE == 3
  // IR Obstacle Sensor Monitor
  static bool lastReportedIr = HIGH;
  bool currentIr = digitalRead(PIN_IR_OUT);
  if (currentIr != lastReportedIr) {
    lastReportedIr = currentIr;
    Serial.print(F("[TEST 3] IR PB12 State Changed: "));
    if (currentIr == LOW) {
      Serial.println(F("BEAM BLOCKED (OBJECT PRESENT - LOW)"));
    } else {
      Serial.println(F("BEAM CLEAR (NO OBJECT - HIGH)"));
    }
  }

#elif TEST_MODE == 4
  // Motor Test Sequence: Forward -> Brake -> Reverse -> Brake
  static uint8_t motorStep = 0;
  if (now - testTimerMs >= 2000) {
    testTimerMs = now;
    switch (motorStep) {
      case 0:
        Serial.println(F("[TEST 4] Motor: FORWARD at DEFAULT_MOTOR_SPEED"));
        motorForward(DEFAULT_MOTOR_SPEED);
        motorStep = 1;
        break;
      case 1:
        Serial.println(F("[TEST 4] Motor: DYNAMIC BRAKE"));
        motorBrake();
        motorStep = 2;
        break;
      case 2:
        Serial.println(F("[TEST 4] Motor: REVERSE at DEFAULT_MOTOR_SPEED"));
        motorReverse(DEFAULT_MOTOR_SPEED);
        motorStep = 3;
        break;
      case 3:
      default:
        Serial.println(F("[TEST 4] Motor: DYNAMIC BRAKE"));
        motorBrake();
        motorStep = 0;
        break;
    }
  }
#endif
}
#endif

