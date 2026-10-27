// Fly-brain car: ONE sketch for all 3 stages. The laptop app picks the stage.
//
// Laptop -> Arduino                         Arduino -> laptop
//   MODE,0   stop                             READY            after power-up
//   MODE,1   stage 1: if-else, here           MODE,n           stage changed
//   MODE,2   stage 2: laptop drives, sonar    P,dir,cm,ms      one sonar ping: dir = LL L C R RR
//   MODE,3   stage 3: laptop drives, camera   W,left,right     wheel speeds (stage 1)
//            (stage 3 also pings straight ahead, P,C,..: only for the laptop's stuck reflex, not the brain)
//                                             STUCK            stage 1 stuck reflex fired
//                                             TRAP,why         stage 1 big escape (deadend / repeat)
//                                             H,mV             heartbeat twice a second + the lowest
//                                                              5 V rail voltage since the last one
//   M,L,R    wheel speeds -255..255 (stage 2 and 3)
//   LOOK,LL / LOOK,L / LOOK,C / LOOK,R / LOOK,RR   point the servo (only when stopped, for testing)

#include <Servo.h>

// ---------- pins ----------
// The "tiiii" sound while driving = the motor power is switched on/off 980 times a second (PWM on
// pins 5 and 6 of the Uno), and the motors sing at that frequency. To make them silent, move 3 wires:
//   L298N ENA: D5 -> D11      L298N ENB: D6 -> D3      servo signal: D3 -> D4
// and set QUIET_WIRING to 1. Then the motor power is switched 31,000 times a second: too high to hear.
#define QUIET_WIRING 0
const int TRIG = A0, ECHO = A1;          // HC-SR04
#if QUIET_WIRING
const int SERVO_PIN = 4;                 // servo signal (orange/yellow wire)
const int ENA = 11, ENB = 3;             // L298N enable pins on timer 2 (31 kHz)
#else
const int SERVO_PIN = 3;                 // servo signal (orange/yellow wire)
const int ENA = 5, ENB = 6;              // L298N enable pins (980 Hz: audible whine)
#endif
const int IN1 = 7, IN2 = 8;              // L298N, left motors
const int IN3 = 9, IN4 = 10;             // L298N, right motors

// ---------- servo: the "radar" ----------
// The sensor sweeps 5 directions, like a ship's radar:
//   name     LL     L     C     R     RR
//   angle   +70   +35     0   -35    -70     (degrees from straight ahead, + = left)
// Order: C L LL L C R RR R ... = every step is 35 degrees, and straight ahead is checked every 4th step.
// The HC-SR04 hears echoes in a cone of about +-15 degrees, so 5 directions cover almost +-85.
const int CENTER_DEG = 90;               // servo position that looks straight ahead
const int NEAR_DEG = 35, WIDE_DEG = 70;  // WIDE_DEG at most 80 (the servo goes 0..180)
const int LEFT_SIGN = 1;                 // if "L" looks to the RIGHT: change to -1
// wait for the servo to arrive before each ping: 30 ms + 2.2 ms per degree moved (35 deg -> 107 ms)
const unsigned long SETTLE_BASE_MS = 30, SETTLE_PER_10DEG = 22;

// ---------- safety / start ----------
const unsigned long WATCHDOG_MS = 300;   // stage 2/3: no M note for 300 ms -> motors stop
const byte START_MODE = 0;               // 1 = start stage 1 by itself at power-up (no laptop)

// ---------- stage 1 rules ----------
const int STOP_CM = 25;                  // something ahead closer than this: back up and turn
const int VEER_CM = 40;                  // L or R (35 deg) closer than this: veer away
const int SIDE_CM = 20;                  // LL or RR (70 deg) closer than this: veer away
const int FAST = 200, SLOW = 90, TURN = 180;
const unsigned long BACK_MS = 400, SPIN_MS = 350;

// ---------- stuck reflex (stage 1) ----------
// Driving forward, but the distance ahead stays the same for 1.5 s -> the car is stuck
// (a corner caught on something). Back up longer, turn, try again. Tells the laptop "STUCK".
const int STUCK_CM = 4;                  // ahead changed less than this ...
const unsigned long STUCK_MS = 1500;     // ... for this long = stuck
const unsigned long ESCAPE_BACK_MS = 700;

// ---------- short-term memory (stage 1) ----------
// The car has no map (it can't measure where it is), but it remembers what happened in the last
// few seconds: escapes close together are "the same trap". Then it keeps turning the SAME way
// (no left-right-left dithering in a corner) and turns further each time; the 3rd time, or in a
// dead end (blocked ahead, left AND right), it backs up longer and turns about half a circle.
const int DEADEND_CM = 35;               // after backing up, both sides still < this = dead end
const int CLOSE_CM = 10;                 // a side this close: pivot away instead of a gentle curve
const unsigned long MEMORY_MS = 3000;    // escapes less than 3 s apart = the same trap
// (was 8 s: in a 2 m arena the car meets a wall every 3-5 s anyway, so EVERY escape counted as
//  'the same trap' -> double turns and a big escape every 3rd time. Simulator: half the touching.)
const unsigned long BIG_BACK_MS = 800;   // big escape: back up this long ...
const byte BIG_SPIN = 3;                 // ... and spin 3 x SPIN_MS (tune: about half a circle)

Servo servo;
byte mode = 0;
char line[32];
byte lineLen = 0;
unsigned long lastCommand = 0;
int wheelL = 0, wheelR = 0;
unsigned long lastWheelReport = 0;

const byte NB = 5;                                        // number of directions
enum { B_LL = 0, B_L = 1, B_C = 2, B_R = 3, B_RR = 4 };
const char* const BEAM_NAME[NB] = {"LL", "L", "C", "R", "RR"};
const int BEAM_DEG[NB] = {WIDE_DEG, NEAR_DEG, 0, -NEAR_DEG, -WIDE_DEG};
const byte SWEEP[8] = {B_C, B_L, B_LL, B_L, B_C, B_R, B_RR, B_R};
byte scanIdx = 0;
bool scanning = false;
unsigned long servoMovedAt = 0, settleMs = 200;
int servoAt = CENTER_DEG;
int dist[NB] = {400, 400, 400, 400, 400};
bool freshCenter = false;

// Missed-echo filter: the HC-SR04 reads 400 ("nothing there") when it hears no echo. That happens
// (1) now and then by chance, and (2) ALWAYS when the sensor is pressed against a wall (closer than
// ~3 cm the echo comes back before it starts listening). Case 2 made the car push into walls forever.
//   - a 400 right after a reading closer than HOLD_NEAR_CM: keep the last value for up to HOLD_MS
//     (driving forward, a wall can't jump from 20 cm to "nothing")
//   - a 400 right after a reading closer than MISS_NEAR_CM: keep the last value once
// After an escape (backed up and turned) the sensor looks somewhere new: 400s are believed again.
const int HOLD_NEAR_CM = 60;
const unsigned long HOLD_MS = 2500;
const int MISS_NEAR_CM = 150;
bool held[NB];                           // the value of this direction is being held
unsigned long heldSince[NB];

byte s1State = 0;                        // 0 drive, 1 back up, 2 spin, 3 wait for a fresh center reading
unsigned long s1Until = 0;
bool spinRight = true;
byte trapCount = 0;                      // escapes in the current trap
unsigned long lastEscape = 0;
bool lastTurnRight = true;
bool sameTrap = false, bigEscape = false;


int stuckRef = -1;                       // the distance ahead when it last changed (-1 = unknown)
unsigned long stuckSince = 0;

void resetStuck() {
  stuckRef = -1;
  stuckSince = millis();
}

void stuckWatch(int cm) {                // called with every distance measured straight ahead
  if (cm >= 400) {                       // nothing in range: can't tell if we move
    resetStuck();
  } else if (stuckRef < 0 || abs(cm - stuckRef) > STUCK_CM) {
    stuckRef = cm;                       // it changed: we are moving
    stuckSince = millis();
  }
}

void aim(byte b) {                       // point the sensor at direction b
  int deg = CENTER_DEG + LEFT_SIGN * BEAM_DEG[b];
  settleMs = SETTLE_BASE_MS + (unsigned long)abs(deg - servoAt) * SETTLE_PER_10DEG / 10;
  servoAt = deg;
  servo.write(deg);
  servoMovedAt = millis();
}

int beamByName(const char* name) {       // "LL" -> B_LL ... ; -1 if unknown
  for (byte b = 0; b < NB; b++)
    if (strcmp(name, BEAM_NAME[b]) == 0) return b;
  return -1;
}

// ---------- 5 V rail watch ----------
// The Arduino measures its own 5 V (= the servo's power) against its built-in 1.1 V reference.
// No extra wires. If it drops below ~4.5 V while the motors run, the servo stops turning.
int vccMin = 0;                          // lowest mV since the last heartbeat (0 = not measured)
#ifdef ADMUX
void vccBegin() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);   // measure the 1.1 V reference against 5 V
  ADCSRA |= _BV(ADEN) | _BV(ADSC);
}
void vccPoll() {                         // non-blocking: read the last conversion, start the next
  if (ADCSRA & _BV(ADSC)) return;
  unsigned int r = ADC;
  if (r > 0 && millis() > 200) {
    int mv = (int)(1125300L / r);
    if (vccMin == 0 || mv < vccMin) vccMin = mv;
  }
  ADCSRA |= _BV(ADSC);
}
#else
void vccBegin() {}
void vccPoll() {}
#endif

int readCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long us = pulseIn(ECHO, HIGH, 25000UL);
  if (us == 0) return 400;
  unsigned long cm = us / 58;
  return cm > 400 ? 400 : (int)cm;
}

// ---------- motors ----------
// The motor battery: write its voltage WHILE THE MOTORS RUN (measure with the multimeter).
// The code then scales the motor power so the motors get at most about 6 V, whatever battery you use.
// 6 alkaline AA cells read 9.2 V at rest but sag to ~8 V when 4 motors pull: 8.0 = full power.
// (Rechargeable lithium 3S pack, 12.6 V: write 12.0 - otherwise the 6 V motors get far too much.)
const float BATTERY_V = 8.0;
const float MOTOR_MAX_V = 6.0;          // yellow TT gear motors are made for 3-6 V
const float MOTOR_MIN_V = 1.8;          // below this the motors only buzz (tune: smallest that still rolls)
const float DRIVER_DROP_V = 2.0;        // the L298N itself loses about 2 V
const float MOTOR_SUPPLY_V = BATTERY_V - DRIVER_DROP_V;
const int MAX_PWM = 255.0 * MOTOR_MAX_V / MOTOR_SUPPLY_V > 255 ? 255 : (int)(255.0 * MOTOR_MAX_V / MOTOR_SUPPLY_V);
const int MIN_PWM = (int)(255.0 * MOTOR_MIN_V / MOTOR_SUPPLY_V);
// (12 V pack: MAX_PWM 153, MIN_PWM 45.  7.4 V pack: 255 and 85.)
// Any real speed command (12..255) is lifted into MIN_PWM..MAX_PWM.
// Soft start: the speed changes at most RAMP_PER_MS per millisecond, and a change of direction
// always passes through stop first. Four motors jumping from forward to reverse pull a huge
// current spike: the battery voltage dips and the Arduino and the camera restart.
const int RAMP_PER_MS = 2;              // 0 -> full speed in about 0.1 s

int outL = 0, outR = 0;                 // what the motors get right now (after ramp)
unsigned long lastMotorUpdate = 0;

int lift(int speed) {
  int a = abs(speed);
  if (a < 12) return 0;                                // tiny = stop
  int out = MIN_PWM + (long)a * (MAX_PWM - MIN_PWM) / 255;
  return speed > 0 ? out : -out;
}

void setSide(int en, int a, int b, int pwm) {           // pwm: -MAX_PWM..MAX_PWM, already lifted
  if (pwm >= 0) {
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
    analogWrite(en, pwm);
  } else {
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
    analogWrite(en, -pwm);
  }
}

int stepToward(int cur, int target, int maxStep) {
  if ((cur > 0 && target < 0) || (cur < 0 && target > 0)) target = 0;   // reversing: stop first
  if (target > cur) return cur + maxStep < target ? cur + maxStep : target;
  return cur - maxStep > target ? cur - maxStep : target;
}

void motorUpdate() {                                    // called every loop: moves the motors toward wheelL/R
  unsigned long now = millis();
  unsigned long dt = now - lastMotorUpdate;
  if (dt == 0) return;
  lastMotorUpdate = now;
  int step = dt > 100 ? 255 : (int)(RAMP_PER_MS * dt);
  outL = stepToward(outL, lift(wheelL), step);
  outR = stepToward(outR, lift(wheelR), step);
  setSide(ENA, IN1, IN2, outL);
  setSide(ENB, IN3, IN4, outR);
}

void drive(int left, int right) {                       // sets the target; motorUpdate() gets there smoothly
  wheelL = constrain(left, -255, 255);
  wheelR = constrain(right, -255, 255);
  motorUpdate();
}

bool timeReached(unsigned long t) {
  return (long)(millis() - t) >= 0;
}

void setMode(byte m) {
  mode = m;
  drive(0, 0);
  s1State = 0;
  resetStuck();
  lastEscape = 0;                        // a new stage forgets old traps
  trapCount = 0;
  scanning = (m >= 1 && m <= 3);         // stage 3: straight ahead only (servo still)
  scanIdx = 0;
  for (byte b = 0; b < NB; b++) {
    held[b] = false;
    dist[b] = 400;
  }
  aim(B_C);
  lastCommand = millis();
  Serial.print("MODE,");
  Serial.println(m);
}

void scanStep() {
  if (!scanning) return;
  if (millis() - servoMovedAt < settleMs) return;
  byte b = (mode == 3) ? B_C : SWEEP[scanIdx];
  int cm = readCm();
  int prev = dist[b];
  bool hold = false;
  if (cm >= 400) {
    if (prev < HOLD_NEAR_CM) hold = !held[b] || millis() - heldSince[b] < HOLD_MS;   // too close / missed
    else if (prev < MISS_NEAR_CM) hold = !held[b];                                   // one missed echo
  }
  if (hold) {
    if (!held[b]) heldSince[b] = millis();
    held[b] = true;
    cm = prev;
  } else if (cm < 400) {
    held[b] = false;
  } else {
    held[b] = true;                      // a believed 400: don't start a new hold from it
    heldSince[b] = 0;
  }
  dist[b] = cm;
  if (b == B_C) {
    freshCenter = true;
    stuckWatch(cm);
  }
  Serial.print("P,");
  Serial.print(BEAM_NAME[b]);
  Serial.print(',');
  Serial.print(cm);
  Serial.print(',');
  Serial.println(millis());
  if (mode == 3) {
    servoMovedAt = millis();             // stage 3: the servo stays straight, just wait for the next ping
    settleMs = SETTLE_BASE_MS + 70;
  } else {
    scanIdx = (scanIdx + 1) % 8;
    aim(SWEEP[scanIdx]);
  }
}

void startEscape(unsigned long backMs) {
  sameTrap = lastEscape != 0 && millis() - lastEscape < MEMORY_MS;
  trapCount = sameTrap ? trapCount + 1 : 1;
  lastEscape = millis();
  bigEscape = trapCount >= 3;                          // 3rd escape in the same trap: big one
  if (bigEscape) {
    Serial.println("TRAP,repeat");
    trapCount = 0;
  }
  for (byte b = 0; b < NB; b++) {        // backing up + turning: the sensor will look somewhere new
    held[b] = true;
    heldSince[b] = 0;
  }
  s1State = 1;
  s1Until = millis() + (bigEscape && backMs < BIG_BACK_MS ? BIG_BACK_MS : backMs);
  drive(-TURN, -TURN);
}

int nearest(byte a, byte b) { return dist[a] < dist[b] ? dist[a] : dist[b]; }
int space(byte a, byte b) { return min(dist[a], 200) + min(dist[b], 200); }

// How far ahead is the nearest thing IN THE CAR'S PATH? Straight ahead, plus the 35-degree beams
// when what they see is less than ~16 cm to the side (closer than 28 cm): then it is in front of
// a wheel, at 0.82 x its distance ahead.
int aheadCm() {
  int a = dist[B_C];
  if (dist[B_L] < 28) a = min(a, dist[B_L] * 82 / 100);
  if (dist[B_R] < 28) a = min(a, dist[B_R] * 82 / 100);
  return a;
}

void stage1() {
  if (s1State == 1) {                                  // backing up
    drive(-TURN, -TURN);
    if (timeReached(s1Until)) {
      s1State = 2;
      // the sides were measured again while backing up: blocked left AND right = dead end
      if (!bigEscape && nearest(B_L, B_LL) < DEADEND_CM && nearest(B_R, B_RR) < DEADEND_CM) {
        Serial.println("TRAP,deadend");
        bigEscape = true;
      }
      // same trap as a moment ago: keep the same direction; otherwise toward the side with more space
      spinRight = sameTrap ? lastTurnRight : (space(B_R, B_RR) > space(B_L, B_LL));
      lastTurnRight = spinRight;
      byte times = bigEscape ? BIG_SPIN : (sameTrap ? 2 : 1);
      s1Until = millis() + SPIN_MS * times;
    }
    return;
  }
  if (s1State == 2) {                                  // spinning
    if (spinRight) drive(TURN, -TURN);
    else drive(-TURN, TURN);
    if (timeReached(s1Until)) {
      s1State = 3;
      freshCenter = false;
      drive(0, 0);
      scanIdx = 0;                                     // look straight ahead right away
      aim(B_C);
    }
    return;
  }
  if (s1State == 3) {                                  // stand still until the sensor has looked ahead again
    drive(0, 0);
    if (freshCenter) {
      s1State = 0;
      resetStuck();
    }
    return;
  }
  if (stuckRef >= 0 && millis() - stuckSince > STUCK_MS) {   // stuck: back up longer, then turn
    Serial.println("STUCK");
    resetStuck();
    startEscape(ESCAPE_BACK_MS);
    return;
  }
  if (aheadCm() < STOP_CM) {
    startEscape(BACK_MS);
    return;
  }
  bool leftClose = dist[B_L] < VEER_CM || dist[B_LL] < SIDE_CM;
  bool rightClose = dist[B_R] < VEER_CM || dist[B_RR] < SIDE_CM;
  if (leftClose && rightClose) {                       // a narrow passage: steer to the wider side,
    int diff = space(B_L, B_LL) - space(B_R, B_RR);    // or straight on if both are about the same
    leftClose = diff < -10;
    rightClose = diff > 10;
  }
  if (leftClose) {
    drive(FAST, nearest(B_L, B_LL) < CLOSE_CM ? -SLOW : SLOW);   // veer (or pivot) right
  } else if (rightClose) {
    drive(nearest(B_R, B_RR) < CLOSE_CM ? -SLOW : SLOW, FAST);   // veer (or pivot) left
  } else {
    drive(FAST, FAST);
  }
}

void reportWheels() {
  if (millis() - lastWheelReport < 200) return;
  lastWheelReport = millis();
  Serial.print("W,");
  Serial.print(wheelL);
  Serial.print(',');
  Serial.println(wheelR);
}

void handleLine() {
  if (strncmp(line, "MODE,", 5) == 0) {
    int m = atoi(line + 5);
    if (m >= 0 && m <= 3) setMode((byte)m);
    return;
  }
  if (strncmp(line, "LOOK,", 5) == 0) {
    int b = beamByName(line + 5);
    if (mode == 0 && b >= 0) aim((byte)b);
    return;
  }
  if (line[0] == 'M' && line[1] == ',') {
    if (mode != 2 && mode != 3) return;
    char* comma = strchr(line + 2, ',');
    if (!comma) return;
    *comma = 0;
    drive(atoi(line + 2), atoi(comma + 1));
    lastCommand = millis();
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLen > 0) {
        line[lineLen] = 0;
        handleLine();
      }
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen] = ch;
      lineLen++;
    } else {
      lineLen = 0;
    }
  }
}

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
#if QUIET_WIRING
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  // timer 2 (pins 3 and 11): PWM at 31 kHz, silent
#endif
  drive(0, 0);
  servo.attach(SERVO_PIN);
  aim(B_C);
  vccBegin();
  Serial.begin(115200);
  Serial.println("READY");                // the laptop sees this mid-run = the Arduino restarted (power dip)
  setMode(START_MODE);
}

unsigned long lastBeat = 0;

void loop() {
  readSerial();
  scanStep();
  motorUpdate();
  vccPoll();
  if (millis() - lastBeat >= 500) {      // heartbeat: "the link and I are alive" + lowest 5 V since last time
    lastBeat = millis();
    Serial.print("H,");
    Serial.println(vccMin);
    vccMin = 0;
  }
  if (mode == 1) {
    stage1();
    reportWheels();
  }
  if ((mode == 2 || mode == 3) && millis() - lastCommand > WATCHDOG_MS) drive(0, 0);
}
