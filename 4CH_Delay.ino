/* 4 channel PWM RC signal delay device

  - Pro Mini 3.3V / 8MHz (a Pro Micro is NOT compatible)
  - Used for my KABOLITE K336 hydraulic excavator in order to make the movements smooth and realistic
  - Uses pin change interrupts to detect the RC servo signals

*/
//
// =======================================================================================================
// LIRBARIES & TABS
// =======================================================================================================
//

#include <Servo.h>

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// RC pins
#define RC_IN_1 A0 // CH1 input
#define RC_IN_2 A1 // CH2 input
#define RC_IN_3 A2 // CH3 input
#define RC_IN_4 A3 // CH4 input

#define RC_OUT_1 9 // CH1 output
#define RC_OUT_2 8 // CH2 output
#define RC_OUT_3 7 // CH3 output
#define RC_OUT_4 6 // CH4 output



// Define global variables
volatile uint8_t prev;            // remembers state of input bits from previous interrupt
volatile uint32_t risingEdge[4];  // time of last rising edge for each channel
volatile uint32_t uSec[4];        // the latest measured pulse width for each channel

// Servo ramp times in microseconds per microsecond signal change
const uint16_t servo1RampTime = 150; // Bucket 150
const uint16_t servo2RampTime = 350; // Dipper 350
const uint16_t servo3RampTime = 7000; // Boom 7000
const uint16_t servo4RampTime = 1800; // Swing 1800

const uint16_t servo3LiftingPoint = 1700; // Boom will drop below this point because of low pump pressure 1700

// RC signal adjustment
const uint16_t servoMin = 1000;
const uint16_t servoNeutral = 1500;
const uint16_t servoMax = 2000;

const uint16_t servoMinLimit = 500;
const uint16_t servoMaxLimit = 2500;

// RC signal impulse duration (in microseconds)
int pulse[5];

// Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

  Serial.begin(9600);

  // Interrupt settings. See: http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
  PCMSK1 |= B00001111; // PinChangeMaskRegister: set the mask to allow pins A0-A3 to generate interrupts (see page 94)
  PCICR |= B00000010;  // PinChangeInterruptControlRegister: enable interupt for port C (Interrupt Enable 2, see page 92)

  // Servo setup
  servo1.attach(RC_OUT_1);
  servo2.attach(RC_OUT_2);
  servo3.attach(RC_OUT_3);
  servo4.attach(RC_OUT_4);

  readReceiver();

  servo1.writeMicroseconds(servoNeutral);
  servo2.writeMicroseconds(servoNeutral);
  servo3.writeMicroseconds(servoNeutral);
  servo4.writeMicroseconds(servoNeutral);

}

//
// =======================================================================================================
// SERVO INPUT PIN CHANGE INTERRUPT ROUTINE (Replaces Arduino pulseIn() ) working on Pro Mini only!
// =======================================================================================================
// Based on: http://ceptimus.co.uk/?p=66

ISR(PCINT1_vect) { // one or more of pins A0-A7 have changed state
  uint32_t now = micros();
  uint8_t curr = PINC; // current state of input pins A0 to A7 (we only use A0 to A3)
  uint8_t changed = curr ^ prev; // bitwise XOR (= bit has changed)
  uint8_t channel = 0;
  for (uint8_t mask = B00000001; mask <= B00001000 ; mask <<= 1) { // do it with bit 0 to 3 masked
    if (changed & mask) { // this pin has changed state
      if (curr & mask) { // Rising edge, so remember time
        risingEdge[channel] = now;
      }
      else { // Falling edge, so store pulse width
        uSec[channel] = now - risingEdge[channel];
      }
    }
    channel++;
  }
  prev = curr;
}

//
// =======================================================================================================
// READ RC SERVO CHANNELS
// =======================================================================================================
//

void readReceiver() {

  // transfer pulse lengths to existing variables
  pulse[1] = uSec[0];
  pulse[2] = uSec[1];
  pulse[3] = uSec[2];
  pulse[4] = uSec[3];

  // center channels, if no valid impulse length was detected!
  if (pulse[1] < servoMinLimit || pulse[1] > servoMaxLimit) pulse[1] = servoNeutral;
  if (pulse[2] < servoMinLimit || pulse[2] > servoMaxLimit) pulse[2] = servoNeutral;
  if (pulse[3] < servoMinLimit || pulse[3] > servoMaxLimit) pulse[3] = servoNeutral;
  if (pulse[4] < servoMinLimit || pulse[4] > servoMaxLimit) pulse[4] = servoNeutral;
}

//
// =======================================================================================================
// DRIVE SERVO OUTPUTS
// =======================================================================================================
//

void driveServo1() { // Bucket

  static uint32_t lastFrameTime = micros();
  static uint16_t servoPos = servoNeutral;

  // Ramp and limit servo position
  if (micros() - lastFrameTime > servo1RampTime) {
    lastFrameTime = micros();
    if (pulse[1] < servoPos) servoPos--;
    if (pulse[1] > servoPos) servoPos++;
    constrain(servoPos, servoMin, servoMax);

    // drive servo
    servo1.writeMicroseconds(servoPos);
    //Serial.print(pulse[1]);
    //Serial.print("  ");
    //Serial.println(servoPos);
  }
}

void driveServo2() { // Dipper

  static uint32_t lastFrameTime = micros();
  static uint16_t servoPos = servoNeutral;

  // Ramp and limit servo position
  if (micros() - lastFrameTime > servo2RampTime) {
    lastFrameTime = micros();
    if (pulse[2] < servoPos) servoPos--;
    if (pulse[2] > servoPos) servoPos++;
    constrain(servoPos, servoMin, servoMax);

    // drive servo
    servo2.writeMicroseconds(servoPos);
  }
}

void driveServo3() { // Dipper

  static uint32_t lastFrameTime = micros();
  static uint16_t servoPos = servoNeutral;
  uint16_t servoOut;

  // Ramp and limit servo position
  if (micros() - lastFrameTime > servo2RampTime) {
    lastFrameTime = micros();
    if (pulse[3] < servoPos) servoPos--;
    if (pulse[3] > servoPos) servoPos++;
    constrain(servoPos, servoMin, servoMax);

  // Valve offset (add dead zone upwards) to prevent boom from falling because of low pump pressure
  if (servoPos > servo3LiftingPoint) servoOut = map(servoPos, servo3LiftingPoint, servoMax, servoNeutral, servoMax);
  else if (servoPos < servoNeutral) servoOut = servoPos;
  else servoOut = servoNeutral;

    // drive servo
    servo3.writeMicroseconds(servoOut);
  }
}

void driveServo4() {

  static uint32_t lastFrameTime = micros();
  static uint16_t servoPos = servoNeutral;

  // Ramp and limit servo position
  if (micros() - lastFrameTime > servo4RampTime ) {
    lastFrameTime = micros();
    if (pulse[4] < servoPos) servoPos--;
    if (pulse[4] > servoPos) servoPos++;
    constrain(servoPos, servoMin, servoMax);

    // drive servo
    servo4.writeMicroseconds(servoPos);
  }
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {

  // Read RC receiver channels
  readReceiver();

  // Write servo outputs
  driveServo1();
  driveServo2();
  driveServo3();
  driveServo4();
}
