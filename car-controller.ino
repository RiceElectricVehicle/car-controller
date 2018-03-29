#include <PID_v1.h>
#include <drv.h>
#include <Logger.h>
#include <SPI.h>

// SPI pins
#define SCS 8 // pin 10 is used for PWM
#define MOSI 11
#define MISO 12
#define CLK 13
#define LED 2

// PWM output pins
#define AIN1 10
#define AIN2 9
#define BIN1 6
#define BIN2 5

// MISC pins
#define SLEEP 7
#define FAULT 4

// input pins
#define PEDAL A0
#define V1 A1
#define V2 A2
#define I1 A3
#define I2 A4
#define VBAT A5

// PID coefficients (what are we doiiiing?)
const double Kp = 0.5;
const double Ki = 0.001;
const double Kd = 0.05;

// PID setpoint, input, output
double setPower;
double currentPower;
double newPower;

// initialize some useful objects
Logger genLog("REV", "info");
drv sailboat(MOSI, MISO, CLK, SCS, LED);
PID control(&currentPower, &newPower, &setPower, Kp, Ki, Kd, DIRECT);



void setup() {
  Serial.begin(115200); 
  
  // set PWM Frequency for PWM outputs to 62.5 kHz
  // TIMER/COUNTER 0
  // PIN 6(OC0A) = NON-INVERTED [_BV(COM0A1)]
  // PIN 5(OC0B) = NON-INVERTED [_BV(COM0B1)
  // PWM MODE    = FAST PWM     [_BV(WGM01) | _BV(WGM00)]
  // PRESCALER   = 1            [_BV(CS00)]
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS00); 

  // TIMER/COUNTER 1
  // PIN 9(OC1A)  = NON-INVERTED [_BV(COM1A1)]
  // PIN 10(OC1B) = NON-INVERTED [_BV(COM1B1)]
  // PWM MODE     = FAST PWM 8BIT[_BV(WGM12) | _BV(WGM10)]
  // PRESCALER    = 1            [_BV(CS10)]
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10); 
  TCCR1B = _BV(WGM12) | _BV(CS10);

  // SPI Pins
  pinMode(SCS, OUTPUT); pinMode(MOSI, OUTPUT); pinMode(MISO, OUTPUT); pinMode(CLK, OUTPUT); 
  
  // PWM Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); 

  // general DRV pins
  pinMode(SLEEP, OUTPUT); pinMode(FAULT, INPUT);

  // DRV registers
  sailboat.setTBlank(0xC1); // 3us
  sailboat.setTOff(0x13); // 10us
  sailboat.setISGain(5);
  sailboat.setTorque(0x8C);
  
  //TDRIVEN and P (ns)
  sailboat.setGDSinkTime(263);
  sailboat.setGDSourceTime(263);

  // IDRIVEN and P (mA)
  sailboat.setGDSinkPkCurrent(100);
  sailboat.setGDSourcePkCurrent(50);
 
  // PWM duty cycle controlled with: 
  // pin 5-6: OCR0A/B
  // pin 9-10: OCR1A/B (dont know which is A or B).


}

void loop() {
  
  // TODO: Check PWM performance

  // TODO: calculate setPower based on pedal position AND pedal position rate of change
  // TODO: caclulcate currentPower based on motors' voltages and currents
  // TODO: run PID.compute()
  // TODO: map newPower to pwm signals (need logic for forward, reverse, coasting)

}


