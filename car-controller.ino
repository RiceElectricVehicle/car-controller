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
  
  // set PWM Frequency for PWM outputs
  TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to 1 for PWM frequency of 62500.00 Hz (pins 5-6)
  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz (pins 9-10)

  // SPI Pins
  pinMode(SCS, OUTPUT); pinMode(MOSI, OUTPUT); pinMode(MISO, OUTPUT); pinMode(CLK, OUTPUT); 
  
  // PWM Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); 

  // general DRV pins
  pinMode(SLEEP, OUTPUT); pinMode(FAULT, INPUT);

  // DRV registers
  sailboat.setTBlank(0xC1); // 3us
  sailboat.setTOff(0x13)); // 10us
  sailboat.setISGain(5);
  sailboat.setTorque(0x8C);
  
  //TDRIVEN and P (ns)
  sailboat.setGDSinkTime(263);
  sailboat.setGDSourceTime(263);

  // IDRIVEN and P (mA)
  sailboat.setGDSinkPkCurrent(100);
  sailboat.setGDSourcePkCurrent(50);
 


}

void loop() {
  
  // TODO: calculate setPower based on pedal position AND pedal position rate of change
  // TODO: caclulcate currentPower based on motors' voltages and currents
  // TODO: run PID.compute()
  // TODO: map newPower to pwm signals (need logic for forward, reverse, coasting)

}


