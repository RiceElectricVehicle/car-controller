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
#define SLEEP 4
#define FAULT 7

// input pins
#define PEDAL A0
#define HALLA A1
#define HALLB A2
#define ISENSEA A3
#define ISENSEB A4
#define VBAT A5

// PID coefficients (what are we doiiiing?)
const double Kp = 0.5;
const double Ki = 0.001;
const double Kd = 0.05;

// PID setpoint, input, output
double throttle_setpoint;
double throttle_new;
double currentA;
double currentB;
double rpmA;
double rpmB;

// initialize some useful objects
Logger genLog("REV", "info");
drv sailboat(MOSI, MISO, CLK, SCS);

//PID won't be used for this one
//PID control(&currentPower, &newPower, &setPower, Kp, Ki, Kd, DIRECT);



void setup() {
  Serial.begin(9600); 
  
  // set PWM Frequency for PWM outputs to 62.5 kHz
  // TIMER/COUNTER 0
  // PIN 6(OC0A) = NON-INVERTED [_BV(COM0A1)]
  // PIN 5(OC0B) = NON-INVERTED [_BV(COM0B1)
  // PWM MODE    = FAST PWM     [_BV(WGM01) | _BV(WGM00)]
  // PRESCALER   = 1            [_BV(CS00)]
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS01);

  // TIMER/COUNTER 1
  // PIN 9(OC1A)  = NON-INVERTED [_BV(COM1A1)]
  // PIN 10(OC1B) = NON-INVERTED [_BV(COM1B1)]
  // PWM MODE     = FAST PWM 8BIT[_BV(WGM12) | _BV(WGM10)]
  // PRESCALER    = 1            [_BV(CS10)]
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10); 
  TCCR1B = _BV(WGM12) | _BV(CS11);

  // SPI Pins
  pinMode(SCS, OUTPUT); pinMode(MOSI, OUTPUT); pinMode(MISO, OUTPUT); pinMode(CLK, OUTPUT); 
  
  // PWM Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); 

  // general DRV pins
  pinMode(SLEEP, OUTPUT); pinMode(FAULT, INPUT);
  
  //wakeup
  digitalWrite(SLEEP, HIGH);
  delay(1000);

  //DRV registers
  sailboat.setISGain(10); // 5 for car (not tested)
  sailboat.setTorque(0x17); // 0x87 for car (not tested yet)

  sailboat.setDTime(410);
  sailboat.setTOff(0x1D); // * 525 ns
  sailboat.setTBlank(0x00); // 1 us
 
  sailboat.setDecMode("auto");
  
  //sailboat.setOCPDeglitchTime(1.05);
  //TDRIVEN and P (ns)
  sailboat.setTDriveN(525);
  sailboat.setTDriveP(525);

  // IDRIVEN and P (mA)
  sailboat.setIDriveN(100);
  sailboat.setIDriveP(50);
 
  // PWM duty cycle controlled with: 
  // pin 5-6: OCR0A/B
  // pin 9-10: OCR1A/B (dont know which is A or B).

}

int i;
void loop() {
  
  // TODO: Check PWM performance

  // TODO: calculate setPower based on pedal position AND pedal position rate of change
  // TODO: caclulcate currentPower based on motors' voltages and currents
  // TODO: run PID.compute()
  // TODO: map newPower to pwm signals (need logic for forward, reverse, coasting)

  if (digitalRead(FAULT) == LOW) {
    // Serial.print("[");
    // for(i = 0; i <= 6; i++) {
    //   Serial.print(sailboat.faults[0]);
    //   Serial.print(", ");
    // }
    // Serial.println("]");
    Serial.println(sailboat.read(0x07) & 0x3F);
  }
  
  // read pedal input
  throttle_setpoint = analogRead(PEDAL);

  // re-map throttle to range 0-255
  if(throttle_setpoint < 100){
    throttle_new = 0;
  }
  else {
    throttle_new = map(throttle_setpoint, 0, 1023, 220, 255);
  }

  throttle_new = constrain(pow(throttle_new, 2) / 255, 0, 255);


  // write PWM to both motors 
  analogWrite(AIN1, throttle_new);
  analogWrite(BIN1, throttle_new);

  // write 0 to xIN2 to go forward
  analogWrite(AIN2, 0);
  analogWrite(BIN2, 0);

  



}




