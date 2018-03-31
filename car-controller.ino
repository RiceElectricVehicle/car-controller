#include <PID_v1.h>
#include <drv.h>
#include <Logger.h>
#include <SPI.h>

// SPI pins
#define SCS 8 // pin 10 is used for PWM
#define MOSI 11
#define MISO 12
#define CLK 13

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
#define I1 A1
#define I2 A2
#define Hall_left 3
#define Hall_right 2

// PID coefficients (what are we doiiiing?)
const double Kp = 0.5;
const double Ki = 0.001;
const double Kd = 0.05;

// PID setpoint, input, output
double inputPower; //power of current pedal value
double setPower; //setpoint of power 
double current_power_1; //current power of motor
double current_power_2;
double newPower;  //power output of PID


// Other PID variables

const int MAX_POWER_RATE_THRESHOLD = 500;
const int KV = 78; //relates rpm and voltage


// Hall Effect Sensor variables
volatile byte rev_count_1;
volatile byte rev_count_2;

unsigned int rpm_1; 
unsigned int rpm_2; 

unsigned int time_old_1;
unsigned int time_old_2;


// initialize some useful objects
Logger genLog("REV", "info");
drv sailboat(MOSI, MISO, CLK, SCS, LED);
PID control(&currentPower, &newPower, &setPower, Kp, Ki, Kd, DIRECT);



void setup() {
  Serial.begin(9600); 
  
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

  // wake up
  digitalWrite(SLEEP, HIGH);

  // DRV registers
  sailboat.setTBlank(0xC1); // 3us
  sailboat.setTOff(0x13); // 10us
  sailboat.setISGain(10);
  sailboat.setTorque(0x8C);
  
  //TDRIVEN and P (ns)
  sailboat.setTDriveN(263);
  sailboat.setTDriveP(263);

  // IDRIVEN and P (mA)
  sailboat.setIDriveN(100);
  sailboat.setIDriveP(50);
 
  // PWM duty cycle controlled with: 
  // pin 5-6: OCR0A/B
  // pin 9-10: OCR1A/B  A = 9 B = 10
  unsigned int read1 = sailboat.read(0x00);
  Serial.println(read1);

  // pin 9
  OCR1A = 250;
  //pin 10 
  OCR1B = 128;
  // OCR0A
  // OCR0B

  // for(OCR1B = 0; OCR1B <= 245; OCR1B++){
  //   delay(2000);
  // }

  attachInterrupt(1, hall_1_ISR, CHANGE);
  attachInterrupt(0, hall_2_ISR, CHANGE);
  rev_count_1 = 0;
  rev_count_2 = 0;
  rpm_1 = 0;
  rpm_2 = 0;
  time_old_1 = 0;
  time_old_2 = 0;

}

int x;
int y;


void loop() {

  y = map(analogRead(A0), 0, 1023, 0, 255);
  
  analogWrite(10, y);


/* 

RPM determination (millis() func returns 1/64 of millis after Timer 0 manipulation)

        #rotations            5 * (rev_count)
RPM =  -------------- =  ----------------------------
        time elapsed      millis() * 64 * 1000 * 60
*/

if(rev_count_1 >= 5){
  rpm_1 = (5 * rev_count_1) / (64000 * 60 * (millis() - time_old_1)); 
  time_old_1 = millis();
  rev_count_1 = 0;
}


if(rev_count_2 >= 5){
  rpm_2 = (5 * rev_count_2) / (64000 * 60 * (millis() - time_old_2));
  time_old_2 = millis();
  rev_count_2 = 0;
}

  // TODO: Check PWM performance

  // TODO: calculate setPower based on pedal position AND pedal position rate of change

/*

setPower is:
  linear map to motor rated power (0 to 1000W) IF rate of depression is not greater than MAX_POWER_RATE_THRESHOLD


*/


inputPower = map(analogRead(PEDAL), 380, 720, 0, 1000);
inputPower = constrain(setPower, 0, 1000);

setPower = inputPower;



  // TODO: caclulcate currentPower based on motors' voltages and currents

current_power_1 = I1 * rpm_1 * KV;

current_power_2 = I2 * rpm_2 * KV;


  // TODO: run PID.compute()
  // TODO: map newPower to pwm signals (need logic for forward, reverse, coasting)

}


void hall_left_detect_ISR(){
  /*
  Increment hall effect sensor counter on left side
  */

rev_count_1++;
Serial.println("LEFT 1 RPM");

}

void hall_right_detect_ISR(){
  /*
  Increment hall effect sensor counter on right side 
  */

rev_count_2++;
Serial.println("RIGHT 1 RPM");

}

