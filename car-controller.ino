#include <PID_v1.h>
#include <drv.h>
#include <Logger.h>
#include <SPI.h>

// PWM SCALER
#define SCALER 8

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
#define HALLA 3
#define HALLB 2
#define ISENSEA A1
#define ISENSEB A2
#define VBAT A5

// misc constants
#define ISENSEREF A3 // read pin for reference point of Amps
#define ISENSEAMP 10// TODO: measure resistance across each R_G, calculate individial gains
const double sense_conductance = 100; // resistance = 1/conductance, to avoid division by zero
const double R_A = 2;
const double R_B = 2;
const int Kv = 75;

// PID coefficients (what are we doiiiing?)
const double Kp = 0.5;
const double Ki = 0.001;
const double Kd = 0.05;

// PID setpoint, input, output
double throttle_setpoint;
double throttle_new;

// Hall Effect Sensor variables
volatile int rev_count_A;
volatile int rev_count_B;

double wheel_rpm_A; 
double wheel_rpm_B;

unsigned int motor_rpm_A;
unsigned int motor_rpm_B;

unsigned long now;
unsigned long time_old_A;
unsigned long time_old_B;

double motor_volt_A;
double motor_volt_B;

double motor_current_A;
double motor_current_B;

int pwm_A;
int pwm_B;

#define GEAR_RATIO 1;


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

  // hall pins
  pinMode(HALLA, INPUT); pinMode(HALLB, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALLA), hall_A_ISR, CHANGE); //maps to pin 3
  attachInterrupt(digitalPinToInterrupt(HALLB), hall_B_ISR, CHANGE); //pin 2

  rev_count_A = 0;
  rev_count_B = 0;
  wheel_rpm_A = 0;
  wheel_rpm_B = 0;
  time_old_A = 0;
  time_old_B = 0;
  
  //wakeup
  digitalWrite(SLEEP, HIGH);
  sailboat.setLogging("error");
  genLog.setLevel("error");
  delay(1000);


  //DRV registers
  sailboat.setISGain(5); // 5 for car (not tested)
  sailboat.setTorque(0x23); // 0x87 for car (not tested yet)

  sailboat.setDTime(670); // ns
  sailboat.setTOff(0x1D); // * 525 ns =  16 us
  sailboat.setTBlank(0xFF); // 5.88 us

  // DRV will limit current at 43 kHz with 37% dutycycle
 
  sailboat.setDecMode("slow");
  
  //sailboat.setOCPDeglitchTime(1.05);
  //TDRIVEN and P (ns)
  sailboat.setTDriveN(2100);
  sailboat.setTDriveP(2100);

  // IDRIVEN and P (mA)
  sailboat.setIDriveN(200);
  sailboat.setIDriveP(100);

  sailboat.setOCPThresh();
  sailboat.setOCPDeglitchTime();

 
  // PWM duty cycle controlled with: 
  // pin 5-6: OCR0A/B
  // pin 9-10: OCR1A/B (dont know which is A or B).

}

bool check = true;
void loop() {
  

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
  if(throttle_setpoint < 75){
    throttle_new = 0;
  }
  else {
    throttle_new = map(throttle_setpoint, 0, 1023, 220, 255);
  }

  throttle_new = constrain(pow(throttle_new, 2) / 255, 0, 255);
  
  // RPM determination (millis() func returns xSCALER of millis after Timer 0 manipulation)
  //unsigned long time = millis() / SCALER;
  if(rev_count_A >= 15) {
    now = millis();
    wheel_rpm_A = (1000 * SCALER * rev_count_A ) / (60 * (now - time_old_A));
    wheel_rpm_A = wheel_rpm_A / 20;
    time_old_A = millis();
    rev_count_A = 0;
  }


  if(rev_count_B >= 15) {
    now = millis();
    wheel_rpm_B = (1000 * SCALER * rev_count_B ) / (60 * (now - time_old_B));
    wheel_rpm_B = wheel_rpm_B / 20;
    time_old_B = millis();
    rev_count_B = 0;
  }
  // DetermineAmotor RPM using wheel RPM
  motor_rpm_A = wheel_rpm_A * GEAR_RATIO;
  motor_rpm_B = wheel_rpm_B * GEAR_RATIO;

  // account for very slow speed (less than 1 full rotation) by constraining RPM to be greater than 0
  if(motor_rpm_A == 0){
    motor_rpm_A = 4;
  }

  if (motor_rpm_B == 0){
    motor_rpm_B = 4;
  }
  

  motor_current_A = get_current(analogRead(ISENSEA), analogRead(ISENSEREF), sense_conductance, ISENSEAMP);
  motor_current_B = get_current(analogRead(ISENSEB), analogRead(ISENSEREF), sense_conductance, ISENSEAMP);
  motor_volt_A = get_voltage(motor_rpm_A, motor_current_A, R_A, Kv);
  motor_volt_B = get_voltage(motor_rpm_B, motor_current_B, R_B, Kv);

  //Serial.print("RPM = ");
  //Serial.println(wheel_rpm_A);

  //Serial.print("CURRENT A: ");
  //Serial.println(motor_current_A);
  //Serial.print("CURRENT B: ");
  //Serial.println(motor_current_B);
  
  //Serial.print("VOLT A: ");
  //Serial.println(motor_volt_A);
  //Serial.print("VOLT B: ");
  //Serial.println(motor_volt_B);

  // write PWM to both motors 
  analogWrite(AIN1, throttle_new);
  analogWrite(BIN1, 0);

  // write 0 to xIN2 to go forward
  analogWrite(AIN2, 0);
  analogWrite(BIN2, 0);

}
double get_voltage(int rpm, int current, int R_m, int Kv) {
  return rpm / Kv + current * R_m;
}

double get_current(int dac, int reference, int conductance, int amplification) {

  return (((double) conductance / (double)(1023 * amplification)) * (double) (dac - reference));
}

void hall_A_ISR(){
  /*
  Interrupt: Increment hall effect sensor counter on left side
  */
  genLog.logi("HALL ISR A tripped");
  rev_count_A++;
}

void hall_B_ISR(){
  /*
  Interrupt: Increment hall effect sensor counter on right side 
  */
  //genLog.logi("HALL ISR B tripped");
  rev_count_B++;
}





