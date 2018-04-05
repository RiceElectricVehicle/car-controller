#include <PID_v1.h>
#include <drv.h>
#include <Logger.h>
#include <SPI.h>

// scaler for millis and delay funcitons
#define SCALER 8

// SPI pins
#define SCS 8
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
#define ISENSEA A1
#define ISENSEB A2
#define ISENSEAMP 20
#define ISENSEOFFSET 0
#define HALLA 3
#define HALLB 2

// PID coefficients (what are we doiiiing?)
const double Kp = 0.05;
const double Ki = 0.001;
const double Kd = 0.005;

// PID setpoint, input, output
double inputPower; //power of current pedal value
double setPower; //setpoint of power 
double current_power_A; //current power of motor
double current_power_B;
double new_power_A;  //power output of PID
double new_power_B;
double setpoint_integrator;
int loop_counter;

// Other PID variables
const int MAX_POWER_RATE_THRESHOLD = 500;
const int KV = 78; // 78 rpm/volt => 1/78 volt/rmp
const float GEAR_RATIO = 5.6;

// Hall Effect Sensor variables
volatile byte rev_count_A;
volatile byte rev_count_B;

// useful vars
unsigned int wheel_rpm_A; 
unsigned int wheel_rpm_B;

unsigned int motor_rpm_A;
unsigned int motor_rpm_B;

unsigned int time_old_A;
unsigned int time_old_B;

double motor_volt_A;
double motor_volt_B;

double motor_current_A;
double motor_current_B;

int pwm_A;
int pwm_B;

const double resistance = 0.01;


// initialize some useful objects
Logger genLog("REV", "info");
drv sailboat(MOSI, MISO, CLK, SCS);
PID control_A(&current_power_A, &new_power_A, &setPower, Kp, Ki, Kd, DIRECT);
PID control_B(&current_power_B, &new_power_B, &setPower, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600); 

  // SPI Pins
  pinMode(SCS, OUTPUT); pinMode(MOSI, OUTPUT); pinMode(MISO, OUTPUT); pinMode(CLK, OUTPUT); 
  
  // PWM Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); 

  // general DRV pins
  pinMode(SLEEP, OUTPUT); pinMode(FAULT, INPUT);

  // hall 
  pinMode(HALLA, INPUT); pinMode(HALLB, INPUT);

  // wake up
  digitalWrite(SLEEP, HIGH);
  
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

  //update timings to account for change to Timer 0;
  control_A.SetSampleTime(SCALER * 200); //200ms update rate
  control_B.SetSampleTime(SCALER * 200); 

  // set up interrupts and variables for hall effect sensors
  attachInterrupt(1, hall_A_ISR, RISING); //maps to pin 3
  attachInterrupt(0, hall_B_ISR, RISING); //pin 2

  rev_count_A = 0;
  rev_count_B = 0;
  wheel_rpm_A = 0;
  wheel_rpm_B = 0;
  time_old_A = 0;
  time_old_B = 0;

  loop_counter = 1;
  setpoint_integrator = 0;

  // PID for motors work on full rated power of the motor
  control_A.SetOutputLimits(0, 1000);
  control_B.SetOutputLimits(0, 1000);

 
}

void loop() {

  
  // RPM determination (millis() func returns 1/SCALER of millis after Timer 0 manipulation)
  if(rev_count_A >= 3){
    wheel_rpm_A = rev_count_A / (SCALER * (millis() - time_old_A)/(1000*60)); 
    time_old_A = millis();
    rev_count_A = 0;
  }


  if(rev_count_B >= 3){
    wheel_rpm_B = rev_count_B/ (SCALER * (millis() - time_old_B)/(1000*60));
    time_old_A = millis();
    rev_count_B = 0;
  }
  // DetermineAmotor RPM using wheel RPM
  motor_rpm_A = wheel_rpm_A * GEAR_RATIO;
  motor_rpm_B = wheel_rpm_B * GEAR_RATIO;

  // account for very slow speed (less than 1 full rotation) by constraining RPM to be greater than 0
  if(motor_rpm_A == 0){
    motor_rpm_A = 1;
  }

  if (motor_rpm_B == 0){
    motor_rpm_B = 1;
  }

  // estimate motor voltage using motor RPM
  motor_volt_A = motor_rpm_A / KV;
  motor_volt_B = motor_rpm_B / KV;

  
  // Calculate set_power
  // set_power is: linear map to motor rated power (0 to 1000W)
  // set_power is averaged over the past 15 values;
  inputPower = map(analogRead(PEDAL), 380, 720, 0, 1000);
  inputPower = constrain(setPower, 0, 1000); 

  setpoint_integrator += inputPower; 

  if (loop_counter > 15){
    loop_counter = 1;
    setpoint_integrator = inputPower; //unwind the averager every 15 samples
  }

  setPower = setpoint_integrator / loop_counter;
  loop_counter++;


 // get current
 motor_current_A = get_current(analogRead(ISENSEA), ISENSEOFFSET, resistance, ISENSEAMP);
 motor_current_B = get_current(analogRead(ISENSEB), ISENSEOFFSET, resistance, ISENSEAMP);


  current_power_A = motor_current_A * motor_volt_A;
  current_power_B = motor_current_B * motor_volt_B;

  //generate new_power values from PID
  control_A.Compute(); 
  control_B.Compute();



  // map new_power to pwm signals (need logic for forward, reverse, coasting)

  pwm_A = map(new_power_A, 0, 1000, 0, 255);
  pwm_B = map(new_power_B, 0, 1000, 0, 255);



  //write values to DRV 
  // Forward - write 0 to xIN2

  analogWrite(AIN1, pwm_A);
  analogWrite(BIN1, pwm_B);

  analogWrite(AIN2, LOW);
  analogWrite(BIN2, LOW);


  }

double get_current(int dacValue, int offset, int resistance, int amplification) {

  return 5 * (double) (dacValue - offset) / (1023 * resistance * amplification);
}

void hall_A_ISR(){
  /*
  Interrupt: Increment hall effect sensor counter on left side
  */

  rev_count_A++;

}

void hall_B_ISR(){
  /*
  Interrupt: Increment hall effect sensor counter on right side 
  */

  rev_count_B++;
}