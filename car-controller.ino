#include <PID_v1.h>
#include <drv.h>
#include <Logger.h>
#include <SPI.h>

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
int pwm_1;
int pwm_2;

// MISC pins
#define SLEEP 7
#define FAULT 4

// input pins
#define PEDAL A0
#define I1 A1
#define I2 A2
#define hall_1 3
#define hall_2 2

// PID coefficients (what are we doiiiing?)
const double Kp = 0.05;
const double Ki = 0.001;
const double Kd = 0.005;

// PID setpoint, input, output
double inputPower; //power of current pedal value
double setPower; //setpoint of power 
double current_power_1; //current power of motor
double current_power_2;
double new_power_1;  //power output of PID
double new_power_2;

double setpoint_integrator;
int loop_counter;

// Other PID variables
const int MAX_POWER_RATE_THRESHOLD = 500;
const int KV = 78; // 78 rpm/volt => 1/78 volt/rmp
const float GEAR_RATIO = 5.6;

// Hall Effect Sensor variables
volatile byte rev_count_1;
volatile byte rev_count_2;

unsigned int wheel_rpm_1; 
unsigned int wheel_rpm_2;

unsigned int motor_rpm_1;
unsigned int motor_rpm_2;

unsigned int time_old_1;
unsigned int time_old_2;

double motor_volt_1;
double motor_volt_2;


// initialize some useful objects
Logger genLog("REV", "info");
drv sailboat(MOSI, MISO, CLK, SCS, 0);
PID control_1(&current_power_1, &new_power_1, &setPower, Kp, Ki, Kd, DIRECT);
PID control_2(&current_power_2, &new_power_2, &setPower, Kp, Ki, Kd, DIRECT);





void setup() {
  Serial.begin(9600); 

  // SPI Pins
  pinMode(SCS, OUTPUT); pinMode(MOSI, OUTPUT); pinMode(MISO, OUTPUT); pinMode(CLK, OUTPUT); 
  
  // PWM Pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); 

  // general DRV pins
  pinMode(SLEEP, OUTPUT); pinMode(FAULT, INPUT);

  // hall 
  pinMode(hall_1, INPUT); pinMode(hall_2, INPUT);

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
  control_1.SetSampleTime(64 * 200);
  control_2.SetSampleTime(64 * 200); 

  // set up interrupts and variables for hall effect sensors
  attachInterrupt(1, hall_1_ISR, RISING); //maps to pin 3
  attachInterrupt(0, hall_2_ISR, RISING); //pin 2
  rev_count_1 = 0;
  rev_count_2 = 0;
  wheel_rpm_1 = 0;
  rpm_2 = 0;
  time_old_1 = 0;
  time_old_2 = 0;


  loop_counter = 0;
  setpoint_integrator = 0;

}

void loop() {

  
  // RPM determination (millis() func returns 1/64 of millis after Timer 0 manipulation)

  if(rev_count_1 >= 3){
    wheel_rpm_1 = rev_count_1 / (64 * (millis() - time_old_1)/(1000*60)); 
    time_old_1 = millis();
    rev_count_1 = 0;
  }


  if(rev_count_2 >= 3){
    rpm_2 = rev_count_2/ (64 * (millis() - time_old_1)/(1000*60));
    time_old_2 = millis();
    rev_count_2 = 0;
  }

  // Determine motor RPM using wheel RPM

  motor_rpm_1 = wheel_rpm_1 * GEAR_RATIO;
  motor_rpm_2 = wheel_rpm_2 * GEAR_RATIO;

  // account for very slow speed (less than 1 full rotation) by constraining RPM to be greater than 0
  if(motor_rpm_1 == 0){
    motor_rpm_1 = 1;
  }

  if (motor_rpm_2 == 0){
    motor_rpm_2 = 1;
  }

  // estimate motor voltage using motor RPM

  motor_volt_1 = motor_rpm_1 / KV;
  motor_volt_2 = motor_rpm_2 / KV;

  
  // Calculate set_power
  // setPower is: linear map to motor rated power (0 to 1000W
  inputPower = map(analogRead(PEDAL), 380, 720, 0, 1000);
  inputPower = constrain(setPower, 0, 1000); 

  loop_counter++;
  setpoint_integrator += inputPower; 


  if (loop_counter > 15){
    loop_counter = 0;
    setpoint_integrator = inputPower; //unwind the averager every 15 samples
  }

  last_time = now;

  setPower = setpoint_integrator / loop_counter;



  // TODO: caclulcate currentPower based on motors' voltages and currents

  current_power_1 = I1 * wheel_rpm_1 * 1/KV;

  current_power_2 = I2 * wheel_rpm_2 * 1/KV;

  //generate new_power values from PID
  control_1.Compute(); 
  control_2.Compute();



  // map new_power to pwm signals (need logic for forward, reverse, coasting)

  pwm_1 = map(new_power_1, 0, 1000, 0, 255);
  pwm_2 = map(new_power_2, 0, 1000, 0, 255);



  //write values to DRV 
  // Forward - write 0 to xIN2

  analogWrite(AIN1, new_power_1);
  analogWrite(BIN1, new_power_2);

  analogWrite(AIN2, LOW);
  analogWrite(BIN2, LOW);


  }


void hall_1_ISR(){
  /*
  Increment hall effect sensor counter on left side
  */

  rev_count_1++;

}

void hall_2_ISR(){
  /*
  Increment hall effect sensor counter on right side 
  */

  rev_count_2++;

}

