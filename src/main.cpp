
#include "Arduino.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

#define ENCODER_PPR 1140
#define ENCODER_PIN_A PB4
#define ENCODER_PIN_B PB5

float D_ind = 0.6/1000;
float Q_ind = 0.7/1000;
float coil_R = 3.28;
int bandwith_A = 380;
int bandwith_V = 40;

int enablePin = PA1;

unsigned long start;
unsigned long finish;
unsigned long looptime;
int loopcounter = 0;
int loopiter = 10;

float recieved_angle = 0.0f;
float recieved_velocity = 0.0f;
float actual_velocity = 0.0f;
float actual_position = 0.0f;
float actual_position_mm = 0.0f;
float follow_error = 0.0f;
float follow_error_mm = 0.0f;
float motor_enable_offset = 0.0f;

float phaseA = 0.0f;
float phaseb = 0.0F;

float velocity_P = 0.55f;
float velocity_I = 4.0f;
float angle_P = 170.0f;
float angle_I = 10.0f;

STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, ENCODER_PIN_A, ENCODER_PIN_B);

//Motor parameters
BLDCMotor motor = BLDCMotor(2);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8,PA9,PA10);

InlineCurrentSense current_sense  = InlineCurrentSense(0.005f, 50.0f, _NC, PA7, PA6);

StepDirListener step_dir = StepDirListener(PA3, PA2, 2*PI/4260);
void onStep() { step_dir.handle(); }

//Command settings
//Commander command = Commander(Serial);
//void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  //Serial.begin(115200);
  //delay(1000);
  pinMode(enablePin, INPUT);
  
  //SimpleFOCDebug::enable();
  SimpleFOC_CORDIC_Config();      // initialize the CORDIC

  //Encoder
  encoder.init();
  motor.linkSensor(&encoder);

  //Driver
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  current_sense.init();
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);

  motor.voltage_sensor_align = 5;
  
  //FOC model selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;

  //Limits
  motor.velocity_limit = 9999;
  motor.voltage_limit = 24;
  motor.current_limit = 6;

  //Velocity
  motor.PID_velocity.P = velocity_P;
  motor.PID_velocity.I = velocity_I;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 0;
  motor.LPF_velocity.Tf = 1.0f/(bandwith_V*_2PI);

  //Angle
  motor.P_angle.P = angle_P; 
  motor.P_angle.I = angle_I;  
  motor.P_angle.D = 0;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf = 0;

  //Current
  motor.PID_current_q.P = Q_ind * bandwith_A * _2PI;                      
  motor.PID_current_q.I = motor.PID_current_q.P * coil_R / Q_ind;
  motor.LPF_current_q.Tf = 1.0f/(2*bandwith_A*_2PI);
  motor.PID_current_d.P = D_ind * bandwith_A * _2PI;
  motor.PID_current_d.I = motor.PID_current_d.P * coil_R / D_ind;
  motor.LPF_current_d.Tf = 1.0f/(2*bandwith_A*_2PI);

  step_dir.init();
  step_dir.enableInterrupt(onStep);
  step_dir.attach(&recieved_angle, &recieved_velocity);
  
  motor.sensor_direction = Direction::CCW;

  motor.init();
  motor.initFOC();

  //command.add('M', onMotor, "motor");

  //Serial.println(F("Motor ready."));

}

void loop() {

  if((digitalRead(enablePin) == LOW) && (motor.enabled == 0)){

    motor_enable_offset = motor.shaft_angle - recieved_angle;
    motor.enable();
  }
  if((digitalRead(enablePin) == HIGH) && (motor.enabled == 1)){

    motor.disable();
  }

  if (loopcounter == loopiter){
    start = micros();
  }

  motor.loopFOC();
  motor.feed_forward_velocity = recieved_velocity;
  motor.move(recieved_angle + motor_enable_offset);  
  
  if (loopcounter == loopiter){
    finish = micros();
    looptime = (finish - start) / 10;
    actual_position = motor.shaft_angle;
    actual_velocity = motor.shaft_velocity; //11.9591676f;
    follow_error = recieved_angle + motor_enable_offset - actual_position;
    actual_position_mm = motor.shaft_angle  * 12.731;
    follow_error_mm = follow_error * 12.731;
    loopcounter = 0;

    motor.PID_velocity.P = velocity_P * 1.0f;
    motor.PID_velocity.I = velocity_I * 1.0f;
    motor.LPF_velocity.Tf = 1.0f/(bandwith_V*_2PI);
    motor.P_angle.P = angle_P * 1.0f;
    motor.P_angle.I = angle_I * 1.0f;

    motor.PID_current_q.P = Q_ind * bandwith_A * _2PI;                      
    motor.PID_current_q.I = motor.PID_current_q.P * coil_R / Q_ind;
    motor.LPF_current_q.Tf = 1.0f/(2*bandwith_A*_2PI);
    motor.PID_current_d.P = D_ind * bandwith_A * _2PI;
    motor.PID_current_d.I = motor.PID_current_d.P * coil_R / D_ind;
    motor.LPF_current_d.Tf = 1.0f/(2*bandwith_A*_2PI);
    
    PhaseCurrent_s currents = current_sense.getPhaseCurrents();
    phaseA = currents.c * 1000;
    phaseb = currents.b * 1000;
  }

  //command.run();
  step_dir.update();
  loopcounter++;
  }