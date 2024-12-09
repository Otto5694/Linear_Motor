#include "Arduino.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

#define ENCODER_PPR 1065
#define ENCODER_PIN_A PB4
#define ENCODER_PIN_B PB5

int enablePin = PA0;

unsigned long start;
unsigned long finish;
unsigned long looptime;

float received_angle = 0.0f;
float received_velocity = 0.0f;
float actual_velocity = 0.0f;
float actual_position = 0.0f;
float follow_error = 0.0f;
float motor_enable_offset = 0.0f;

float phaseA = 0.0f;
float phaseb = 0.0F;

STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, ENCODER_PIN_A, ENCODER_PIN_B);

//Motor parameters
BLDCMotor motor = BLDCMotor(2);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8,PA9,PA10);

LowsideCurrentSense current_sense  = LowsideCurrentSense(0.005f, 50.0f, PB0, PB1);

StepDirListener step_dir = StepDirListener(PA2, PA1, 2*PI/4260);
void onStep() { step_dir.handle(); }

//Command settings
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  Serial.begin(115200);
  delay(1000);
  pinMode(enablePin, INPUT);
  
  //SimpleFOCDebug::enable();
  SimpleFOC_CORDIC_Config();      // initialize the CORDIC

  //Encoder
  encoder.init();
  motor.linkSensor(&encoder);

  //Driver
  driver.pwm_frequency = 40000;
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  current_sense.init();
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);

  motor.voltage_sensor_align = 2;
  
  //FOC model selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;
  motor.feed_forward_velocity = received_velocity;

  //Limits
  motor.velocity_limit = 9999;
  motor.voltage_limit = 24;
  motor.current_limit = 6;

  //Velocity
  motor.PID_velocity.P = 0.5f;
  motor.PID_velocity.I = 20.0f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 0;
  motor.LPF_velocity.Tf = 1.0f/(30*_2PI);

  //Angle
  motor.P_angle.P = 200; 
  motor.P_angle.I = 120;  
  motor.P_angle.D = 0.0f;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf = 0;

  //Current
  motor.PID_current_q.P = 2.5f;                      
  motor.PID_current_q.I = 120;
  motor.LPF_current_q.Tf = 1.0f/(250*_2PI);
  motor.PID_current_d.P = 2.5f;
  motor.PID_current_d.I = 120;
  motor.LPF_current_d.Tf = 1.0f/(250*_2PI);

  step_dir.init();
  step_dir.enableInterrupt(onStep);
  step_dir.attach(&received_angle, &received_velocity);
  
  motor.sensor_direction = Direction::CCW;

  motor.init();
  motor.initFOC();

  command.add('M', onMotor, "motor");

  Serial.println(F("Motor ready."));

}

void loop() {

  if((digitalRead(enablePin) == HIGH) && (motor.enabled == 0)){

    motor_enable_offset = motor.shaft_angle - received_angle;
    motor.enable();
  }
  if((digitalRead(enablePin) == LOW) && (motor.enabled == 1)){

    motor.disable();
  }

  start = micros();
  motor.loopFOC();
  actual_position = motor.shaft_angle;
  actual_velocity = motor.shaft_velocity /* 11.9591676f*/;
  follow_error = received_angle +motor_enable_offset;
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  phaseA = currents.a * 1000;
  phaseb = currents.b * 1000;
  motor.move(received_angle + motor_enable_offset);
  
  command.run();
  finish = micros();
  looptime = finish - start;
  step_dir.update();
  }
