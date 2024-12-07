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
float actual_velocity = 0.0f;
float actual_position = 0.0f;
float follow_error = 0.0f;

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
  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  current_sense.init();
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);

  motor.voltage_sensor_align = 2.0f;
  
  //FOC model selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;

  //Limits
  motor.velocity_limit = 9999;
  motor.voltage_limit = 24;
  motor.current_limit = 1;

  //Velocity
  //motor.PID_velocity.P = 0.35;
  //motor.PID_velocity.I = 1.0;
  //motor.PID_velocity.D = 0.0;
  //motor.PID_velocity.output_ramp = 0;
  //motor.LPF_velocity.Tf = 0.01;

  //Angle
  motor.P_angle.P = 10; 
  motor.P_angle.I = 0;  
  motor.P_angle.D = 0;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf = 0;

  //Current
  motor.PID_current_q.P = 2.5f;                      
  motor.PID_current_q.I = 50.0f;
  motor.LPF_current_q.Tf = 0.0f;
  motor.PID_current_d.P = 2.5f;
  motor.PID_current_d.I = 50.0f;
  motor.LPF_current_d.Tf = 0.0f;

  step_dir.init();
  step_dir.enableInterrupt(onStep);
  step_dir.attach(&received_angle);
  
  motor.sensor_direction = Direction::CCW;

  motor.init();
  motor.initFOC();

  command.add('M', onMotor, "motor");

  Serial.println(F("Motor ready."));

}

void loop() {

  start = micros();
  motor.loopFOC();
  actual_position = motor.shaft_angle;
  actual_velocity = motor.shaft_velocity * 11.919832;
  follow_error = received_angle - actual_position;
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  phaseA = currents.a * 1000;
  phaseb = currents.b * 1000;
  motor.move(received_angle);
  
  command.run();
  finish = micros();
  looptime = finish - start;
  }
