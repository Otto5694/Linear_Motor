#include "Arduino.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

#define ENCODER_PPR 1065
#define ENCODER_PIN_A PB4
#define ENCODER_PIN_B PB5

unsigned long start;
unsigned long finish;
unsigned long looptime;

float received_angle = 0.0f;
float actual_velocity = 0.0f;
float actual_position = 0.0f;

float phaseA = 0.0f;
float phaseb = 0.0F;

STM32HWEncoder encoder = STM32HWEncoder(ENCODER_PPR, ENCODER_PIN_A, ENCODER_PIN_B);

//Motor parameters
BLDCMotor motor = BLDCMotor(2);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8,PA9,PA10);

InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, PB0, PA7);

StepDirListener step_dir = StepDirListener(PA2, PA1, 2*PI/4260);
void onStep() { step_dir.handle(); }

//Command settings
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  SimpleFOCDebug::enable();
  SimpleFOC_CORDIC_Config();      // initialize the CORDIC

  //Encoder
  encoder.init();
  motor.linkSensor(&encoder);

  //Driver
  driver.pwm_frequency = 30000;
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  current_sense.init();
  //current_sense.gain_b *= -1;
  //current_sense.gain_a *= -1;
  current_sense.skip_align = true;
  motor.linkCurrentSense(&current_sense);
  
  //FOC model selection
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;

  //Limits
  motor.velocity_limit = 99999;
  motor.voltage_limit = 24;
  motor.current_limit = 3.5;

  //Velocity
  motor.PID_velocity.P = 0.6;
  motor.PID_velocity.I = 0.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 0;
  motor.LPF_velocity.Tf = 0.001;

  //Angle
  motor.P_angle.P = 40; 
  motor.P_angle.I = 5;  
  motor.P_angle.D = 0.1;
  motor.P_angle.output_ramp = 0;
  motor.LPF_angle.Tf = 0;

  //Current
  motor.PID_current_q.P = 0.5;                      
  motor.PID_current_q.I = 10;
  motor.LPF_current_q.Tf = 0.0003;
  motor.PID_current_d.P = 0.5;
  motor.PID_current_d.I = 10;
  motor.LPF_current_d.Tf = 0.0003;

  Serial.begin(115200);
  //motor.useMonitoring(Serial);

  step_dir.init();
  step_dir.enableInterrupt(onStep);
  step_dir.attach(&received_angle);
  
  motor.sensor_direction = Direction::CCW;

  motor.init();
  motor.initFOC();

  command.add('M', onMotor, "motor");

  encoder.update();
  received_angle = encoder.getAngle();

  Serial.println(F("Motor ready."));

}

void loop() {

  start = micros();
  motor.loopFOC();
  actual_position = motor.shaft_angle;
  actual_velocity = motor.shaft_velocity * 11.919832;
  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  phaseA = currents.a * 1000;
  phaseb = currents.b * 1000;
  motor.move(received_angle);
  
  //motor.monitor();
  command.run();
  finish = micros();
  looptime = finish - start;
  }
