/**
 * B-G431B-ESC1 position motion control example with encoder
 *
 */

#include <SimpleFOC.h>
#define NUMBER_OF_PAIR_POLES 10

// Motor instance
BLDCMotor motor = BLDCMotor(NUMBER_OF_PAIR_POLES);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// encoder instance
HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, NUMBER_OF_PAIR_POLES);

// Interrupt routine intialisation
// channel A and B callbacks
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.motion(&motor, cmd); }

void setup()
{
  // use monitoring with serial
  Serial.begin(1000000);

  // initialize sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 18;

  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  motor.velocity_limit = 50; // [rad/s]

  // set motion control loop to be used
  // motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity; // velocity_openloop;//torque

  /*
    motor.voltage_limit = 3; // [V]
    motor.PID_velocity.P = 0.3;
    motor.PID_velocity.I = 2; // 1.5;
    motor.LPF_velocity.Tf = 0.1;
  */

  // Coeffs pour TorqueControlType::foc_current
  motor.torque_controller = TorqueControlType::foc_current;
  motor.voltage_limit = 18; // [V]
  motor.PID_velocity.P = 0.4;
  motor.PID_velocity.I = 0.0;//0.1; // 1.5;
  motor.LPF_velocity.Tf = 0.05;

  // motor.PID_velocity.output_ramp = 100;// en A par seconde

  /*motor.PID_current_q.P = 5;
  motor.PID_current_q.I= 300;
  motor.PID_current_d.P= 5;
  motor.PID_current_d.I = 300;
  motor.LPF_current_q.Tf = 0.01;
  motor.LPF_current_d.Tf = 0.01;*/

  //  comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE | _MON_CURR_Q

  motor.sensor_direction = Direction::CCW; // CW or CCW

  // motor.motion_downsample = 5; // - times (default 0 - disabled)

  // initialize motor
  motor.init();

  // It is very important that the the current sensing init function is called after the BLDCDriver init function is called
  //  current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);
  // currentSense.driverAlign(3);

  // align encoder and start FOC
  motor.initFOC();
  command.add('T', doTarget, "target angle");

  _delay(1000);

  motor.PID_velocity.limit = 10;
  motor.current_limit = 10;
}

uint32_t last_time = micros();
void loop()
{
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();
  sensor.update();
  // Serial.println(sensor.getVelocity());
  //  currentSense.update();
  //   display the angle and the angular velocity to the terminal
  //  Serial.println(sensor.getAngle());
  //  delay(100);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!


  if(micros()-last_time > 1000){
    last_time = micros();
    motor.monitor();
  }

  // user communication
  /*PhaseCurrent_s currents = currentSense.getPhaseCurrents();
  float current_magnitude = currentSense.getDCCurrent();

  Serial.print(currents.a * 1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents.b * 1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents.c * 1000); // milli Amps
  Serial.print("\t");
  Serial.println(current_magnitude * 1000); // milli Amps*/
  command.run();
}
