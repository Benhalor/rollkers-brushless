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
  Serial.begin(115200);

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
  motor.voltage_limit = 2;  // [V]
  motor.velocity_limit = 3; // [rad/s]
  // motor.current_limit = 0.02;  // Amps - default 0.2Amps

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity; // velocity_openloop;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  /*motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // default voltage_power_supply
  motor.voltage_limit = 2;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle P controller
  motor.P_angle.P = 20;
  //  maximal velocity of the position control
  motor.velocity_limit = 4;*/

  motor.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 2;
  motor.LPF_velocity.Tf = 0.1;

  // motor.PI_s.voltage_limit=xx;
  //  comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE

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
  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);

}

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

  motor.monitor();

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
