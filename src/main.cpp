#include <SimpleFOC.h>

// NUMBER OF POLE PAIRS, NOT POLES
BLDCMotor motor = BLDCMotor(8);
// MUST USE 6PWM FOR B-G431 DRIVER
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

void setup() {

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 22;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  motor.voltage_limit = 3;   // [V]
  motor.velocity_limit = 3; // [rad/s]

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

}

void loop() {
  motor.move(2);
}