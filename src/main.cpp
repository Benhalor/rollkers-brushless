/**
 * B-G431B-ESC1 position motion control example with encoder
 *
 */

#include <SimpleFOC.h>
#define NUMBER_OF_PAIR_POLES 10

#define WHEEL_DIAMETER_M 0.083
#define MAX_SPEED_KMPH 7
#define MIN_SPEED_RADPS 0.0
#define MAX_SPEED_RADPS 2 * (MAX_SPEED_KMPH / 3.6) / (WHEEL_DIAMETER_M)
#define MIN_PPM_DURATION 1080.0
#define MAX_PPM_DURATION 1860.0
#define SAFETY_MAX_PPM_DURATION 3000.0
#define PERMANENT_SPEED_RADPS 10.0
#define ZERO_SPEED_RADPS 0.5

// Motor instance
BLDCMotor motor = BLDCMotor(NUMBER_OF_PAIR_POLES, 0.39, 80, 0.00018); // 0.39, 65, 0.00018
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// encoder instance
HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, NUMBER_OF_PAIR_POLES);
float maxCurrent = 0.0;
// Interrupt routine intialisation
// channel A and B callbacks
void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

uint32_t durationPpm = 0;
uint32_t lastPpmRising = micros();
float targetSpeedRadianParSeconde = 0.0;

enum Step
{
  SETUP,
  STOPPED,
  STARTING,
  PERMANENT_SPEED
};
Step msmstep = SETUP;
Step previousmsmstep = SETUP;

void escEdge()
{
  uint32_t micr = micros();
  if (digitalRead(A_PWM))
  {
    lastPpmRising = micr;
  }
  else
  {
    durationPpm = micr - lastPpmRising;
  }
}

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.motion(&motor, cmd); }

void setup()
{
  // use monitoring with serial
  Serial.begin(1000000);
  /*//Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  delay(5000);*/

  // initialize sensor hardware
  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 18;
  driver.pwm_frequency = 20000;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // aligning voltage [V]
  motor.voltage_sensor_align = 2;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  motor.velocity_limit = 50; // [rad/s]

  // set motion control loop to be used
  // motor.torque_controller = TorqueControlType::foc_current;

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.PID_current_q.P = 3.0;   // 3    - Arduino UNO/MEGA
  motor.PID_current_q.I = 300.0; // 300  - Arduino UNO/MEGA
  motor.PID_current_q.D = 0.0;
  motor.PID_current_q.limit = 7.0; // sortie du pid en courant en V
  // motor.PID_current_q.output_ramp = 0.05;
  motor.LPF_current_q.Tf = 0.005;

  motor.PID_current_d.P = 3.0; // 3    - Arduino UNO/MEGA
  motor.PID_current_d.I = 300; // 300  - Arduino UNO/MEGA
  motor.PID_current_d.D = 0.0;
  motor.PID_current_d.limit = 2.0;
  motor.PID_current_d.output_ramp = 0.0;

  // Ziegler nichols
  // Ku = 0.25
  // Tu = 54ms
  //0.13*Ku/Tu
  motor.voltage_limit = 8.5;     // [V]
  motor.PID_velocity.P = 0.11;   // 1.0
  motor.PID_velocity.I = 0.61;   // 10
  motor.PID_velocity.limit = 10; // 10 // à refaire après initFoc() aussi
  motor.LPF_velocity.Tf = 0.05;
  motor.PID_velocity.output_ramp = 100.1;


  //  comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_TARGET | _MON_VEL; // default _MON_TARGET | _MON_VOLT_Q | _MON_VEL | _MON_ANGLE | _MON_CURR_Q

  motor.sensor_direction = Direction::CW; // CW or CCW

  motor.motion_downsample = 20; // - times (default 0 - disabled)

  // initialize motor
  // motor.current_limit = 2;
  motor.init();

  // It is very important that the the current sensing init function is called after the BLDCDriver init function is called
  //  current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);
  motor.zero_electric_angle = 0.0;

  // align encoder and start FOC
  motor.initFOC();
  motor.PID_velocity.limit = 22;

  command.add('T', doTarget, "target angle");

  _delay(1000);
  pinMode(A_PWM, INPUT);
  attachInterrupt(digitalPinToInterrupt(A_PWM), escEdge, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(A_PWM), escFalling, FALLING);

  // Par sécurité, on attend que la consigne soit à 0 au démarrage
  while (targetSpeedRadianParSeconde > 0.001 || targetSpeedRadianParSeconde < -0.001)
  {
    delay(1);
  }
  msmstep = STOPPED;
  motor.monitor_downsample = 1;
}

uint32_t last_time = micros();
void loop()
{

  if (micros() - last_time > 1000)
  {

    // Serial.println(durationPpm);
    // PhaseCurrent_s  current = currentSense.getPhaseCurrents();
    /*maxCurrent = max(current.a, maxCurrent);
    maxCurrent = max(current.b, maxCurrent);
    maxCurrent = max(current.c, maxCurrent);
    Serial.println(maxCurrent);*/
    last_time = micros();
    //sensor.getVelocitySmooth();
     motor.monitor();
   /* Serial.print(sensor.getVelocitySmooth());
    Serial.print(",");
    Serial.print(sensor.getVelocity());
    Serial.println();*/
  }

  // Watchdog sur la commande
  uint32_t micr = micros();
  uint32_t test = micr - lastPpmRising;
  if (micr > lastPpmRising && (test) > 200000)
  {
    targetSpeedRadianParSeconde = 0.0;
    msmstep = STOPPED;
  }

  // Calcul de la vitesse
  if (durationPpm < MIN_PPM_DURATION)
  {
    targetSpeedRadianParSeconde = MIN_SPEED_RADPS;
  }
  else if (durationPpm > SAFETY_MAX_PPM_DURATION)
  {
    targetSpeedRadianParSeconde = MIN_SPEED_RADPS;
  }
  else if (durationPpm > MAX_PPM_DURATION)
  {
    targetSpeedRadianParSeconde = MAX_SPEED_RADPS;
  }
  else
  {
    targetSpeedRadianParSeconde = ((float)durationPpm - MIN_PPM_DURATION) * (MAX_SPEED_RADPS - MIN_SPEED_RADPS) / (MAX_PPM_DURATION - MIN_PPM_DURATION);
  }

  motor.loopFOC();
  float measuredSpeedRadianParSeconde = sensor.getVelocity();
  if (msmstep == STOPPED)
  {
    motor.PID_velocity.P = 0.11;   // 1.0
    motor.PID_velocity.I = 0.61;   // 10
    motor.PID_velocity.limit = 22; // 10
    motor.LPF_velocity.Tf = 0.05;
    motor.PID_velocity.output_ramp = 200.1;
    motor.PID_velocity.negative_softener = 1.0;
    motor.move(0.0);
    if (abs(targetSpeedRadianParSeconde) > ZERO_SPEED_RADPS)
    {
      msmstep = STARTING;
    }
  }
  else if (msmstep == STARTING)
  {
    // ku = 0.05
    // Tu = 120
    motor.PID_velocity.P = 0.11;   // 1.0
    motor.PID_velocity.I = 0.61;   // 10
    motor.PID_velocity.limit = 22; // 10
    motor.LPF_velocity.Tf = 0.05;
    motor.PID_velocity.output_ramp = 100.1;
    motor.PID_velocity.negative_softener = 1.0;
    motor.move(targetSpeedRadianParSeconde);
    if (abs(targetSpeedRadianParSeconde) > PERMANENT_SPEED_RADPS && abs(measuredSpeedRadianParSeconde) > PERMANENT_SPEED_RADPS)
    {
      msmstep = PERMANENT_SPEED;
    }
    else if (abs(targetSpeedRadianParSeconde) < ZERO_SPEED_RADPS && abs(measuredSpeedRadianParSeconde) < ZERO_SPEED_RADPS)
    {
      msmstep == STOPPED;
    }
  }
  else if (msmstep == PERMANENT_SPEED)
  {
    motor.PID_velocity.P = 0.11;   // 1.0
    motor.PID_velocity.I = 0.61;   // 10
    motor.PID_velocity.limit = 22; // 10
    motor.LPF_velocity.Tf = 0.05;
    motor.PID_velocity.output_ramp = 100.1;
    motor.PID_velocity.negative_softener = 1.0;
    motor.move(targetSpeedRadianParSeconde);
    if (abs(targetSpeedRadianParSeconde) < PERMANENT_SPEED_RADPS && abs(measuredSpeedRadianParSeconde) < PERMANENT_SPEED_RADPS)
    {
      msmstep = STARTING;
    }
  }

  previousmsmstep = msmstep;

  // Serial.println(msmstep);
}
