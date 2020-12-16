/****************************************************************
*
 ***************************************************************/
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h"
#include "SparkFun_VL53L1X.h"
#include "ICM_20948.h"
#include <math.h>
#include "BLE_example.h"
#include "commands.h"
#include "related_funcs.h"

#define SERIAL_PORT Serial
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0

// Attached peripherals
SFEVL53L1X distanceSensor; // ToF
SCMD myMotorDriver; // motor driver
ICM_20948_I2C myICM;  // IMU

// Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

// PID variables to keep track of
double error0 = 0;
double integrate_error0 = 0;
double diff_error = 0;
double t0_PID = micros();
double t0_step = micros();
double reference = 0;

// IMU variables to keep track of
double rollAccel0 = 0;
double pitchAccel0 = 0;
double rollGyro0 = 0;
double pitchGyro0 = 0;
double yawGyro0 = 0;
double rollCompFilt0 = 0;
double pitchCompFilt0 = 0;
double yawRateGyro0 = 0;
double yaw_rate_data = 0;
double yaw_rate_data_filt = 0;
double yaw_angle_data = 0;
double yaw_angle_start = 0;
double t0_pitch = micros();
double t0_roll = micros();
double t0_yaw = micros();
double t0_compFilt = micros();

// ToF variables to keep track of
double distance = 0;

// Motor variables to keep track of
#define LEFT_MOTOR 1
#define RIGHT_MOTOR 0
#define FWD 1
#define REV 0
#define LEFT_MIN 74
#define RIGHT_MIN 84
int motor_speed = 0;
boolean ramp_up = true;
double t0_motors = micros();
double t_motors_start = micros();
double t0_rotate = micros();
boolean motors_start = false;

// Bluetooth vairables to keep track of
#define MAXREPLY 100 // maximum length of reply / data message
#define TODO_VAL 0
uint8_t val[MAXREPLY]; // buffer to reply to client
uint8_t *val_data = &val[2]; // start of optional data area
uint8_t *val_len = &val[1];  // store length of optional data
String s_Rev = "Rev 1.0";
String s_Rcvd = "100"; //KHE declare extern in BLE_example_funcs.cpp to accept messages, if 100 don't bother checking case statements
uint16_t l_Rcvd = 0;
uint8_t *m_Rcvd = NULL;
String s_AdvName = "MyRobot"; //KHE 2 0 TOTAL CHARACHTERS ONLY!!  any more will be dropped
uint16_t package = 0;
cmd_t empty_cmd = {NOT_A_COMMAND, 1, {0}};
cmd_t *cmd = &empty_cmd;
cmd_t *res_cmd = &empty_cmd;
bt_debug_msg_t *bt_debug_head = NULL;
bt_debug_msg_t *bt_debug_tail = NULL;
present_t presentSensors = {
    .motorDriver = 0,
    .ToF_sensor = 0,
    .prox_sensor = 0,
    .IMU = 0};
int bytestream_active = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  Wire.setClock(400000);
  
  /// *** Set-up Bluetooth Connection *** ///
  #ifdef BLE_SHOW_DATA
  #endif
  #ifdef AM_DEBUG_PRINTF
    enable_print_interface();
  #endif
  Serial.print("Revision = ");
  Serial.print(s_Rev);
  Serial.printf("  ECE 4960 Robot Compiled: %s   %s\n", __DATE__, __TIME__);
  analogWriteResolution(16); //Set AnalogWrite resolution to 16 bit = 0 - 65535 (but make max 64k or trouble)
  set_Adv_Name(); //Set Advertising name.  BLE_example_funcs.cpp near end of file, fn declared extern in BLE_example.h
  HciDrvRadioBoot(0); // Boot the radio
  exactle_stack_init(); // Initialize the main ExactLE stack: BLE_example_funcs.cpp
  HciVsA3_SetRfPowerLevelEx(TX_POWER_LEVEL_PLUS_3P0_dBm); //= 15 decimal = max power WORKS..default = 0
  AmdtpStart(); // Start the "Amdtp" (AmbiqMicro Data Transfer Protocol) profile
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 20; i++){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
  }
  pinMode(blinkPin, OUTPUT);
  digitalWrite(blinkPin, LOW);
  setupWdt(); // configure the watchdog
  am_hal_wdt_init(&g_sWatchdogConfig);
  NVIC_EnableIRQ(WDT_IRQn); // Enable WDT interrupt in nested vector interrupt controller.
  uint8_t a = 0;
  m_Rcvd = &a;
  am_hal_interrupt_master_enable();
  
  /// *** Set-up ToF Sensor *** ///
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Distance sensor online!");
  distanceSensor.setOffset(24); // Manually set the offset for the sensor
  distanceSensor.setTimingBudgetInMs(20); // Manually set the timing budget
  distanceSensor.setIntermeasurementPeriod(20); // Manually set the intermeasurment period
  distanceSensor.setDistanceModeShort(); // Set the sensor to 1.3 m range
  
  /// *** Set-up Motor Driver ***///
  pinMode(8, INPUT_PULLUP); //Use to halt motor movement (ground)
  //  .commInter face is I2C_MODE 
  myMotorDriver.settings.commInterface = I2C_MODE;
  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D
  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;
  // initialize the driver get wait for idle
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "Motor driver ID matches 0xA9" );
  //  Check to make sure the driver is done looking for slaves before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();
  //Set application settings and enable driver
  //Uncomment code for motor 1 inversion
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1
  while ( myMotorDriver.busy() );
  myMotorDriver.enable(); //Enables the output driver hardware
  
  /// *** Set-up IMU *** ///
  while(!Serial){};
  bool initialized = false;
  while( !initialized ){
    myICM.begin( WIRE_PORT, AD0_VAL );
    Serial.print( F("Initialization of the sensor returned: ") );
    Serial.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      Serial.println( "Trying again..." );
      delay(10);
    }else{
      initialized = true;
    }
  }
}

void loop() {
  /// *** Get Sensor Measurements *** ///
  // IMU
  if( myICM.dataReady() ){
    // Update IMU values
    myICM.getAGMT();
    // Get yaw rate
    yaw_rate_data = getFormattedFloat( myICM.gyrZ(), 5, 2).toDouble();
    yaw_rate_data_filt = lowPass("yaw_rate_gyro", yaw_rate_data, 0.1, 0.16);
    // Get yaw angle
    yaw_angle_data = getYawGyro();
  }
  else{
    Serial.println("Waiting for data");
    delay(500);
  }
  // ToF
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  byte rangeStatus = distanceSensor.getRangeStatus();

  /// *** Perform Control Action *** ///
  //rampMotors();
  //stepMotors(220, 10);
  //motor_feedback(200 , yaw_rate_data, 0.6, 5, 0, 10);
  //single_motor_feedback("left", 30, yaw_rate_data, 0.6,  5, 0, 10);
  //set_reference_step(200, 10, 20);
  set_reference_rotate(720, 50, yaw_angle_data, 15);
  motor_feedback_2(reference, yaw_rate_data, 0.6, 5, 0);
  
  /// *** Print Stuff in Serial for Debugging *** ///
  Serial.print(yaw_rate_data);
  Serial.print(",");
  Serial.print(yaw_angle_data);
  Serial.print(",");
  Serial.print(motor_speed);
  Serial.print(",");
  Serial.print(reference);
  Serial.println();

  /// *** Send Data Back Over Bluetooth *** ///
  if (l_Rcvd > 1){ //Check if we have a new message from amdtps_main.c through BLE_example_funcs.cpp
    cmd = (cmd_t *)m_Rcvd;
    switch (cmd->command_type){
        case SET_MOTORS:
            Serial.println("Placeholder: Set Motors");
            break;
        case GET_MOTORS:
            Serial.println("Placeholder: Set Motors");
            //amdtpsSendData((uint8_t *)res_cmd, *val_len);
            break;
        case SER_RX:
            Serial.println("Got a serial message");
            pushMessage((char *)&cmd->data, cmd->length);
            break;
        case REQ_FLOAT:
            Serial.println("Going to send a float");
            //TODO: Put a float (perhaps pi) into a command response and send it.
            res_cmd->command_type = GIVE_FLOAT;
            ((float *)res_cmd->data)[0] = 3.14159265354f;
            amdtpsSendData((uint8_t *)res_cmd, 6);
            break;
        case PING:
            Serial.println("Ping Pong");
            cmd->command_type = PONG;
            amdtpsSendData(m_Rcvd, l_Rcvd);
            break;
        case START_BYTESTREAM_TX:
            bytestream_active = 1; //(int)cmd->data[0];
            //Serial.printf("Start bytestream with active %d \n", bytestream_active);
            ((uint32_t *)res_cmd->data)[0] = 0;
            break;
        case STOP_BYTESTREAM_TX:
            bytestream_active = 0;
            break;
        default:
            Serial.printf("Unsupported Command 0x%x \n", cmd->command_type);
            break;
        }

        l_Rcvd = 0;
        am_hal_wdt_restart();
        free(m_Rcvd);
    } 
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '7'))
    {
        s_Rcvd[0] = 0;
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.printf("Connected, length was %d", l_Rcvd);
    }
    else if ((s_Rcvd[0] == '6' && s_Rcvd[1] == '8'))
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("disconnected");
        //Decimal value of D for Disconnect
        //Serial.println("got disconnect from case in ino file - set_Stop");
        digitalWrite(LED_BUILTIN, LOW);
        //amdtps_conn_close();
        DmDevReset();
    }
    if (availableMessage())
    {
        Serial.println("Bluetooth Message:");
        Serial.println(pullMessage());
        printOverBluetooth("Message Received.");
    }
    if (bytestream_active)
    {
        res_cmd->command_type = BYTESTREAM_TX;
        res_cmd->length = 52;
        double t1 = micros()*1e-6;
        int motor_data = motor_speed;
        double yaw_rate = yaw_rate_data;
        double yaw_data = yaw_angle_data;
        double yaw_start = yaw_angle_start;
        double ref_sig = reference;
        double distance_data = distance;
        memcpy(res_cmd->data,&t1,8);
        memcpy(res_cmd->data+8,&motor_data,4);
        memcpy(res_cmd->data+12,&yaw_rate,8);
        memcpy(res_cmd->data+20,&yaw_data,8);
        memcpy(res_cmd->data+28,&yaw_start,8);
        memcpy(res_cmd->data+36,&ref_sig,8);
        memcpy(res_cmd->data+44, &distance_data,8);
        amdtpsSendData((uint8_t *)res_cmd, 54);
        // Log time
        long t = micros();
        Serial.printf("Package: %3d, Time: %3d\n",package,t);
        package++;
    }
    trigger_timers();
    delay(5);
}

/// *** Helper Functions for PID control *** ///
// sets the reference signal to the angular_set_point at time_start until the robot has rotated specified angle
void set_reference_rotate(double degrees_rotation, double angular_set_point, double yaw_angle, double time_start){
  double t = micros();
  if (!motors_start){ // current time < time_start
    yaw_angle_start = yaw_angle;
    reference = 0;
  }
  if (t - t0_rotate >= time_start*1e6){ // current time > time_start
    motors_start = true;
    if ((degrees_rotation + yaw_angle_start) >= abs(yaw_angle)){ // current yaw angle < degrees_rotation
      reference = angular_set_point;
    }
    else{ // current yaw angle > degrees_rotation
      reference = 0;
    }
  }
}

// creates a square wave from 0 to the set_point at time_start, back to 0 at time_end
void set_reference_step(double set_point, double time_start, double time_end){
  double t = micros();
  if ((t - t0_step >= time_start*1e6) && (t - t0_step <= time_end*1e6)){
    reference = set_point;
  }
  else{
    reference = 0;
  }
}

// PID control for dual motors, always following the set_point
void motor_feedback_2(double set_point, double sensor, double Kp, double Ki, double Kd){
  // local variables
  String motor_direction = "";
  double t_PID = micros();
  double tolerance = 20;
  // compute error
  double error = set_point - sensor;
  double error_filt = lowPass("yaw_rate_gyro", sensor, 0.1, 0.16);
  double integrate_error = integrate_error0 + error*(t_PID - t0_PID)*1e-6;
  diff_error = (error_filt - error0)/((t_PID - t0_PID)*1e-6);
  // sum controller as the sum of the P, I, D terms
  double controller = Kp*error + Ki*integrate_error + Kd*diff_error;
  // multiply the controller by the plant
  double plant = 1;
  double output = controller * plant;
  // implement anti-windup
  if (output > 255){
    motor_direction = "left";
    motor_speed = 255;
    integrate_error = 0;
  }
  else if (output < -255){
    motor_direction = "right";
    motor_speed = 255;
    integrate_error = 0;
  }
  else if (output >= 0 - tolerance){
    motor_direction = "left";
    motor_speed = round(output);
  }
  else if (output < 0 - tolerance){
    motor_direction = "right";
    motor_speed = round(abs(output));
  }
  // set the motor speed
  if (motor_direction == "left"){
    myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
    delay(5);
  }
  else if (motor_direction == "right"){
    myMotorDriver.setDrive( LEFT_MOTOR, FWD, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, REV, motor_speed);
    delay(5);
  }
  // update time, error, and set-point
  error0 = error;
  integrate_error0 = integrate_error;
  t0_PID = t_PID;
  reference = set_point;
}

// PID control for dual motors.  Apply reference signal after waiting 10 seconds for as long as defined by time_step.
void motor_feedback(double set_point, double sensor, double Kp, double Ki, double Kd, double time_step){
  double t = micros();
  // waits 10 seconds before starting
  if ((t - t0_motors >= 10*1e6) && (!motors_start)){
    motors_start = true;
    t_motors_start = micros();
  }
  // run PID control
  else if((motors_start) && (t - t_motors_start <= time_step*1e6)){
    // compute error
    double t_PID = micros();
    double error = set_point - sensor;
    double error_filt = lowPass("yaw_rate_gyro", sensor, 0.1, 0.16);
    double integrate_error = integrate_error0 + error*(t_PID - t0_PID)*1e-6;
    diff_error = (error_filt - error0)/((t_PID - t0_PID)*1e-6);
    // sum controller as the sum of the P, I, D terms
    double controller = Kp*error + Ki*integrate_error + Kd*diff_error;
    // multiply the controller by the plant
    double plant = 1;
    double output = controller * plant;
    // implement anti-windup
    if (output < 0){
      motor_speed = 0;
      integrate_error = 0;
    }
    else if (output > 255){
      motor_speed = 255;
      integrate_error = 0;
    }
    else{
      motor_speed = round(output);
    }
    // set the motor speed
    myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
    delay(5);
    // update time, error, and set-point
    Serial.print((t_PID - t0_PID)*1e-6);
    Serial.println();
    error0 = error;
    integrate_error0 = integrate_error;
    t0_PID = t_PID;
    reference = set_point;
  }
  // otherwise set the motors to speed 0
  else{
    error0 = 0;
    integrate_error0 = 0;
    diff_error = 0;
    motor_speed = 0;
    reference = 0;
    myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
  }
}

// PID control for single motor, specified by motor.  Apply reference signal after waiting 10 seconds for as long as defined by time_step.
void single_motor_feedback(String motor, double set_point, double sensor, double Kp, double Ki, double Kd, double time_step){
  double t = micros();
  // waits 10 seconds before starting
  if ((t - t0_motors >= 10*1e6) && (!motors_start)){
    motors_start = true;
    t_motors_start = micros();
  }
  // run PID control
  else if((motors_start) && (t - t_motors_start <= time_step*1e6)){
    // compute error
    double t_PID = micros();
    double error = set_point - sensor;
    double error_filt = lowPass("yaw_rate_gyro", sensor, 0.1, 0.16);
    double integrate_error = integrate_error0 + error*(t_PID - t0_PID)*1e-6;
    diff_error = (error_filt - error0)/((t_PID - t0_PID)*1e-6);
    // sum controller as the sum of the P, I, D terms
    double controller = Kp*error + Ki*integrate_error + Kd*diff_error;
    // multiply the controller by the plant
    double plant = 1;
    double output = controller * plant;
    // implement anti-windup
    if (output < 0){
      motor_speed = 0;
      integrate_error = 0;
    }
    else if (output > 255){
      motor_speed = 255;
      integrate_error = 0;
    }
    else{
      motor_speed = round(output);
    }
    // set the motor speed
    if (motor == "left"){
      myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
      myMotorDriver.setDrive( RIGHT_MOTOR, FWD, 50);
    }
    else if (motor == "right"){
      myMotorDriver.setDrive( LEFT_MOTOR, REV, 50);
      myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
    }
    else{
      Serial.print("incorrect motor specifed");
    }
    delay(5);
    // update time, error, and set-point
    Serial.print((t_PID - t0_PID)*1e-6);
    Serial.println();
    error0 = error;
    integrate_error0 = integrate_error;
    t0_PID = t_PID;
    reference = set_point;
  }
  // otherwise set the motors to speed 0
  else{
    error0 = 0;
    integrate_error0 = 0;
    diff_error = 0;
    motor_speed = 0;
    reference = 0;
    myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
  }
}

/// *** Helper Functions for Controlling the Motors *** ///
// steps the motor according to the speed defined by motor_input [0-255] and for time in time_step (s)
void stepMotors(double motor_input, double time_step){
  // local variables
  double t = micros();
  // waits 10 seconds before starting
  if ((t - t0_motors >= 10*1e6) && (!motors_start)){
    motors_start = true;
    t_motors_start = micros();
  }
  // steps motors after 10 sec for as long as specified
  else if ((motors_start) && (t - t_motors_start <= time_step*1e6)){
    motor_speed = motor_input;
    myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
    delay(50);
  }
  // otherwise set the motors to speed 0
  else{
    motor_speed = 0;
    myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
  }
}

// ramps the motors from 0 to 255 to 0
void rampMotors(){
  // ramp up
  if (ramp_up){
    myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
    delay(50);
    motor_speed++;
    if (motor_speed > 255){
      ramp_up = false;
    }
  }
  // ramp down
  else{
    myMotorDriver.setDrive( LEFT_MOTOR, REV, motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, motor_speed);
    delay(50);
    motor_speed--;
    if (motor_speed < 0){
      ramp_up = true;
    }
  }
}

/// *** Helper Functions for Reading from the IMU ***///
// format the raw output
String getFormattedFloat(float val, uint8_t leading, uint8_t decimals){
  String output = "";

  float aval = abs(val);
  if(val < 0){
    output = output + "-";
  }else{
    output = output + "0";
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      output = output + "0";
    }else{
      break;
    }
  }
  if(val < 0){
    output = output + String(-val);
  }else{
    output = output + String(val);
  }
  
  return output;
}

// returns the pitch in degrees using the acceleration data
double getPitchAccel(){
  double aX = getFormattedFloat( myICM.accX(), 5, 2).toDouble();
  double aY = getFormattedFloat( myICM.accY(), 5, 2).toDouble();
  double aZ = getFormattedFloat( myICM.accZ(), 5, 2).toDouble();
  double aMag = sqrt(pow(aX,2) + pow(aY,2) + pow(aZ,2));

  double theta = -asin(aX/aMag) * 57.295779513;
  
  return theta;
}

// returns the roll in degrees using the acceleration data
double getRollAccel(){
  double aX = getFormattedFloat( myICM.accX(), 5, 2).toDouble();
  double aY = getFormattedFloat( myICM.accY(), 5, 2).toDouble();
  double aZ = getFormattedFloat( myICM.accZ(), 5, 2).toDouble();
  double aMag = sqrt(pow(aX,2) + pow(aY,2) + pow(aZ,2));

  double phi = -asin(aY/aMag) * 57.295779513;

  return phi;
}

// returns the pitch in degrees using the gyro data
double getPitchGyro(){
  if (pitchGyro0 == 0){
    pitchGyro0 = getPitchAccel(); // initialize to the pitch from accel on start
  }
  double omegaY = getFormattedFloat( myICM.gyrY(), 5, 2).toDouble(); // read raw
  double t = micros();  
  
  double pitchGyro = -(pitchGyro0 - omegaY*(t-t0_pitch)*1e-6);
  
  pitchGyro0 = pitchGyro; // update variables
  t0_pitch = t;

  return pitchGyro;
}

// returns the roll in degrees using the gyro data
double getRollGyro(){
  if (rollGyro0 == 0){
    rollGyro0 == getRollAccel(); // initialize to the roll from accel on start
  }
  double omegaX = getFormattedFloat( myICM.gyrX(), 5, 2).toDouble(); // read raw
  double t = micros();  
  
  double rollGyro = -(rollGyro0 + omegaX*(t-t0_roll)*1e-6);
  
  rollGyro0 = rollGyro; // update variables
  t0_roll = t;

  return rollGyro;
}

// returns the yaw in degrees using the gyro data
double getYawGyro(){
  double omegaZ = getFormattedFloat( myICM.gyrZ(), 5, 2).toDouble(); // read raw
  double t = micros();  
  
  double yawGyro = yawGyro0 + omegaZ*(t-t0_yaw)*1e-6;
  
  yawGyro0 = yawGyro; // update variables
  t0_yaw = t;

  return yawGyro;
}

// returns a low pass filtered data 
double lowPass(String dataType, double raw, double Tau, double Fc){
  double filterPrior = 0;
  
  if (dataType == "roll_accel"){
    filterPrior = rollAccel0;
  }
  else if (dataType == "pitch_accel"){
    filterPrior = pitchAccel0;
  }
  else if (dataType == "yaw_rate_gyro"){
    filterPrior = yawRateGyro0;
  }
  
  double RC = 1/(2*M_PI*Fc);
  double alpha = Tau/(Tau+RC);
  double filtered = alpha*raw + (1-alpha)*filterPrior; // LPF formula

  if (dataType == "roll_accel"){
    rollAccel0 = filtered;
  }
  else if (dataType == "pitch_accel"){
    pitchAccel0 = filtered;
  }
  else if (dataType == "yaw_rate_gyro"){
    yawRateGyro0 = filtered;
  }
  
  return filtered;
}

// fuses angles from accelerometer and gyro
double compFilter(String dataType, double alpha){
  double angleAccel = 0;
  double angleGyro = 0;
  double angle0 = 0;
  double angleFilt = 0;
  double t = micros();

  // set the variables to correct values
  if (dataType == "roll"){
    angleAccel = lowPass("roll_accel", getRollAccel(), 0.1, 0.16); // get low pass filtered angle from accel values
    angleGyro = getRollGyro();
    angle0 = rollCompFilt0;
  }
  else if (dataType == "pitch"){
    angleAccel = lowPass("pitch_accel", getPitchAccel(), 0.1, 0.16); // get low pass filtered angle from accel values
    angleGyro = getPitchGyro();
    angle0 = pitchCompFilt0;
  }

  // compute the filtered angle value
  angleFilt = (angle0 + angleGyro*(t - t0_compFilt)*(1e-6))*(1 - alpha) + angleAccel*alpha; // comp filt formula

  // update global variables
  if (dataType == "roll"){
    rollCompFilt0 = angleFilt;
  }
  else if (dataType == "pitch"){
    pitchCompFilt0 = angleFilt;
  }
  t0_compFilt = t;
  
  return angleFilt; 
}

// returns the yaw from the magnetometer
double getYawMag(){
  // normalize the mag readings
  double magnX = getFormattedFloat( myICM.magX(), 5, 2).toDouble()*10e-6; // read raw
  double magnY = getFormattedFloat( myICM.magY(), 5, 2).toDouble()*10e-6; // read raw
  double magnZ = getFormattedFloat( myICM.magZ(), 5, 2).toDouble()*10e-6; // read raw
  double magn_norm = sqrt((magnX*magnX) + (magnY*magnY) + (magnZ*magnZ))*10e-6;
  magnX = magnX/magn_norm;
  magnY = magnY/magn_norm;
  magnZ = magnZ/magn_norm;

  // get filtered roll and pitch from accelerometer in radians
  double pitchAccelFilt = lowPass("pitch_accel", getPitchAccel(), 0.1, 0.16)*M_PI/180;
  double rollAccelFilt = lowPass("roll_accel", getRollAccel(), 0.1, 0.16)*M_PI/180;
  
  double xm = magnX*cos(pitchAccelFilt) - magnY*sin(rollAccelFilt)*sin(pitchAccelFilt) + magnZ*cos(rollAccelFilt)*sin(pitchAccelFilt); // theta=pitch and roll=phi
  double ym = magnY*cos(rollAccelFilt) + magnZ*sin(rollAccelFilt);
  double yawMag = atan2(ym, xm)*180/M_PI; 

  return yawMag;
}

// prints accel (mg), gyroscope (deg/sec), mag flux density (uT)
void printScaledAGMT( ICM_20948_AGMT_t agmt){
  String accelX = getFormattedFloat( myICM.accX(), 5, 2);
  String accelY = getFormattedFloat( myICM.accY(), 5, 2);
  String accelZ = getFormattedFloat( myICM.accZ(), 5, 2);
  String gyroX = getFormattedFloat( myICM.gyrX(), 5, 2);
  String gyroY = getFormattedFloat( myICM.gyrY(), 5, 2);
  String gyroZ = getFormattedFloat( myICM.gyrZ(), 5, 2);
  String magnX = getFormattedFloat( myICM.magX(), 5, 2);
  String magnY = getFormattedFloat( myICM.magY(), 5, 2);
  String magnZ = getFormattedFloat( myICM.magZ(), 5, 2);
  String temper = getFormattedFloat( myICM.temp(), 5, 2);
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  SERIAL_PORT.print(accelX);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(accelY);
  SERIAL_PORT.print(",");
  SERIAL_PORT.print(accelZ);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  SERIAL_PORT.print(gyroX);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(gyroY);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(gyroZ);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  SERIAL_PORT.print(magnX);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(magnY);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(magnZ);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  SERIAL_PORT.print(temper);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}
