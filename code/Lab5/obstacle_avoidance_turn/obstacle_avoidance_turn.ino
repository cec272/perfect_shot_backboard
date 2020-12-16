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

/*
 * Avoids obstacles by braking when too close using ToF sensor
 * 
 * Christopher Chan
 */

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

//Loop variables
#define LEFT_MOTOR 1
#define RIGHT_MOTOR 0
#define FWD 1
#define REV 0
#define LEFT_MIN 74
#define RIGHT_MIN 84
int left_motor_speed = 0;
int right_motor_speed = 0;
int too_close = 600;
boolean turn = false;
boolean brake = false;
boolean has_braked = false;

//Attached peripherals
SFEVL53L1X distanceSensor;
SCMD myMotorDriver;

void setup() {
  // *** Set-up distance sensor *** //
  Wire.begin();
  Serial.begin(9600);
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

  // *** Set-up motor driver ***//
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
}

void loop() {
  // *** Get sensor measurements *** //
  // Distance sensor
  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  byte rangeStatus = distanceSensor.getRangeStatus();

  // *** Plan control output from data *** //
  if (distance > too_close){
    // move forward
    left_motor_speed = LEFT_MIN + 70;
    right_motor_speed = RIGHT_MIN + 70;
    turn = false; // do not turn
    brake = false; // do not brake
    has_braked = false; // reset has_braked
    Serial.println("Move forward");
  }
  else{
    left_motor_speed = 0;
    right_motor_speed = 0;
    turn = true; // do turn
    brake = true; // do brake
    Serial.println("Too Close!");
  }

  // *** Perform control action *** //
  while (digitalRead(8) == 0); //Hold if jumper is placed between pin 8 and ground
  if (rangeStatus == 0){ // perform action only if sensor measurement is valid
    if (!has_braked && brake){ // do this only if the braking flag is true and once per braking event
      delay(100);
      myMotorDriver.setDrive( LEFT_MOTOR, REV, LEFT_MIN + 20);
      myMotorDriver.setDrive( RIGHT_MOTOR, REV, RIGHT_MIN + 20);
      delay(80);
      has_braked = true;
      Serial.println("Braking");
    }
    if (turn){
      myMotorDriver.setDrive( LEFT_MOTOR, REV, LEFT_MIN + 150);
      myMotorDriver.setDrive( RIGHT_MOTOR, FWD, RIGHT_MIN + 150);
      delay(100);
      Serial.println("Turning");
    }
    myMotorDriver.setDrive( LEFT_MOTOR, FWD, left_motor_speed);
    myMotorDriver.setDrive( RIGHT_MOTOR, FWD, right_motor_speed);
    delay(50);
  }
}
