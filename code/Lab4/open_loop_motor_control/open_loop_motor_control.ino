/******************************************************************************
  MotorTest.ino
  Serial Controlled Motor Driver
  Marshall Taylor @ SparkFun Electronics
  Sept 15, 2016
  https://github.com/sparkfun/Serial_Controlled_Motor_Driver
  https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library

  Resources:
  Uses Wire.h for i2c operation
  Uses SPI.h for SPI operation

  Development environment specifics:
  Arduino IDE 1.6.7
  Teensy loader 1.27

  This code is released under the [MIT License](http://opensource.org/licenses/MIT).
  Please review the LICENSE.md file included with this example. If you have any questions
  or concerns with licensing, please contact techsupport@sparkfun.com.
  Distributed as-is; no warranty is given.
******************************************************************************/
//This example steps through all motor positions moving them forward, then backwards.
//To use, connect a redboard to the user port, as many slaves as desired on the expansion
//port, and run the sketch.
//
// Notes:
//    While using SPI, the defualt LEDPIN will not toggle
//    This steps through all 34 motor positions, which takes a few seconds to loop.

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

#define LEDPIN 13

SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

void setup()
{
  Serial.begin(9600);
  pinMode(LEDPIN, OUTPUT);

  Serial.println("Starting sketch.");

  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern "0101" on board for address 0x5A

  myMotorDriver.enable();


}

void loop()
{
  //***** Operate the Motor Driver *****//
  //  This walks through all 34 motor positions driving them forward and back.
  //  It uses .setDrive( motorNum, direction, level ) to drive the motors.

  Serial.println("Now stepping through the motors.");
  digitalWrite( LEDPIN, 1 );
  myMotorDriver.setDrive( 0, 1, 255); //Drive motor i forward at full speed
  delay(250);
  digitalWrite( LEDPIN, 0 );
  myMotorDriver.setDrive( 1, 0, 255); //Drive motor i backward at full speed
  delay(250);
}
