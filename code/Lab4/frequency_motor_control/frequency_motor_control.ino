/* Author: Christopher Chan
  Created: September 13, 2020

  Credit: Nathan Seidle, Owen Lyke
  License: MIT. See SparkFun Arduino Apollo3 Project for more information

  This example demonstrates how to use the pulse density microphone (PDM) on Artemis boards.
  This library and example are heavily based on Example1_Blink and Example1_Microphone
*/

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values
#include "Wire.h"

SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.

//#define blinkPin 5
#define lightPin LED_BUILTIN

//define light-up frequency threshold in Hz
#define lightFreq 2000

//Global variables needed for PDM library
#define pdmDataBufferSize 4096 //Default is array of 4096 * 32bit
uint16_t pdmDataBuffer[pdmDataBufferSize];

//Global variables needed for the FFT in this sketch
float g_fPDMTimeDomain[pdmDataBufferSize * 2];
float g_fPDMFrequencyDomain[pdmDataBufferSize * 2];
float g_fPDMMagnitudes[pdmDataBufferSize * 2];
uint32_t sampleFreq;

//Enable these defines for additional debug printing
#define PRINT_PDM_DATA 0
#define PRINT_FFT_DATA 0

#include <PDM.h> //Include PDM library included with the Aruino_Apollo3 core
AP3_PDM myPDM;   //Create instance of PDM class

//Math library needed for FFT
#define ARM_MATH_CM4
#include <arm_math.h>

void setup()
{
  pinMode(8, INPUT_PULLUP); //Use to halt motor movement (ground)
  
  Serial.begin(115200);
  Serial.println("SparkFun PDM Example");

  if (myPDM.begin() == false) // Turn on PDM with default settings, start interrupts
  {
    Serial.println("PDM Init failed. Are you sure these pins are PDM capable?");
    while (1)
      ;
  }
  Serial.println("PDM Initialized");

  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face is I2C_MODE 
  myMotorDriver.settings.commInterface = I2C_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for slaves before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  //Uncomment code for motor 0 inversion
  //while( myMotorDriver.busy() );
  //myMotorDriver.inversionMode(0, 1); //invert motor 0

  //Uncomment code for motor 1 inversion
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1

  while ( myMotorDriver.busy() );
  myMotorDriver.enable(); //Enables the output driver hardware
  
  printPDMConfig();

  pinMode(lightPin, OUTPUT);
}

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define FWD 0
#define REV 1
#define LEFT_MIN 74+80
#define RIGHT_MIN 84+80

boolean hasRun = false;
int runTime = 100;

void loop()
{
  if (myPDM.available())
  {
    myPDM.getData(pdmDataBuffer, pdmDataBufferSize);

    uint32_t currentFreq = getLoudest();

    //pass setDrive() a motor number, direction as 0(call 0 forward) or 1, and level from 0 to 255
    myMotorDriver.setDrive( LEFT_MOTOR, 0, 0); //Stop motor
    myMotorDriver.setDrive( RIGHT_MOTOR, 0, 0); //Stop motor
    while (digitalRead(8) == 0); //Hold if jumper is placed between pin 8 and ground

    //***** Operate the Motor Driver *****//
    //  This walks through all 34 motor positions driving them forward and back.
    //  It uses .setDrive( motorNum, direction, level ) to drive the motors.

    //Smoothly move one motor up to speed and back (drive level 0 to 255)
    // Light the LED if loudest frequency is above threshold
    if (currentFreq > lightFreq)
    {
        myMotorDriver.setDrive( LEFT_MOTOR, FWD, LEFT_MIN);
        myMotorDriver.setDrive( RIGHT_MOTOR, FWD, RIGHT_MIN);
        delay(runTime);
        myMotorDriver.setDrive( LEFT_MOTOR, 0, 0); //Stop motor
        myMotorDriver.setDrive( RIGHT_MOTOR, 0, 0); //Stop motor
    }
    else{
        myMotorDriver.setDrive( LEFT_MOTOR, 0, 0); //Stop motor
        myMotorDriver.setDrive( RIGHT_MOTOR, 0, 0); //Stop motor  
    }
    
  }


  // Go to Deep Sleep until the PDM ISR or other ISR wakes us.
  am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
}

//*****************************************************************************
//
// Analyze and print frequency data.
//
//*****************************************************************************
uint32_t getLoudest()
{
  float fMaxValue;
  uint32_t ui32MaxIndex;
  int16_t *pi16PDMData = (int16_t *)pdmDataBuffer;
  uint32_t ui32LoudestFrequency;

  //
  // Convert the PDM samples to floats, and arrange them in the format
  // required by the FFT function.
  //
  for (uint32_t i = 0; i < pdmDataBufferSize; i++)
  {
    
    if (PRINT_PDM_DATA)
    {
      Serial.printf("%d\n", pi16PDMData[i]);
    }
    

    g_fPDMTimeDomain[2 * i] = pi16PDMData[i] / 1.0;
    g_fPDMTimeDomain[2 * i + 1] = 0.0;
  }
  
  if (PRINT_PDM_DATA)
  {
    Serial.printf("END\n");
  }

  //
  // Perform the FFT.
  //
  arm_cfft_radix4_instance_f32 S;
  arm_cfft_radix4_init_f32(&S, pdmDataBufferSize, 0, 1);
  arm_cfft_radix4_f32(&S, g_fPDMTimeDomain);
  arm_cmplx_mag_f32(g_fPDMTimeDomain, g_fPDMMagnitudes, pdmDataBufferSize);

  if (PRINT_FFT_DATA)
  {
    for (uint32_t i = 0; i < pdmDataBufferSize / 2; i++)
    {
      Serial.printf("%f\n", g_fPDMMagnitudes[i]);
    }

    Serial.printf("END\n");
  }

  //
  // Find the frequency bin with the largest magnitude.
  //
  arm_max_f32(g_fPDMMagnitudes, pdmDataBufferSize / 2, &fMaxValue, &ui32MaxIndex);

  ui32LoudestFrequency = (sampleFreq * ui32MaxIndex) / pdmDataBufferSize;

  if (PRINT_FFT_DATA)
  {
    Serial.printf("Loudest frequency bin: %d\n", ui32MaxIndex);
  }

  Serial.printf("Loudest frequency: %d         \n", ui32LoudestFrequency);

  return ui32LoudestFrequency;
}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
void printPDMConfig(void)
{
  uint32_t PDMClk;
  uint32_t MClkDiv;
  float frequencyUnits;

  //
  // Read the config structure to figure out what our internal clock is set
  // to.
  //
  switch (myPDM.getClockDivider())
  {
  case AM_HAL_PDM_MCLKDIV_4:
    MClkDiv = 4;
    break;
  case AM_HAL_PDM_MCLKDIV_3:
    MClkDiv = 3;
    break;
  case AM_HAL_PDM_MCLKDIV_2:
    MClkDiv = 2;
    break;
  case AM_HAL_PDM_MCLKDIV_1:
    MClkDiv = 1;
    break;

  default:
    MClkDiv = 0;
  }

  switch (myPDM.getClockSpeed())
  {
  case AM_HAL_PDM_CLK_12MHZ:
    PDMClk = 12000000;
    break;
  case AM_HAL_PDM_CLK_6MHZ:
    PDMClk = 6000000;
    break;
  case AM_HAL_PDM_CLK_3MHZ:
    PDMClk = 3000000;
    break;
  case AM_HAL_PDM_CLK_1_5MHZ:
    PDMClk = 1500000;
    break;
  case AM_HAL_PDM_CLK_750KHZ:
    PDMClk = 750000;
    break;
  case AM_HAL_PDM_CLK_375KHZ:
    PDMClk = 375000;
    break;
  case AM_HAL_PDM_CLK_187KHZ:
    PDMClk = 187000;
    break;

  default:
    PDMClk = 0;
  }

  //
  // Record the effective sample frequency. We'll need it later to print the
  // loudest frequency from the sample.
  //
  sampleFreq = (PDMClk / (MClkDiv * 2 * myPDM.getDecimationRate()));

  frequencyUnits = (float)sampleFreq / (float)pdmDataBufferSize;

  Serial.printf("Settings:\n");
  Serial.printf("PDM Clock (Hz):         %12d\n", PDMClk);
  Serial.printf("Decimation Rate:        %12d\n", myPDM.getDecimationRate());
  Serial.printf("Effective Sample Freq.: %12d\n", sampleFreq);
  Serial.printf("FFT Length:             %12d\n\n", pdmDataBufferSize);
  Serial.printf("FFT Resolution: %15.3f Hz\n", frequencyUnits);
}
