/****************************************************************
*
 ***************************************************************/
#include "ICM_20948.h"
#include <math.h>
#define SERIAL_PORT Serial
#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

// variables to keep track of
double rollAccel0 = 0;
double pitchAccel0 = 0;
double rollGyro0 = 0;
double pitchGyro0 = 0;
double yawGyro0 = 0;
double rollCompFilt0 = 0;
double pitchCompFilt0 = 0;
double t0_pitch = micros();
double t0_roll = micros();
double t0_yaw = micros();
double t0_compFilt = micros();

void setup() {
  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT){};
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  bool initialized = false;
  while( !initialized ){
    myICM.begin( WIRE_PORT, AD0_VAL );
    
    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      SERIAL_PORT.println( "Trying again..." );
      delay(10);
    }else{
      initialized = true;
    }
  }
}

void loop() {

  if( myICM.dataReady() ){
    myICM.getAGMT();                // The values are only updated when you call 'getAGMT'
    // Print pitch and roll from raw accel values
    Serial.print(getPitchAccel());
    Serial.print(",");
    Serial.print(getRollAccel());
    Serial.print(",");
    // Print pitch and roll from accel values with a low pass filter
    Serial.print(lowPass("pitch_accel", getPitchAccel(), 0.1, 0.16));
    Serial.print(",");
    Serial.print(lowPass("roll_accel", getRollAccel(), 0.1, 0.16));
    Serial.print(",");
    // Print pitch, roll, and yaw from raw gyro values
    Serial.print(getPitchGyro());
    Serial.print(",");
    Serial.print(getRollGyro());
    Serial.print(",");
    Serial.print(getYawGyro());
    Serial.print(",");
    // Print pitch, roll with a complementary filter, fusing gyro and accel values
    Serial.print(compFilter("pitch", .2));
    Serial.print(",");
    Serial.print(compFilter("roll", .2));
    Serial.print(",");
    // Print yaw from magnetometer values
    Serial.print(getYawMag());
    Serial.println();
    //printScaledAGMT( myICM.agmt);   // This function takes into account the sclae settings from when the measurement was made to calculate the values with units
    delay(10);
  }else{
    Serial.println("Waiting for data");
    delay(500);
  }
}


/// *** Below here are some helper functions to print the data nicely! ***///
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
  
  double RC = 1/(2*M_PI*Fc);
  double alpha = Tau/(Tau+RC);
  double filtered = alpha*raw + (1-alpha)*filterPrior; // LPF formula

  if (dataType == "roll_accel"){
    rollAccel0 = filtered;
  }
  else if (dataType == "pitch_accel"){
    pitchAccel0 = filtered;
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
