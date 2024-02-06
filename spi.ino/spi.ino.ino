// Include the library
#include <AS5X47.h>

// Define where the CSN Pin in connected.
int slaveSelectPin = 10;

// Start connection to the sensor.
AS5X47 as5047p(slaveSelectPin);


void setup() {
  Serial.begin(115200);
  Settings1 settings1;

  // Parameters from SETTINGS1 can be set like this:
  // According to manufacturer documentation, UVW_ABI enables UVW output.
  /*settings1.values.daecdis = 1; 
  // Write settings to the encoder:
  as5047p.writeSettings1(settings1);

   as5047p.writeRegister(0x003, 0x01);
  // Start OTP : write PROGOTP
  as5047p.writeRegister(0x003, 0x08);
  
  // Wait for OTP to finish...
  bool otpComplete = false;
  for (int i=0; i<100; i++) {
    delay(100);
    ReadDataFrame df = as5047p.readRegister(0x003);
    if (df.values.data == 0x01) {
      otpComplete = true;
      break;
    }
  }
  if (otpComplete) {
    Serial.println("Configuration written successfully !");
  }*/

  
  // put your setup code here, to run once:
  as5047p.printDebugString();

}

void loop() {
  // put your main code here, to run repeatedly:
  float angle = as5047p.readAngle();

  // Show the measure angle on the Serial Port
  //Serial.println(angle);
  delay(100);

}
