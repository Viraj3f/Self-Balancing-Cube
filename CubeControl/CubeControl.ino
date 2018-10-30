#include <Servo.h> //Using servo library to control ESC
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

Servo esc; //Creating a servo class with name as esc
int serialVal = 1000;

bool firstLoop = true;
int startAngle = 0;

void setup() {
  /*ESC stuff*/
  esc.attach(9); //Specify the esc signal pin,Here as D8
  esc.writeMicroseconds(1500);
  delay(2000);
  Serial.begin(9600);

  /*BNO stuff*/
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}


void loop() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if (firstLoop == true){
    startAngle = euler.x();
    firstLoop = false;
  }
  float angle = euler.x() - startAngle;
  if (angle > 180) {
    angle = 0; 
  }
  Serial.print("Angle: ");
  Serial.println(angle);
  serialVal = angle * 4 + 1500;
  Serial.print("serialVal: ");
  Serial.println(serialVal);
  Serial.println("");
  if (serialVal != 0) 
  esc.writeMicroseconds(serialVal);
  delay(20);
}
  
