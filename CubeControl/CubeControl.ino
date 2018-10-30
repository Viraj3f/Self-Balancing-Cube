/*
   Connections
   ===========
   --From IMU--
   Connect SCL to D21
   Connect SDA to D20
   Connect VDD to 3.3V DC
   Connect GROUND to GND
   --From ESC--
   Connect ESC to D9 and GND
*/

#include <Adafruit_BNO055.h>
#include <Servo.h> //Using servo library to control ESC

Adafruit_BNO055 bno = Adafruit_BNO055();
Servo esc;
const int angleBound = 50;
double angleOffset = 0;

void setup() {
    Serial.begin(9600);
    
    // Esc setup
    esc.attach(9);
    esc.writeMicroseconds(1500);
    delay(2000);
    Serial.println("Esc is setup");
    
    // BNO setup
    if(!bno.begin()) {
        printErrorAndExit("Could not connect to BNO.");
    }
    delay(1000);
    Serial.println("BNO is attached");
    bno.setExtCrystalUse(true);
    
    delay(2000);
    double initialAngle = getAngleFromIMU(bno);
    if (initialAngle <= 45){
        angleOffset = 45.0 + initialAngle;
    }
    else {
        angleOffset = initialAngle - 45.0;
    }
    Serial.println("BNO is calibrated with offset: " + String(angleOffset));
}


void loop() {
    double angle = getAngleFromIMU(bno);
    if (fabs(angle) > angleBound) {
      printErrorAndExit("Angle is past max angle. Current angle: " + String(angle) + " Angle bound: " + String(angleBound));
    }

    int escValue = getEscValFromAngle(angle); 
    Serial.println(angle);
    
    //esc.writeMicroseconds(escValue);
}

double getAngleFromIMU(Adafruit_BNO055 &bno){
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  sensors_event_t event; 
  double angle = euler.y();
  return euler.y() - angleOffset;
}

int getEscValFromAngle(double angle) {
    const int scalingValue = 1000;
    const int minESC = 1400;
    const int maxESC = 1600;
    int scaledAngle = (int)(angle * scalingValue);
    int escValue = map(scaledAngle, -angleBound * scalingValue, angleBound * scalingValue, minESC, maxESC);
    return escValue;
}

void printErrorAndExit(String message) {
    Serial.println(message);
    while(true);
}
  
