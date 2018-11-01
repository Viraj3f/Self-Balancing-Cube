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
#include <Servo.h>

// The IMU object.
Adafruit_BNO055 bno;

// The ESC object with common signals.
const int escBreakSignal = 1500;
const int minEscSignal = 1400;
const int maxEscSignal = 1600;
Servo esc;

// The maximum angle magnitude the cube should ever reach.
const int angleBound = 50;

void setup()
{
    Serial.begin(9600);
    
    // Esc setup
    esc.attach(9);
    esc.writeMicroseconds(escBreakSignal);
    delay(2000);
    Serial.println("Esc is setup.");
    
    // BNO setup
    if(!bno.begin()) {
        printErrorAndExit("Could not connect to BNO.");
    }
    Serial.println("BNO is attached.");
    bno.setExtCrystalUse(true);
    delay(1000);
    double initialAngle = getAngleFromIMU(bno);
    Serial.println("BNO has intial angle: " + String(initialAngle));
}


void loop()
{
    double angle = getAngleFromIMU(bno);
    if (fabs(angle) > angleBound)
    {
         printErrorAndExit("Angle is past max angle. Current angle: " + String(angle) + " Angle bound: " + String(angleBound));
    }

    int escValue = getEscValFromAngle(angle); 
    Serial.println("Angle:" + String(angle) + " Esc value: " + String(escValue));
    
    //esc.writeMicroseconds(escValue);
}

double getAngleFromIMU(Adafruit_BNO055& bno)
{
   const double angleOffset = 45.0;
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   return euler.y() - angleOffset;
}

int getEscValFromAngle(double angle)
{
    const int scalingValue = 1000;
    int scaledAngle = (int)(angle * scalingValue);
    int escValue = map(scaledAngle, -angleBound * scalingValue, angleBound * scalingValue, minEscSignal, maxEscSignal);
    escValue = constrain(escValue, minEscSignal, maxEscSignal);
    return escValue;
}

void printErrorAndExit(String message)
{
    esc.writeMicroseconds(escBreakSignal);
    Serial.println(message);
    while(true);
}
  
