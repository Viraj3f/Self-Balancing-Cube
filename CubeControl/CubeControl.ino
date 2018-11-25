/*
   Connections
   ===========
   --From IMU 1--
   Connect SCL to D21
   Connect SDA to D20
   Connect VDD to 3.3V DC
   Connect GROUND to GND
   --From ESC--
   Connect ESC to D9 and GND
*/

#include <Adafruit_BNO055.h>
#include <Servo.h>

#include "PID.hpp"

/*
 * Constant settings for the system.
 */
namespace Settings
{
    // The break signal to the ESC.
    const int escBreakSignal = 1100;

    // The maximimum signal to add or subtract relative to
    // the break signal.
    const int escUpperBound = 100;

    // The minimum signal to add  or subtract relative to 
    // the break signal, since BLDCs are unstable at low speeds.
    const int escLowerBound = 30;

    // The angle at which the cube system should just quit.
    const double unsafeAngle = 360;

    // The angle at which the cube should begin balancing at.
    const double breakAngle = 15;

    // The sampling time of the PID controller in ms.
    const int PIDSampleTime = 15;
};

/*
 * Defining the state of the system.
 */
 struct State
 {
    // The current angle of the cube.
    double currentAngle;

    // The reference angle for the state.
    double referenceAngle;

    State() : currentAngle(0), referenceAngle(0) {};
 };

// Controller
// 8 2 0.1
PID controller(-8, 0, 0, Settings::PIDSampleTime);

// The state of the system.
State state;

// The IMU objects. 1 is the upper IMU, 2 is the lower one.
Adafruit_BNO055 imu_1;
Adafruit_BNO055 imu_2;

// The esc object
Servo esc;

void setup()
{
    Serial.begin(115200);
    
    // Setup ESC.
    esc.attach(11);
    delay(1000);
    esc.writeMicroseconds(Settings::escBreakSignal);
    delay(5000);
    Serial.println("Esc is setup.");
    
    // Setup BNOs.
    if(!imu_1.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
    {
        printErrorAndExit("Could not connect to imu 1.");
    }

    if(!imu_2.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
    {
        printErrorAndExit("Could not connect to imu 2.");
    }

    imu_1.setExtCrystalUse(true);
    imu_2.setExtCrystalUse(true);
    while (!imu_1.isFullyCalibrated() || !imu_2.isFullyCalibrated())
    {
        Serial.println("Waiting for imu calibration");
        delay(500);
    }

    Serial.println("Both imus are calibrated");
    delay(1000);

    state.currentAngle = getAngleFromIMUs(imu_1, imu_2);
    Serial.println("System has intial angle: " + String(state.currentAngle));
}


void loop()
{
    state.currentAngle = getAngleFromIMUs(imu_1, imu_2);
    Serial.print("Theta: " + String(state.currentAngle) + " ");

    bool areBothIMUsCalibrated = imu_1.isFullyCalibrated() && imu_2.isFullyCalibrated();
    if (!areBothIMUsCalibrated || isnan(state.currentAngle) || fabs(state.currentAngle) >= Settings::unsafeAngle)
    {
        // Some invalid reading was found, just exit.
        controller.reset();
        printErrorAndExit("Angle is past " + String(Settings::unsafeAngle) + 
                          " degrees. Current angle: "+ String(state.currentAngle));
    }
    else if (fabs(state.currentAngle) >= Settings::breakAngle)
    {
        // Send the break signal and do nothing.
        Serial.print(state.currentAngle);
        Serial.print(" ");
        Serial.print(map(Settings::escBreakSignal, Settings::escBreakSignal, 2 * Settings::escUpperBound, -30, 30));
        Serial.print('\n');
        esc.writeMicroseconds(Settings::escBreakSignal);
        controller.reset();
        delay(500);
    }
    else
    {
        double roundedAngle = (float)((int)(state.currentAngle * 100))/100.0;
        double escDiff = controller.compute(roundedAngle, state.referenceAngle);

        if (abs(escDiff) > Settings::escUpperBound)
        {
            escDiff = escDiff > 0 ? Settings::escUpperBound : -Settings::escUpperBound;
        }

        long escVal = Settings::escBreakSignal + Settings::escUpperBound + escDiff + Settings::escLowerBound + 10;
        esc.writeMicroseconds(escVal);

        Serial.print(roundedAngle);
        Serial.print(" ");
        Serial.print(map(escVal, 
                         Settings::escBreakSignal + Settings::escUpperBound + Settings::escLowerBound + 10,
                         Settings::escBreakSignal + 2 * Settings::escUpperBound + Settings::escLowerBound + 10,
                         -30, 30));
        Serial.print('\n');
        //Serial.println("Esc signal: " + String(escVal));
    }
    delay(10);
}

double getAngleFromIMUs(Adafruit_BNO055& imu_1, Adafruit_BNO055& imu_2)
{
    const double r1 = 178.89;  // Distance to top IMU in mm
    const double r2 = 33.24;   // Distance to bottom IMU in mm
    const double mu = r1/r2;
    
    imu::Vector<3> acceleration_1 = imu_1.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> acceleration_2 = imu_2.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    double mx = acceleration_1.x() + mu * acceleration_2.x();
    double my = acceleration_1.y() - mu * acceleration_2.y();

    double theta = atan(-mx/my) * 180/PI;

    return theta;
}

void printErrorAndExit(const String& message)
{
    esc.writeMicroseconds(Settings::escBreakSignal);
    Serial.println(message);
    while(true);
}
