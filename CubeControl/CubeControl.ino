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
    const int escUpperBound = 200;

    // The minimum signal to add  or subtract relative to 
    // the break signal, since BLDCs are unstable at low speeds.
    const int escLowerBound = 50;

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
// 21, 0.18,  0.27
// 28, 0.18,  0.045
// 21, 0.18,  0.045
// 28, 0,  0.045

// Best ranked
// 0.6 * Ku, 0.5 * Tu, 0.75 * Tu

double Ku = 36;
double Tu = 0.357;
PID controller(0.6 * Ku, 0.6 * Tu, 0.83 * Tu, Settings::PIDSampleTime);

// The state of the system.
State state;

// The IMU objects. 1 is the upper IMU, 2 is the lower one.
// The ADR Pin of imu 2 needs to be connected to 3.3V
// SCL is connected to A5, SDA to A4, VIN to 5V for both.
Adafruit_BNO055 bno;

// The esc object
Servo esc;

void setup()
{
    Serial.begin(115200);
    
    // Setup ESC.
    esc.attach(11);
    delay(1000);
    esc.writeMicroseconds(Settings::escBreakSignal);
    delay(2000);
    Serial.println("Esc is setup.");
    
    // Setup BNOs.
    if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
    {
        printErrorAndExit("Could not connect to imu 1.");
    }

    bno.setExtCrystalUse(true);

    while (!isAccelerationCalibrated(bno))
    {
        Serial.println("Waiting for accleration calibration");
        delay(500);
    }

    Serial.println("Both imus are calibrated");
    delay(1000);

    state.currentAngle = getAngleFromIMU(bno);
    Serial.println("System has intial angle: " + String(state.currentAngle));

    Serial.println(0);
    Serial.println(Settings::breakAngle);
    Serial.println(-Settings::breakAngle);
}


void loop()
{
    state.currentAngle = getAngleFromIMU(bno);

    if (isnan(state.currentAngle) || fabs(state.currentAngle) >= Settings::unsafeAngle)
    {
        // Some invalid reading was found, just exit.
        controller.reset();
        printErrorAndExit("Angle is past " + String(Settings::unsafeAngle) + 
                          " degrees. Current angle: "+ String(state.currentAngle));
    }
    else if (fabs(state.currentAngle) >= Settings::breakAngle)
    {
        // Send the break signal and do nothing.
        /*
        Serial.print(state.currentAngle);
        Serial.print(" ");
        Serial.print(0);
        Serial.print('\n');
        */
        esc.writeMicroseconds(Settings::escBreakSignal);
        controller.reset();
        delay(500);
    }
    else
    {
        double roundedAngle = (float)((int)(state.currentAngle * 10))/10.0;
        double escDiff = controller.compute(state.currentAngle, state.referenceAngle);

        if (abs(escDiff) > Settings::escUpperBound)
        {
            escDiff = escDiff > 0 ? Settings::escUpperBound : -Settings::escUpperBound;
        }

        long escVal = Settings::escBreakSignal + Settings::escUpperBound + escDiff + Settings::escLowerBound;
        esc.writeMicroseconds(escVal);

        Serial.print(roundedAngle);
        Serial.print(" ");
        Serial.print(map(escDiff, 
                         -Settings::escUpperBound,
                         Settings::escUpperBound,
                         -Settings::breakAngle, Settings::breakAngle));
        Serial.print('\n');
    }
    delay(Settings::PIDSampleTime);
}

double getAngleFromIMU(Adafruit_BNO055& bno)
{
    imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    double mx = acceleration.x();
    double my = acceleration.y();
    double theta = atan2(mx, my) * 180/PI;

    return theta + 3;
}

bool isAccelerationCalibrated(Adafruit_BNO055& bno)
{
    uint8_t acceleration = 0, gyro = 0;
    bno.getCalibration(nullptr, nullptr, &acceleration, &gyro);
    return acceleration > 0;
}

void printErrorAndExit(const String& message)
{
    esc.writeMicroseconds(Settings::escBreakSignal);
    Serial.println(message);
    while(true);
}
