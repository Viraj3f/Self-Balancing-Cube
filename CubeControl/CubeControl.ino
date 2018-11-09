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
#include <PID_v1.h>

/*
 * Constant settings for the system.
 */
namespace Settings
{
    // The break signal to the ESC.
    const int escBreakSignal = 1500;

    // The maximimum signal to add or subtract relative to
    // the break signal.
    const int escUpperBound = 150;

    // The minimum signal to add  or subtract relative to 
    // the break signal, since BLDCs are unstable at low speeds.
    const int escLowerBound = 30;

    // The angle at which the cube system should just quit.
    const int unsafeAngle = 60;

    // The angle at which the cube should begin balancing at.
    const int breakAngle = 10;

    // The sampling time of the PID controller in ms.
    const int PIDSampleTime = 10;
};

/*
 * Defining the state of the system.
 */
 struct State
 {
    // The current angle of the cube.
    double currentAngle;

    // The reference angle for the state.
    const double referenceAngle = 0;

    // The value that the PID controller will update.
    double pidValue;

    State() : currentAngle(0), pidValue (0) {};
 };

// The state of the system.
State state;

// The PID controller.
PID controller(&state.currentAngle,
               &state.pidValue,
               &state.referenceAngle,
               1, 0, 0, DIRECT);

// The IMU object.
Adafruit_BNO055 bno;

// The esc object
Servo esc;

void setup()
{
    Serial.begin(9600);

    // Set PID settings.
    controller.SetSampleTime(Settings::PIDSampleTime);
    controller.SetOutputLimits(-Settings::escUpperBound, Settings::escUpperBound);
    
    // Setup ESC.
    esc.attach(9);
    esc.writeMicroseconds(Settings::escBreakSignal);
    delay(2000);
    Serial.println("Esc is setup.");
    
    // Setup BNO.
    if(!bno.begin())
    {
        printErrorAndExit("Could not connect to BNO.");
    }
    Serial.println("BNO is attached.");
    bno.setExtCrystalUse(true);
    delay(1000);
    
    state.currentAngle = getAngleFromIMU(bno);
    Serial.println("BNO has intial angle: " + String(state.currentAngle));
}


void loop()
{
    state.currentAngle = getAngleFromIMU(bno);
    Serial.println("Theta: " + String(state.currentAngle));
    if (fabs(state.currentAngle) >= Settings::unsafeAngle)
    {
        // Some invalid reading was found, just exit.
        controller.SetMode(MANUAL);
        printErrorAndExit("Angle is past " + String(Settings::unsafeAngle) + 
                          " degrees. Current angle: "+ String(state.currentAngle));
    }
    else if (fabs(state.currentAngle) >= Settings::breakAngle)
    {
        // Send the break signal and do nothing.
        esc.writeMicroseconds(Settings::escBreakSignal);
        controller.SetMode(MANUAL);
        Serial.println("Esc signal: " + String(Settings::escBreakSignal));
    }
    else
    {
        controller.SetMode(AUTOMATIC);
        controller.Compute();
        int escDiff = state.pidValue;

        if (abs(escDiff) < Settings::escLowerBound)
        {
            escDiff = escDiff > 0 ? Settings::escLowerBound : -Settings::escLowerBound;
        }

        int escValue = Settings::escBreakSignal + escDiff;
        esc.writeMicroseconds(escValue);
        Serial.println("Esc signal: " + String(escValue));
    }
}

double getAngleFromIMU(Adafruit_BNO055& bno)
{
   const double angleOffset = 45.0;
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   return euler.y() - angleOffset;
}

void printErrorAndExit(const String& message)
{
    esc.writeMicroseconds(Settings::escBreakSignal);
    Serial.println(message);
    while(true);
}
  
