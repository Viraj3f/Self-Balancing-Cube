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

/*
 * Constant settings for the system.
 */
namespace Settings
{
    // The break signal to the ESC.
    const int escBreakSignal = 1480;

    // The maximimum signal to add or subtract relative to
    // the break signal.
    const int escUpperBound = 80;

    // The minimum signal to add  or subtract relative to 
    // the break signal, since BLDCs are unstable at low speeds.
    const int escLowerBound = 30;

    // The angle at which the cube system should just quit.
    const double unsafeAngle = 360;

    // The angle at which the cube should begin balancing at.
    const double breakAngle = 20;

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
    double referenceAngle;

    State() : currentAngle(0), referenceAngle(0) {};
 };

 /*
  * PID implementation class.
  */
class PID
{
  public:
    unsigned long lastTime;
    double output;
    double errSum, lastInput;
    double kp, ki, kd;
    unsigned long sampleTime;
    bool wasReset;

    PID(double _kp, double _ki, double _kd, unsigned long _sampleTime)
    {
        this->sampleTime = _sampleTime;
        double sampleTimeInSec = ((double)sampleTime)/1000;
        this->kp = _kp;
        this->ki = _ki * sampleTimeInSec;
        this->kd = _kd / sampleTimeInSec;
        this->reset();
    }

    void reset()
    {
        wasReset = true;
        lastTime = 0;
        output = 0;
        errSum = 0;
        lastInput = 0;
    }

    double compute(double input, double setpoint)
    {
        if (wasReset)
        {
            // If the reset method was called,
            // set lastInput to input to prevent a
            // potentially large derivative term.
            lastInput = input;
            wasReset = false;
        }

        unsigned long now = millis();
        unsigned long timeChange = (now - lastTime);
        if (timeChange >= sampleTime)
        {
            /*Compute all the working error variables*/
            double error = setpoint - input;
            errSum += error;
            double dInput = (input - lastInput);

            if (input * lastInput < 0)
            {
                // zero crossing
                errSum = 0;
            }

            /*Compute PID Output*/
            output = kp * error + ki * errSum - kd * dInput;

            /*Remember some variables for next time*/
            lastInput = input;
            lastTime = now;
        }

        return output;
    }
};

// Controller
PID controller(8, 2, 0, Settings::PIDSampleTime);

// The state of the system.
State state;

// The IMU object.
Adafruit_BNO055 bno;

// The esc object
Servo esc;

void setup()
{
    Serial.begin(9600);
    
    // Setup ESC.
    esc.attach(11);
    delay(1000);
    esc.writeMicroseconds(1500);
    delay(5000);
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

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    while (gyro != 0x03)
    {
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.println("Waiting for Gyro calibration");
        delay(500);
    }
    Serial.println("Gyro is calibrated");
    
}


void loop()
{
    state.currentAngle = getAngleFromIMU(bno);
    Serial.print("Theta: " + String(state.currentAngle) + " ");

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if (gyro != 0x03 || isnan(state.currentAngle) || fabs(state.currentAngle) >= Settings::unsafeAngle)
    {
        // Some invalid reading was found, just exit.
        controller.reset();
        printErrorAndExit("Angle is past " + String(Settings::unsafeAngle) + 
                          " degrees. Current angle: "+ String(state.currentAngle));
    }
    else if (fabs(state.currentAngle) >= Settings::breakAngle)
    {
        // Send the break signal and do nothing.
        esc.writeMicroseconds(Settings::escBreakSignal);
        controller.reset();
        Serial.println("Esc signal: " + String(Settings::escBreakSignal));
        delay(500);
    }
    else
    {
        double escDiff = controller.compute(state.currentAngle, state.referenceAngle);

        if (abs(escDiff) > Settings::escUpperBound)
        {
            escDiff = escDiff > 0 ? Settings::escUpperBound : -Settings::escUpperBound;
        }

        long escVal = Settings::escBreakSignal + Settings::escUpperBound + escDiff + Settings::escLowerBound;
        esc.writeMicroseconds(escVal);
        Serial.println("Esc signal: " + String(escVal));
    }

    Serial.print('\n');
    delay(Settings::PIDSampleTime);
}

double getAngleFromIMU(Adafruit_BNO055& bno)
{
   const double angleOffset = 39.5;
   imu::Quaternion q = bno.getQuat();
   imu::Vector<3> euler = q.toEuler();
   return euler.y() * 180.0/PI + angleOffset;
}

void printErrorAndExit(const String& message)
{
    esc.writeMicroseconds(Settings::escBreakSignal);
    Serial.println(message);
    while(true);
}
