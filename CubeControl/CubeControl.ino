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

 /*
  * PID implementation class.
  */
class PID
{
  public:
    unsigned long lastTime;
    double output;
    double errSum, lastErr;
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
        lastTime = 0;
        output = 0;
        errSum = 0;
        lastErr = 0;
    }

    double compute(double input, double setpoint)
    {
        unsigned long now = millis();
        unsigned long timeChange = (now - lastTime);
        if (timeChange >= sampleTime)
        {
            /*Compute all the working error variables*/
            double error = setpoint - input;
            errSum += error;

            /*Compute PID Output*/
            output = kp * error + ki * errSum + kd * (error - lastErr);

            /*Remember some variables for next time*/
            lastErr = error;
            lastTime = now;
        }

        return output;
    }
};

// Controller
// 8 2 0.1
PID controller(-8, 0, 0, Settings::PIDSampleTime);

// The state of the system.
State state;

// The IMU object.
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
    //Serial.print("Theta: " + String(state.currentAngle) + " ");

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

double getAngleFromIMU(Adafruit_BNO055& bno)
{
   const double angleOffset = 20;
   imu::Quaternion q = bno.getQuat();
   imu::Vector<3> euler = q.toEuler();
   return euler.y() * 180.0/PI; //+ angleOffset;
}

void printErrorAndExit(const String& message)
{
    esc.writeMicroseconds(Settings::escBreakSignal);
    Serial.println(message);
    while(true);
}
