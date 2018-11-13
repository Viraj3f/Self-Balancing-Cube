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
    const int escUpperBound = 500;

    // The minimum signal to add  or subtract relative to 
    // the break signal, since BLDCs are unstable at low speeds.
    const int escLowerBound = 30;

    // The angle at which the cube system should just quit.
    const int unsafeAngle = 360;

    // The angle at which the cube should begin balancing at.
    const int breakAngle = 30;

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
  * 
  */

class PID
{
  public:
    unsigned long lastTime;
    double output;
    double errSum, lastInput;
    double kp, ki, kd;
    unsigned long sampleTime;

    PID(double _kp, double _ki, double _kd, unsigned long _sampleTime)
    {
        this->sampleTime = _sampleTime;
        double sampleTimeInSec = ((double)sampleTime)/1000;
        this->kp = _kp;
        this->ki = _ki * sampleTimeInSec;
        this->kd = _kd / sampleTimeInSec;
    }

    void reset()
    {
      lastTime = 0;
      output = 0;
      errSum = 0;
      lastInput = 0;
    }


    double compute(double Input, double Setpoint)
    {
       unsigned long now = millis();
       unsigned long timeChange = (now - lastTime);
       if (timeChange >= sampleTime)
       {
          /*Compute all the working error variables*/
          double error = Setpoint - Input;
          errSum += error;
          double dInput = (Input - lastInput);

          if (Input * lastInput < 0){
            //zero crossing
            errSum = 0;
          }
      
          /*Compute PID Output*/
          output = kp * error + ki * errSum - kd * dInput;
      
          /*Remember some variables for next time*/
          lastInput = Input;
          lastTime = now;
       }

       return output;
    }
};

// Controller
PID controller(-30, -6, 0, Settings::PIDSampleTime);

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
}


void loop()
{
    state.currentAngle = getAngleFromIMU(bno);
    Serial.print("Theta: " + String(state.currentAngle) + " ");
    if (fabs(state.currentAngle) >= Settings::unsafeAngle)
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
        delay(1000);
    }
    else
    {
        double escDiff = controller.compute(state.currentAngle, state.referenceAngle);

        if (abs(escDiff) > Settings::escUpperBound)
        {
            escDiff = escDiff > 0 ? Settings::escUpperBound : -Settings::escUpperBound;
        }
        else if (abs(escDiff) < Settings::escLowerBound)
        {
            escDiff = escDiff > 0 ? Settings::escLowerBound : -Settings::escLowerBound;
        }

        long escValue = Settings::escBreakSignal + escDiff;
        esc.writeMicroseconds(escValue);
        Serial.println("Relative esc signal: " + String(escDiff));
    }

    Serial.print('\n');
    delay(Settings::PIDSampleTime);
}



double getAngleFromIMU(Adafruit_BNO055& bno)
{
   const double angleOffset = 44.0;
   imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
   return euler.y() - angleOffset;
}

void printErrorAndExit(const String& message)
{
    esc.writeMicroseconds(Settings::escBreakSignal);
    Serial.println(message);
    while(true);
}
  

