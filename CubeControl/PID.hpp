#include <Arduino.h>

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
