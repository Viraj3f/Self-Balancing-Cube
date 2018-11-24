#include <Arduino.h>

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
