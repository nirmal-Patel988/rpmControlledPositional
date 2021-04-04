#include <PID_v1.h>
#include <rpmPID.h>
#define pulseMode 0
#define angleMode 1
class positional
{
public:
    RPM *rpm;
    Motor *mtr1;
    int mode = pulseMode;
    double Setpoint = 0, Input, Output;
    int targetAngle = 0, targetPulse = 0;
    //Specify the links and initial tuning parameters
    double aggKp = 0.03, aggKi = 0, aggKd = 0.00;
    double softKp = 0.01, softKi = 0, softKd = 0.00;

    int softThreshold = 0;
    long diff = 0;
    bool enable = true;
    int speed = 0;
    PID *myPID = new PID(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, DIRECT);
    int sampleTime = 0;

    positional(Motor *mtr1)
    {
        this->mtr1 = mtr1;
        rpm = new RPM(this->mtr1);
        myPID->SetMode(AUTOMATIC);
        myPID->SetSampleTime(this->sampleTime);
    }
    long getReadings()
    {
        // Serial.println("data OF reading:");
        // Serial.println(mtr1->getReadings());
        return mtr1->getReadings();
    }
    positional()
    {
    }

    void setThreshold(int targetThreshold)
    {
        softThreshold = targetThreshold;
    }
    void setPulse(int targetPulse)
    {
        this->Setpoint = targetPulse;
        // Serial.println(this->Setpoint);
    }
    void setOutputLimits(int min, int max)
    {
        myPID->SetOutputLimits(min, max);
    }

    void setAggTunings(double Kp, double Ki, double Kd)
    {
        this->aggKp = Kp;
        this->aggKi = Ki;
        this->aggKd = Kd;
    }

    void setSoftTunings(double Kp, double Ki, double Kd)
    {
        this->softKp = Kp;
        this->softKi = Ki;
        this->softKd = Kd;
    }

    void setTunings(double Kp, double Ki, double Kd)
    {
        myPID->SetTunings(Kp, Ki, Kd);
        // Serial.println("object works00");
    }
    void compute()
    {
        if (enable)
        {
            Input = mtr1->getReadings();

            if (abs(Input) < softThreshold)
            {
                this->setTunings(softKp, softKi, softKd);
            }

            else
            {
                this->setTunings(aggKp, aggKi, aggKd);
            }

            myPID->Compute();
            rpm->setAggTunings(0.5,0,0);
            rpm->setSoftTunings(0.5,0,0);
            rpm->setRPM(Output);
            rpm->compute();
        }
    }
};