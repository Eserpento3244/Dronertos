#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

inline float limit(float min, float max, float value);

class PID
{
  private:

    float prevError;
    float integral;
    float integralMin, integralMax;
    float outputMin, outputMax;
    float derivativeFiltered;
    float alpha;
    float prevOutput;
  public:
    float Kp, Ki, Kd;
    PID();
    PID(float Kpv, float Kiv, float Kdv, float outMin, float outMax, float intMin, float intMax, float filtAlpha = 0.1);
    float calculate(float current_value, float point, float dt);
};

#endif
