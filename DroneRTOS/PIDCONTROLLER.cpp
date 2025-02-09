#include "PIDCONTROLLER.h"

inline float limit(float min, float max, float value) 
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

PID::PID(float Kpv, float Kiv, float Kdv, float outMin, float outMax, float intMin, float intMax, float filtAlpha) 
{
    Kp = Kpv;
    Ki = Kiv;
    Kd = Kdv;
    prevError = 0;
    integral = 0;
    integralMin = intMin;
    integralMax = intMax;
    outputMin = outMin;
    outputMax = outMax;
    alpha = (filtAlpha >= 0.0f && filtAlpha <= 1.0f) ? filtAlpha : 0.1f;
    derivativeFiltered = 0;
    prevOutput = 0;
}

float PID::calculate(float current_value, float setpoint, float dt) 
{
    if (dt <= 0.001) return prevOutput;

    float error = setpoint - current_value;

    float Pout = Kp * error;

    integral += error * dt;
    integral = limit(integralMin, integralMax, integral);
    float Iout = Ki * integral;

    float derivative = (error - prevError) / dt;
    derivativeFiltered = alpha * derivativeFiltered + (1 - alpha) * derivative;
    float Dout = Kd * derivativeFiltered;

    prevError = error;

    float output = Pout + Iout + Dout;
    output = limit(outputMin, outputMax, output);

    prevOutput = output;
    return -output;
}
