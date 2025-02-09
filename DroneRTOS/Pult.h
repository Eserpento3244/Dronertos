#ifndef PULT_H
#define PULT_H

#include <Arduino.h>

inline float limit(float min, float max, float value);

struct PultData
{
  int8_t x;
  int8_t y;
  int8_t z;
  int16_t speed;
};

class Pult
{
  private:
    const float alpha = 0.2;

    int ch1pin = 0;
    int ch2pin = 0;
    int ch3pin = 0;
    int ch4pin = 0;

  public:
    Pult(int p1, int p2, int p3, int p4);
    float Sch1 = 0, preSch1 = 0;
    float Sch2 = 0, preSch2 = 0;
    float Sch3 = 0, preSch3 = 0;
    float Sch4 = 0, preSch4 = 0;
    void Filterdata();
    void getPultData(PultData* pd);
};

#endif
