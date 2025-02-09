#include "Pult.h"

inline float limit(float min, float max, float value)
{
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

Pult::Pult(int p1, int p2, int p3, int p4)
{
  ch1pin = p1;
  ch2pin = p2;
  ch3pin = p3;
  ch4pin = p4;

  preSch1 = 0;
  preSch2 = 0;
  preSch3 = 0;
  preSch4 = 0;
}

void Pult::Filterdata()
{
  int d1 = pulseIn(ch1pin, HIGH, 1000000);
  int d2 = pulseIn(ch2pin, HIGH, 1000000);
  int d3 = pulseIn(ch3pin, HIGH, 1000000);
  int d4 = pulseIn(ch4pin, HIGH, 1000000);


  if (d1 + d2 + d3 + d4 <= 100)
  {
    Sch1 = 0;
    Sch2 = 0;
    Sch3 = 0;
    Sch4 = 0;
    return;
  }

  preSch1 = limit(-15, 15, ((float)(d1 - 1565) * 32.0f) / 500.0f);
  preSch2 = limit(-15, 15, ((float)(d2  - 1492) * 32.0f) / 500.0f);
  preSch3 = limit(0, 1000, (float)(d3 - 1010) * 0.3);
  preSch4 = limit(-30, 30, ((float)(d4 - 1502) * 60.0f) / 500.0f);

  Sch1 = alpha * Sch1 + (1.0f - alpha) * preSch1;
  Sch2 = alpha * Sch2 + (1.0f - alpha) * preSch2;
  Sch3 = alpha * Sch3 + (1.0f - alpha) * preSch3;
  Sch4 = alpha * Sch4 + (1.0f - alpha) * preSch4;
}

void Pult::getPultData(PultData* pd)
{
  pd->x = limit(-15, 15, Sch1);
  pd->y = limit(-15, 15, Sch2);
  pd->z = limit(-30, 30, Sch4);
  pd->speed = limit(0, 1000, Sch3);
}
