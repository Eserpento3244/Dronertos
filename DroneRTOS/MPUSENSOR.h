#ifndef MPUSENSOR_H
#define MPUSENSOR_H
#include <Arduino.h>
#include <Wire.h>

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

#define GYRO_CALIBRATE_RANGE 1000

inline float limit(float min, float max, float value);
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);

struct SensorData
{
  float x;
  float y;
  float z;
};

class MPUSENSOR
{
  private:

    float roll = 0 , pitch = 0, yaw = 0;
    float nullroll = 0, nullpitch = 0, nullyaw = 0;
    float alpha = 0.9;
    unsigned long lastTime = 0;
  public:
    bool Load();
    void Filterdata();
    bool Calibrate();
    bool getAngles(SensorData* s);
};



#endif
