#include "MPUSENSOR.h"

float limit(float min, float max, float value)
{
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

bool MPUSENSOR::Load()
{
  if (Wire.begin() == NULL) return false;
  I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);
  I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);

  I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_250_DPS);
  I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  //I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);
  lastTime = millis();



  return true;
}

bool MPUSENSOR::Calibrate()
{
  int errors = 0;
  alpha = 0.4;

  for (int i = 0; i <= GYRO_CALIBRATE_RANGE; i++)
  {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

    int16_t ax = -(Buf[0] << 8 | Buf[1]);
    int16_t ay = -(Buf[2] << 8 | Buf[3]);
    int16_t az = Buf[4] << 8 | Buf[5];

    int16_t gx = -(Buf[8] << 8 | Buf[9]);
    int16_t gy = -(Buf[10] << 8 | Buf[11]);
    int16_t gz = Buf[12] << 8 | Buf[13];

    /*uint8_t ST1;
    do
    {
      I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
    }
    while (!(ST1 & 0x01));

    uint8_t Mag[7];
    I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

    int16_t mx = -(Mag[3] << 8 | Mag[2]);
    int16_t my = -(Mag[1] << 8 | Mag[0]);
    int16_t mz = -(Mag[5] << 8 | Mag[4]);*/

    if (ax == 0 && ay == 0 && az == 0) errors++;

    nullroll = alpha * (gx * dt / 131.0) + (1.0 - alpha) * nullroll;
    nullpitch = alpha * (gy * dt / 131.0) + (1.0 - alpha) * nullpitch;
    nullyaw = alpha * ((gz * dt / 131.0) * 3) + (1.0 - alpha) * nullyaw;
  }
  alpha = 0.7;
  lastTime = millis();
  return (errors < GYRO_CALIBRATE_RANGE / 10);
}

void MPUSENSOR::Filterdata()
{
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  uint8_t Buf[14];
  I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
  //I2CwriteByte(MAG_ADDRESS, 0x0A, 0x16);

  int16_t ax = -(Buf[0] << 8 | Buf[1]);
  int16_t ay = -(Buf[2] << 8 | Buf[3]);
  int16_t az = Buf[4] << 8 | Buf[5];

  int16_t gx = -(Buf[8] << 8 | Buf[9]);
  int16_t gy = -(Buf[10] << 8 | Buf[11]);
  int16_t gz = Buf[12] << 8 | Buf[13];


  float accRoll = atan2(ay, az) * 180.0 / PI;
  float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

  roll = alpha * (roll + (gx * dt / 131.0) - nullroll) + (1.0 - alpha) * accRoll;
  pitch = alpha * (pitch + (gy * dt / 131.0) - nullpitch) + (1.0 - alpha) * accPitch;
  yaw = alpha * yaw + (1.0 - alpha) * (((gz * dt / 131.0) * 3) - nullyaw);
  
}

bool MPUSENSOR::getAngles(SensorData* s)
{
  if (s == NULL) return false;
  s->z = limit(-180, 180, yaw);
  s->y = limit(-90, 90, pitch);
  s->x = limit(-90, 90, roll);
  return true;
}
