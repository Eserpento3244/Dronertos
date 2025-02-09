#include "Errors.h"

const char* MacrosString(char n)
{
  switch (n)
  {
    case ERROR_NONE:
      return "";

    case ERROR_LOAD_IMU:
      return "Load imu error";

    case ERROR_CALIBRATE_IMU:
      return "Imu calibrate error";

    case ERROR_PULT_TASK_FALED:
      return "Pult task error";

    case ERROR_SEND_SENSOR_QUEUE:
      return "Sensor queue send error";

    case ERROR_SEND_PULT_QUEUE:
      return "Pult queue send error";

    case ERROR_SENSOR_TASK_FALED:
      return "Sensor task error";

    case ERROR_RECEIVE_SENSOR_QUEUE:
      return "Sensor queue receive error";

    case ERROR_RECEIVE_PULT_QUEUE:
      return "Pult queue receive error";

    case DEBUG_START_IMU_LOAD:
      return "Load imu";

    case DEBUG_START_IMU_CALIBRATE:
      return "Imu calibrate";

    case DEBUG_START_PULT_TASK:
      return "Start pult task";

    case DEBUG_START_MOTOR_TASK:
      return "Start motor task";

    case DEBUG_LOAD_QUEUE:
      return "Queue load";

    case DEBUG_START_ERROR_TASK:
      return "Start error task";

    case DEBUG_START_SENSOR_TASK:
      return "Start sensor task";

    case DEBUG_START_PULT_LOAD:
      return "Pult load";

    default:
      return "Error";
  }
}
