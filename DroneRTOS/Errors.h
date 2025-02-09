#ifndef ERRORS_H
#define ERRORS_H

#define ERROR_NONE 0
#define ERROR_LOAD_IMU 3
#define ERROR_CALIBRATE_IMU 4
#define ERROR_PULT_TASK_FALED 5
#define ERROR_SENSOR_TASK_FALED 6
#define ERROR_SEND_SENSOR_QUEUE 7
#define ERROR_SEND_PULT_QUEUE 8
#define ERROR_RECEIVE_SENSOR_QUEUE 9
#define ERROR_RECEIVE_PULT_QUEUE 10

#define DEBUG_LOAD_QUEUE 11
#define DEBUG_START_SENSOR_TASK 12
#define DEBUG_START_PULT_TASK 13
#define DEBUG_START_MOTOR_TASK 14
#define DEBUG_START_ERROR_TASK 15
#define DEBUG_START_IMU_LOAD 16
#define DEBUG_START_IMU_CALIBRATE 17
#define DEBUG_START_PULT_LOAD 18

const char* MacrosString(char n);

#define TYPE_IMU_X 'x'
#define TYPE_IMU_Y 'y'
#define TYPE_IMU_Z 'z'
#define TYPE_PID_X 'X'
#define TYPE_PID_Y 'Y'
#define TYPE_PID_Z 'Z'
#define TYPE_PULT_SPEED 's'
#define TYPE_PULT_X 'a'
#define TYPE_PULT_Y 'b'
#define TYPE_PULT_Z 'c'
#define TYPE_MESSAGE 'm'

struct InfoData
{
  char type;
  float data;
};

#endif
