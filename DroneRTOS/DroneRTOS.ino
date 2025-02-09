#include <freertos/FreeRTOS.h>
#include <ESP32Servo.h>
#include "MPUSENSOR.h"
#include "Pult.h"
#include "PIDCONTROLLER.h"
#include "Errors.h"

#define Debug
//#define Release

#define ESCPin1 13
#define ESCPin2 12
#define ESCPin3 27
#define ESCPin4 14

#define PultPin1 18
#define PultPin2 5
#define PultPin3 17
#define PultPin4 16

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

int Errorcode = ERROR_NONE;
bool taskmotorcontrollstarted = false;

TaskHandle_t TaskMotorControllerHandle = NULL;
TaskHandle_t TaskPultUpdateHandle = NULL;
TaskHandle_t TaskSensorUpdateHandle = NULL;

QueueHandle_t SensorQueue;
QueueHandle_t PultQueue;
QueueHandle_t InformationQueue;//использовалось для отладки данных через интернет


//---------------------------------------------------------------------------------------------------------------------------------------------------------
/* Получение значений сенсора и отпаврка их к задаче с pid
   --------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void TaskSensorUpdate(void *pvParameters)
{
  SensorData sdata;
  MPUSENSOR mpu;
  if (mpu.Load() == false) Errorcode = ERROR_LOAD_IMU;
  if (mpu.Calibrate() == false) Errorcode = ERROR_LOAD_IMU;

  int er = 0;
  do
  {
    sdata.x = 0;
    sdata.z = 0;
    sdata.y = 0;
    mpu.Filterdata();
    mpu.getAngles(&sdata);

    er++;
    if (er > 1000) Errorcode = ERROR_SENSOR_TASK_FALED;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  while (sdata.x == 0 && sdata.y == 0 && sdata.z == 0);

  xTaskNotifyGive(TaskPultUpdateHandle);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  int errors = 0;
  while (true)
  {


    mpu.Filterdata();
    mpu.getAngles(&sdata);

    if (taskmotorcontrollstarted)
    {
      if (xQueueSend(SensorQueue, &sdata, pdMS_TO_TICKS(1)) != pdPASS )
      {
        errors++;
        if (errors > 100000)
        {
          Errorcode = ERROR_SEND_SENSOR_QUEUE;
        }
      }
      else
      {
        errors = 0;
      }
    }
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));

  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
/* Получение значений пульта и отпаврка их к задаче с pid
   --------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void TaskPultUpdate(void *pvParameters)
{
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  PultData pultdata;
  pultdata.x = 0;
  pultdata.y = 0;
  pultdata.z = 0;
  pultdata.speed = 0;
  Pult pult(PultPin1, PultPin2, PultPin3, PultPin4);
  int er = 0;

  do
  {
    pultdata.x = 0;
    pultdata.z = 0;
    pultdata.y = 0;
    pult.Filterdata();
    pult.getPultData(&pultdata);
    er++;
    if (er > 10000) Errorcode = ERROR_PULT_TASK_FALED;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  while (pultdata.x == 0 && pultdata.y == 0 && pultdata.z == 0);
  xTaskNotifyGive(TaskMotorControllerHandle);
  while (true)
  {

    pult.Filterdata();
    pult.getPultData(&pultdata);
    if (taskmotorcontrollstarted)
    {
      if (xQueueSend(PultQueue, &pultdata, pdMS_TO_TICKS(1)) != pdPASS)
      {
        Errorcode = ERROR_SEND_PULT_QUEUE;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));

  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
/* Получение значений из задач пульта и сенсора и обработка их с pid
   --------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void TaskMotorController(void *pvParameters)
{
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

  PultData pultdata, respult;
  pultdata.x = 0;
  pultdata.y = 0;
  pultdata.z = 0;
  pultdata.speed = 0;

  SensorData sdata, _sdata;
  sdata.x = 0;
  sdata.z = 0;
  sdata.y = 0;

  int errors = 0, errors2 = 0;
  int taskerrors = 0;
  uint32_t lastTime = millis();
  // Значения пид
  /* 1 параметр коэффициент p
     2 параметр коэффициент i
     3 параметр коэффициент d
     4-5 параметр минимальное и максимальное значение вычиления pid
     6-7 параметр минимальное и максимальное значение I
     8 параметр влияет на плавность d(лучше не трогать а изменять коэффициент)
  */
  PID pidx(2.5, 0.05, 0.06, -50, 50, -200, 200, 0.1),
      pidy(2.5, 0.05, 0.06, -50, 50, -200, 200, 0.1),
      pidz(1, 0.01, 0.01, -2, 2, -10, 10, 0.1);

  taskmotorcontrollstarted = true;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (true)
  {

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    respult = pultdata;

    if (xQueueReceive(PultQueue, &pultdata, pdMS_TO_TICKS(1)) == pdPASS)
    {
      errors = 0;
    }
    else
    {
      pultdata = respult;

      errors++;
      if (errors > 1000)
      {
        Errorcode = ERROR_RECEIVE_PULT_QUEUE;
      }
    }

    _sdata = sdata;
    errors2++;
    if (errors2 > 1000)
    {
      Errorcode = ERROR_RECEIVE_SENSOR_QUEUE;
    }
    while (xQueueReceive(SensorQueue, &sdata, pdMS_TO_TICKS(1)) == pdPASS)
    {
      if (errors2 > 0) errors2--;
      _sdata = sdata;
    }
    sdata = _sdata;

    int fx = pidx.calculate(sdata.x, pultdata.x, dt);
    int fy = pidy.calculate(sdata.y, pultdata.y, dt);
    int fz = pidz.calculate(sdata.z, pultdata.z, dt);


    if (pultdata.speed > 70) // если значение пульта выше 70 то работает
    {
      int throttle = pultdata.speed + 1000 - 70; // задается скорость оборотов

      int m1 = throttle + fx - fy + fz;
      int m2 = throttle + fx + fy - fz;
      int m3 = throttle - fx + fy + fz;
      int m4 = throttle - fx - fy - fz;

      if (m1 < 1000) m1 = 1000;
      else if (m1 > 2000) m1 = 2000;

      if (m2 < 1000) m2 = 1000;
      else if (m2 > 2000) m2 = 2000;

      if (m3 < 1000) m3 = 1000;
      else if (m3 > 2000) m3 = 2000;

      if (m4 < 1000) m4 = 1000;
      else  if (m4 > 2000) m4 = 2000;

      esc1.writeMicroseconds(m1);
      esc2.writeMicroseconds(m2);
      esc3.writeMicroseconds(m3);
      esc4.writeMicroseconds(m4);
    }
    else
    {
      Otkl(); // отключение моторов
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 10 мс задержка

  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
/* Получение ошибок с 3 предыдущих задач
   --------------------------------------------------------------------------------------------------------------------------------------------------------
*/


void Otkl()
{
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
}


void TaskErrorsUpdate(void *pvParameters)
{
  InfoData infd;
  infd.type = 0;
  infd.data = 0;

  while (true)
  {



    /* Serial.print("t1"); Serial.print(sch.getpercent(0));
      Serial.print("t2"); Serial.print(sch.getpercent(1));
      Serial.print("t3"); Serial.print(sch.getpercent(2));
      Serial.print("t4"); Serial.println(sch.getpercent(3)); */

    switch (Errorcode)
    {
      case ERROR_NONE:
        break;

      case ERROR_LOAD_IMU:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;

      case ERROR_CALIBRATE_IMU:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;

      case ERROR_PULT_TASK_FALED:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;

      case ERROR_SEND_SENSOR_QUEUE:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;

      case ERROR_SEND_PULT_QUEUE:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;

      case ERROR_SENSOR_TASK_FALED:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;

      case ERROR_RECEIVE_SENSOR_QUEUE:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;

      case ERROR_RECEIVE_PULT_QUEUE:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;

      default:
        vTaskSuspend(TaskPultUpdateHandle);
        vTaskSuspend(TaskMotorControllerHandle);
        vTaskSuspend(TaskSensorUpdateHandle);
#ifdef Debug
        Serial.println(MacrosString(Errorcode));
#endif
        while (1) Otkl();
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(200));

  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------
/* Инициализация
   --------------------------------------------------------------------------------------------------------------------------------------------------------
*/

void setup()
{
#ifdef Debug
  Serial.begin(115200);
#endif


  esc1.attach(ESCPin1);
  esc2.attach(ESCPin2);
  esc3.attach(ESCPin3);
  esc4.attach(ESCPin4);
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
  delay(6000);

  do
  {
    SensorQueue = xQueueCreate(30, sizeof(SensorData));
  } while (SensorQueue == NULL);

  do
  {
    PultQueue = xQueueCreate(30, sizeof(PultData));
  } while (PultQueue == NULL);

  do
  {
    InformationQueue = xQueueCreate(20, sizeof(InfoData));
  } while (InformationQueue == NULL);

  xTaskCreatePinnedToCore(
    TaskSensorUpdate,
    "T1",
    16384,
    NULL,
    1,
    &TaskSensorUpdateHandle,
    0
  );

  xTaskCreatePinnedToCore(
    TaskPultUpdate,
    "T2",
    8192,
    NULL,
    1,
    &TaskPultUpdateHandle,
    0
  );

  xTaskCreatePinnedToCore(
    TaskMotorController,
    "T3",
    8192,
    NULL,
    1,
    &TaskMotorControllerHandle,
    1
  );

  xTaskCreatePinnedToCore(
    TaskErrorsUpdate,
    "T4",
    8192,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {}
