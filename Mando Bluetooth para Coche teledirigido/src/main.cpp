#include <Arduino.h>
#include <driver/adc.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "BluetoothSerial.h"
#include "YoistickRawData.h"
#include "YoistickDecodedData.h"

#define PIN_YOISTICK_PUSH 5
#define DECODED_MAX_VALUE 1024
#define DECODED_MIN_VALUE 0
#define RAW_MAX_VALUE 2048
#define RAW_MIN_VALUE 0

QueueHandle_t rawDataQueue;
QueueHandle_t decodedDataQueue;

BluetoothSerial SerialBT;

void vTaskReadYoistick(void *pvParameters) {

  //UBaseType_t uxHighWaterMark;
  YoistickRawData yrd;
  portBASE_TYPE result;

  for (;;) {
    yrd.valX = adc1_get_raw(ADC1_CHANNEL_5) - 2048;
    yrd.valY = adc1_get_raw(ADC1_CHANNEL_4) - 2048;
    result = xQueueSendToBack(rawDataQueue, &yrd, 200/portTICK_RATE_MS);
    if (result != pdPASS) {
      printf("[TaskReadYoistick]: no se ha podido enviar a la cola el dato leido.\r\n");
    }
    /*uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printf("[TaskReadYoistick]: uxHigWaterMark = %d\r\n", uxHighWaterMark);*/
    vTaskDelay(250/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);

}

void vTaskMsgEncoder(void *pvParameters) {

  //UBaseType_t uxHighWaterMark;
  YoistickRawData yrd;
  YoistickDecodedData ydd;
  portBASE_TYPE result;

  for (;;) {

    result = xQueueReceive(rawDataQueue, &yrd, 100/portTICK_RATE_MS);
    if (result == pdPASS) {
      if (yrd.valX > 0) {
        ydd.dirX = true;
      } else {
        ydd.dirX = false;
        yrd.valX = yrd.valX * -1;
      }
      if (yrd.valY > 0) {
        ydd.dirY = true;
      } else {
        ydd.dirY = false;
        yrd.valY = yrd.valY * -1;
      }
      ydd.velX = (DECODED_MIN_VALUE + (((DECODED_MAX_VALUE - DECODED_MIN_VALUE) * yrd.valX) / RAW_MAX_VALUE));
      ydd.velY = (DECODED_MIN_VALUE + (((DECODED_MAX_VALUE - DECODED_MIN_VALUE) * yrd.valY) / RAW_MAX_VALUE));
      
      result = xQueueSendToBack(decodedDataQueue, &ydd, 1000/portTICK_RATE_MS);
      if (result != pdPASS) {
        printf("[TaskMsgEncoder]: no se ha podido enviar a la cola el dato decodificado");
      }
    } else {
      printf("[TaskMsgEncoder]: no se ha podido recibir de la cola");
    }
    /*uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printf("[TaskMsgEncoder]: uxHigWaterMark = %d\r\n", uxHighWaterMark);*/

  }

  vTaskDelete(NULL);

}

void vTaskSender(void *pvParameters) {

  //UBaseType_t uxHighWaterMark;
  YoistickDecodedData ydd;
  portBASE_TYPE result;

  for (;;) {

    result = xQueueReceive(decodedDataQueue, &ydd, 1000/portTICK_RATE_MS);
    if (result == pdPASS) {
      SerialBT.printf("%04d%d%04d%d", ydd.velX, ydd.dirX, ydd.velY, ydd.dirY); //10 caracteres.
    } else {
      printf("[TaskSender]: no se ha podido recibir de la cola.\r\n");
    }
    /*uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    printf("[TaskSender]: uxHigWaterMark = %d\r\n", uxHighWaterMark);*/

  }
  vTaskDelete(NULL);

}

void app_main(void) {

  if(xTaskGetSchedulerState()==taskSCHEDULER_RUNNING)
    printf("Scheduler is running\n");

  rawDataQueue = xQueueCreate(2, sizeof(YoistickRawData));
  decodedDataQueue = xQueueCreate(2, sizeof(YoistickDecodedData));

  if (rawDataQueue != NULL && decodedDataQueue != NULL) {
    xTaskCreatePinnedToCore(vTaskReadYoistick, "TaskReadYoistick", 768, NULL, 4, NULL, 0); //tarea principal. Mayor prioridad. Lectura de los datos del yoistick
    xTaskCreatePinnedToCore(vTaskMsgEncoder, "TaskEncoder", 1024, NULL, 3, NULL, 1); //tarea que transforma la informacion original en informacion lista para enviar. Prioridad media
    xTaskCreatePinnedToCore(vTaskSender, "TaskSender", 1792, NULL, 2, NULL, 0); //Tarea que envia los datos por bluetooth. Prioridad baja.
  }

}

void setup() {

  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten((adc1_channel_t)ADC2_CHANNEL_5, ADC_ATTEN_11db);
  adc1_config_channel_atten((adc1_channel_t)ADC2_CHANNEL_4, ADC_ATTEN_11db);
  pinMode(PIN_YOISTICK_PUSH, INPUT);
  Serial.begin(115200);
  SerialBT.begin("P2_G1", true);
  SerialBT.setPin("1234");
  uint8_t addrBT12[] = {0xd4, 0x55, 0x00, 0x00, 0x81, 0x82};
  bool result = SerialBT.connect(addrBT12);
  if (result) {
    printf("resultTrue");
  } else {
    printf("resultFalse");
  }

  app_main();

}

void loop() {

}