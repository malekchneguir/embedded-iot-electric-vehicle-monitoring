/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* Flag : Modbus TCP initialisé et prêt */
volatile bool modbusReady = false;
volatile SensorData_t sensorData;
/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 8,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SensorsTask */
osThreadId_t SensorsTaskHandle;
const osThreadAttr_t SensorsTask_attributes = {
  .name = "SensorsTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorsTask */
  SensorsTaskHandle = osThreadNew(StartTask02, NULL, &SensorsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* 1. Initialise LwIP */
  MX_LWIP_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* 2. Attends un peu que le lien soit stable */
  osDelay(2000);

  /* 3. Initialise le serveur Modbus TCP */
  if (eMBTCPInit(502) == MB_ENOERR) {
    eMBEnable();
    lastModbusTick = HAL_GetTick(); /* Initialisation ICI */
    modbusReady = true;
  }

  /* Infinite loop */
  for(;;)
  {
    /* Traitement Modbus TCP */
    eMBPoll();

    /* --- AUTO-RÉCUPÉRATION ETHERNET --- */
    extern ETH_HandleTypeDef heth;
    if (HAL_ETH_GetState(&heth) == HAL_ETH_STATE_ERROR) {
        HAL_ETH_Stop_IT(&heth);
        HAL_ETH_Start_IT(&heth);
    }

    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin); /* Témoin Tâche Modbus (Verte) */
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the StartTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Variables Globales definies dans main.c/gps.c */
  extern float gpsLat;
  extern float gpsLon;
  extern char  gpsBuffer[256];
  extern volatile uint8_t dhtError;
  
  uint32_t dht11_counter = 0;
  
  /* Infinite loop */
  for(;;)
  {
    /* 1. Lecture des capteurs RÉELS */
    sensorData.gas_ppm     = ReadGasSensor();
    sensorData.distance_cm = ReadUltrasound();
    sensorData.speed_kmh   = ReadSpeed();
    
    /* Lecture DHT11 toutes les 2 secondes (10 * 200ms) */
    if (dht11_counter++ >= 10) {
        float temp = 0.0f, hum = 0.0f;
        /* --- UTILISATION DE PB10 (D6) --- */
        ReadDHT11_On_PB10(&temp, &hum);
        if (temp > 0.0f || hum > 0.0f) {
            sensorData.temperature = temp;
            sensorData.humidity = hum;
        } else {
            extern volatile uint8_t dhtError;
            sensorData.temperature = (float)dhtError + 0.1f;
        }
        dht11_counter = 0;
        osDelay(50); // Laisse l'Ethernet respirer après le DHT11
    }
    
    /* Lecture GPS */
    GPS_Update();
    sensorData.gps_lat = gpsLat;
    sensorData.gps_lon = gpsLon;
    
    /* Rétablissement de la distance réelle */
    sensorData.distance_cm = ReadUltrasound();

    /* 3. Mise à jour des registres Modbus */
    if (modbusReady) {
      UpdateModbusRegisters();
    }
    
    /* 4. Témoin d'activité (Bleue) */
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

    osDelay(300);  /* Cycle allongé à 300 ms pour la stabilité */
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
