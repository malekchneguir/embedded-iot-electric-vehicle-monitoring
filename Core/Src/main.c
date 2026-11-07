/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Système IoT embarqué — STM32H753ZI
  *                   Capteurs + Ethernet + Modbus TCP/IP
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "lwip.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mb.h"
#include "mbtcp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MODBUS_REG_COUNT     10
#define GPS_BUFFER_SIZE      128
#define TEMP_SENSOR_ADDR     0x90

/* Broches HC-SR04 ultrason */
#define TRIG_PIN             GPIO_PIN_8
#define TRIG_PORT            GPIOA
#define ECHO_PIN             GPIO_PIN_9
#define ECHO_PORT            GPIOA
/* USER CODE END PD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Données capteurs partagées entre tâches FreeRTOS */
volatile uint32_t lastModbusTick = 0;


/* Registres Modbus (10 x 16 bits) */
/* Reg 0 : gaz (ppm x10)           */
/* Reg 1 : température (°C x10)    */
/* Reg 2 : vitesse (km/h x10)      */
/* Reg 3 : GPS lat entier           */
/* Reg 4 : GPS lat décimal x10000  */
/* Reg 5 : GPS lon entier           */
/* Reg 6 : GPS lon décimal x10000  */
/* Reg 7 : distance (cm x10)       */
/* Reg 8-9 : réservés               */
uint16_t modbusRegisters[MODBUS_REG_COUNT] = {0};



/* Handle tâche FreeRTOS */
osThreadId_t sensorsTaskHandle;
const osThreadAttr_t sensorsTask_attributes = {
    .name       = "SensorsTask",
    .stack_size = 1024 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void     SensorsTask(void *argument);
float    ReadGasSensor(void);
float    ReadSpeed(void);
float    ReadUltrasound(void);
void     GPS_ParseNMEA(char *sentence);
void     GPS_StartReceive(void);
void     UpdateModbusRegisters(void);
void     DWT_Init(void);
uint32_t DWT_GetMicros(void);
static void MPU_Config(void);  /* Configuration cache D2 SRAM -> non-cacheable */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------------------
 * MPU_Config — Rend le D2 SRAM non-cacheable pour le DMA Ethernet
 *
 * OBLIGATOIRE sur STM32H7 : le D-Cache Cortex-M7 corrompt silencieusement
 * les descripteurs DMA ETH si cette region n'est pas marquee non-cacheable.
 * Doit etre appele AVANT HAL_Init().
 * -------------------------------------------------------------------------- */
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Desactive le MPU pendant la configuration */
    HAL_MPU_Disable();

    /* Region 0 : D2 SRAM entier (0x30000000, 512 KB couvre 288 KB reel)
     * Marque comme Non-Cacheable, Non-Bufferable (Normal, non-shared)
     * Ceci couvre :
     *   - LwIP heap     @ 0x30004000 (lwipopts LWIP_RAM_HEAP_POINTER)
     *   - ETH Rx desc   @ 0x30040000 (linker .RxDecripSection)
     *   - ETH Tx desc   @ 0x30040060 (linker .TxDecripSection)
     *   - LwIP Rx pool  @ 0x30040200 (linker .Rx_PoolSection)        */
    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress      = 0x30000000UL;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable = 0x00U;
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE; /* Pas d'exécution de code ici */
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;           /* Crucial pour la synchro DMA */
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Active le MPU — acces non-privilegies utilise la carte memoire par defaut */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* --------------------------------------------------------------------------
 * DWT — compteur de cycles pour mesures de temps précises (ultrason)
 * -------------------------------------------------------------------------- */
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Retourne le temps écoulé en microsecondes depuis le dernier reset DWT */
uint32_t DWT_GetMicros(void) {
    return (DWT->CYCCNT / (SystemCoreClock / 1000000));
}

/**
  * @brief  Délai précis en microsecondes utilisant le compteur de cycles DWT.
  */
void DWT_Delay_us(volatile uint32_t microseconds)
{
    uint32_t clk_cycle_start = DWT->CYCCNT;
    /* Calcul du nombre de cycles nécessaires */
    uint32_t cycles = microseconds * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - clk_cycle_start) < cycles);
}

/* --------------------------------------------------------------------------
 * ReadGasSensor — Lecture ADC3 capteur de gaz MQ-2
 * -------------------------------------------------------------------------- */
float ReadGasSensor(void) {
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
        uint32_t raw = HAL_ADC_GetValue(&hadc3);
        HAL_ADC_Stop(&hadc3);
        
        /* Conversion : ADC 16-bit sur H7 (0..65535) -> 0..3.3V */
        float voltage = (raw * 3.3f) / 65535.0f;
        
        /* Calibration simplifiée : mapping 0-3.3V vers 0-5000 ppm */
        float ppm = (voltage / 3.3f) * 5000.0f;
        return ppm;
    }
    HAL_ADC_Stop(&hadc3);
    return 0.0f;
}

/* --------------------------------------------------------------------------
 * ReadSpeed — Lecture encodeur incrémental sur TIM2
 * Retourne la vitesse en km/h
 * -------------------------------------------------------------------------- */
float ReadSpeed(void) {
    /* Lit le compteur encodeur et le remet à zéro */
    uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* Conversion en km/h :
     * - Roue de 0.5 m de circonférence
     * - 20 impulsions par tour de roue
     * - Tâche appelée toutes les 100 ms (0.1 s)
     * tours/s = count / (20 * 0.1) = count / 2
     * vitesse m/s = tours/s * 0.5
     * vitesse km/h = vitesse m/s * 3.6 */
    float tours_par_seconde = count / (20.0f * 0.1f);
    float vitesse_ms  = tours_par_seconde * 0.5f;
    float vitesse_kmh = vitesse_ms * 3.6f;
    return vitesse_kmh;
}

/* --------------------------------------------------------------------------
 * ReadUltrasound — Capteur HC-SR04 (Trig PF12, Echo PF13)
 * -------------------------------------------------------------------------- */
float ReadUltrasound(void) {
    uint32_t start_time, stop_time;
    uint32_t timeout = 1000000; // Sécurité pour éviter de bloquer la boucle
    
    /* 1. Pulse de 10us sur TRIG (PF12) */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);
    DWT_Delay_us(10);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);
    
    /* 2. Attendre que ECHO (PF13) passe à HIGH */
    while(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13) == GPIO_PIN_RESET && timeout > 0) {
        timeout--;
    }
    start_time = DWT->CYCCNT;
    
    /* 3. Attendre que ECHO repasse à LOW */
    timeout = 1000000;
    while(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13) == GPIO_PIN_SET && timeout > 0) {
        timeout--;
    }
    stop_time = DWT->CYCCNT;
    
    /* 4. Calcul de la distance */
    /* Vitesse du son = 343 m/s => 0.0343 cm/us */
    uint32_t duration_cycles = stop_time - start_time;
    float distance = ((float)duration_cycles * 0.0343f) / (2.0f * (SystemCoreClock / 1000000.0f));
    
    /* Filtrage des erreurs (portée max 400cm) */
    if (distance > 400.0f || distance < 2.0f) return 0.0f;
    
    return distance;
}

/* --------------------------------------------------------------------------
 * DHT11 — Capteur Température/Humidité sur PE3
 * -------------------------------------------------------------------------- */
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Activation du Pull-up interne
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

volatile uint8_t dhtError = 0;

void ReadDHT11(float *temp, float *hum) {
    uint8_t data[5] = {0};
    uint32_t timeout;
    dhtError = 0;
    
    /* 1. Signal de démarrage */
    Set_Pin_Output(GPIOE, GPIO_PIN_3);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_Delay(25); 
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    DWT_Delay_us(30);
    Set_Pin_Input(GPIOE, GPIO_PIN_3);
    
    /* 2. Attente réponse du DHT11 */
    timeout = 10000;
    while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET) {
        if (--timeout == 0) { dhtError = 1; return; }
    }
    
    /* Attente de la fin du LOW (80us) */
    timeout = 10000;
    while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET) {
        if (--timeout == 0) { dhtError = 2; return; }
    }
    
    /* Attente de la fin du HIGH (80us) */
    timeout = 10000;
    while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET) {
        if (--timeout == 0) { dhtError = 2; return; }
    }
    
    /* 3. Lecture des 40 bits */
    for (int i=0; i<40; i++) {
        timeout = 10000;
        while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_RESET) {
            if (--timeout == 0) { dhtError = 3; return; }
        }
        
        DWT_Delay_us(40); // Seuil entre bit 0 et bit 1
        
        if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET) {
            data[i/8] |= (1 << (7 - (i%8)));
            timeout = 10000;
            while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3) == GPIO_PIN_SET) {
                if (--timeout == 0) break;
            }
        }
    }
    
    /* 4. Vérification Checksum */
    if (data[4] == (uint8_t)(data[0] + data[1] + data[2] + data[3])) {
        if (data[0] != 0 || data[2] != 0) { // On ignore si tout est à zéro (cas d'erreur rare)
            *hum = (float)data[0] + (float)data[1]/10.0f;
            *temp = (float)data[2] + (float)data[3]/10.0f;
        }
    } else {
        dhtError = 4;
    }
}

/* --------------------------------------------------------------------------
 * GPS_ParseNMEA — Parse une trame NMEA $GPRMC ou $GNRMC
 * Met à jour sensorData.gps_lat et gps_lon
 * -------------------------------------------------------------------------- */


/* --------------------------------------------------------------------------
 * ReadDHT11_On_PB10 — Lecture Niveau Registre
 * -------------------------------------------------------------------------- */
void ReadDHT11_On_PB10(float *temp, float *hum) {
    uint8_t data[5] = {0};
    uint32_t timeout;
    dhtError = 0;
    
    /* 1. Mode Sortie : PB10 */
    GPIOB->MODER &= ~(3UL << (10 * 2)); 
    GPIOB->MODER |=  (1UL << (10 * 2)); 
    
    /* 2. Signal LOW (20ms) */
    GPIOB->BSRR = (1UL << (10 + 16)); 
    HAL_Delay(20);
    
    /* 3. Signal HIGH court et passage immédiat en INPUT */
    GPIOB->BSRR = (1UL << 10);
    DWT_Delay_us(2);
    GPIOB->MODER &= ~(3UL << (10 * 2)); // Mode Entrée
    
    /* 4. Attente réponse (LOW puis HIGH) */
    timeout = 20000;
    while((GPIOB->IDR & (1UL << 10))) if (--timeout == 0) { dhtError = 1; return; }
    
    timeout = 20000;
    while(!(GPIOB->IDR & (1UL << 10))) if (--timeout == 0) { dhtError = 2; return; }
    
    timeout = 20000;
    while((GPIOB->IDR & (1UL << 10))) if (--timeout == 0) { dhtError = 2; return; }
    
    /* 5. Lecture des 40 bits */
    for (int i=0; i<40; i++) {
        timeout = 20000;
        while(!(GPIOB->IDR & (1UL << 10))) if (--timeout == 0) { dhtError = 3; return; }
        
        DWT_Delay_us(40);
        
        if(GPIOB->IDR & (1UL << 10)) {
            data[i/8] |= (1 << (7 - (i%8)));
            timeout = 20000;
            while((GPIOB->IDR & (1UL << 10))) if (--timeout == 0) break;
        }
    }
    
    if (data[4] == (uint8_t)(data[0] + data[1] + data[2] + data[3])) {
        if (data[0] != 0 || data[2] != 0) {
            *hum = (float)data[0] + (float)data[1]/10.0f;
            *temp = (float)data[2] + (float)data[3]/10.0f;
        }
    } else {
        dhtError = 4;
    }
}

void UpdateModbusRegisters(void) {
    modbusRegisters[0] = (uint16_t)(sensorData.gas_ppm     * 10.0f);
    modbusRegisters[1] = (uint16_t)(sensorData.temperature * 10.0f);
    modbusRegisters[2] = (uint16_t)(sensorData.speed_kmh   * 10.0f);
    modbusRegisters[3] = (uint16_t)(fabsf(sensorData.gps_lat));
    modbusRegisters[4] = (uint16_t)((fabsf(sensorData.gps_lat)
                          - (float)modbusRegisters[3]) * 10000.0f);
    modbusRegisters[5] = (uint16_t)(fabsf(sensorData.gps_lon));
    modbusRegisters[6] = (uint16_t)((fabsf(sensorData.gps_lon)
                          - (float)modbusRegisters[5]) * 10000.0f);
    modbusRegisters[7] = (uint16_t)(sensorData.distance_cm * 10.0f);
    modbusRegisters[8] = (uint16_t)(sensorData.humidity    * 10.0f);
    
    uint16_t diag = 0;
    if (sensorData.temperature == 0.0f) diag |= 0x01;
    if (sensorData.gps_lat == 0.0f)     diag |= 0x04;
    modbusRegisters[9] = diag;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 1. Configurer le MPU AVANT HAL_Init() pour eviter la corruption
   *    du DMA Ethernet par le D-Cache Cortex-M7 (bug silencieux STM32H7) */
  MPU_Config();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
    /* Priorité MAXIMALE pour le Systick pour que le watchdog de secours (it)
     * puisse redémarrer la carte même si l'Ethernet bloque tout. */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    /* Initialise DWT pour mesures de temps précises (ultrason) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55; /* Déverrouillage DWT sur H7 */
    DWT->CYCCNT = 0;
    DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;

    /* Démarre l'encodeur de vitesse */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* NOTE : MX_LWIP_Init() et eMBTCPInit() sont appelés DANS la tâche
     *        FreeRTOS (StartDefaultTask) car LwIP nécessite le scheduler. */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Clignotement rapide pour indiquer un crash systeme */
  while (1)
  {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
      HAL_Delay(50);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
