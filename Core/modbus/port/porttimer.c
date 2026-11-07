/**
  ******************************************************************************
  * @file    porttimer.c
  * @brief   Timer freemodbus pour STM32H753ZI
  *          Utilise HAL_GetTick() — pas besoin de timer matériel supplémentaire
  ******************************************************************************
  */

#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "main.h"

/* Durée du timer Modbus en millisecondes */
static uint32_t usTimerTimeoutMS = 0;
static uint32_t usTimerStartTick = 0;
static BOOL     xTimerRunning    = FALSE;

/* --------------------------------------------------------------------------
 * xMBPortTimersInit
 * Initialise le timer avec un timeout en unités de 50µs
 * freemodbus appelle cette fonction au démarrage
 * -------------------------------------------------------------------------- */
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
    /* Convertit les unités 50µs en millisecondes */
    usTimerTimeoutMS = ((uint32_t)usTim1Timerout50us * 50) / 1000;
    if (usTimerTimeoutMS == 0) usTimerTimeoutMS = 1;
    xTimerRunning = FALSE;
    return TRUE;
}

/* --------------------------------------------------------------------------
 * xMBPortTimersEnable
 * Démarre le timer — appelé par freemodbus pour démarrer le timeout
 * -------------------------------------------------------------------------- */
void vMBPortTimersEnable(void)
{
    usTimerStartTick = HAL_GetTick();
    xTimerRunning    = TRUE;
}

/* --------------------------------------------------------------------------
 * vMBPortTimersDisable
 * Arrête le timer
 * -------------------------------------------------------------------------- */
void vMBPortTimersDisable(void)
{
    xTimerRunning = FALSE;
}

/* --------------------------------------------------------------------------
 * vMBPortTimerPoll
 * À appeler régulièrement dans la boucle principale ou tâche FreeRTOS
 * Vérifie si le timeout est expiré et notifie freemodbus
 * -------------------------------------------------------------------------- */
void vMBPortTimerPoll(void)
{
    if (xTimerRunning)
    {
        if ((HAL_GetTick() - usTimerStartTick) >= usTimerTimeoutMS)
        {
            xTimerRunning = FALSE;
            pxMBPortCBTimerExpired();
        }
    }
}
