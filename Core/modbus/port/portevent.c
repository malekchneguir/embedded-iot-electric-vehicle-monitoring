/**
  ******************************************************************************
  * @file    portevent.c
  * @brief   Gestion des événements freemodbus pour STM32H753ZI + FreeRTOS
  *          Utilise une queue FreeRTOS pour transmettre les événements Modbus
  ******************************************************************************
  */

#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "cmsis_os.h"

/* Queue FreeRTOS pour les événements Modbus */
static osMessageQueueId_t xQueueModbusEvent = NULL;

/* --------------------------------------------------------------------------
 * xMBPortEventInit
 * Initialise la queue d'événements — appelé au démarrage de freemodbus
 * -------------------------------------------------------------------------- */
BOOL xMBPortEventInit(void)
{
    /* Crée une queue FreeRTOS de 8 événements de type eMBEventType */
    xQueueModbusEvent = osMessageQueueNew(8,
                                          sizeof(eMBEventType),
                                          NULL);
    return (xQueueModbusEvent != NULL) ? TRUE : FALSE;
}

/* --------------------------------------------------------------------------
 * xMBPortEventPost
 * Envoie un événement dans la queue — appelé depuis les interruptions ou
 * callbacks freemodbus
 * -------------------------------------------------------------------------- */
BOOL xMBPortEventPost(eMBEventType eEvent)
{
    osStatus_t status;

    if (xQueueModbusEvent == NULL) return FALSE;

    status = osMessageQueuePut(xQueueModbusEvent,
                               &eEvent,
                               0,      /* priorité message */
                               0);     /* timeout = immédiat */

    return (status == osOK) ? TRUE : FALSE;
}

/* --------------------------------------------------------------------------
 * xMBPortEventGet
 * Lit le prochain événement depuis la queue — appelé par eMBPoll()
 * Bloque jusqu'à 10 ms en attendant un événement
 * -------------------------------------------------------------------------- */
BOOL xMBPortEventGet(eMBEventType *eEvent)
{
    osStatus_t status;

    if (xQueueModbusEvent == NULL) return FALSE;

    status = osMessageQueueGet(xQueueModbusEvent,
                               eEvent,
                               NULL,   /* priorité message (non utilisé) */
                               10);    /* timeout 10 ms */

    return (status == osOK) ? TRUE : FALSE;
}
