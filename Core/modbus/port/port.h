/**
  ******************************************************************************
  * @file    port.h
  * @brief   Adaptation freemodbus pour STM32H753ZI + FreeRTOS + LwIP
  ******************************************************************************
  */

#ifndef PORT_H
#define PORT_H

#include "cmsis_os.h"
#include <stdint.h>
#include <assert.h>

/* ---- Types de base requis par freemodbus --------------------------------- */
#define INLINE                      inline
#define PR_BEGIN_EXTERN_C           extern "C" {
#define PR_END_EXTERN_C             }

typedef uint8_t                     BOOL;
typedef unsigned char               UCHAR;
typedef char                        CHAR;
typedef uint16_t                    USHORT;
typedef int16_t                     SHORT;
typedef uint32_t                    ULONG;
typedef int32_t                     LONG;

#ifndef TRUE
#define TRUE                        1
#endif
#ifndef FALSE
#define FALSE                       0
#endif

/* ---- Section critique (protection contre les interruptions) -------------- */
/* Utilise les primitives FreeRTOS pour entrer/sortir de section critique     */
#define ENTER_CRITICAL_SECTION()    taskENTER_CRITICAL()
#define EXIT_CRITICAL_SECTION()     taskEXIT_CRITICAL()

/* ---- Assert (désactivé en production) ------------------------------------ */
#define MB_PORT_HAS_CLOSE           0
#define ASSERT( x )                 do { if(!(x)) { while(1){} } } while(0)

#endif /* PORT_H */
