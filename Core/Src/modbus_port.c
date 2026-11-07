#include "mb.h"
#include "mbport.h"
#include "main.h"

// Registres Modbus (10 registres de 16 bits)
// Registre 0 : gaz (ppm * 10)
// Registre 1 : température (°C * 10)
// Registre 2 : vitesse (km/h * 10)
// Registre 3 : GPS latitude entier (degrés * 1000)
// Registre 4 : GPS latitude décimal
// Registre 5 : GPS longitude entier
// Registre 6 : GPS longitude décimal
// Registre 7 : distance ultrason (cm * 10)
// Registre 8-9 : réservés

extern uint16_t modbusRegisters[10];

// Appelé par la lib Modbus quand le maître lit les registres
eMBErrorCode eMBRegHoldingCB(UCHAR* pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
  lastModbusTick = HAL_GetTick();

  eMBErrorCode eStatus = MB_ENOERR;
  int iRegIndex;
  
  /* usAddress est 1-based dans FreeModbus (ex: 40001 -> address 1) */
  if ((usAddress >= 1) && (usAddress + usNRegs <= 10 + 1))
  {
    iRegIndex = (int)(usAddress - 1);
    switch (eMode)
    {
      case MB_REG_READ:
        while (usNRegs > 0)
        {
          *pucRegBuffer++ = (unsigned char)(modbusRegisters[iRegIndex] >> 8);
          *pucRegBuffer++ = (unsigned char)(modbusRegisters[iRegIndex] & 0xFF);
          iRegIndex++;
          usNRegs--;
        }
        break;
      case MB_REG_WRITE:
        while (usNRegs > 0)
        {
          modbusRegisters[iRegIndex] = (unsigned short)(*pucRegBuffer++ << 8);
          modbusRegisters[iRegIndex] |= (unsigned short)(*pucRegBuffer++);
          iRegIndex++;
          usNRegs--;
        }
        break;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}
// Stub for other callbacks to avoid linker errors
eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs ) { return MB_ENOREG; }
eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode ) { return MB_ENOREG; }
eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs ) { return MB_ENOREG; }
