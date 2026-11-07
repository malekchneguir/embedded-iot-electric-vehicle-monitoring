#include "main.h"
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart2;
int last_gps_idx = 0;

char gpsBuffer[256];
float gpsLat = 0.0f;
float gpsLon = 0.0f;

// Conversion degrés minutes -> degrés décimaux
float ConvertNMEA(float raw) {
  int degrees = (int)(raw / 100);
  float minutes = raw - (degrees * 100);
  return degrees + (minutes / 60.0f);
}

void GPS_Parse(char* sentence) {
  char* start = strchr(sentence, '$');
  if (start == NULL) return;

  char copy[256];
  strncpy(copy, start, 255);
  copy[255] = '\0';
  char* token;
  int field = 0;

  /* Support RMC */
  if(strncmp(start, "$GPRMC", 6) == 0 || strncmp(start, "$GNRMC", 6) == 0) {
    token = strtok(copy, ",");
    while(token != NULL) {
      if(field == 2 && token[0] == 'V') return; 
      if(field == 3 && strlen(token) > 0) gpsLat = ConvertNMEA(atof(token));
      if(field == 4 && token[0] == 'S') gpsLat = -gpsLat; 
      if(field == 5 && strlen(token) > 0) gpsLon = ConvertNMEA(atof(token));
      if(field == 6 && token[0] == 'W') gpsLon = -gpsLon; 
      field++;
      token = strtok(NULL, ",");
    }
  }
  /* Support GGA */
  else if(strncmp(start, "$GPGGA", 6) == 0 || strncmp(start, "$GNGGA", 6) == 0) {
    token = strtok(copy, ",");
    while(token != NULL) {
      if(field == 6 && token[0] == '0') return; // Quality 0 = No Fix
      if(field == 2 && strlen(token) > 0) gpsLat = ConvertNMEA(atof(token));
      if(field == 3 && token[0] == 'S') gpsLat = -gpsLat;
      if(field == 4 && strlen(token) > 0) gpsLon = ConvertNMEA(atof(token));
      if(field == 5 && token[0] == 'W') gpsLon = -gpsLon;
      field++;
      token = strtok(NULL, ",");
    }
  }
}

void GPS_Update(void) {
  uint8_t rxChar;
  int idx = 0;
  uint32_t startTick = HAL_GetTick();
  
  /* Reset UART en cas d'erreur (Overrun) */
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(&huart2);
  }
  
  memset(gpsBuffer, 0, sizeof(gpsBuffer));
  
  /* On cherche le début d'une trame '$' pendant max 150ms */
  while ((HAL_GetTick() - startTick) < 150) {
    if (HAL_UART_Receive(&huart2, &rxChar, 1, 10) == HAL_OK) {
      /* Diagnostic : on allume la LED rouge si on reçoit des données */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
      
      if (rxChar == '$') {
        gpsBuffer[idx++] = '$';
        /* On lit la suite de la ligne jusqu'à '\n' ou buffer plein */
        while (idx < 254) {
          if (HAL_UART_Receive(&huart2, &rxChar, 1, 50) == HAL_OK) {
            gpsBuffer[idx++] = rxChar;
            if (rxChar == '\n') break;
          } else break;
        }
        last_gps_idx = idx;
        GPS_Parse(gpsBuffer);
        return;
      }
    }
  }
  /* Si on arrive ici sans avoir trouvé de '$', on éteint la LED rouge */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
