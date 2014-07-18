#ifndef _TMP112_SENSOR_H
#define _TMP112_SENSOR_H

#include "hal_types.h"


#endif

bool TMP112_open(void);
void readTemperature(int8 *pTemperature, uint16 *pTemperatureRaw);