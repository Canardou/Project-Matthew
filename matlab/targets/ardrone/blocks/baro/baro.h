#ifndef _BARO_H
#define _BARO_H

#ifndef MATLAB_MEX_FILE

#include <paparazzi/Navdata.h>

#endif

void Barometer_Initialization();

int32_t Barometer_Get_Pressure ();

int32_t Barometer_Get_Temperature ();

#endif