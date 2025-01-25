#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H
#include <iostream>
#include <iomanip>
#include <string>
#include <initializer_list>
#include <SPI.h>
#include "ICM_20948.h" 

#define SPI_PORT SPI
const int CS_PIN{17};
const int SCK_PIN{18};
const int MOSI_PIN{19};
const int MISO_PIN{16};

ICM_20948_SPI myICM;

#endif