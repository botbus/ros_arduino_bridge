#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H
#include <iostream>
#include <iomanip> // For std::fixed and std::setprecision
#include <string>
#include <initializer_list>
#include <SPI.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define SPI_PORT SPI

const int CS_PIN{17};
const int SCK_PIN{18};
const int MOSI_PIN{19};
const int MISO_PIN{16};

ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object

#endif