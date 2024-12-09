#include "include/imu_handler.h"
// char IMUBuffer[500];
void initIMU()
{
  SPI_PORT.setSCK(SCK_PIN);
  SPI_PORT.setMOSI(MOSI_PIN);
  SPI_PORT.setMISO(MISO_PIN);
  SPI_PORT.begin();

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(CS_PIN, SPI_PORT);
    Serial.print(F("Initialization status: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

// Below here are some helper functions to print the data nicely!

// void printPaddedInt16b(int16_t val) {
//   if (val > 0) {
//     Serial.print(" ");
//     if (val < 10000) {
//       Serial.print("0");
//     }
//     if (val < 1000) {
//       Serial.print("0");
//     }
//     if (val < 100) {
//       Serial.print("0");
//     }
//     if (val < 10) {
//       Serial.print("0");
//     }
//   } else {
//     Serial.print("-");
//     if (abs(val) < 10000) {
//       Serial.print("0");
//     }
//     if (abs(val) < 1000) {
//       Serial.print("0");
//     }
//     if (abs(val) < 100) {
//       Serial.print("0");
//     }
//     if (abs(val) < 10) {
//       Serial.print("0");
//     }
//   }
//   Serial.print(abs(val));
// }

// void printRawAGMT(ICM_20948_AGMT_t agmt) {
//   Serial.print("RAW. Acc [ ");
//   printPaddedInt16b(agmt.acc.axes.x);
//   Serial.print(", ");
//   printPaddedInt16b(agmt.acc.axes.y);
//   Serial.print(", ");
//   printPaddedInt16b(agmt.acc.axes.z);
//   Serial.print(" ], Gyr [ ");
//   printPaddedInt16b(agmt.gyr.axes.x);
//   Serial.print(", ");
//   printPaddedInt16b(agmt.gyr.axes.y);
//   Serial.print(", ");
//   printPaddedInt16b(agmt.gyr.axes.z);
//   Serial.print(" ], Mag [ ");
//   printPaddedInt16b(agmt.mag.axes.x);
//   Serial.print(", ");
//   printPaddedInt16b(agmt.mag.axes.y);
//   Serial.print(", ");
//   printPaddedInt16b(agmt.mag.axes.z);
//   Serial.print(" ], Tmp [ ");
//   printPaddedInt16b(agmt.tmp.val);
//   Serial.print(" ]");
//   Serial.println();
// }
// void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
// {
//   // float aval = abs(val);
//   // if (val < 0) {
//   //   Serial.print("-");
//   // } else {
//   //   Serial.print(" ");
//   // }
//   // for (uint8_t indi = 0; indi < leading; indi++) {
//   //   uint32_t tenpow = 0;
//   //   if (indi < (leading - 1)) {
//   //     tenpow = 1;
//   //   }
//   //   for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
//   //     tenpow *= 10;
//   //   }
//   //   if (aval < tenpow) {
//   //     Serial.print("0");
//   //   } else {
//   //     break;
//   //   }
//   // }
//   if (val < 0)
//   {
//     Serial.print(-val, decimals);
//   }
//   else
//   {
//     Serial.print(val, decimals);
//   }
// }

void printScaledAGMT(ICM_20948_SPI *sensor)
{

  Serial.print("{\"ACC\":[");
  Serial.print(sensor->accX(), 2);
  Serial.print(",");
  Serial.print(sensor->accY(), 2);
  Serial.print(",");
  Serial.print(sensor->accZ(), 2);
  Serial.print("],\"GYR\":[");
  Serial.print(sensor->gyrX(), 2);
  Serial.print(",");
  Serial.print(sensor->gyrY(), 2);
  Serial.print(",");
  Serial.print(sensor->gyrZ(), 2);
  Serial.print("],\"MAG\":[");
  Serial.print(sensor->magX(), 2);
  Serial.print(",");
  Serial.print(sensor->magY(), 2);
  Serial.print(",");
  Serial.print(sensor->magZ(), 2);
  Serial.print("],\"TMP\":[");
  Serial.print(sensor->temp(), 2);
  Serial.print("],");
  // Serial.println();
}

void readIMU()
{
  if (myICM.dataReady())
  {
    myICM.getAGMT();
    printScaledAGMT(&myICM);
  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }
}