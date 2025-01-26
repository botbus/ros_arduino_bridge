#include <string>
#include <iomanip>
#include <sstream>
#include "include/imu_handler.h"

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
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());

        if (myICM.status != ICM_20948_Stat_Ok)
        {
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }

    Serial.println(F("Device connected!"));
    myICM.swReset();

    if (myICM.status != ICM_20948_Stat_Ok)
    {
        Serial.print(F("Software Reset FAILED: "));
        Serial.println(myICM.statusString());
    }
    delay(250);

    myICM.sleep(false);
    myICM.lowPower(false);

    bool success = true;
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);

    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok);

    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    if (!success)
    {

        Serial.println(F("Enable DMP failed!"));
        Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        while (1)
            ;
    }
}

std::string jsonFormat(const std::string &name, long value1, long value2)
{
    std::stringstream oss;
    oss << "\"" << name << "\":[" << value1 << "," << value2 << "],";
    return oss.str();
}

std::string jsonFormat(const std::string &name, double value1, double value2, double value3)
{
    std::stringstream oss;
    oss << "\"" << name << "\":[" << value1 << "," << value2 << "," << value3 << "],";
    return oss.str();
}
std::string jsonFormat(const std::string &name, double value1, double value2, double value3, double value4)
{
    std::stringstream oss;
    oss << "\"" << name << "\":[" << value1 << "," << value2 << "," << value3 << "," << value4 << "],";
    return oss.str();
}

std::string readIMU()
{
    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);

    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        std::string jsonString{"{"};
        // if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
        // {
        //     double x = (double)data.Raw_Accel.Data.X / 32768.0f * 4 * 9.80665; // Extract the raw accelerometer data
        //     double y = (double)data.Raw_Accel.Data.Y / 32768.0f * 4 * 9.80665;
        //     double z = (double)data.Raw_Accel.Data.Z / 32768.0f * 4 * 9.80665;

        //     jsonString += jsonFormat("ACC", x, y, z);
        // }

        if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
        {
            // double x = (double)data.Raw_Gyro.Data.X / 32768.0f * 2000 * M_PI / 180; // Extract the raw gyro data
            // double y = (double)data.Raw_Gyro.Data.Y / 32768.0f * 2000 * M_PI / 180;
            double x{};
            double y{};
            double z = (double)data.Raw_Gyro.Data.Z / 32768.0f * 2000 * M_PI / 180;
            jsonString += jsonFormat("GYR", x, y, z);
        }

        if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {

            double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
            double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
            double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            jsonString += jsonFormat("QUAT", q0, q1, q2, q3);
        }
        jsonString += jsonFormat("ENC", readEncoder(LEFT), readEncoder(RIGHT));
        jsonString.pop_back();
        jsonString += "}|";
        return jsonString;
    }
    return "";
}
