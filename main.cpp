#include "mbed.h"
#include "platform/mbed_thread.h"
#include "bmx160.h"

Serial pc(USBTX, USBRX);

int main()
{
    pc.baud(9600);
    // Initialise I2C bus (PC_1 and PC_0 pins being used in this demo for Nucleo L476RG)
    I2C i2cBus(PC_1, PC_0);
    i2cBus.frequency(400000);
    BMI160_I2C imu(i2cBus, BMI160_I2C::I2C_ADRS_SDO_LO);

    //check if accel an gyro are ok
    uint32_t failures = 0;
    if(imu.setSensorPowerMode(BMI160::GYRO, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        pc.printf("Failed to set gyroscope power mode\n");
        failures++;
    }
    thread_sleep_for(100);
    if(imu.setSensorPowerMode(BMI160::ACC, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        pc.printf("Failed to set accelerometer power mode\n");
        failures++;
    }
    thread_sleep_for(100);
    if(imu.setSensorPowerMode(BMI160::MAG, BMI160::NORMAL) != BMI160::RTN_NO_ERROR) {
        pc.printf("Failed to set magnetometer power mode\n");
        failures++;
    }
    imu.setMagnConf(); //initialize magnetometer for regular preset.
    thread_sleep_for(100);

    BMI160::AccConfig accConfig;
    //example of using getSensorConfig
    if(imu.getSensorConfig(accConfig) == BMI160::RTN_NO_ERROR) {
        pc.printf("ACC Range = %d\n", accConfig.range);
        pc.printf("ACC UnderSampling = %d\n", accConfig.us);
        pc.printf("ACC BandWidthParam = %d\n", accConfig.bwp);
        pc.printf("ACC OutputDataRate = %d\n\n", accConfig.odr);
    } else {
        pc.printf("Failed to get accelerometer configuration\n");
        failures++;
    }

    //example of setting user defined configuration
    accConfig.range = BMI160::SENS_4G;
    accConfig.us = BMI160::ACC_US_OFF;
    accConfig.bwp = BMI160::ACC_BWP_2;
    accConfig.odr = BMI160::ACC_ODR_8;
    if(imu.setSensorConfig(accConfig) == BMI160::RTN_NO_ERROR) {
        pc.printf("ACC Range = %d\n", accConfig.range);
        pc.printf("ACC UnderSampling = %d\n", accConfig.us);
        pc.printf("ACC BandWidthParam = %d\n", accConfig.bwp);
        pc.printf("ACC OutputDataRate = %d\n\n", accConfig.odr);
    } else {
        pc.printf("Failed to set accelerometer configuration\n");
        failures++;
    }

    BMI160::GyroConfig gyroConfig;
    if(imu.getSensorConfig(gyroConfig) == BMI160::RTN_NO_ERROR) {
        pc.printf("GYRO Range = %d\n", gyroConfig.range);
        pc.printf("GYRO BandWidthParam = %d\n", gyroConfig.bwp);
        pc.printf("GYRO OutputDataRate = %d\n\n", gyroConfig.odr);
    } else {
        pc.printf("Failed to get gyroscope configuration\n");
        failures++;
    }
    thread_sleep_for(1000);
    if(failures == 0) {
        float imuTemperature;
        BMI160::SensorData accData;
        BMI160::SensorData gyroData;
        BMI160::SensorData magData;
        BMI160::SensorTime sensorTime;

        while(1) {
            imu.getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, accConfig.range, gyroConfig.range);

            pc.printf("ACC xAxis (g) = %4.3f\n", accData.xAxis.scaled);
            pc.printf("ACC yAxis (g) = %4.3f\n", accData.yAxis.scaled);
            pc.printf("ACC zAxis (g) = %4.3f\n\n", accData.zAxis.scaled);

            pc.printf("GYRO xAxis (dps) = %5.1f\n", gyroData.xAxis.scaled);
            pc.printf("GYRO yAxis (dps) = %5.1f\n", gyroData.yAxis.scaled);
            pc.printf("GYRO zAxis (dps) = %5.1f\n\n",gyroData.zAxis.scaled);

            imu.getMagSensorXYZ(magData);

            pc.printf("Mag xAxis (uT) = %5.1f\n", magData.xAxis.scaled);
            pc.printf("Mag yAxis (uT) = %5.1f\n", magData.yAxis.scaled);
            pc.printf("Mag zAxis (uT) = %5.1f\n\n",magData.zAxis.scaled);

            imu.getTemperature(&imuTemperature);

            pc.printf("Sensor Time = %f\n",sensorTime.seconds);
            pc.printf("Sensor Temperature = %5.3f\n",  imuTemperature);

            thread_sleep_for(500);


        }
    } else {
        while(1) {
            pc.printf("Error\n");
        }
    }


}
