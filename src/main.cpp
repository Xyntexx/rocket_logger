//
// Created by Markus on 6.2.2024.
//
#include <Arduino.h>
#include "Arduino_LSM9DS1.h"
#include "ArduinoBLE.h"
// sd card
#include <SD.h>
#include <SPI.h>

#define LSM9DS1_ADDRESS            0x6b

#define LSM9DS1_WHO_AM_I           0x0f
#define LSM9DS1_CTRL_REG1_G        0x10
#define LSM9DS1_STATUS_REG         0x17
#define LSM9DS1_OUT_X_G            0x18
#define LSM9DS1_CTRL_REG6_XL       0x20
#define LSM9DS1_CTRL_REG8          0x22
#define LSM9DS1_OUT_X_XL           0x28

// magnetometer
#define LSM9DS1_ADDRESS_M          0x1e

#define LSM9DS1_CTRL_REG1_M        0x20
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_STATUS_REG_M       0x27
#define LSM9DS1_OUT_X_L_M          0x28


unsigned long time_millis = 0;
unsigned long start_time = 0;

File file;

void setup() {
    // put your setup code here, to run once:

    SerialUSB.begin(115200);
    while (!SerialUSB);
    SerialUSB.print("Booting");
    for (int i = 0; i < 20; i++) {
        SerialUSB.print(".");
        delay(100);
    }

    if (!SD.begin(5)) {
        SerialUSB.println("Card Mount Failed");
        return;
    }

    // Open a file for writing
    int i = 0;
    while (SD.exists("/log" + String(i) + ".txt")) {
        i++;
    }
    file = SD.open("/log" + String(i) + ".txt", FILE_WRITE);
    if (!file) {
        SerialUSB.println("Failed to open file for writing");
        return;
    }


    if (!IMU.begin()) {
        SerialUSB.println("Failed to initialize IMU!");
        while (1);
    }
    // Set the accelerometer range to 16G
    //Linear acceleration sensor Control Register - CTRL_REG6_XL (20h)
    //ODR_XL
    //[2:0]

    //FS_XL
    //[1:0]

    //BW_SCAL_
    //ODR

    //BW_XL
    //[1:0]

    // ODR_XL = 119 Hz
    // FS_XL = 16g
    // BW_SCAL_ODR = 0
    // BW_XL = 0

    // 0b01101000
    IMU.writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0b01101000);
    start_time = millis();
}
bool logging = true; // logging is off
unsigned long loggin_time = 120000; // 2 minutes

void logData() {
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);
    file.print(time_millis);
    file.print(",");
    file.print(ax);
    file.print(",");
    file.print(ay);
    file.print(",");
    file.print(az);
    file.print(",");
    file.print(gx);
    file.print(",");
    file.print(gy);
    file.print(",");
    file.print(gz);
    file.print(",");
    file.print(mx);
    file.print(",");
    file.print(my);
    file.print(",");
    file.println(mz);
}

void advertise() {
    BLE.begin();
    BLE.setAdvertisingInterval(320); // 200 * 0.625 ms
    BLE.advertise();
    }

void loop() {
    // put your main code here, to run repeatedly:

    time_millis = millis();

    if (logging) {
        if(time_millis - start_time > loggin_time) {
            logging = false;
            file.close();
            SerialUSB.println("Logging stopped");
            advertise();
        }
        logData();
    }
    delay(10);

}