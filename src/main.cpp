//
// Created by Markus on 6.2.2024.
//
#include "ArduinoBLE.h"
#include "Arduino_LSM9DS1.h"
#include <Arduino.h>
// sd card
#include <SD.h>
#include <SPI.h>

unsigned long time_millis = 0;
unsigned long start_time = 0;

unsigned long loggin_time = 300000; // 2 minutes

File file;
LSM9DS1Class IMUU(Wire1);

void advertise() {
  // set power
  BLE.begin();
  BLE.setLocalName("Rocket");
  BLE.setAdvertisingInterval(320); // 200 * 0.625 ms
  BLE.advertise();
  SerialUSB.println("Advertising started");
}

void setup() {
  // put your setup code here, to run once:

  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;
  SerialUSB.print("Booting");
  for (int i = 0; i < 20; i++) {
    SerialUSB.print(".");
    delay(100);
  }

  // initialize the digital Pin as an output
  pinMode(LEDR, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LED_PWR, OUTPUT);

  digitalWrite(LED_PWR, HIGH); // turn the LED on (HIGH is the voltage level)
  digitalWrite(LEDR, HIGH);    // turn the LED on (HIGH is the voltage level)
  digitalWrite(LEDB, HIGH);    // turn the LED on (HIGH is the voltage level)
  digitalWrite(LEDG, HIGH);    // turn the LED on (HIGH is the voltage level)
  if (!SD.begin(10)) {
    SerialUSB.println("Card Mount Failed");
    while (1) {
      digitalWrite(LEDR, LOW); // turn the LED on (HIGH is the voltage level)
      delay(100);
    }
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
  } else {
    SerialUSB.println("Logging to file /log" + String(i) + ".txt");
    SerialUSB.println("Logging time is " + String(loggin_time / 1000) + " s");
  }

  if (!IMUU.begin()) {
    SerialUSB.println("Failed to initialize IMUU!");
    while (1) {
      digitalWrite(LEDR, LOW); // turn the LED on (HIGH is the voltage level)
      delay(100);
    }
  }
  SerialUSB.println("IMUU initialized!");
  IMUU.setAccelerometerRange(LSM9DS1Class::AccelerometerRange::RANGE_16G);
  advertise();
  SerialUSB.println("Starting logging");
  start_time = millis();
}
bool logging = true; // logging is off

void logData() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  IMUU.readAcceleration(ax, ay, az);
  IMUU.readGyroscope(gx, gy, gz);
  IMUU.readMagneticField(mx, my, mz);
  char data[200];
  sprintf(data, "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f", time_millis, ax, ay, az, gx,
          gy, gz, mx, my, mz);
  file.println(data);
}

int count = 1000;

void loop() {
  // put your main code here, to run repeatedly:
  time_millis = millis();

  if (logging) {
    if (time_millis - start_time > loggin_time) {
      // if(count-- == 0){
      logging = false;
      file.close();
      SerialUSB.println("Logging stopped");
      SerialUSB.println("Took " + String(time_millis - start_time) + " ms");
    }
    logData();
  } else {
    delay(1);
  }
}