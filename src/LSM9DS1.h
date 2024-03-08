/*
  This file is part of the Arduino_LSM9DS1 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <Arduino.h>
#include <Wire.h>

class LSM9DS1Class {

public:
  enum class AccelerometerRange : uint8_t {
    RANGE_2G = 0b00,
    RANGE_16G = 0b01,
    RANGE_4G = 0b10,
    RANGE_8G = 0b11
  };
  LSM9DS1Class(TwoWire &wire);
  virtual ~LSM9DS1Class();

  int begin();
  void end();

  // Controls whether a FIFO is continuously filled, or a single reading is
  // stored. Defaults to one-shot.
  void setContinuousMode();
  void setOneShotMode();

  // Accelerometer
  virtual int readAcceleration(float &x, float &y,
                               float &z); // Results are in g (earth gravity).
  virtual int accelerationAvailable();    // Number of samples in the FIFO.
  virtual float accelerationSampleRate(); // Sampling rate of the sensor.
  virtual int accelerationRange();
  virtual AccelerometerRange accelerometerRangeEnum();

  // Gyroscope
  virtual int readGyroscope(float &x, float &y,
                            float &z); // Results are in degrees/second.
  virtual int gyroscopeAvailable();    // Number of samples in the FIFO.
  virtual float gyroscopeSampleRate(); // Sampling rate of the sensor.

  // Magnetometer
  virtual int readMagneticField(float &x, float &y,
                                float &z); // Results are in uT (micro Tesla).
  virtual int magneticFieldAvailable();    // Number of samples in the FIFO.
  virtual float magneticFieldSampleRate(); // Sampling rate of the sensor.

  int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);

  bool setAccelerometerRange(AccelerometerRange range);

private:
  bool continuousMode;
  int readRegister(uint8_t slaveAddress, uint8_t address);
  int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t *data,
                    size_t length);

private:
  TwoWire *_wire;
  AccelerometerRange accelerometer_range_ = AccelerometerRange::RANGE_4G;
};

extern LSM9DS1Class IMU_LSM9DS1;
#undef IMU
#define IMU IMU_LSM9DS1