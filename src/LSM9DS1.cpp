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

#include "LSM9DS1.h"

#define LSM9DS1_ADDRESS 0x6b

#define LSM9DS1_WHO_AM_I 0x0f
#define LSM9DS1_CTRL_REG1_G 0x10
#define LSM9DS1_STATUS_REG 0x17
#define LSM9DS1_OUT_X_G 0x18
#define LSM9DS1_CTRL_REG6_XL 0x20
#define LSM9DS1_CTRL_REG8 0x22
#define LSM9DS1_OUT_X_XL 0x28

// magnetometer
#define LSM9DS1_ADDRESS_M 0x1e

#define LSM9DS1_CTRL_REG1_M 0x20
#define LSM9DS1_CTRL_REG2_M 0x21
#define LSM9DS1_CTRL_REG3_M 0x22
#define LSM9DS1_STATUS_REG_M 0x27
#define LSM9DS1_OUT_X_L_M 0x28

LSM9DS1Class::LSM9DS1Class(TwoWire &wire)
    : continuousMode(false), _wire(&wire) {}

LSM9DS1Class::~LSM9DS1Class() {}

int LSM9DS1Class::begin() {
  _wire->begin();

  // reset
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG8, 0x05);
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x0c);

  delay(10);

  if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_WHO_AM_I) != 0x68) {
    end();

    return 0;
  }

  if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_WHO_AM_I) != 0x3d) {
    end();

    return 0;
  }

  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G,
                0x78); // 119 Hz, 2000 dps, 16 Hz BW
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x70); // 119 Hz, 4g

  writeRegister(
      LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M,
      0xb4); // Temperature compensation enable, medium performance, 20 Hz
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 gauss
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M,
                0x00); // Continuous conversion mode

  return 1;
}

void LSM9DS1Class::setContinuousMode() {
  // Enable FIFO (see docs
  // https://www.st.com/resource/en/datasheet/DM00103319.pdf)
  writeRegister(LSM9DS1_ADDRESS, 0x23, 0x02);
  // Set continuous mode
  writeRegister(LSM9DS1_ADDRESS, 0x2E, 0xC0);

  continuousMode = true;
}

void LSM9DS1Class::setOneShotMode() {
  // Disable FIFO (see docs
  // https://www.st.com/resource/en/datasheet/DM00103319.pdf)
  writeRegister(LSM9DS1_ADDRESS, 0x23, 0x00);
  // Disable continuous mode
  writeRegister(LSM9DS1_ADDRESS, 0x2E, 0x00);

  continuousMode = false;
}

void LSM9DS1Class::end() {
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x03);
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG1_G, 0x00);
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0x00);

  _wire->end();
}

int LSM9DS1Class::readAcceleration(float &x, float &y, float &z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_XL, (uint8_t *)data,
                     sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  switch (accelerometer_range_) {
  case LSM9DS1Class::AccelerometerRange::RANGE_2G:
    // 4g
    x = data[0] * 2.0 / 32768.0;
    y = data[1] * 2.0 / 32768.0;
    z = data[2] * 2.0 / 32768.0;
    break;
  case LSM9DS1Class::AccelerometerRange::RANGE_16G:
    x = data[0] * 16.0 / 32768.0;
    y = data[1] * 16.0 / 32768.0;
    z = data[2] * 16.0 / 32768.0;
    break;
  case LSM9DS1Class::AccelerometerRange::RANGE_4G:
    x = data[0] * 4.0 / 32768.0;
    y = data[1] * 4.0 / 32768.0;
    z = data[2] * 4.0 / 32768.0;
    break;
  case LSM9DS1Class::AccelerometerRange::RANGE_8G:
    x = data[0] * 8.0 / 32768.0;
    y = data[1] * 8.0 / 32768.0;
    z = data[2] * 8.0 / 32768.0;
    break;
  }
  return 1;
}

int LSM9DS1Class::accelerationAvailable() {
  if (continuousMode) {
    // Read FIFO_SRC. If any of the rightmost 8 bits have a value, there is
    // data.
    if (readRegister(LSM9DS1_ADDRESS, 0x2F) & 63) {
      return 1;
    }
  } else {
    if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x01) {
      return 1;
    }
  }

  return 0;
}

float LSM9DS1Class::accelerationSampleRate() { return 119.0F; }

int LSM9DS1Class::readGyroscope(float &x, float &y, float &z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS, LSM9DS1_OUT_X_G, (uint8_t *)data,
                     sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

int LSM9DS1Class::gyroscopeAvailable() {
  if (readRegister(LSM9DS1_ADDRESS, LSM9DS1_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

float LSM9DS1Class::gyroscopeSampleRate() { return 119.0F; }

int LSM9DS1Class::readMagneticField(float &x, float &y, float &z) {
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t *)data,
                     sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 * 100.0 / 32768.0;
  y = data[1] * 4.0 * 100.0 / 32768.0;
  z = data[2] * 4.0 * 100.0 / 32768.0;

  return 1;
}

int LSM9DS1Class::magneticFieldAvailable() {
  if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_STATUS_REG_M) & 0x08) {
    return 1;
  }

  return 0;
}

float LSM9DS1Class::magneticFieldSampleRate() { return 20.0; }

int LSM9DS1Class::readRegister(uint8_t slaveAddress, uint8_t address) {
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  if (_wire->endTransmission() != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, 1) != 1) {
    return -1;
  }

  return _wire->read();
}

int LSM9DS1Class::readRegisters(uint8_t slaveAddress, uint8_t address,
                                uint8_t *data, size_t length) {
  _wire->beginTransmission(slaveAddress);
  _wire->write(0x80 | address);
  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }

  return 1;
}

int LSM9DS1Class::writeRegister(uint8_t slaveAddress, uint8_t address,
                                uint8_t value) {
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}

bool LSM9DS1Class::setAccelerometerRange(
    LSM9DS1Class::AccelerometerRange range) {

  // Set the accelerometer range to 16G
  // Linear acceleration sensor Control Register - CTRL_REG6_XL (20h)
  // ODR_XL
  //[2:0]

  // FS_XL
  //[1:0]

  // BW_SCAL_
  // ODR

  // BW_XL
  //[1:0]

  // ODR_XL = 119 Hz
  // FS_XL = 16g
  // BW_SCAL_ODR = 0
  // BW_XL = 0

  // 0b01101000
  uint8_t reg6 = readRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL);
  if (reg6 < 0) {
    return false;
  }
  // clear and set the range bits
  reg6 &= ~(0b11 << 3);
  reg6 |= (uint8_t)range << 3;
  // clear and set the ODR bits
  //reg6 &= ~(0b111 << 5);
  // set the ODR bits to 952 Hz
  //reg6 |= (0b011 << 5);

  bool ret = writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, reg6);
  if (!ret) {
    return false;
  }
  accelerometer_range_ = range;
  return true;

  reg6 &= 0b11001111;
  reg6 |= (uint8_t)range << 4;
  return writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG6_XL, reg6);
}
int LSM9DS1Class::accelerationRange() {
  switch (accelerometer_range_) {
        case LSM9DS1Class::AccelerometerRange::RANGE_2G:
        return 2;
        case LSM9DS1Class::AccelerometerRange::RANGE_16G:
        return 16;
        case LSM9DS1Class::AccelerometerRange::RANGE_4G:
        return 4;
        case LSM9DS1Class::AccelerometerRange::RANGE_8G:
        return 8;
  }
}
LSM9DS1Class::AccelerometerRange LSM9DS1Class::accelerometerRangeEnum() {
  return accelerometer_range_;
}


#ifdef ARDUINO_ARDUINO_NANO33BLE
LSM9DS1Class IMU_LSM9DS1(Wire1);
#else
LSM9DS1Class IMU_LSM9DS1(Wire);
#endif
