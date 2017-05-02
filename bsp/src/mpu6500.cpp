//
// Created by Fan Jiang on 2017/5/2.
//

/*  10 DOF stick featuring MPU6050, HMC5883L, MS5611
    According to MPU6050 datasheet, this chip should support I2C speeds up to 400kHz (Fast-mode Fm)

    However on Teensy 3.0 i am able to reach 2.4MHz (High-speed mode) without any problems.
    (which cuts down the reading time of accel + gyro to about 180us)

    Please note that external pullup resistors are required on Teensy 3.0,
    while they are not "required" on other platforms, i highly recommend adding them.
    1000 ohm pullup seems to work best in my case.
*/

extern "C" {
#include "cmsis_os.h"
#include "spi.h"
#include "gpio.h"
}

#include "bsp/mpu6500.h"

#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

uint8_t txbuff[20] = {0xff};

MPU6500::MPU6500(SPI_HandleTypeDef *spi) {
  gyroSamples = 0;
  accelSamples = 0;

  port = spi;
}

uint8_t MPU6500::writeReg(uint8_t const addr, uint8_t value){
  uint8_t txdata[2] = {addr & 0x7F, value};
  uint8_t rxdata[2];
  MPU_NSS_LOW;
  HAL_SPI_TransmitReceive(port, txdata, rxdata, 2, 55);
  MPU_NSS_HIGH;
  return rxdata[2];
}

uint8_t MPU6500::readReg(uint8_t const addr){
  uint8_t txdata[2] = {addr | 0x80, addr | 0x80};
  uint8_t rxdata[2];
  MPU_NSS_LOW;
  HAL_SPI_TransmitReceive(port, txdata, rxdata, 2, 55);
  MPU_NSS_HIGH;
  return rxdata[2];
}

uint8_t MPU6500::readReg(uint8_t const addr, uint8_t* buffer, uint16_t length){
  uint8_t txdata = addr | 0x80;
  uint8_t rxdata;
  MPU_NSS_LOW;
  HAL_SPI_TransmitReceive(port, &txdata, &rxdata, 1, 55);
  txbuff[0] = txdata;
  rxdata = HAL_SPI_TransmitReceive(port, txbuff, buffer, length, 55);

  MPU_NSS_HIGH;
  return rxdata;
}

void MPU6500::readGyroTemperature() {

}

void MPU6500::setAccelRange(AccelRange range) {

  // Accel scale factor = 9.81 m/s^2 / scale
  switch(range){
    case G_2:
      accelScaleFactor = 16384;
      break;
    case G_4:
      accelScaleFactor = 8192;
      break;
    case G_8:
      accelScaleFactor = 4096;
      break;
    case G_16:
      accelScaleFactor = 2048;
      break;

    default:
      return;
  }

  writeReg(MPU6500_ACCEL_CONFIG, range << 3);
}

void MPU6500::setGyroRange(GyroRange range) {

  switch(range){
    case DPS_250:
      gyroScaleFactor = 131.0;
      break;
    case DPS_500:
      gyroScaleFactor = 65.50;
      break;
    case DPS_1000:
      gyroScaleFactor = 32.8;
      break;
    case DPS_2000:
      gyroScaleFactor = 16.4;
      break;
    default:
      return;
  }

  writeReg(MPU6500_GYRO_CONFIG, range << 3);
}

void MPU6500::readRawData(){

  readReg(MPU6500_ACCEL_XOUT_H, buffer, 14);

  mpu_data.ax   = buffer[0] << 8 | buffer[1];
  mpu_data.ay   = buffer[2] << 8 | buffer[3];
  mpu_data.az   = buffer[4] << 8 | buffer[5];

  mpu_data.temp = buffer[6] << 8 | buffer[7];

  mpu_data.gx = buffer[8] << 8 | buffer[9];
  mpu_data.gy = buffer[10] << 8 | buffer[11];
  mpu_data.gz = buffer[12] << 8 | buffer[13];

  data.temp = 21 + mpu_data.temp / 333.87f;
  data.wx   = mpu_data.gx / gyroScaleFactor * 0.0174533;
  data.wy   = mpu_data.gy / gyroScaleFactor * 0.0174533;
  data.wz   = mpu_data.gz / gyroScaleFactor * 0.0174533;

  data.ax   = mpu_data.ax * 9.81 / accelScaleFactor;
  data.ay   = mpu_data.ay * 9.81 / accelScaleFactor;
  data.az   = mpu_data.az * 9.81 / accelScaleFactor;

}

int MPU6500::initialize() {

//  if(readReg(MPU6500_WHO_AM_I) != MPU6500_ID){
//    return -1;
//  }

  uint8_t MPU6500_Init_Data[10][2] = {
      { MPU6500_PWR_MGMT_1, 0x80 }, // Reset Device
      { MPU6500_PWR_MGMT_1, 0x03 }, // Clock Source - Gyro-Z
      { MPU6500_PWR_MGMT_2, 0x00 }, // Enable Acc & Gyro
      { MPU6500_CONFIG, 0x04 }, // LPF 41Hz//gyro bandwidth 41Hz
      { MPU6500_GYRO_CONFIG, 0x18 }, // +-2000dps
      { MPU6500_ACCEL_CONFIG, 0x10 }, // +-8G
      { MPU6500_ACCEL_CONFIG_2, 0x02 }, // enable LowPassFilter  Set Acc LPF
      { MPU6500_USER_CTRL, 0x20 }, // Enable AUX
  };

  for(int i = 0; i < 10; i++){
    writeReg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
    osDelay(1);
  }

  setGyroRange(DPS_2000);
  osDelay(1);
  setAccelRange(G_8);

  // Enable auxiliary I2C bus bypass
  // *NOT* Necessary for all setups, but some boards have magnetometer attached to the auxiliary I2C bus
  // and without this settings magnetometer won't be accessible.
  return 0;
}

//void MPU6500::calibrate_gyro() {
//  static uint8_t retry = 0;
//  uint8_t i, count = 128;
//  int16_t xSum = 0, ySum = 0, zSum = 0;
//
//  for (i = 0; i < count; i++) {
//    readRawData();
//    xSum += gyroRaw[XAXIS];
//    ySum += gyroRaw[YAXIS];
//    zSum += gyroRaw[ZAXIS];
//    delay(10);
//  }
//
//  gyro_offset[XAXIS] = -xSum / count;
//  gyro_offset[YAXIS] = -ySum / count;
//  gyro_offset[ZAXIS] = -zSum / count;
//
//  // Calibration sanity check
//  // if suitable offset couldn't be established, break out of the loop after 10 retries
//  if ((abs(gyro_offset[XAXIS]) > 300 || abs(gyro_offset[YAXIS]) > 300 || abs(gyro_offset[ZAXIS]) > 300) && retry < 10) {
//    // gyro calibration failed, run again
//    retry++;
//
//    // small delay before next gyro calibration
//    delay(500);
//
//    calibrate_gyro();
//  }
//}
//
//void MPU6500::calibrate_accel() {
//  uint8_t i, count = 128;
//  int32_t xSum = 0, ySum = 0, zSum = 0;
//
//  for (i = 0; i < count; i++) {
//    readRawData();
//    xSum += accelRaw[XAXIS];
//    ySum += accelRaw[YAXIS];
//    zSum += accelRaw[ZAXIS];
//    delay(10);
//  }
//
//  CONFIG.data.ACCEL_BIAS[XAXIS] = xSum / count;
//  CONFIG.data.ACCEL_BIAS[YAXIS] = ySum / count;
//  CONFIG.data.ACCEL_BIAS[ZAXIS] = (zSum / count) - 8192; // - 1G;
//
//  // Reverse calibration forces
//  CONFIG.data.ACCEL_BIAS[XAXIS] *= -1;
//  CONFIG.data.ACCEL_BIAS[YAXIS] *= -1;
//  CONFIG.data.ACCEL_BIAS[ZAXIS] *= -1;
//}
//
//void MPU6500::evaluateGyro() {
//  // Calculate average
//  gyro[XAXIS] = gyroSum[XAXIS] / gyroSamples;
//  gyro[YAXIS] = gyroSum[YAXIS] / gyroSamples;
//  gyro[ZAXIS] = gyroSum[ZAXIS] / gyroSamples;
//
//  // Apply offsets
//  gyro[XAXIS] += gyro_offset[XAXIS];
//  gyro[YAXIS] += gyro_offset[YAXIS];
//  gyro[ZAXIS] += gyro_offset[ZAXIS];
//
//  // Apply correct scaling (at this point gyro is in radians)
//  gyro[XAXIS] *= gyroScaleFactor;
//  gyro[YAXIS] *= gyroScaleFactor;
//  gyro[ZAXIS] *= gyroScaleFactor;
//
//  // Reset SUM variables
//  gyroSum[XAXIS] = 0;
//  gyroSum[YAXIS] = 0;
//  gyroSum[ZAXIS] = 0;
//  gyroSamples = 0;
//}
//
//void MPU6500::evaluateAccel() {
//  // Calculate average
//  accel[XAXIS] = accelSum[XAXIS] / accelSamples;
//  accel[YAXIS] = accelSum[YAXIS] / accelSamples;
//  accel[ZAXIS] = accelSum[ZAXIS] / accelSamples;
//
//  // Apply offsets
//  accel[XAXIS] += CONFIG.data.ACCEL_BIAS[XAXIS];
//  accel[YAXIS] += CONFIG.data.ACCEL_BIAS[YAXIS];
//  accel[ZAXIS] += CONFIG.data.ACCEL_BIAS[ZAXIS];
//
//  // Apply correct scaling (at this point accel reprensents +- 1g = 9.81 m/s^2)
//  accel[XAXIS] *= accelScaleFactor;
//  accel[YAXIS] *= accelScaleFactor;
//  accel[ZAXIS] *= accelScaleFactor;
//
//  // Reset SUM variables
//  accelSum[XAXIS] = 0;
//  accelSum[YAXIS] = 0;
//  accelSum[ZAXIS] = 0;
//  accelSamples = 0;
//}
