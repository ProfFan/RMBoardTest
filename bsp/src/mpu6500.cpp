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

#define MPUDelay osDelay

uint8_t txbuff[20] = {0xff};

MPU6500::MPU6500(SPI_HandleTypeDef *spi) {
  gyroSamples = 0;
  accelSamples = 0;

  port = spi;
}

uint8_t MPU6500::writeReg(uint8_t const addr, uint8_t value) {
  uint8_t txdata[2] = {addr & 0x7F, value};
  uint8_t rxdata[2];
  MPU_NSS_LOW;
  HAL_SPI_TransmitReceive(port, txdata, rxdata, 2, 55);
  MPU_NSS_HIGH;
  return rxdata[1];
}

uint8_t MPU6500::readReg(uint8_t const addr) {
  uint8_t rxdata;
  readReg(addr, &rxdata, 1);
  return rxdata;
}

uint8_t MPU6500::readReg(uint8_t const addr, uint8_t *buffer, uint16_t length) {
  uint8_t txdata = addr | 0x80;
  uint8_t rxdata;
  MPU_NSS_LOW;
  HAL_SPI_TransmitReceive(port, &txdata, &rxdata, 1, 55);
  txbuff[0] = txdata;
  rxdata = HAL_SPI_TransmitReceive(port, txbuff, buffer, length, 55);

  MPU_NSS_HIGH;
  return rxdata;
}

void MPU6500::setAccelRange(AccelRange range) {

  // Accel scale factor = 9.81 m/s^2 / scale
  switch (range) {
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

  switch (range) {
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

void MPU6500::writeIST8310Reg(uint8_t addr, uint8_t data) {
  writeReg(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPUDelay(100);
  writeReg(MPU6500_I2C_SLV1_REG, addr);
  MPUDelay(100);
  writeReg(MPU6500_I2C_SLV1_DO, data);
  MPUDelay(100);
  // Enable SLV1 transmit
  writeReg(MPU6500_I2C_SLV1_CTRL, 0x81);
  MPUDelay(100);
}

uint8_t MPU6500::readIST8310Reg(uint8_t addr) {
  uint8_t value;
  MPUDelay(100);
  writeReg(MPU6500_I2C_SLV0_REG, addr);
  MPUDelay(100);
  writeReg(MPU6500_I2C_SLV0_CTRL, 0x81);
  MPUDelay(100);
  value = readReg(MPU6500_EXT_SENS_DATA_00);
  //turn off slave4 after read
  MPUDelay(100);
  return value;
}

void MPU6500::configureI2CAutoRead(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num) {
  //configure the device address of the IST8310
  //use slave1,auto transmit single measure mode.
  writeReg(MPU6500_I2C_SLV1_ADDR, device_address);
  MPUDelay(2);
  writeReg(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
  MPUDelay(2);
  writeReg(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
  MPUDelay(2);

  //use slave0,auto read data
  writeReg(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
  MPUDelay(2);
  writeReg(MPU6500_I2C_SLV0_REG, reg_base_addr);
  MPUDelay(2);

  //every 10 mpu6500 internal samples one i2c master read
  writeReg(MPU6500_I2C_SLV4_CTRL, 0x09);
  MPUDelay(2);

  //enable slave 0 and 1 access delay
  writeReg(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
  MPUDelay(2);
  //enable slave 1 auto transmit
  writeReg(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
  MPUDelay(6); //Wait 6ms (minimum waiting time for 16 times internal average setup)
  //enable slave 0 with data_num bytes reading
  writeReg(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
  MPUDelay(2);
}

int MPU6500::initIST8310() {

  HAL_GPIO_WritePin(IST_RST_GPIO_Port, IST_RST_Pin, GPIO_PIN_RESET);

  writeReg(MPU6500_USER_CTRL, 0x30); //enable iic passthrough mode
  MPUDelay(10);
  writeReg(MPU6500_I2C_MST_CTRL, 0x0d); //enable iic 400khz
  MPUDelay(10);

  HAL_GPIO_WritePin(IST_RST_GPIO_Port, IST_RST_Pin, GPIO_PIN_SET);

  //turn on slave 1 for ist write and slave 4 to ist read
  writeReg(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS); //enable iic 400khz
  MPUDelay(10);
  writeReg(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS); //enable iic 400khz
  MPUDelay(10);
  writeReg(MPU6500_I2C_SLV0_ADDR, 0x80 | IST8310_ADDRESS);
  MPUDelay(40);
  // IST8310_R_CONFB 0x01	= device rst
  // writeIST8310Reg(IST8310_R_CONFB, 0x01); //soft rst
  // MPUDelay(10);
  if (IST8310_DEVICE_ID_A != readIST8310Reg(IST8310_WHO_AM_I))
    return 1; //wrong
    //return readReg(MPU6500_I2C_SLV0_CTRL);
  writeIST8310Reg(IST8310_R_CONFB, 0x01); //rst
  MPUDelay(10);

  writeIST8310Reg(IST8310_R_CONFA, 0x00); //config as ready mode to access reg
  if (readIST8310Reg(IST8310_R_CONFA) != 0x00)
    return 2;
  MPUDelay(10);

  writeIST8310Reg(IST8310_R_CONFB, 0x00); //normal state, no int
  if (readIST8310Reg(IST8310_R_CONFB) != 0x00)
    return 3;
  MPUDelay(10);

# define CTRL3_SAMPLEAVG_16		0x24	/* Sample Averaging 16 */
# define CTRL3_SAMPLEAVG_8		0x1b	/* Sample Averaging 8 */
# define CTRL3_SAMPLEAVG_4		0x12	/* Sample Averaging 4 */
# define CTRL3_SAMPLEAVG_2		0x09	/* Sample Averaging 2 */
  //config  low noise mode, x,y,z axis 16 time 1 avg,
  writeIST8310Reg(IST8310_AVGCNTL, CTRL3_SAMPLEAVG_2); //100100
  if (readIST8310Reg(IST8310_AVGCNTL) != CTRL3_SAMPLEAVG_2)
    return 4;
  MPUDelay(10);

  //Set/Reset pulse duration setup,normal mode
  writeIST8310Reg(IST8310_PDCNTL, 0xc0);
  if (readIST8310Reg(IST8310_PDCNTL) != 0xc0)
    return 5;
  MPUDelay(10);

  //turn off slave1 & slave 4
  writeReg(MPU6500_I2C_SLV1_CTRL, 0x00);
  MPUDelay(10);
  writeReg(MPU6500_I2C_SLV4_CTRL, 0x00);
  MPUDelay(10);

  //configure and turn on slave 0
  configureI2CAutoRead(IST8310_ADDRESS, IST8310_R_XL, 0x06);
  MPUDelay(100);
  return 0;
}

void MPU6500::readRawData() {

  readReg(MPU6500_ACCEL_XOUT_H, buffer, 14);

  mpu_data.ax = buffer[0] << 8 | buffer[1];
  mpu_data.ay = buffer[2] << 8 | buffer[3];
  mpu_data.az = buffer[4] << 8 | buffer[5];

  mpu_data.temp = buffer[6] << 8 | buffer[7];

  mpu_data.gx = buffer[8] << 8 | buffer[9];
  mpu_data.gy = buffer[10] << 8 | buffer[11];
  mpu_data.gz = buffer[12] << 8 | buffer[13];

  data.temp = 21 + mpu_data.temp / 333.87f;
  data.wx = mpu_data.gx / gyroScaleFactor * 0.0174533;
  data.wy = mpu_data.gy / gyroScaleFactor * 0.0174533;
  data.wz = mpu_data.gz / gyroScaleFactor * 0.0174533;

  data.ax = mpu_data.ax * 9.81 / accelScaleFactor;
  data.ay = mpu_data.ay * 9.81 / accelScaleFactor;
  data.az = mpu_data.az * 9.81 / accelScaleFactor;

  readReg(MPU6500_EXT_SENS_DATA_00, (uint8_t *) &(mpu_data.mx), 6);

  data.mx = mpu_data.mx * 0.003;
  data.my = mpu_data.my * 0.003;
  data.mz = mpu_data.mz * 0.003;

}

int MPU6500::initialize() {

//  if(readReg(MPU6500_WHO_AM_I) != MPU6500_ID){
//    return -1;
//  }

  uint8_t MPU6500_Init_Data[10][2] = {
      {MPU6500_PWR_MGMT_1,     0x80}, // Reset Device
      {MPU6500_PWR_MGMT_1,     0x01}, // Clock Source - Gyro-Z
      {MPU6500_PWR_MGMT_2,     0x00}, // Enable Acc & Gyro
      {MPU6500_CONFIG,         0x04}, // LPF 43Hz
      {MPU6500_SMPLRT_DIV,     0x00}, // 1000Hz sampling rate
      {MPU6500_GYRO_CONFIG,    0x18}, // +-2000dps
      {MPU6500_ACCEL_CONFIG,   0x10}, // +-8G
      {MPU6500_ACCEL_CONFIG_2, 0x02}, // enable ACC LPF
      {MPU6500_USER_CTRL,      0x20}, // Enable AUX I2C
  };

  for (int i = 0; i < 10; i++) {
    writeReg(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
    osDelay(1);
  }

  setGyroRange(DPS_2000);
  osDelay(1);
  setAccelRange(G_8);

  int error = 100;
  int tries = 0;

  while((error != 0) && (tries < 10)) {
    error = initIST8310();
    osDelay(100);
    tries++;
  }

  // Enable auxiliary I2C bus bypass
  // *NOT* Necessary for all setups, but some boards have magnetometer attached to the auxiliary I2C bus
  // and without this settings magnetometer won't be accessible.
  return error;
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
