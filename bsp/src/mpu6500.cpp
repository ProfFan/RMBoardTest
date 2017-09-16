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

#include <bsp/param.h>
#include "arm_math.h"

#include "bsp/mpu6500.h"
#include "bsp/param.h"

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

uint8_t MPU6500::setRegBit(uint8_t const addr, uint8_t bit, uint8_t state) {

  if (bit > 7)
    return 1;

  if (state > 1)
    return 1;

  uint8_t data;
  data = readReg(addr);
  data |= state << bit;

  writeReg(addr, data);

  return 0;
}

uint8_t MPU6500::getRegBit(uint8_t const addr, uint8_t bit) {

  if (bit > 7)
    return 2;

  uint8_t data;
  data = readReg(addr);

  return (data >> bit) & 1;
}

static inline uint8_t getbits(uint8_t value, uint8_t offset, uint8_t n) {

  if (offset >= 8)
    return 0; /* value is padded with infinite zeros on the left */

  value >>= offset; /* drop offset bits */

  if (n >= 8)
    return value; /* all  bits requested */

  const unsigned mask = (1u << n) - 1; /* n '1's */

  return value & mask;
}

uint8_t MPU6500::getRegBits(uint8_t const addr, uint8_t offset, uint8_t size) {

  if (offset > 7)
    return 2;

  uint8_t data;
  data = readReg(addr);

  return getbits(data, offset, size);
}

uint8_t MPU6500::setRegBits(uint8_t const addr, uint8_t offset, uint8_t size, uint8_t value) {

  if (size > 7)
    return 1;

  if (offset > 7)
    return 1;

  uint8_t data, temp = 0;
  data = readReg(addr);

  uint8_t mask = ((1 << size) - 1) << (offset - size + 1);
  data <<= (offset - size + 1); // shift data into correct position
  data &= mask; // zero all non-important bits in data
  temp &= ~(mask); // zero all important bits in existing byte
  temp |= data; // combine data with existing byte
  writeReg(addr, temp);

  return 0;
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

/** Get the I2C address of the specified slave (0-3).
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * In read mode, the result of the read is placed in the lowest available
 * EXT_SENS_DATA register. For further information regarding the allocation of
 * read results, please refer to the EXT_SENS_DATA register description
 * (Registers 73 - 96).
 *
 * The MPU-6500 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions (getSlave4* and setSlave4*).
 *
 * I2C data transactions are performed at the Sample Rate, as defined in
 * Register 25. The user is responsible for ensuring that I2C data transactions
 * to and from each enabled Slave can be completed within a single period of the
 * Sample Rate.
 *
 * The I2C slave access rate can be reduced relative to the Sample Rate. This
 * reduced access rate is determined by I2C_MST_DLY (Register 52). Whether a
 * slave's access rate is reduced relative to the Sample Rate is determined by
 * I2C_MST_DELAY_CTRL (Register 103).
 *
 * The processing order for the slaves is fixed. The sequence followed for
 * processing the slaves is Slave 0, Slave 1, Slave 2, Slave 3 and Slave 4. If a
 * particular Slave is disabled it will be skipped.
 *
 * Each slave can either be accessed at the sample rate or at a reduced sample
 * rate. In a case where some slaves are accessed at the Sample Rate and some
 * slaves are accessed at the reduced rate, the sequence of accessing the slaves
 * (Slave 0 to Slave 4) is still followed. However, the reduced rate slaves will
 * be skipped if their access rate dictates that they should not be accessed
 * during that particular cycle. For further information regarding the reduced
 * access rate, please refer to Register 52. Whether a slave is accessed at the
 * Sample Rate or at the reduced rate is determined by the Delay Enable bits in
 * Register 103.
 *
 * @param num Slave number (0-3)
 * @return Current address for specified slave
 * @see MPU6500_RA_I2C_SLV0_ADDR
 */
uint8_t MPU6500::GetSlaveAddress(uint8_t num) {
  if (num > 3)
    return 0;

  return readReg(MPU6500_RA_I2C_SLV0_ADDR + num * 3);
}

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU6500_RA_I2C_SLV0_ADDR
 */
uint8_t MPU6500::SetSlaveAddress(uint8_t num, uint8_t address) {
  if (num > 3)
    return 1;

  writeReg(MPU6500_RA_I2C_SLV0_ADDR + num * 3, address);

  return 0;
}

/** Get the active internal register for the specified slave (0-3).
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * The MPU-6500 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions.
 *
 * @param num Slave number (0-3)
 * @return Current active register for specified slave
 * @see MPU6500_RA_I2C_SLV0_REG
 */
uint8_t MPU6500::GetSlaveRegister(uint8_t num) {
  if (num > 3)
    return 0;

  return readReg(MPU6500_RA_I2C_SLV0_REG + num * 3);
}

/** Set the active internal register for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param reg New active register for specified slave
 * @see getSlaveRegister()
 * @see MPU6500_RA_I2C_SLV0_REG
 */
uint8_t MPU6500::SetSlaveRegister(uint8_t num, uint8_t reg) {
  if (num > 3)
    return 0;

  return writeReg(MPU6500_RA_I2C_SLV0_REG + num * 3, reg);
}

/** Get the enabled value for the specified slave (0-3).
 * When set to 1, this bit enables Slave 0 for data transfer operations. When
 * cleared to 0, this bit disables Slave 0 from data transfer operations.
 * @param num Slave number (0-3)
 * @return Current enabled value for specified slave
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
bool MPU6500::GetSlaveEnabled(uint8_t num) {
  if (num > 3)
    return 0;

  return getRegBit(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_EN_BIT);
}

/** Set the enabled value for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New enabled value for specified slave
 * @see getSlaveEnabled()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
uint8_t MPU6500::SetSlaveEnabled(uint8_t num, bool enabled) {
  if (num > 3)
    return 0;
  return setRegBit(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_EN_BIT,
                   enabled);
}

/** Get word pair byte-swapping enabled for the specified slave (0-3).
 * When set to 1, this bit enables byte swapping. When byte swapping is enabled,
 * the high and low bytes of a word pair are swapped. Please refer to
 * I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
 * bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
 * registers in the order they were transferred.
 *
 * @param num Slave number (0-3)
 * @return Current word pair byte-swapping enabled value for specified slave
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
bool MPU6500::GetSlaveWordByteSwap(uint8_t num) {
  if (num > 3)
    return 0;
  return getRegBit(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_BYTE_SW_BIT);
}

/** Set word pair byte-swapping enabled for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair byte-swapping enabled value for specified slave
 * @see getSlaveWordByteSwap()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
uint8_t MPU6500::SetSlaveWordByteSwap(uint8_t num, bool enabled) {
  if (num > 3)
    return 0;
  return setRegBit(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_BYTE_SW_BIT, enabled);
}

/** Get write mode for the specified slave (0-3).
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @param num Slave number (0-3)
 * @return Current write mode for specified slave (0 = register address + data, 1 = data only)
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
bool MPU6500::GetSlaveWriteMode(uint8_t num) {
  if (num > 3)
    return 0;
  return getRegBit(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_REG_DIS_BIT);
}

/** Set write mode for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param mode New write mode for specified slave (0 = register address + data, 1 = data only)
 * @see getSlaveWriteMode()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
uint8_t MPU6500::SetSlaveWriteMode(uint8_t num, bool mode) {
  if (num > 3)
    return 0;
  return setRegBit(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_REG_DIS_BIT,
                   mode);
}

/** Get word pair grouping order offset for the specified slave (0-3).
 * This sets specifies the grouping order of word pairs received from registers.
 * When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
 * then odd register addresses) are paired to form a word. When set to 1, bytes
 * from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
 * register addresses) are paired to form a word.
 *
 * @param num Slave number (0-3)
 * @return Current word pair grouping order offset for specified slave
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
bool MPU6500::GetSlaveWordGroupOffset(uint8_t num) {
  if (num > 3)
    return 0;
  return getRegBit(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_GRP_BIT);
}

/** Set word pair grouping order offset for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair grouping order offset for specified slave
 * @see getSlaveWordGroupOffset()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
uint8_t MPU6500::SetSlaveWordGroupOffset(uint8_t num, bool enabled) {
  if (num > 3)
    return 0;
  return setRegBit(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_GRP_BIT,
                   enabled);
}

/** Get number of bytes to read for the specified slave (0-3).
 * Specifies the number of bytes transferred to and from Slave 0. Clearing this
 * bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN.
 * @param num Slave number (0-3)
 * @return Number of bytes to read for specified slave
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
uint8_t MPU6500::GetSlaveDataLength(uint8_t num) {
  if (num > 3)
    return 0;
  return getRegBits(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_LEN_BIT,
                    MPU6500_I2C_SLV_LEN_LENGTH);
}

/** Set number of bytes to read for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see getSlaveDataLength()
 * @see MPU6500_RA_I2C_SLV0_CTRL
 */
uint8_t MPU6500::SetSlaveDataLength(uint8_t num, uint8_t length) {
  if (num > 3)
    return 1;
  return setRegBits(MPU6500_RA_I2C_SLV0_CTRL + num * 3, MPU6500_I2C_SLV_LEN_BIT,
                    MPU6500_I2C_SLV_LEN_LENGTH, length);
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

# define CTRL3_SAMPLEAVG_16    0x24  /* Sample Averaging 16 */
# define CTRL3_SAMPLEAVG_8    0x1b  /* Sample Averaging 8 */
# define CTRL3_SAMPLEAVG_4    0x12  /* Sample Averaging 4 */
# define CTRL3_SAMPLEAVG_2    0x09  /* Sample Averaging 2 */
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

  float mx = mpu_data.mx - MagXOffset;
  float my = mpu_data.my - MagYOffset;
  float mz = mpu_data.mz - MagZOffset;

  data.mx = MagEllipsoidAInv[0][0] * mx + MagEllipsoidAInv[0][1] * my + MagEllipsoidAInv[0][2] * mz;
  data.my = MagEllipsoidAInv[1][0] * mx + MagEllipsoidAInv[1][1] * my + MagEllipsoidAInv[1][2] * mz;
  data.mz = MagEllipsoidAInv[2][0] * mx + MagEllipsoidAInv[2][1] * my + MagEllipsoidAInv[2][2] * mz;

}

int MPU6500::initialize() {
  params->LoadPresets();
  params->ReadParams();

  MagXOffset = params->raw.MagXOffset;
  MagYOffset = params->raw.MagYOffset;
  MagZOffset = params->raw.MagZOffset;

  memcpy(MagEllipsoidAInv, params->raw.MagEllipsoidAInv, sizeof(MagEllipsoidAInv));

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

  while ((error != 0) && (tries < 10)) {
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
