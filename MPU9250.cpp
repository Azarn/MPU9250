#include "MPU9250.h"


MPU9250::MPU9250() : MPU9250(MPU9250_DEFAULT_ADDRESS, MPU9250_MAG_ADDRESS) {
}

MPU9250::MPU9250(const char* i2cAddress) : MPU9250(I2Cdev::DEFAULT_I2C_ADDRESS, MPU9250_DEFAULT_ADDRESS, MPU9250_MAG_ADDRESS) {
}

MPU9250::MPU9250(uint8_t mpuAddress): MPU9250(mpuAddress, MPU9250_MAG_ADDRESS) {
}

MPU9250::MPU9250(uint8_t mpuAddress, uint8_t magAddress) : MPU9250(I2Cdev::DEFAULT_I2C_ADDRESS, mpuAddress, magAddress) {
}

MPU9250::MPU9250(const char* i2cAddress, uint8_t mpuAddress, uint8_t magAddress) : _i2c(i2cAddress), _mpuAddress(mpuAddress),
                                                                                   _magAddress(magAddress) {
}

template<typename T, typename F>
void MPU9250::updateRegister(F func) {
    T reg;
    readRegister(reg);
    func(reg);
    writeRegister(reg);
}

// template<typename T, typename F>
// auto MPU9250::parseRegister(F func) -> decltype(auto) {
//     T reg;
//     readRegister(reg);
//     return func(reg);
// }

template<typename T>
T& MPU9250::readRegister(T& t) {
    static_assert(t.IOFlag & IO_FLAG::R);
    if (!_i2c.readByte(t.Type == REG_TYPE::GEN ? _mpuAddress : _magAddress, t.Address, &t.getData().Raw) > 0) {
        throw std::runtime_error("Error while reading from register!");
    }
    return t;
}

template<typename T>
void MPU9250::writeRegister(T& t) {
    static_assert(t.IOFlag & IO_FLAG::W);
    if (!_i2c.writeByte(t.Type == REG_TYPE::GEN ? _mpuAddress : _magAddress, t.Address, t.getData().Raw)) {
        throw std::runtime_error("Error while writing to register!");
    }
}

template<typename H, typename L>
uint16_t MPU9250::mergeTwoRegistersU() {
    H high;
    L low;
    readRegister(high);
    readRegister(low);
    uint16_t res = high.getData().Raw << 8 | low.getData().Raw;
    return res;
}

template<typename H, typename L>
int16_t MPU9250::mergeTwoRegistersS() {
    uint16_t res = mergeTwoRegistersU<H, L>();
    if (res <= INT16_MAX) {
        return static_cast<int16_t>(res);
    } else if (res >= INT16_MIN){
        return static_cast<int16_t>(res - INT16_MIN) + INT16_MIN;
    }

    throw std::runtime_error("No possible convertion from uint16_t to int16_t on this platform");
}

void MPU9250::initialize() {
    setClockSource(PWR_MGMT1_DATA::CLKSEL_AUTO_PLL);
    setFullScaleGyroRange(GYRO_CONFIG_DATA::FULL_SCALE_250DPS);
    setFullScaleAccelRange(ACCEL_CONFIG_DATA::FULL_SCALE_2G);
    setSleepEnabled(false);
    setI2CBypassEnabled(true);
    setMagnetometerMode(MAG_CNTL1_DATA::MODE_SINGLE_MEASURE);
}

uint8_t MPU9250::getRate() {
    SMPLRT_DIV reg;
    return readRegister(reg).getData().Raw;
}

void MPU9250::setRate(uint8_t rate) {
    updateRegister<SMPLRT_DIV>([=](auto& reg) {
        reg.setData(rate);
    });
}

// CONFIG register
uint8_t MPU9250::getExternalFrameSync() {
    CONFIG reg;
    return readRegister(reg).getData().Structed.EXT_SYNC_SET;
}

void MPU9250::setExternalFrameSync(uint8_t sync) {
    updateRegister<CONFIG>([=](auto& reg) {
        reg.getData().Structed.EXT_SYNC_SET = sync;
    });
}

uint8_t MPU9250::getDLPFMode() {
    CONFIG reg;
    return readRegister(reg).getData().Structed.DLPF_CFG;
}

void MPU9250::setDLPFMode(uint8_t mode) {
    updateRegister<CONFIG>([=](auto& reg) {
        reg.getData().Structed.DLPF_CFG = mode;
    });
}

uint8_t MPU9250::getFullScaleGyroRange() {
    GYRO_CONFIG reg;
    return readRegister(reg).getData().Structed.GYRO_FS_SEL;
}

void MPU9250::setFullScaleGyroRange(uint8_t range) {
    updateRegister<GYRO_CONFIG>([=](auto& reg){
        reg.getData().Structed.GYRO_FS_SEL = range;
    });
}

// ACCEL_CONFIG register

/** Get self-test enabled setting for accelerometer X axis.
 * @return Self-test enabled value
 * @see MPU9250_SELF_TEST_X_ACCEL
 */
bool MPU9250::getAccelXSelfTest(void) {
    uint8_t test_result = readRegister(MPU9250_SELF_TEST_X_ACCEL); //MPU9250_ACONFIG_XA_ST_BIT, buffer); //check if ACCEL CONFIG2 is relevant
    return (test_result != 0);
}

/** Get self-test enabled setting for accelerometer X axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_SELF_TEST_X_ACCEL
 */
bool MPU9250::setAccelXSelfTest(const uint8_t enabled) {
    return writeRegister(MPU9250_SELF_TEST_X_ACCEL, enabled); //, MPU9250_ACONFIG_XA_ST_BIT, enabled);//check if ACCEL CONFIG2 is relevant
}

/** Get self-test enabled value for accelerometer Y axis.
 * @return Self-test enabled value
 * @see MPU9250_SELF_TEST_Y_ACCEL
 */
bool MPU9250::getAccelYSelfTest(void) {
    uint8_t test_result = readRegister(MPU9250_SELF_TEST_Y_ACCEL); //, MPU9250_ACONFIG_YA_ST_BIT, buffer);//check if ACCEL CONFIG2 is relevant
    return (test_result != 0);
}

/** Get self-test enabled value for accelerometer Y axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_SELF_TEST_Y_ACCEL
 */
bool MPU9250::setAccelYSelfTest(const uint8_t enabled) {
    return writeRegister(MPU9250_SELF_TEST_Y_ACCEL, enabled); //, MPU9250_ACONFIG_YA_ST_BIT, enabled);//check if ACCEL CONFIG2 is relevant
}

/** Get self-test enabled value for accelerometer Z axis.
 * @return Self-test enabled value
 * @see MPU9250_SELF_TEST_Z_ACCEL
 */
bool MPU9250::getAccelZSelfTest(void) {
    uint8_t test_result = readRegister(MPU9250_SELF_TEST_Z_ACCEL); //, MPU9250_ACONFIG_ZA_ST_BIT, buffer);//check if ACCEL CONFIG2 is relevant
    return (test_result != 0);
}

/** Set self-test enabled value for accelerometer Z axis.
 * @param enabled Self-test enabled value
 * @see MPU9250_SELF_TEST_Z_ACCEL
 */
bool MPU9250::setAccelZSelfTest(const uint8_t enabled) {
    return writeRegister(MPU9250_SELF_TEST_Z_ACCEL, enabled); //MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_ZA_ST_BIT, enabled);//check if ACCEL CONFIG2 is relevant
}

uint8_t MPU9250::getFullScaleAccelRange() {
    ACCEL_CONFIG reg;
    return readRegister(reg).getData().Structed.ACCEL_FS_SEL;
}

void MPU9250::setFullScaleAccelRange(uint8_t range) {
    updateRegister<ACCEL_CONFIG>([=](auto& reg) {
        reg.getData().Structed.ACCEL_FS_SEL = range;
    });
}

bool MPU9250::getFIFOEnabled() {
    USER_CTRL reg;
    return readRegister(reg).getData().Structed.FIFO_EN;
}

void MPU9250::setFIFOEnabled(bool enabled) {
    updateRegister<USER_CTRL>([=](auto& reg) {
        reg.getData().Structed.FIFO_EN = enabled;
    });
}

// FIFO_EN register
bool MPU9250::getTempFIFOEnabled() {
    FIFO_EN reg;
    return readRegister(reg).getData().Structed.TEMP_FIFO_EN;
}

void MPU9250::setTempFIFOEnabled(bool enabled) {
    updateRegister<FIFO_EN>([=](auto& reg) {
        reg.getData().Structed.TEMP_FIFO_EN = enabled;
    });
}

bool MPU9250::getXGyroFIFOEnabled() {
    FIFO_EN reg;
    return readRegister(reg).getData().Structed.GYRO_XOUT;
}

void MPU9250::setXGyroFIFOEnabled(bool enabled) {
    updateRegister<FIFO_EN>([=](auto& reg) {
        reg.getData().Structed.GYRO_XOUT = enabled;
    });
}

bool MPU9250::getYGyroFIFOEnabled() {
    FIFO_EN reg;
    return readRegister(reg).getData().Structed.GYRO_YOUT;
}

void MPU9250::setYGyroFIFOEnabled(bool enabled) {
    updateRegister<FIFO_EN>([=](auto& reg) {
        reg.getData().Structed.GYRO_YOUT = enabled;
    });
}

bool MPU9250::getZGyroFIFOEnabled() {
    FIFO_EN reg;
    return readRegister(reg).getData().Structed.GYRO_ZOUT;
}

void MPU9250::setZGyroFIFOEnabled(bool enabled) {
    updateRegister<FIFO_EN>([=](auto& reg) {
        reg.getData().Structed.GYRO_ZOUT = enabled;
    });
}

bool MPU9250::getAccelFIFOEnabled() {
    FIFO_EN reg;
    return readRegister(reg).getData().Structed.ACCEL;
}

void MPU9250::setAccelFIFOEnabled(bool enabled) {
    updateRegister<FIFO_EN>([=](auto& reg) {
        reg.getData().Structed.ACCEL = enabled;
    });
}

bool MPU9250::getSlave2FIFOEnabled() {
    FIFO_EN reg;
    return readRegister(reg).getData().Structed.SLV2;
}

void MPU9250::setSlave2FIFOEnabled(bool enabled) {
    updateRegister<FIFO_EN>([=](auto& reg) {
        reg.getData().Structed.SLV2 = enabled;
    });
}

bool MPU9250::getSlave1FIFOEnabled() {
    FIFO_EN reg;
    return readRegister(reg).getData().Structed.SLV1;
}

void MPU9250::setSlave1FIFOEnabled(bool enabled) {
    updateRegister<FIFO_EN>([=](auto& reg) {
        reg.getData().Structed.SLV1 = enabled;
    });
}

bool MPU9250::getSlave0FIFOEnabled() {
    FIFO_EN reg;
    return readRegister(reg).getData().Structed.SLV0;
}

void MPU9250::setSlave0FIFOEnabled(bool enabled) {
    updateRegister<FIFO_EN>([=](auto& reg) {
        reg.getData().Structed.SLV0 = enabled;
    });
}

// I2C_MST_CTRL register

bool MPU9250::getMultiMasterEnabled() {
    I2C_MST_CTRL reg;
    return readRegister(reg).getData().Structed.MULT_MST_EN;
}

void MPU9250::setMultiMasterEnabled(bool enabled) {
    updateRegister<I2C_MST_CTRL>([=](auto& reg) {
        reg.getData().Structed.MULT_MST_EN = enabled;
    });
}

bool MPU9250::getWaitForExternalSensorEnabled() {
    I2C_MST_CTRL reg;
    return readRegister(reg).getData().Structed.WAIT_FOR_ES;
}

void MPU9250::setWaitForExternalSensorEnabled(bool enabled) {
    updateRegister<I2C_MST_CTRL>([=](auto& reg) {
        reg.getData().Structed.WAIT_FOR_ES = enabled;
    });
}

/** Get Slave 3 FIFO enabled value.
 * When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96)
 * associated with Slave 3 to be written into the FIFO buffer.
 * @return Current Slave 3 FIFO enabled value
 * @see MPU9250_RA_MST_CTRL
 */
bool MPU9250::getSlave3FIFOEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_CTRL, MPU9250_SLV_3_FIFO_EN_MASK);
    return (response != 0);
}

/** Set Slave 3 FIFO enabled value.
 * @param enabled New Slave 3 FIFO enabled value
 * @see getSlave3FIFOEnabled()
 * @see MPU9250_RA_MST_CTRL
 */
bool MPU9250::setSlave3FIFOEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_I2C_MST_CTRL, MPU9250_SLV_3_FIFO_EN_MASK, enabled);
}

/** Get slave read/write transition enabled value.
 * The I2C_MST_P_NSR bit configures the I2C Master's transition from one slave
 * read to the next slave read. If the bit equals 0, there will be a restart
 * between reads. If the bit equals 1, there will be a stop followed by a start
 * of the following read. When a write transaction follows a read transaction,
 * the stop followed by a start of the successive write will be always used.
 *
 * @return Current slave read/write transition enabled value
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250::getSlaveReadWriteTransitionEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_MASK);
    return (response != 0);
}

/** Set slave read/write transition enabled value.
 * @param enabled New slave read/write transition enabled value
 * @see getSlaveReadWriteTransitionEnabled()
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250::setSlaveReadWriteTransitionEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_P_NSR_MASK, enabled);
}

/** Get I2C master clock speed.
 * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
 * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
 * the following table:
 *
 * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 *
 * @return Current I2C master clock speed
 * @see MPU9250_RA_I2C_MST_CTRL
 */
uint8_t MPU9250::getMasterClockSpeed(void) {
    return readMaskedRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_MASK);
}

/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU9250_RA_I2C_MST_CTRL
 */
bool MPU9250::setMasterClockSpeed(const uint8_t speed) {
    if(speed > 15) {
        return 0;
    }
    return writeMaskedRegister(MPU9250_I2C_MST_CTRL, MPU9250_I2C_MST_CLK_MASK, speed);
}

// I2C_SLV* registers (Slave 0-3)

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
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
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
 * @see MPU9250_RA_I2C_SLV0_ADDR
 */
uint8_t MPU9250::getSlaveAddress(uint8_t num) {
    if(num > 3) {
        return 0;
    }
    return readRegister(MPU9250_I2C_SLV0_ADDR + num*3);
}

/** Set the I2C address of the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param address New address for specified slave
 * @see getSlaveAddress()
 * @see MPU9250_RA_I2C_SLV0_ADDR
 */
bool MPU9250::setSlaveAddress(uint8_t num, uint8_t address) {
    if(num > 3) {
        return 0;
    }
    return writeRegister(MPU9250_I2C_SLV0_ADDR + num*3, address);
}

/** Get the active internal register for the specified slave (0-3).
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions.
 *
 * @param num Slave number (0-3)
 * @return Current active register for specified slave
 * @see MPU9250_RA_I2C_SLV0_REG
 */
uint8_t MPU9250::getSlaveRegister(uint8_t num) {
    if(num > 3) {
        return 0;
    }
    return readRegister(MPU9250_I2C_SLV0_REG + num*3);
}

/** Set the active internal register for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param reg New active register for specified slave
 * @see getSlaveRegister()
 * @see MPU9250_RA_I2C_SLV0_REG
 */
bool MPU9250::setSlaveRegister(uint8_t num, uint8_t reg) {
    if(num > 3) {
        return 0;
    }
    return writeRegister(MPU9250_I2C_SLV0_REG + num*3, reg);
}

/** Get the enabled value for the specified slave (0-3).
 * When set to 1, this bit enables Slave 0 for data transfer operations. When
 * cleared to 0, this bit disables Slave 0 from data transfer operations.
 * @param num Slave number (0-3)
 * @return Current enabled value for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::getSlaveEnabled(uint8_t num) {
    if(num > 3) {
        return 0;
    }
    uint8_t response = readMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_EN_MASK);
    return (response != 0);
}

/** Set the enabled value for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New enabled value for specified slave
 * @see getSlaveEnabled()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::setSlaveEnabled(uint8_t num, const bool enabled) {
    if(num > 3) {
        return 0;
    }
    return writeMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_EN_MASK, enabled);
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
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::getSlaveWordByteSwap(uint8_t num) {
    if(num > 3) {
        return 0;
    }
    uint8_t response = readMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_BYTE_SW_MASK);
    return (response != 0);
}

/** Set word pair byte-swapping enabled for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair byte-swapping enabled value for specified slave
 * @see getSlaveWordByteSwap()
 * @see MPU9250_I2C_SLV0_CTRL
 */
bool MPU9250::setSlaveWordByteSwap(uint8_t num, const bool enabled) {
    if(num > 3) {
        return 0;
    }
    return writeMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_BYTE_SW_MASK, enabled);
}

/** Get write mode for the specified slave (0-3).
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @param num Slave number (0-3)
 * @return Current write mode for specified slave (0 = register address + data, 1 = data only)
 * @see MPU9250_I2C_SLV0_CTRL
 */
bool MPU9250::getSlaveWriteMode(uint8_t num) {
    if(num > 3) {
        return 0;
    }
    uint8_t response = readMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_REG_DIS_MASK);
    return (response != 0);
}

/** Set write mode for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param mode New write mode for specified slave (0 = register address + data, 1 = data only)
 * @see getSlaveWriteMode()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::setSlaveWriteMode(uint8_t num, const bool mode) {
    if(num > 3) {
        return 0;
    }
    return writeMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_REG_DIS_MASK, mode);
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
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::getSlaveWordGroupOffset(uint8_t num) {
    if(num > 3) {
        return 0;
    }
    uint8_t response = readMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_GRP_MASK);
    return (response != 0);
}

/** Set word pair grouping order offset for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param enabled New word pair grouping order offset for specified slave
 * @see getSlaveWordGroupOffset()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::setSlaveWordGroupOffset(uint8_t num, const bool enabled) {
    if(num > 3) {
        return 0;
    }
    return writeMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_GRP_MASK, enabled);
}

/** Get number of bytes to read for the specified slave (0-3).
 * Specifies the number of bytes transferred to and from Slave 0. Clearing this
 * bit to 0 is equivalent to disabling the register by writing 0 to I2C_SLV0_EN.
 * @param num Slave number (0-3)
 * @return Number of bytes to read for specified slave
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
uint8_t MPU9250::getSlaveDataLength(uint8_t num) {
    if(num > 3) {
        return 0;
    }
    return readMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_LENG_MASK);
}

/** Set number of bytes to read for the specified slave (0-3).
 * @param num Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see getSlaveDataLength()
 * @see MPU9250_RA_I2C_SLV0_CTRL
 */
bool MPU9250::setSlaveDataLength(uint8_t num, const uint8_t length) {
    if(num > 3) {
        return false;
    }
    return writeMaskedRegister(MPU9250_I2C_SLV0_CTRL + num*3, MPU9250_I2C_SLV0_LENG_MASK, length);
}

// I2C_SLV* registers (Slave 4)

/** Get the I2C address of Slave 4.
 * Note that Bit 7 (MSB) controls read/write mode. If Bit 7 is set, it's a read
 * operation, and if it is cleared, then it's a write operation. The remaining
 * bits (6-0) are the 7-bit device address of the slave device.
 *
 * @return Current address for Slave 4
 * @see getSlaveAddress()
 * @see MPU9250_RA_I2C_SLV4_ADDR
 */
uint8_t MPU9250::getSlave4Address(void) {
    return readRegister(MPU9250_I2C_SLV4_ADDR);
}

/** Set the I2C address of Slave 4.
 * @param address New address for Slave 4
 * @see getSlave4Address()
 * @see MPU9250_RA_I2C_SLV4_ADDR
 */
bool MPU9250::setSlave4Address(const uint8_t address) {
    return writeRegister(MPU9250_I2C_SLV4_ADDR, address);
}

/** Get the active internal register for the Slave 4.
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * @return Current active register for Slave 4
 * @see MPU9250_RA_I2C_SLV4_REG
 */
uint8_t MPU9250::getSlave4Register(void) {
    return readRegister(MPU9250_I2C_SLV4_REG);
}

/** Set the active internal register for Slave 4.
 * @param reg New active register for Slave 4
 * @see getSlave4Register()
 * @see MPU9250_RA_I2C_SLV4_REG
 */
bool MPU9250::setSlave4Register(const uint8_t reg) {
    return writeRegister(MPU9250_I2C_SLV4_REG, reg);
}

/** Set new byte to write to Slave 4.
 * This register stores the data to be written into the Slave 4. If I2C_SLV4_RW
 * is set 1 (set to read), this register has no effect.
 * @param data New byte to write to Slave 4
 * @see MPU9250_RA_I2C_SLV4_DO
 */
bool MPU9250::setSlave4OutputByte(const uint8_t data) {
    return writeRegister(MPU9250_I2C_SLV4_DO, data);
}

/** Get the enabled value for the Slave 4.
 * When set to 1, this bit enables Slave 4 for data transfer operations. When
 * cleared to 0, this bit disables Slave 4 from data transfer operations.
 * @return Current enabled value for Slave 4
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250::getSlave4Enabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_EN_MASK);
    return (response != 0);
}

/** Set the enabled value for Slave 4.
 * @param enabled New enabled value for Slave 4
 * @see getSlave4Enabled()
 * @see MPU9250_I2C_SLV4_CTRL
 */
bool MPU9250::setSlave4Enabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_EN_MASK, enabled);
}

/** Get the enabled value for Slave 4 transaction interrupts.
 * When set to 1, this bit enables the generation of an interrupt signal upon
 * completion of a Slave 4 transaction. When cleared to 0, this bit disables the
 * generation of an interrupt signal upon completion of a Slave 4 transaction.
 * The interrupt status can be observed in Register 54.
 *
 * @return Current enabled value for Slave 4 transaction interrupts.
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250::getSlave4InterruptEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_SLV4_CTRL, MPU9250_SLV4_DONE_INT_EN_MASK);
    return (response != 0);
}

/** Set the enabled value for Slave 4 transaction interrupts.
 * @param enabled New enabled value for Slave 4 transaction interrupts.
 * @see getSlave4InterruptEnabled()
 * @see MPU9250_I2C_SLV4_CTRL
 */
bool MPU9250::setSlave4InterruptEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_I2C_SLV4_CTRL, MPU9250_SLV4_DONE_INT_EN_MASK, enabled);
}

/** Get write mode for Slave 4.
 * When set to 1, the transaction will read or write data only. When cleared to
 * 0, the transaction will write a register address prior to reading or writing
 * data. This should equal 0 when specifying the register address within the
 * Slave device to/from which the ensuing data transaction will take place.
 *
 * @return Current write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250::getSlave4WriteMode(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_REG_DIS_MASK);
    return (response != 0);
}

/** Set write mode for the Slave 4.
 * @param mode New write mode for Slave 4 (0 = register address + data, 1 = data only)
 * @see getSlave4WriteMode()
 * @see MPU9250_I2C_SLV4_CTRL
 */
bool MPU9250::setSlave4WriteMode(const bool mode) {
    return writeMaskedRegister(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_SLV4_REG_DIS_MASK, mode);
}

/** Get Slave 4 master delay value.
 * This configures the reduced access rate of I2C slaves relative to the Sample
 * Rate. When a slave's access rate is decreased relative to the Sample Rate,
 * the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register 25) and
 * DLPF_CFG (register 26). Whether a slave's access rate is reduced relative to
 * the Sample Rate is determined by I2C_MST_DELAY_CTRL (register 103). For
 * further information regarding the Sample Rate, please refer to register 25.
 *
 * @return Current Slave 4 master delay value
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
uint8_t MPU9250::getSlave4MasterDelay(void) {
    return readMaskedRegister(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_MST_DLY_MASK);
}

/** Set Slave 4 master delay value.
 * @param delay New Slave 4 master delay value
 * @see getSlave4MasterDelay()
 * @see MPU9250_RA_I2C_SLV4_CTRL
 */
bool MPU9250::setSlave4MasterDelay(const uint8_t delay) {
    return writeMaskedRegister(MPU9250_I2C_SLV4_CTRL, MPU9250_I2C_MST_DLY_MASK, delay);
}

/** Get last available byte read from Slave 4.
 * This register stores the data read from Slave 4. This field is populated
 * after a read transaction.
 * @return Last available byte read from to Slave 4
 * @see MPU9250_I2C_SLV4_DI
 */
uint8_t MPU9250::getSlate4InputByte(void) {
    return readRegister(MPU9250_I2C_SLV4_DI);
}

// I2C_MST_STATUS register

/** Get FSYNC interrupt status.
 * This bit reflects the status of the FSYNC interrupt from an external device
 * into the MPU-60X0. This is used as a way to pass an external interrupt
 * through the MPU-60X0 to the host application processor. When set to 1, this
 * bit will cause an interrupt if FSYNC_INT_EN is asserted in INT_PIN_CFG
 * (Register 55).
 * @return FSYNC interrupt status
 * @see MPU9250_I2C_MST_STATUS
 */
bool MPU9250::getPassthroughStatus(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_STATUS, MPU9250_PASS_THROUGH_MASK);
    return (response != 0);
}

/** Get Slave 4 transaction done status.
 * Automatically sets to 1 when a Slave 4 transaction has completed. This
 * triggers an interrupt if the I2C_MST_INT_EN bit in the INT_ENABLE register
 * (Register 56) is asserted and if the SLV_4_DONE_INT bit is asserted in the
 * I2C_SLV4_CTRL register (Register 52).
 * @return Slave 4 transaction done status
 * @see MPU9250_I2C_MST_STATUS
 */
bool MPU9250::getSlave4IsDone(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_STATUS, MPU9250_I2C_SLV4_DONE_MASK);
    return (response != 0);
}

/** Get master arbitration lost status.
 * This bit automatically sets to 1 when the I2C Master has lost arbitration of
 * the auxiliary I2C bus (an error condition). This triggers an interrupt if the
 * I2C_MST_INT_EN bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Master arbitration lost status
 * @see MPU9250_I2C_MST_STATUS
 */
bool MPU9250::getLostArbitration(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_STATUS, MPU9250_I2C_LOST_ARB_MASK);
    return (response != 0);
}

/** Get Slave 4 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 4. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 4 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave4Nack(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_STATUS, MPU9250_I2C_SLV4_NACK_MASK);
    return (response != 0);
}

/** Get Slave 3 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 3. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 3 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave3Nack(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_STATUS, MPU9250_I2C_SLV3_NACK_MASK);
    return (response != 0);
}

/** Get Slave 2 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 2. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 2 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave2Nack(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_STATUS, MPU9250_I2C_SLV2_NACK_MASK);
    return (response != 0);
}

/** Get Slave 1 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 1. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 1 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave1Nack(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_STATUS, MPU9250_I2C_SLV1_NACK_MASK);
    return (response != 0);
}

/** Get Slave 0 NACK status.
 * This bit automatically sets to 1 when the I2C Master receives a NACK in a
 * transaction with Slave 0. This triggers an interrupt if the I2C_MST_INT_EN
 * bit in the INT_ENABLE register (Register 56) is asserted.
 * @return Slave 0 NACK interrupt status
 * @see MPU9250_RA_I2C_MST_STATUS
 */
bool MPU9250::getSlave0Nack(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_STATUS, MPU9250_I2C_SLV0_NACK_MASK);
    return (response != 0);
}

// INT_PIN_CFG register

/** Get interrupt logic level mode.
 * Will be set 0 for active-high, 1 for active-low.
 * @return Current interrupt mode (0=active-high, 1=active-low)
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_ACTL_MASK
 */
bool MPU9250::getInterruptMode(void) {
    uint8_t response = readMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_ACTL_MASK);
    return (response != 0);
}

/** Set interrupt logic level mode.
 * @param mode New interrupt mode (0=active-high, 1=active-low)
 * @see getInterruptMode()
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_ACTL_MASK
 */
bool MPU9250::setInterruptMode(const bool mode) {
   return writeMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_ACTL_MASK, mode);
}

/** Get interrupt drive mode.
 * Will be set 0 for push-pull, 1 for open-drain.
 * @return Current interrupt drive mode (0=push-pull, 1=open-drain)
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_OPEN_MASK
 */
bool MPU9250::getInterruptDrive(void) {
    uint8_t response = readMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_OPEN_MASK);
    return (response != 0);
}
/** Set interrupt drive mode.
 * @param drive New interrupt drive mode (0=push-pull, 1=open-drain)
 * @see getInterruptDrive()
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_OPEN_MASK
 */
bool MPU9250::setInterruptDrive(const bool drive) {
    return writeMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_OPEN_MASK, drive);
}

/** Get interrupt latch mode.
 * Will be set 0 for 50us-pulse, 1 for latch-until-int-cleared.
 * @return Current latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_LATCH_INT_EN_MASK
 */
bool MPU9250::getInterruptLatch(void) {
    uint8_t response = readMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_LATCH_INT_EN_MASK);
    return (response != 0);
}

/** Set interrupt latch mode.
 * @param latch New latch mode (0=50us-pulse, 1=latch-until-int-cleared)
 * @see getInterruptLatch()
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_LATCH_INT_EN_MASK
 */
bool MPU9250::setInterruptLatch(const bool latch) {
    return writeMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_LATCH_INT_EN_MASK, latch);
}

/** Get interrupt latch clear mode.
 * Will be set 0 for status-read-only, 1 for any-register-read.
 * @return Current latch clear mode (0=status-read-only, 1=any-register-read)
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_INT_ANYRD_2CLEAR_MASK
 */
bool MPU9250::getInterruptLatchClear(void) {
    uint8_t response = readMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_INT_ANYRD_2CLEAR_MASK);
    return (response != 0);
}

/** Set interrupt latch clear mode.
 * @param clear New latch clear mode (0=status-read-only, 1=any-register-read)
 * @see getInterruptLatchClear()
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_INT_ANYRD_2CLEAR_MASK
 */
bool MPU9250::setInterruptLatchClear(const bool clear) {
    return writeMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_INT_ANYRD_2CLEAR_MASK, clear);
}

/** Get FSYNC interrupt logic level mode.
 * @return Current FSYNC interrupt mode (0=active-high, 1=active-low)
 * @see getFSyncInterruptMode()
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_FSYNC_INT_MODE_EN_MASK
 */
bool MPU9250::getFSyncInterruptLevel(void) {
    uint8_t response = readMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_FSYNC_INT_MODE_EN_MASK);
    return (response != 0);
}

/** Set FSYNC interrupt logic level mode.
 * @param mode New FSYNC interrupt mode (0=active-high, 1=active-low)
 * @see getFSyncInterruptMode()
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_FSYNC_INT_MODE_EN_MASK
 */
bool MPU9250::setFSyncInterruptLevel(const bool level) {
    return writeMaskedRegister(MPU9250_INT_PIN_CFG, MPU9250_FSYNC_INT_MODE_EN_MASK, level);
}

/** Get FSYNC pin interrupt enabled setting.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled setting
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_FSYNC_INT_EN_MASK
 */
bool MPU9250::getFSyncInterruptEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, MPU9250_FSYNC_INT_EN_MASK);
    return (response != 0);
}

/** Set FSYNC pin interrupt enabled setting.
 * @param enabled New FSYNC pin interrupt enabled setting
 * @see getFSyncInterruptEnabled()
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_FSYNC_INT_EN_MASK
 */
bool MPU9250::setFSyncInterruptEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_INT_ENABLE, MPU9250_FSYNC_INT_EN_MASK, enabled);
}

/** Get I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @return Current I2C bypass enabled status
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_BYPASS_EN_MASK
 */
bool MPU9250::getI2CBypassEnabled() {
    INT_PIN_CFG reg;
    // TODO: check I2C_MST_EN bit (?)
    return readRegister(reg).getData().Structed.BYPASS_EN;
}

/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_BYPASS_EN_MASK
 */
void MPU9250::setI2CBypassEnabled(bool enabled) {
    updateRegister<INT_PIN_CFG>([=](auto reg&) {
        reg.getData().Structed.BYPASS_EN = enabled;
    });
}

/** Get reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
 * further information regarding CLKOUT, please refer to the MPU-60X0 Product
 * Specification document.
 * @return Current reference clock output enabled status
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_INTCFG_CLKOUT_EN_BIT
 */
bool MPU9250::getClockOutputEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_PIN_CFG, uint8_t mask); //MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_CLKOUT_EN_BIT, buffer);
    //return (response != 0);
}

/** Set reference clock output enabled status.
 * When this bit is equal to 1, a reference clock output is provided at the
 * CLKOUT pin. When this bit is equal to 0, the clock output is disabled. For
 * further information regarding CLKOUT, please refer to the MPU-60X0 Product
 * Specification document.
 * @param enabled New reference clock output enabled status
 * @see MPU9250_INT_PIN_CFG
 * @see MPU9250_INTCFG_CLKOUT_EN_BIT
 */
bool MPU9250::setClockOutputEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(MPU9250_INT_PIN_CFG, uint8_t mask, enabled); //MPU9250_INT_PIN_CFG, MPU9250_INTCFG_CLKOUT_EN_BIT, enabled);
}

// INT_ENABLE register

/** Get full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit will be
 * set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
uint8_t MPU9250::getIntEnabled(void) {
    return readRegister(MPU9250_INT_ENABLE);
}

/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
bool MPU9250::setIntEnabled(const uint8_t enabled) {
    return writeRegister(MPU9250_INT_ENABLE, enabled);
}

/** Get Free Fall interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
bool MPU9250::getIntFreefallEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_INT_ENABLE, MPU9250_INTERRUPT_FF_BIT, buffer);
    //return (response != 0);
}

/** Set Free Fall interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_FF_BIT
 **/
bool MPU9250::setIntFreefallEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask, enabled); //MPU9250_INT_ENABLE, MPU9250_INTERRUPT_FF_BIT, enabled);
}

/** Get Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_MOT_BIT
 **/
bool MPU9250::getIntMotionEnabled(void) {
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_INT_ENABLE, MPU9250_INTERRUPT_MOT_BIT, buffer);
    //return (response != 0);
}

/** Set Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntMotionEnabled()
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_MOT_BIT
 **/
bool MPU9250::setIntMotionEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask, enabled); //MPU9250_INT_ENABLE, MPU9250_INTERRUPT_MOT_BIT, enabled);
}

/** Get Zero Motion Detection interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 **/
bool MPU9250::getIntZeroMotionEnabled(void) {
    return 0;
    //uint8_t respo = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_INT_ENABLE, MPU9250_INTERRUPT_ZMOT_BIT, buffer);
    //return (response != 0);
}

/** Set Zero Motion Detection interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntZeroMotionEnabled()
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 **/
bool MPU9250::setIntZeroMotionEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask, enabled); //MPU9250_INT_ENABLE, MPU9250_INTERRUPT_ZMOT_BIT, enabled);
}

/** Get FIFO Buffer Overflow interrupt enabled status.
 * Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_FIFO_OFLOW_EN_MASK
 **/
bool MPU9250::getIntFIFOBufferOverflowEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, MPU9250_FIFO_OFLOW_EN_MASK);
    return (response != 0);
}

/** Set FIFO Buffer Overflow interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntFIFOBufferOverflowEnabled()
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_FIFO_OFLOW_EN_MASK
 **/
bool MPU9250::setIntFIFOBufferOverflowEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_INT_ENABLE, MPU9250_FIFO_OFLOW_EN_MASK, enabled);
}

/** Get I2C Master interrupt enabled status.
 * This enables any of the I2C Master interrupt sources to generate an
 * interrupt. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_USER_CTRL
 * @see MPU9250_I2C_MST_EN_MASK
 **/
bool MPU9250::getIntI2CMasterEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN_MASK);
    return (response != 0);
}

/** Set I2C Master interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntI2CMasterEnabled()
 * @see MPU9250_USER_CTRL
 * @see MPU9250_I2C_MST_EN_MASK
 **/
bool MPU9250::setIntI2CMasterEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN_MASK, enabled);
}

/** Get Data Ready interrupt enabled setting.
 * This event occurs each time a write operation to all of the sensor registers
 * has been completed. Will be set 0 for disabled, 1 for enabled.
 * @return Current interrupt enabled status
 * @see MPU9250_INT_ENABLE
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
bool MPU9250::getIntDataReadyEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, buffer);
    //return (response != 0);
}

/** Set Data Ready interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see MPU9250_RA_INT_CFG
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
bool MPU9250::setIntDataReadyEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask, enabled); //MPU9250_INT_ENABLE, MPU9250_INTERRUPT_DATA_RDY_BIT, enabled);
}

// INT_STATUS register

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 */
uint8_t MPU9250::getIntStatus(void) {
    return readRegister(MPU9250_INT_STATUS);
}

/** Get Free Fall interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_FF_BIT
 */
bool MPU9250::getIntFreefallStatus(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_FF_BIT, buffer);
    //return (response != 0);
}

/** Get Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Motion Detection interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_MOT_BIT
 */
bool MPU9250::getIntMotionStatus(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_MOT_BIT, buffer);
    //return (response != 0);
}

/** Get Zero Motion Detection interrupt status.
 * This bit automatically sets to 1 when a Zero Motion Detection interrupt has
 * been generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_ZMOT_BIT
 */
bool MPU9250::getIntZeroMotionStatus(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_ZMOT_BIT, buffer);
    //return (response != 0);
}

/** Get FIFO Buffer Overflow interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_FIFO_OFLOW_BIT
 */
bool MPU9250::getIntFIFOBufferOverflowStatus(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_FIFO_OFLOW_BIT, buffer);
    //return (response != 0);
}

/** Get I2C Master interrupt status.
 * This bit automatically sets to 1 when an I2C Master interrupt has been
 * generated. For a list of I2C Master interrupts, please refer to Register 54.
 * The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_I2C_MST_INT_BIT
 */
bool MPU9250::getIntI2CMasterStatus(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_I2C_MST_INT_BIT, buffer);
    //eturn (response != 0);
}

/** Get Data Ready interrupt status.
 * This bit automatically sets to 1 when a Data Ready interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU9250_RA_INT_STATUS
 * @see MPU9250_INTERRUPT_DATA_RDY_BIT
 */
bool MPU9250::getIntDataReadyStatus(void) {
    return 0;
    //uint8_t response = readMaskedRegister(MPU9250_INT_ENABLE, uint8_t mask); //MPU9250_RA_INT_STATUS, MPU9250_INTERRUPT_DATA_RDY_BIT, buffer);
    //return (response != 0);
}

// EXT_SENS_DATA_* registers

/** Read single byte from external sensor data register.
 * These registers store data read from external sensors by the Slave 0, 1, 2,
 * and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
 * I2C_SLV4_DI (Register 53).
 *
 * External sensor data is written to these registers at the Sample Rate as
 * defined in Register 25. This access rate can be reduced by using the Slave
 * Delay Enable registers (Register 103).
 *
 * External sensor data registers, along with the gyroscope measurement
 * registers, accelerometer measurement registers, and temperature measurement
 * registers, are composed of two sets of registers: an internal register set
 * and a user-facing read register set.
 *
 * The data within the external sensors' internal register set is always updated
 * at the Sample Rate (or the reduced access rate) whenever the serial interface
 * is idle. This guarantees that a burst read of sensor registers will read
 * measurements from the same sampling instant. Note that if burst reads are not
 * used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Data is placed in these external sensor data registers according to
 * I2C_SLV0_CTRL, I2C_SLV1_CTRL, I2C_SLV2_CTRL, and I2C_SLV3_CTRL (Registers 39,
 * 42, 45, and 48). When more than zero bytes are read (I2C_SLVx_LEN > 0) from
 * an enabled slave (I2C_SLVx_EN = 1), the slave is read at the Sample Rate (as
 * defined in Register 25) or delayed rate (if specified in Register 52 and
 * 103). During each Sample cycle, slave reads are performed in order of Slave
 * number. If all slaves are enabled with more than zero bytes to be read, the
 * order will be Slave 0, followed by Slave 1, Slave 2, and Slave 3.
 *
 * Each enabled slave will have EXT_SENS_DATA registers associated with it by
 * number of bytes read (I2C_SLVx_LEN) in order of slave number, starting from
 * EXT_SENS_DATA_00. Note that this means enabling or disabling a slave may
 * change the higher numbered slaves' associated registers. Furthermore, if
 * fewer total bytes are being read from the external sensors as a result of
 * such a change, then the data remaining in the registers which no longer have
 * an associated slave device (i.e. high numbered registers) will remain in
 * these previously allocated registers unless reset.
 *
 * If the sum of the read lengths of all SLVx transactions exceed the number of
 * available EXT_SENS_DATA registers, the excess bytes will be dropped. There
 * are 24 EXT_SENS_DATA registers and hence the total read lengths between all
 * the slaves cannot be greater than 24 or some bytes will be lost.
 *
 * Note: Slave 4's behavior is distinct from that of Slaves 0-3. For further
 * information regarding the characteristics of Slave 4, please refer to
 * Registers 49 to 53.
 *
 * EXAMPLE:
 * Suppose that Slave 0 is enabled with 4 bytes to be read (I2C_SLV0_EN = 1 and
 * I2C_SLV0_LEN = 4) while Slave 1 is enabled with 2 bytes to be read so that
 * I2C_SLV1_EN = 1 and I2C_SLV1_LEN = 2. In such a situation, EXT_SENS_DATA _00
 * through _03 will be associated with Slave 0, while EXT_SENS_DATA _04 and 05
 * will be associated with Slave 1. If Slave 2 is enabled as well, registers
 * starting from EXT_SENS_DATA_06 will be allocated to Slave 2.
 *
 * If Slave 2 is disabled while Slave 3 is enabled in this same situation, then
 * registers starting from EXT_SENS_DATA_06 will be allocated to Slave 3
 * instead.
 *
 * REGISTER ALLOCATION FOR DYNAMIC DISABLE VS. NORMAL DISABLE:
 * If a slave is disabled at any time, the space initially allocated to the
 * slave in the EXT_SENS_DATA register, will remain associated with that slave.
 * This is to avoid dynamic adjustment of the register allocation.
 *
 * The allocation of the EXT_SENS_DATA registers is recomputed only when (1) all
 * slaves are disabled, or (2) the I2C_MST_RST bit is set (Register 106).
 *
 * This above is also true if one of the slaves gets NACKed and stops
 * functioning.
 *
 * @param position Starting position (0-23)
 * @return Byte read from register
 */
uint8_t MPU9250::getExternalSensorByte(int position) {
    return readRegister(MPU9250_EXT_SENS_DATA_00 + position);
}

/** Read word (2 bytes) from external sensor data registers.
 * @param position Starting position (0-21)
 * @return Word read from register
 * @see getExternalSensorByte()
 */
uint16_t MPU9250::getExternalSensorWord(int position) {
    return readRegisters(MPU9250_EXT_SENS_DATA_00 + position, MPU9250_EXT_SENS_DATA_01 + position);
}

/** Read double word (4 bytes) from external sensor data registers.
 * @param position Starting position (0-20)
 * @return Double word read from registers
 * @see getExternalSensorByte()
 */
uint32_t MPU9250::getExternalSensorDWord(int position) {
    uint16_t word_msb = getExternalSensorWord(position);
    uint16_t word_lsb = getExternalSensorWord(position + 2);
    return (((uint32_t)word_msb) << 16) | word_lsb;
}

// MOT_DETECT_STATUS register

/** Get X-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_XNEG_BIT
 */
bool MPU9250::getXNegMotionDetected(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_XNEG_BIT, buffer);
    //return (response != 0);
}

/** Get X-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_XPOS_BIT
 */
bool MPU9250::getXPosMotionDetected(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_XPOS_BIT, buffer);
    //return (response != 0);
}

/** Get Y-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_YNEG_BIT
 */
bool MPU9250::getYNegMotionDetected(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_YNEG_BIT, buffer);
    //return (response != 0);
}

/** Get Y-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_YPOS_BIT
 */
bool MPU9250::getYPosMotionDetected(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_YPOS_BIT, buffer);
    //return (response != 0);
}

/** Get Z-axis negative motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZNEG_BIT
 */
bool MPU9250::getZNegMotionDetected(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZNEG_BIT, buffer);
    //return (response != 0);
}

/** Get Z-axis positive motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZPOS_BIT
 */
bool MPU9250::getZPosMotionDetected(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZPOS_BIT, buffer);
    //return (response != 0);
}

/** Get zero motion detection interrupt status.
 * @return Motion detection status
 * @see MPU9250_RA_MOT_DETECT_STATUS
 * @see MPU9250_MOTION_MOT_ZRMOT_BIT
 */
bool MPU9250::getZeroMotionDetected(void) {
    return 0; //MPU9250_MOT_DETECT_CTRL
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_STATUS, MPU9250_MOTION_MOT_ZRMOT_BIT, buffer);
    //return (response != 0);
}

// I2C_SLV*_DO register

/** Write byte to Data Output container for specified slave.
 * This register holds the output data written into Slave when Slave is set to
 * write mode. For further information regarding Slave control, please
 * This register is used to specify the timing of external sensor data
 * refer to Registers 37 to 39 and immediately following.
 * @param num Slave number (0-3)
 * @param data Byte to write
 * @see MPU9250_I2C_SLV0_DO
 */
bool MPU9250::setSlaveOutputByte(uint8_t num, const uint8_t data) {
    if (num > 3) {
        return 0;
    }
    return writeRegister(MPU9250_I2C_SLV0_DO + num, data);
}

// I2C_MST_DELAY_CTRL register

/** Get external data shadow delay enabled status.
 * This register is used to specify the timing of external sensor data
 * shadowing. When DELAY_ES_SHADOW is set to 1, shadowing of external
 * sensor data is delayed until all data has been received.
 * @return Current external data shadow delay enabled status.
 * @see MPU9250_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT
 */
bool MPU9250::getExternalShadowDelayEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_DELAY_CTRL, MPU9250_DELAY_ES_SHADOW_MASK);
    return (response != 0);
}

/** Set external data shadow delay enabled status.
 * @param enabled New external data shadow delay enabled status.
 * @see getExternalShadowDelayEnabled()
 * @see MPU9250_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT
 */
bool MPU9250::setExternalShadowDelayEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_I2C_MST_DELAY_CTRL, MPU9250_DELAY_ES_SHADOW_MASK, enabled);
}

/** Get slave delay enabled status.
 * When a particular slave delay is enabled, the rate of access for the that
 * slave device is reduced. When a slave's access rate is decreased relative to
 * the Sample Rate, the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) Samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register  * 25)
 * and DLPF_CFG (register 26).
 *
 * For further information regarding I2C_MST_DLY, please refer to register 52.
 * For further information regarding the Sample Rate, please refer to register 25.
 *
 * @param num Slave number (0-4)
 * @return Current slave delay enabled status.
 * @see MPU9250_RA_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
bool MPU9250::getSlaveDelayEnabled(const uint8_t num) {
    // MPU9250_DELAYCTRL_I2C_SLV4_DLY_EN_BIT is 4, SLV3 is 3, etc. //sweet jesus something is wrong here
    if(num > 4) {
        return 0;
    }
    uint8_t response = readMaskedRegister(MPU9250_I2C_MST_DELAY_CTRL, 0x01 << num); //MPU9250_RA_I2C_MST_DELAY_CTRL, num, buffer);
    return (response != 0);
}

/** Set slave delay enabled status.
 * @param num Slave number (0-4)
 * @param enabled New slave delay enabled status.
 * @see MPU9250_I2C_MST_DELAY_CTRL
 * @see MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */
bool MPU9250::setSlaveDelayEnabled(const uint8_t num, const bool enabled) {
    if(num > 4) {
        return 0;
    }
    return writeMaskedRegister(MPU9250_I2C_MST_DELAY_CTRL, 0x01 << num, enabled);
}

// SIGNAL_PATH_RESET register

/** Reset gyroscope signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_SIGNAL_PATH_RESET
 * @see MPU9250_GYRO_RST_MASK
 */
bool MPU9250::resetGyroscopePath(void) {
    return writeMaskedRegister(MPU9250_SIGNAL_PATH_RESET, MPU9250_GYRO_RST_MASK, true);
}

/** Reset accelerometer signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_SIGNAL_PATH_RESET
 * @see MPU9250_ACCEL_RST_MASK
 */
bool MPU9250::resetAccelerometerPath(void) {
    return writeMaskedRegister(MPU9250_SIGNAL_PATH_RESET, MPU9250_ACCEL_RST_MASK, true);
}

/** Reset temperature sensor signal path.
 * The reset will revert the signal path analog to digital converters and
 * filters to their power up configurations.
 * @see MPU9250_SIGNAL_PATH_RESET
 * @see MPU9250_TEMP_RST_MASK
 */
bool MPU9250::resetTemperaturePath(void) {
    return writeMaskedRegister(MPU9250_SIGNAL_PATH_RESET, MPU9250_TEMP_RST_MASK, true);
}

// MOT_DETECT_CTRL register

/** Get accelerometer power-on delay.
 * The accelerometer data path provides samples to the sensor registers, Motion
 * detection, Zero Motion detection, and Free Fall detection modules. The
 * signal path contains filters which must be flushed on wake-up with new
 * samples before the detection modules begin operations. The default wake-up
 * delay, of 4ms can be lengthened by up to 3ms. This additional delay is
 * specified in ACCEL_ON_DELAY in units of 1 LSB = 1 ms. The user may select
 * an    readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, buffer);
y value above zero unless instructed otherwise by InvenSense. Please refer
 * to Section 8 of the MPU-6000/MPU-6050 Product Specification document for
 * further information regarding the detection modules.
 * @return Current accelerometer power-on delay
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_ACCEL_ON_DELAY_BIT
 */
uint8_t MPU9250::getAccelerometerPowerOnDelay(void) {
    return 0;
    //return readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_ACCEL_ON_DELAY_BIT, MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH, buffer);
}

/** Set accelerometer power-on delay.
 * @param delay New accelerometer power-on delay (0-3)
 * @see getAccelerometerPowerOnDelay()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_ACCEL_ON_DELAY_BIT
 */
bool MPU9250::setAccelerometerPowerOnDelay(const uint8_t delay) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, delay); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_ACCEL_ON_DELAY_BIT, MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}

/** Get Free Fall detection counter decrement configuration.
 * Detection is registered by the Free Fall detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring FF_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * FF_COUNT | Counter Decrement
 * ---------+------------------
 * 0        | Reset
 * 1        | 1
 * 2        | 2
 *    readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, buffer);
 3        | 4
 * </pre>
 *
 * When FF_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Free Fall detection,
 * please refer to Registers 29 to 32.
 *
 * @return Current decrement configuration
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_FF_COUNT_BIT
 */
uint8_t MPU9250::getFreefallDetectionCounterDecrement(void) {
    return 0;
    //return readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_FF_COUNT_BIT, MPU9250_DETECT_FF_COUNT_LENGTH, buffer);
}

/** Set Free Fall detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getFreefallDetectionCounterDecrement()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_FF_COUNT_BIT
 */
bool MPU9250::setFreefallDetectionCounterDecrement(const uint8_t decrement) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, decrement); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_FF_COUNT_BIT, MPU9250_DETECT_FF_COUNT_LENGTH, decrement);
}

/** Get Motion detection counter decrement configuration.
 * Detection is registered by the Motion detection module after accelerometer
 * measurements meet their respective threshold conditions over a specified
 * number of samples. When the threshold conditions are met, the corresponding
 * detection counter increments by 1. The user may control the rate at which the
 * detection counter decrements when the threshold condition is not met by
 * configuring MOT_COUNT. The decrement rate can be set according to the
 * following table:
 *
 * <pre>
 * MOT_COUNT | Counter Decrement
 * ----------+------------------
 * 0         | Reset
 * 1         | 1
 * 2         | 2
 * 3       readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, buffer);
      | 4
 * </pre>
 *
 * When MOT_COUNT is configured to 0 (reset), any non-qualifying sample will
 * reset the counter to 0. For further information on Motion detection,
 * please refer to Registers 29 to 32.
 *
 */
uint8_t MPU9250::getMotionDetectionCounterDecrement(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, buffer);
    //return (response != 0);
}

/** Set Motion detection counter decrement configuration.
 * @param decrement New decrement configuration value
 * @see getMotionDetectionCounterDecrement()
 * @see MPU9250_RA_MOT_DETECT_CTRL
 * @see MPU9250_DETECT_MOT_COUNT_BIT
 */
void MPU9250::setMotionDetectionCounterDecrement(const uint8_t decrement) {
    //writeMaskedRegister(uint8_t register_addr, uint8_t mask, decrement); //MPU9250_RA_MOT_DETECT_CTRL, MPU9250_DETECT_MOT_COUNT_BIT, MPU9250_DETECT_MOT_COUNT_LENGTH, decrement);
}

// USER_CTRL register

/** Get I2C Master Mode enabled status.
 * When this mode is enabled, the MPU-60X0 acts as the I2C Master to the
 * external sensor slave devices on the auxiliary I2C bus. When this bit is
 * cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically
 * driven by the primary I2C bus (SDA and SCL). This is a precondition to
 * enabling Bypass Mode. For further information regarding Bypass Mode, please
 * refer to Register 55.
 * @return Current I2C Master Mode enabled status
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_EN_BIT
 */
bool MPU9250::getI2CMasterModeEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN_MASK);
    return (response != 0);
}

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_EN_BIT
 */
bool MPU9250::setI2CMasterModeEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_EN_MASK, enabled);
}

/** Switch from I2C to SPI mode (MPU-6000 only)
 * If this is set, the primary SPI interface will be enabled in place of the
 * disabled primary I2C interface.
 */
bool MPU9250::switchSPIEnabled(const bool enabled) {
    return writeMaskedRegister(MPU9250_USER_CTRL, MPU9250_I2C_IF_DIS_MASK, enabled);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_FIFO_RESET_BIT
 */
bool MPU9250::resetFIFO(void) {
    return writeMaskedRegister(MPU9250_USER_CTRL, MPU9250_FIFO_RST_MASK, true);
}

/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_I2C_MST_RESET_BIT
 */
bool MPU9250::resetI2CMaster(void) {
    return writeMaskedRegister(MPU9250_USER_CTRL, MPU9250_I2C_MST_RST_MASK, true);
    // Set X-axis accelerometer standby enabled status.
}

/** Reset all sensor registers and signal paths.
 * When set to 1, this bit resets the signal paths for all sensors (gyroscopes,
 * accelerometers, and temperature sensor). This operation will also clear the
 * sensor registers. This bit automatically clears to 0 after the reset has been
 * triggered.
 *
 * When resetting only the signal path (and not the sensor registers), please
 * use Register 104, SIGNAL_PATH_RESET.
 *
 * @see MPU9250_RA_USER_CTRL
 * @see MPU9250_USERCTRL_SIG_COND_RESET_BIT
 */
bool MPU9250::resetSensors(void) {
    return writeMaskedRegister(MPU9250_USER_CTRL, MPU9250_SIG_COND_RST_MASK, true);
}

///** Set X-axis accelerometer standby enabled status.

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU9250_RA_PWR_MGMT_1
 * @see MPU9250_PWR1_DEVICE_RESET_BIT
 */
bool MPU9250::reset(void) {
    return writeMaskedRegister(MPU9250_PWR_MGMT_1, MPU9250_H_RESET_MASK, true);
}

bool MPU9250::getSleepEnabled() {
    PWR_MGMT1 reg;
    return readRegister(reg).getData().Structed.SLEEP;
}

void MPU9250::setSleepEnabled(bool enabled) {
    updateRegister<PWR_MGMT1>([=](auto& reg) {
        reg.getData().Structed.SLEEP = enabled;
    });
}

/** Get wake cycle enabled status.
 * When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle
 * between sleep mode and waking up to take a single sample of data from active
 * sensors at a rate determined by LP_WAKE_CTRL (register 108).
 * @return Current sleep mode enabled status
 * @see MPU9250_PWR_MGMT_1
 * @see MPU9250_CYCLE_MASK
 */
bool MPU9250::getWakeCycleEnabled(void) {
    uint8_t response = readMaskedRegister(MPU9250_PWR_MGMT_1, MPU9250_CYCLE_MASK);
    return (response != 0);
}

/** Set wake cycle enabled status.
 * @param enabled New sleep mode enabled status
 * @see getWakeCycleEnabled()
 * @see MPU9250_PWR_MGMT_1
 * @see MPU9250_CYCLE_MASK
 */
bool MPU9250::setWakeCycleEnabled(const bool enabled) {
    // Set X-axis accelerometer standby enabled status.
    return writeMaskedRegister(MPU9250_PWR_MGMT_1, MPU9250_CYCLE_MASK, enabled);
}

uint8_t MPU9250::getClockSource() {
    PWR_MGMT1 reg;
    return readRegister(reg).getData().Structed.CLKSEL;
}

void MPU9250::setClockSource(uint8_t source) {
    updateRegister<PWR_MGMT1>([=](auto& reg) {
        reg.getData().Structed.CLKSEL = source;
    });
}

/** Set X-axis accelerometer standby enabled status.

// PWR_MGMT_2 register

/** Get wake frequency in Accel-Only Low Power Mode.
 * The MPU-60X0 can be put into Accerlerometer Only Low Power Mode by setting
 * PWRSEL to 1 in the Power Management 1 register (Register 107). In this mode,
 * the device will power off all devices except for the primary I2C interface,
 * waking only the accelerometer at fixed intervals to take a single
 * measurement. The frequency of wake-ups can be configured with LP_WAKE_CTRL
 * as shown below:
 *
 * <pre>
 * LP_WAKE_CTRL | Wake-up Frequency
 * -------------+------------------
 * 0            | 1.25 Hz
 * 1            | 2.5 Hz
 * 2            | 5 Hz
 * 3            | 10 Hz
 * <pre>
 *
 * For further information regarding the MPU-60X0's power modes, please refer to
 * Register 107.
 *
 * @return Current wake frequency
 * @see MPU9250_RA_PWR_MGMT_2
 */
uint8_t MPU9250::getWakeFrequency(void) {
    return 0;
    //return readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_LP_WAKE_CTRL_BIT, MPU9250_PWR2_LP_WAKE_CTRL_LENGTH, buffer);
}

/** Set wake frequency in Accel-Only Low Power Mode.
 * @param frequency New wake frequency
 * @see MPU9250_RA_PWR_MGMT_2
 */
bool MPU9250::setWakeFrequency(const uint8_t frequency) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, frequency); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_LP_WAKE_CTRL_BIT, MPU9250_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}

/** Get X-axis accelerometer standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XA_BIT
 */
bool MPU9250::getStandbyXAccelEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XA_BIT, buffer);
    //return (response != 0);
}

/** Set X-axis accelerometer standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XA_BIT
 */
bool MPU9250::setStandbyXAccelEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, enabled); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XA_BIT, enabled);
}

/** Get Y-axis accelerometer standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YA_BIT
 */
bool MPU9250::getStandbyYAccelEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YA_BIT, buffer);
    //return (response != 0);
}

/** Set Y-axis accelerometer standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YA_BIT
 */
bool MPU9250::setStandbyYAccelEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, enabled); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YA_BIT, enabled);
}

/** Get Z-axis accelerometer standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZA_BIT
 */
bool MPU9250::getStandbyZAccelEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZA_BIT, buffer);
    //return (response != 0);
}

/** Set Z-axis accelerometer standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZAccelEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZA_BIT
 */
bool MPU9250::setStandbyZAccelEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, enabled); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZA_BIT, enabled);
}

/** Get X-axis gyroscope standby enabled status.
 * If enabled, the X-axis will not gather or report data (or use power).
 * @return Current X-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XG_BIT
 */
bool MPU9250::getStandbyXGyroEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XG_BIT, buffer);
    //return (response != 0);
}

/** Set X-axis gyroscope standby enabled status.
 * @param New X-axis standby enabled status
 * @see getStandbyXGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_XG_BIT
 */
bool MPU9250::setStandbyXGyroEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, enabled); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_XG_BIT, enabled);
}

/** Get Y-axis gyroscope standby enabled status.
 * If enabled, the Y-axis will not gather or report data (or use power).
 * @return Current Y-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YG_BIT
 */
bool MPU9250::getStandbyYGyroEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YG_BIT, buffer);
    //return (response != 0);
}

/** Set Y-axis gyroscope standby enabled status.
 * @param New Y-axis standby enabled status
 * @see getStandbyYGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_YG_BIT
 */
bool MPU9250::setStandbyYGyroEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, enabled); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_YG_BIT, enabled);
}

/** Get Z-axis gyroscope standby enabled status.
 * If enabled, the Z-axis will not gather or report data (or use power).
 * @return Current Z-axis standby enabled status
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZG_BIT
 */
bool MPU9250::getStandbyZGyroEnabled(void) {
    return 0;
    //uint8_t response = readMaskedRegister(uint8_t register_addr, uint8_t mask); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZG_BIT, buffer);
    //return (response != 0);
}

/** Set Z-axis gyroscope standby enabled status.
 * @param New Z-axis standby enabled status
 * @see getStandbyZGyroEnabled()
 * @see MPU9250_RA_PWR_MGMT_2
 * @see MPU9250_PWR2_STBY_ZG_BIT
 */
bool MPU9250::setStandbyZGyroEnabled(const bool enabled) {
    return 0;
    //return writeMaskedRegister(uint8_t register_addr, uint8_t mask, enabled); //MPU9250_RA_PWR_MGMT_2, MPU9250_PWR2_STBY_ZG_BIT, enabled);
}

// FIFO_COUNT* registers

/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t MPU9250::getFIFOCount(void) {
    return readRegisters(MPU9250_FIFO_COUNTH, MPU9250_FIFO_COUNTL);
}

// FIFO_R_W register

/** Get byte from FIFO buffer.
 * This register is used to read and write data from the FIFO buffer. Data is
 * written to the FIFO in order of register number (from lowest to highest). If
 * all the FIFO enable flags (see below) are enabled and all External Sensor
 * Data registers (Registers 73 to 96) are associated with a Slave device, the
 * contents of registers 59 through 96 will be written in order at the Sample
 * Rate.
 *
 * The contents of the sensor data registers (Registers 59 to 96) are written
 * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
 * in FIFO_EN (Register 35). An additional flag for the sensor data registers
 * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
 *
 * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
 * automatically set to 1. This bit is located in INT_STATUS (Register 58).
 * When the FIFO buffer has overflowed, the oldest data will be lost and new
 * data will be written to the FIFO.
 *
 * If the FIFO buffer is empty, reading this register will return the last byte
 * that was previously read from the FIFO until new data is available. The user
 * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
 * empty.
 *
 * @return Byte from FIFO buffer
 */
uint8_t MPU9250::getFIFOByte(void) {
    return readRegister(MPU9250_FIFO_R_W);
}

//uint8_t MPU9250::getFIFOBytes(uint8_t *data, uint8_t length) {
//    readRegister(uint8_t register_addr); //remove buffer, add read commands which handle the number of subsequent registers listed here: //MPU9250_RA_FIFO_R_W, length, data);
//}


/** Write byte to FIFO buffer.
 * @see getFIFOByte()
 * @see MPU9250_FIFO_R_W
 */
bool MPU9250::setFIFOByte(const uint8_t data) {
    return writeRegister(MPU9250_FIFO_R_W, data);
}

int16_t MPU9250::getXGyroOffset(void) {
    uint8_t msb = readRegister(MPU9250_XG_OFFSET_H);
    uint8_t lsb = readRegister(MPU9250_XG_OFFSET_L);
    return (msb << 8 | lsb); //I think
}

bool MPU9250::setXGyroOffset(const int16_t offset) {
    uint8_t msb_offset = offset >> 8;
    uint8_t lsb_offset = offset & 0x00FF;
    bool msb_check = writeRegister(MPU9250_XG_OFFSET_H, msb_offset);
    bool lsb_check = writeRegister(MPU9250_XG_OFFSET_L, lsb_offset);
    return msb_check | lsb_check;
}

// YG_OFFS_TC register
int16_t MPU9250::getYGyroOffset(void) {
    uint8_t msb = readRegister(MPU9250_YG_OFFSET_H);
    uint8_t lsb = readRegister(MPU9250_YG_OFFSET_L);
    return (msb << 8 | lsb); //I think
}

bool MPU9250::setYGyroOffset(const int16_t offset) {
    uint8_t msb_offset = offset >> 8;
    uint8_t lsb_offset = offset & 0x00FF;
    bool msb_check = writeRegister(MPU9250_YG_OFFSET_H, msb_offset);
    bool lsb_check = writeRegister(MPU9250_YG_OFFSET_L, lsb_offset);
    return msb_check | lsb_check;
}

// ZG_OFFS_TC register
int16_t MPU9250::getZGyroOffset(void) {
    uint8_t msb = readRegister(MPU9250_ZG_OFFSET_H);
    uint8_t lsb = readRegister(MPU9250_ZG_OFFSET_L);
    return (msb << 8 | lsb); //I think
}

bool MPU9250::setZGyroOffset(const int16_t offset) {
    uint8_t msb_offset = offset >> 8;
    uint8_t lsb_offset = offset & 0x00FF;
    bool msb_check = writeRegister(MPU9250_ZG_OFFSET_H, msb_offset);
    bool lsb_check = writeRegister(MPU9250_ZG_OFFSET_L, lsb_offset);
    return msb_check | lsb_check;
}

// XA_OFFS_* registers
int16_t MPU9250::getXAccelOffset(void) {
    return readRegisters(MPU9250_XA_OFFSET_H, MPU9250_XA_OFFSET_L);
}

bool MPU9250::setXAccelOffset(const int16_t offset) {
    return writeRegisters(MPU9250_XA_OFFSET_H, offset >> 8, MPU9250_XA_OFFSET_L, offset);
}

// YA_OFFS_* register
int16_t MPU9250::getYAccelOffset(void) {
    return readRegisters(MPU9250_YA_OFFSET_H, MPU9250_YA_OFFSET_L);
}

bool MPU9250::setYAccelOffset(const int16_t offset) {
    return writeRegisters(MPU9250_YA_OFFSET_H, offset >> 8, MPU9250_YA_OFFSET_L, offset);
}

// ZA_OFFS_* register
int16_t MPU9250::getZAccelOffset(void) {
    return readRegisters(MPU9250_ZA_OFFSET_H, MPU9250_ZA_OFFSET_L);
}

bool MPU9250::setZAccelOffset(const int16_t offset) {
    return writeRegisters(MPU9250_ZA_OFFSET_H, offset >> 8, MPU9250_ZA_OFFSET_L, offset);
}

// XG_OFFS_USR* registers
int16_t MPU9250::getXGyroOffsetUser(void) {
    return 0;
    //return readRegisters(MPU9250_RA_XG_OFFSET_USRH, UNKNOWN_REGISTER);
}

void MPU9250::setXGyroOffsetUser(const int16_t offset) {
    //I2Cdev::writeWord(_address, MPU9250_RA_XG_OFFSET_USRH, offset);
}

// YG_OFFS_USR* register
int16_t MPU9250::getYGyroOffsetUser(void) {
    return 0;
    //return readRegisters(MPU9250_RA_YG_OFFS_USRH, UNKNOWN_REGSITER);
}

void MPU9250::setYGyroOffsetUser(const int16_t offset) {
    //I2Cdev::writeWord(_address, MPU9250_RA_YG_OFFS_USRH, offset);
}

// ZG_OFFS_USR* register
int16_t MPU9250::getZGyroOffsetUser(void) {
    //return readRegisters(MPU9250_RA_ZG_OFFS_USRH, UNKNOWN_REGISTER);
}
// * @param duration New free-fall duration threshold value (LSB = 1ms)

void MPU9250::setZGyroOffsetUser(const int16_t offset) {
    //I2Cdev::writeWord(_address, MPU9250_RA_ZG_OFFS_USRH, offset);
}

uint8_t MPU9250::getDeviceID(void) {
    _whoami = readRegister(MPU9250_WHO_AM_I);
    return _whoami;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool MPU9250::testConnection(void) {
    return getDeviceID() == WHOAMI_DEFAULT_VAL;
}

void MPU9250::selectClock(uint8_t clock_type) {

}

//Accelerometer functions
bool MPU9250::enableAccelerometerAxis(const uint8_t axis) {
    //arguments are : MPU9250_DISABLE_XA_MASK, MPU9250_DISABLE_YA_MASK, MPU9250_DISABLE_ZA_MASK
    //WARNING: this should be verified here
    return writeRegister(MPU9250_PWR_MGMT_2, axis);
}

bool MPU9250::disableAccelerometerAxis(const uint8_t axis) {
    //arguments are : MPU9250_DISABLE_XA_MASK, MPU9250_DISABLE_YA_MASK, MPU9250_DISABLE_ZA_MASK
    //WARNING: this should be verified here
    return writeRegister(MPU9250_PWR_MGMT_2, axis);
}

bool MPU9250::enableAccelerometer(void) {
    //WARNING: should read register value first and modify accordingly
    //WARNING: wrong register value
    return writeRegister(MPU9250_PWR_MGMT_2, MPU9250_DISABLE_XYZA_MASK);
}

bool MPU9250::disableAccelerometer(void) {
    //WARNING: should read register value first and modify accordingly
    return writeRegister(MPU9250_PWR_MGMT_2, MPU9250_DISABLE_XYZA_MASK);
}

void MPU9250::getAccelerometerTestData(uint8_t ax, uint8_t ay, uint8_t az) {
        ax = readRegister(MPU9250_SELF_TEST_X_ACCEL);
        ay = readRegister(MPU9250_SELF_TEST_Y_ACCEL);
        az = readRegister(MPU9250_SELF_TEST_Z_ACCEL);
}

bool MPU9250::accelerometerXIsEnabled() {
    PWR_MGMT2 reg;
    return readRegister(reg).getData().Structed.DISABLE_XA;
}

bool MPU9250::accelerometerYIsEnabled() {
    PWR_MGMT2 reg;
    return readRegister(reg).getData().Structed.DISABLE_YA;
}

bool MPU9250::accelerometerZIsEnabled() {
    PWR_MGMT2 reg;
    return readRegister(reg).getData().Structed.DISABLE_ZA;
}

int16_t MPU9250::getAccelerationX() {
    if (!accelerometerXIsEnabled()) {
        return 0;
    }
    return mergeTwoRegistersS<ACCEL_XOUT_H, ACCEL_XOUT_L>();
}

int16_t MPU9250::getAccelerationY() {
    if (!accelerometerYIsEnabled()) {
        return 0;
    }
    return mergeTwoRegistersS<ACCEL_YOUT_H, ACCEL_YOUT_L>();
}

int16_t MPU9250::getAccelerationZ() {
    if (!accelerometerZIsEnabled()) {
        return 0;
    }
    return mergeTwoRegistersS<ACCEL_ZOUT_H, ACCEL_ZOUT_L>();
}

void MPU9250::getAcceleration(int16_t& ax, int16_t& ay, int16_t& az) {
    ax = getAccelerationX();
    ay = getAccelerationY();
    az = getAccelerationZ();
}

//Gyroscope functions

bool MPU9250::enableGyroAxis(const uint8_t axis) {
    //arguments are : MPU9250_DISABLE_XG_MASK, MPU9250_DISABLE_YG_MASK, MPU9250_DISABLE_ZG_MASK
    //WARNING: this should be verified here

    return writeRegister(MPU9250_PWR_MGMT_2, axis);
}

bool MPU9250::disableGyroAxis(const uint8_t axis) {
    //arguments are : MPU9250_DISABLE_XG_MASK, MPU9250_DISABLE_YG_MASK, MPU9250_DISABLE_ZG_MASK
    //WARNING: this should be verified here
    return writeRegister(MPU9250_PWR_MGMT_2, axis);
}

bool MPU9250::enableGyro(void) {
    //WARNING: should read register value first and modify accordingly
    //WARNING: wrong register value
    return writeRegister(MPU9250_PWR_MGMT_2, MPU9250_DISABLE_XYZG_MASK);
}

bool MPU9250::disableGyro(void) {
    //WARNING: should read register value first and modify accordingly
    return writeRegister(MPU9250_PWR_MGMT_2, MPU9250_DISABLE_XYZG_MASK);
}

void MPU9250::getGyroTestData(uint8_t gx, uint8_t gy, uint8_t gz) {
        gx = readRegister(MPU9250_SELF_TEST_X_GYRO);
        gy = readRegister(MPU9250_SELF_TEST_Y_GYRO);
        gz = readRegister(MPU9250_SELF_TEST_Z_GYRO);
}

bool MPU9250::gyroXIsEnabled() {
    PWR_MGMT2 reg;
    return readRegister(reg).getData().Structed.DISABLE_XG;
}

bool MPU9250::gyroYIsEnabled() {
    PWR_MGMT2 reg;
    return readRegister(reg).getData().Structed.DISABLE_YG;
}

bool MPU9250::gyroZIsEnabled() {
    PWR_MGMT2 reg;
    return readRegister(reg).getData().Structed.DISABLE_ZG;
}

int16_t MPU9250::getRotationX() {
    if (!gyroXIsEnabled()) {
        return 0;
    }
    return mergeTwoRegistersS<GYRO_XOUT_H, GYRO_XOUT_L>();
}

int16_t MPU9250::getRotationY() {
    if (!gyroYIsEnabled()) {
        return 0;
    }
    return mergeTwoRegistersS<GYRO_YOUT_H, GYRO_YOUT_L>();
}

int16_t MPU9250::getRotationZ() {
    if (!gyroZIsEnabled()) {
        return 0;
    }
    return mergeTwoRegistersS<GYRO_ZOUT_H, GYRO_ZOUT_L>();
}

// GYRO_*OUT_* registers

/** Get 3-axis gyroscope readings.
 * These gyroscope measurement registers, along with the accelerometer
 * measurement registers, temperature measurement registers, and external sensor
 * data registers, are composed of two sets of registers: an internal register
 * set and a user-facing read register set.
 * The data within the gyroscope sensors' internal register set is always
 * updated at the Sample Rate. Meanwhile, the user-facing read register set
 * duplicates the internal register set's data values whenever the serial
 * interface is idle. This guarantees that a burst read of sensor registers will
 * read measurements from the same sampling instant. Note that if burst reads
 * are not used, the user is responsible for ensuring a set of single byte reads
 * correspond to a single sampling instant by checking the Data Ready interrupt.
 *
 * Each 16-bit gyroscope measurement has a full scale defined in FS_SEL
 * (Register 27). For each full scale setting, the gyroscopes' sensitivity per
 * LSB in GYRO_xOUT is shown in the table below:
 *
 * <pre>
 * FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 * </pre>
 *
 * @param x 16-bit signed integer container for X-axis rotation
 * @param y 16-bit signed integer container for Y-axis rotation
 * @param z 16-bit signed integer container for Z-axis rotation
 * @see getMotion6()
 * @see MPU9250_RA_GYRO_XOUT_H
 * @see MPU9250_RA_GYRO_XOUT_L
 */

void MPU9250::getRotation(int16_t& gx, int16_t& gy, int16_t& gz) {
    gx = getRotationX();
    gy = getRotationY();
    gz = getRotationZ();
}

void MPU9250::getMotion6(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
    getAcceleration(ax, ay, az);
    getRotation(gx, gy, gz);
}

// ACCEL_*OUT_* registers

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * FUNCTION NOT FULLY IMPLEMENTED YET.
 */
void MPU9250::getMotion9(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz, int16_t& mx, int16_t& my, int16_t& mz) {
    // Get accel and gyro
    getMotion6(ax, ay, az, gx, gy, gz);

    // Read magnetometer
    mx = getMagneticX();
    my = getMagneticX();
    mz = getMagneticX();
}


// Magnetometer functions
uint8_t MPU9250::getMagnetometerMode() {
    MAG_CNTL1 reg;
    return readRegister(reg).getData().Structed.MODE;
}

void MPU9250::setMagnetometerMode(uint8_t mode) {
    updateRegister<MAG_CNTL1>([=](auto& reg) {
        reg.getData().Structed.MODE = mode;
    });
}

int16_t MPU9250::getMagneticX() {
    if (getMagnetometerMode() == MAG_CNTL1_DATA::MODE_POWERDOWN) {
        return 0;
    }
    return mergeTwoRegistersS<MAG_XOUT_H, MAG_XOUT_L>();
}

int16_t MPU9250::getMagneticY() {
    if (getMagnetometerMode() == MAG_CNTL1_DATA::MODE_POWERDOWN) {
        return 0;
    }
    return mergeTwoRegistersS<MAG_YOUT_H, MAG_YOUT_L>();
}

int16_t MPU9250::getMagneticZ() {
    if (getMagnetometerMode() == MAG_CNTL1_DATA::MODE_POWERDOWN) {
        return 0;
    }
    return mergeTwoRegistersS<MAG_ZOUT_H, MAG_ZOUT_L>();
}


//Temperature functions
bool MPU9250::enableTemperature(void) {
    return writeMaskedRegister(MPU9250_FIFO_EN, MPU9250_TEMP_FIFO_EN_MASK, true);
}

bool MPU9250::disableTemperature(void) {
    return writeMaskedRegister(MPU9250_FIFO_EN, MPU9250_TEMP_FIFO_EN_MASK, false);
}

bool MPU9250::temperatureIsEnabled(void) {
    uint8_t status = readMaskedRegister(MPU9250_FIFO_EN, MPU9250_TEMP_FIFO_EN_MASK); //these need to be defined
    return (status != 0);
}

int16_t MPU9250::getTemperature(void) {
    if(temperatureIsEnabled()){
        //get currnet internal temperature reading in 16-bit 2's complement format
        return (int16_t) readRegisters(MPU9250_TEMP_OUT_H, MPU9250_TEMP_OUT_L);
        //WARNING readRegisters function currently returns uint16_t instead of int16_t
        //((Temp_out - room_temp_offset)/temp_sensitivity) + 21; //celcius
    } else {
        return 0;
    }
}
