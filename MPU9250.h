/** Based on InvenSense MPU-9250 register map document rev. 1.4, 9/9/2013 (RM-MPU-9250A-00)
* 13/04/2014 by Conor Forde <me@conorforde.com>
* Updates should be available at https://github.com/Snowda/MPU9250
*
* Changelog:
*     ... - ongoing development release

* NOTE: THIS IS ONLY A PARIAL RELEASE.
* THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE DEVELOPMENT AND IS MISSING MOST FEATURES.
* PLEASE KEEP THIS IN MIND IF YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.
*/

#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdint.h>
#include <stdio.h>
// #include <functional>

#include "I2Cdev.h"
// #include <avr/pgmspace.h>

#define pgm_read_byte(p) (*(uint8_t *)(p))

//MPU9250 Register map


#define MPU9250_ADDRESS_AD0_LOW         0x68    // address pin low (GND), default for InvenSense evaluation board
#define MPU9250_ADDRESS_AD0_HIGH        0x69    // address pin high (VCC)
#define MPU9250_DEFAULT_ADDRESS         MPU9250_ADDRESS_AD0_LOW
#define MPU9250_MAG_ADDRESS             0x0C

// Reset values
#define WHOAMI_RESET_VAL                0x71
#define POWER_MANAGMENT_1_RESET_VAL     0x01
#define DEFAULT_RESET_VALUE             0x00
#define WHOAMI_DEFAULT_VAL              0x71
#define MAG_WIA_DEFAULT_VAL             0x48


enum IO_FLAG {
    R = 1,
    W = 2,
    RW = 3
};

template<uint8_t A, IO_FLAG F, typename T=uint8_t>
struct REGISTER_DESCRIPTION {
    union DATA {
        T Structed;
        uint8_t Raw;
        explicit DATA(T data) : Structed(data) {};
        DATA(uint8_t value) : Raw(value) {};
        DATA() : DATA(0) {};
    };

    static_assert(sizeof(DATA) == sizeof(uint8_t));

    DATA& getData() {
        return Data;
    }

    void setData(const DATA data) {
        Data = data;
    }

    static const uint8_t Address = A;
    static const IO_FLAG IOFlag = F;

private:
    DATA Data;
};

// TODO: Can we fix this "copy-paste"?
template<uint8_t A, IO_FLAG F>
struct REGISTER_DESCRIPTION<A, F, uint8_t> {
    union DATA {
        uint8_t Raw;
        DATA(uint8_t value) : Raw(value) {};
        DATA() : DATA(0) {};
    };

    DATA& getData() {
        return Data;
    }

    void setData(const DATA data) {
        Data = data;
    }

    static const uint8_t Address = A;
    static const IO_FLAG IOFlag = F;

private:
    DATA Data;
};


struct CONFIG_DATA {
    uint8_t DLPF_CFG: 3;        // 0 bit
    uint8_t EXT_SYNC_SET: 3;
    uint8_t FIFO_MODE: 1;       // 6 bit
    uint8_t : 1;                // 7 bit, Reserved

    static const uint8_t EXT_SYNC_DISABLED      = 0x0;
    static const uint8_t EXT_SYNC_TEMP_OUT_L    = 0x1;
    static const uint8_t EXT_SYNC_GYRO_XOUT_L   = 0x2;
    static const uint8_t EXT_SYNC_GYRO_YOUT_L   = 0x3;
    static const uint8_t EXT_SYNC_GYRO_ZOUT_L   = 0x4;
    static const uint8_t EXT_SYNC_ACCEL_XOUT_L  = 0x5;
    static const uint8_t EXT_SYNC_ACCEL_YOUT_L  = 0x6;
    static const uint8_t EXT_SYNC_ACCEL_ZOUT_L  = 0x7;
};

struct GYRO_CONFIG_DATA {
    uint8_t FCHOICE_B: 2;
    uint8_t : 1;
    uint8_t GYRO_FS_SEL : 2;
    uint8_t ZGYRO_CTEN : 1;
    uint8_t YGYRO_CTEN : 1;
    uint8_t XGYRO_CTEN : 1;

    static const uint8_t FULL_SCALE_250DPS     = 0x0;
    static const uint8_t FULL_SCALE_500DPS     = 0x1;
    static const uint8_t FULL_SCALE_1000DPS    = 0x2;
    static const uint8_t FULL_SCALE_2000DPS    = 0x3;
};

struct ACCEL_CONFIG_DATA {
    uint8_t : 3;
    uint8_t ACCEL_FS_SEL : 2;
    uint8_t AZ_ST_EN : 1;
    uint8_t AY_ST_EN : 1;
    uint8_t AX_ST_EN : 1;

    static const uint8_t FULL_SCALE_2G  = 0x0;
    static const uint8_t FULL_SCALE_4G  = 0x1;
    static const uint8_t FULL_SCALE_8G  = 0x2;
    static const uint8_t FULL_SCALE_16G = 0x3;
};

struct ACCEL_CONFIG2_DATA {
    uint8_t A_DLPFCFG : 3;
    uint8_t ACCEL_FCHOICE_B : 1;
    uint8_t : 4;
};

struct LP_ACCEL_ODR_DATA {
    uint8_t LPOSC_CLKSEL : 4;
    uint8_t : 4;
};

struct FIFO_EN_DATA {
    uint8_t SLV0 : 1;
    uint8_t SLV1 : 1;
    uint8_t SLV2 : 1;
    uint8_t ACCEL : 1;
    uint8_t GYRO_ZOUT : 1;
    uint8_t GYRO_YOUT : 1;
    uint8_t GYRO_XOUT : 1;
    uint8_t TEMP_FIFO_EN : 1;
};

struct I2C_MST_CTRL_DATA {
    uint8_t I2C_MST_CLK : 4;
    uint8_t I2C_MST_P_NSR : 1;
    uint8_t SLV_3_FIFO_EN : 1;
    uint8_t WAIT_FOR_ES : 1;
    uint8_t MULT_MST_EN : 1;
};

struct I2C_SLV_ADDR_DATA {
    uint8_t I2C_ID : 7;
    uint8_t I2C_SLV_RNW : 1;
};

struct I2C_SLV_CTRL_DATA {
    uint8_t I2C_SLV_LENG : 4;
    uint8_t I2C_SLV_GRP : 1;
    uint8_t I2C_SLV_REG_DIS : 1;
    uint8_t I2C_SLV_BYTE_SW : 1;
    uint8_t I2C_SLV_EN : 1;
};

struct I2C_SLV4_CTRL_DATA {
    uint8_t I2C_MST_DLY : 5;
    uint8_t I2C_SLV4_REG_DIS : 1;
    uint8_t SLV4_DONE_INT_EN : 1;
    uint8_t I2C_SLV4_EN : 1;
};

struct I2C_MST_STATUS_DATA {
    uint8_t I2C_SLV0_NACK : 1;
    uint8_t I2C_SLV1_NACK : 1;
    uint8_t I2C_SLV2_NACK : 1;
    uint8_t I2C_SLV3_NACK : 1;
    uint8_t I2C_SLV4_NACK : 1;
    uint8_t I2C_LOST_ARB : 1;
    uint8_t I2C_SLV4_DONE : 1;
    uint8_t PASS_THROUGH : 1;
};

struct INT_PIN_CFG_DATA {
    uint8_t : 1;
    uint8_t BYPASS_EN : 1;
    uint8_t FSYNC_INT_MODE_EN : 1;
    uint8_t ACTL_FSYNC : 1;
    uint8_t INT_ANYRD_2CLEAR : 1;
    uint8_t LATCH_INT_EN : 1;
    uint8_t OPEN : 1;
    uint8_t ACTL : 1;
};

struct INT_ENABLE_DATA {
    uint8_t RAW_RDY_EN : 1;
    uint8_t : 2;
    uint8_t FSYNC_INT_EN : 1;
    uint8_t FIFO_OVERFLOW_EN : 1;
    uint8_t : 1;
    uint8_t WOM_EN : 1;
    uint8_t : 1;
};

struct INT_STATUS_DATA {
    uint8_t RAW_DATA_RDY_INT : 1;
    uint8_t : 2;
    uint8_t FSYNC_INT : 1;
    uint8_t FIFO_OVERFLOW_INT : 1;
    uint8_t : 1;
    uint8_t WOM_INT : 1;
    uint8_t : 1;
};

struct I2C_MST_DELAY_CTRL_DATA {
    uint8_t I2C_SLV0_DLY_EN : 1;
    uint8_t I2C_SLV1_DLY_EN : 1;
    uint8_t I2C_SLV2_DLY_EN : 1;
    uint8_t I2C_SLV3_DLY_EN : 1;
    uint8_t I2C_SLV4_DLY_EN : 1;
    uint8_t : 2;
    uint8_t DELAY_ES_SHADOW : 1;
};

struct SIGNAL_PATH_RESET_DATA {
    uint8_t TEMP_RST : 1;
    uint8_t ACCEL_RST : 1;
    uint8_t GYRO_RST : 1;
    uint8_t : 5;
};

struct ACCEL_INTEL_CTRL_DATA {
    uint8_t : 6;
    uint8_t ACCEL_INTEL_MODE : 1;
    uint8_t ACCEL_INTEL_EN : 1;
};

struct USER_CTRL_DATA {
    uint8_t SIG_COND_RST : 1;
    uint8_t I2C_MST_RST : 1;
    uint8_t FIFO_RST : 1;
    uint8_t : 1;
    uint8_t I2C_IF_DIS : 1;
    uint8_t I2C_MST_EN : 1;
    uint8_t FIFO_EN : 1;
    uint8_t : 1;
};

struct PWR_MGMT1_DATA {
    uint8_t CLKSEL : 3;
    uint8_t PD_PTAT : 1;
    uint8_t GYRO_STANDBY : 1;
    uint8_t CYCLE : 1;
    uint8_t SLEEP : 1;
    uint8_t H_RESET : 1;

    static const uint8_t CLKSEL_INTERNAL    = 0x0;
    static const uint8_t CLKSEL_AUTO_PLL    = 0x01;
    static const uint8_t CLKSEL_RESET       = 0x7;
};

struct PWR_MGMT2_DATA {
    uint8_t DISABLE_ZG : 1;
    uint8_t DISABLE_YG : 1;
    uint8_t DISABLE_XG : 1;
    uint8_t DISABLE_ZA : 1;
    uint8_t DISABLE_YA : 1;
    uint8_t DISABLE_XA : 1;
    uint8_t : 2;
};

struct FIFO_COUNTH_DATA {
    uint8_t FIFO_CNT : 5;
    uint8_t : 3;
};

struct A_OFFS_L_DATA {
    uint8_t : 1;
    uint8_t A_OFFS : 7;
};

struct MAG_ST1_DATA {
    uint8_t DRDY : 1;
    uint8_t DOR : 1;
    uint8_t : 6;
};

struct MAG_ST2_DATA {
    uint8_t : 3;
    uint8_t HOFL : 1;
    uint8_t BITM : 1;
    uint8_t : 3;
};

struct MAG_CNTL1_DATA {
    uint8_t MODE : 4;
    uint8_t BIT : 1;
    uint8_t : 3;

    static const uint8_t MODE_POWERDOWN = 0x0;
    static const uint8_t MODE_SINGLE_MEASURE = 0x1;
    static const uint8_t MODE_CONTINIOUS_MEASURE_1 = 0x2;
    static const uint8_t MODE_EXTERNAL_MEASURE = 0x4;
    static const uint8_t MODE_CONTINIOUS_MEASURE_2 = 0x6;
    static const uint8_t MODE_SELF_TEST = 0x8;
    static const uint8_t MODE_FUSE_ROM = 0xF;
    static const uint8_t BIT_14BIT = 0x0;
    static const uint8_t BIT_16BIT = 0x1;
};

struct MAG_CNTL2_DATA {
    uint8_t SRST : 1;
    uint8_t : 7;
};

struct MAG_ASTC_DATA {
    uint8_t : 6;
    uint8_t SELF : 1;
    uint8_t : 1;
};

using SELF_TEST_X_GYRO = REGISTER_DESCRIPTION<0x00, IO_FLAG::RW>;
using SELF_TEST_Y_GYRO = REGISTER_DESCRIPTION<0x01, IO_FLAG::RW>;
using SELF_TEST_Z_GYRO = REGISTER_DESCRIPTION<0x02, IO_FLAG::RW>;
using SELF_TEST_X_ACCEL = REGISTER_DESCRIPTION<0x0D, IO_FLAG::RW>;
using SELF_TEST_Y_ACCEL = REGISTER_DESCRIPTION<0x0E, IO_FLAG::RW>;
using SELF_TEST_Z_ACCEL = REGISTER_DESCRIPTION<0x0F, IO_FLAG::RW>;
using XG_OFFSET_H = REGISTER_DESCRIPTION<0x13, IO_FLAG::RW>;
using XG_OFFSET_L = REGISTER_DESCRIPTION<0x14, IO_FLAG::RW>;
using YG_OFFSET_H = REGISTER_DESCRIPTION<0x15, IO_FLAG::RW>;
using YG_OFFSET_L = REGISTER_DESCRIPTION<0x16, IO_FLAG::RW>;
using ZG_OFFSET_H = REGISTER_DESCRIPTION<0x17, IO_FLAG::RW>;
using ZG_OFFSET_L = REGISTER_DESCRIPTION<0x18, IO_FLAG::RW>;
using SMPLRT_DIV = REGISTER_DESCRIPTION<0x19, IO_FLAG::RW>;
using CONFIG = REGISTER_DESCRIPTION<0x1A, IO_FLAG::RW, CONFIG_DATA>;
using GYRO_CONFIG = REGISTER_DESCRIPTION<0x1B, IO_FLAG::RW, GYRO_CONFIG_DATA>;
using ACCEL_CONFIG = REGISTER_DESCRIPTION<0x1C, IO_FLAG::RW, ACCEL_CONFIG_DATA>;
using ACCEL_CONFIG2 = REGISTER_DESCRIPTION<0x1D, IO_FLAG::RW, ACCEL_CONFIG2_DATA>;
using LP_ACCEL_ODR = REGISTER_DESCRIPTION<0x1E, IO_FLAG::RW, LP_ACCEL_ODR_DATA>;
using WOM_THR = REGISTER_DESCRIPTION<0x1F, IO_FLAG::RW>;
using FIFO_EN = REGISTER_DESCRIPTION<0x23, IO_FLAG::RW, FIFO_EN_DATA>;
using I2C_MST_CTRL = REGISTER_DESCRIPTION<0x24, IO_FLAG::RW, I2C_MST_CTRL_DATA>;
using I2C_SLV0_ADDR = REGISTER_DESCRIPTION<0x25, IO_FLAG::RW, I2C_SLV_ADDR_DATA>;
using I2C_SLV0_REG = REGISTER_DESCRIPTION<0x26, IO_FLAG::RW>;
using I2C_SLV0_CTRL = REGISTER_DESCRIPTION<0x27, IO_FLAG::RW, I2C_SLV_CTRL_DATA>;
using I2C_SLV1_ADDR = REGISTER_DESCRIPTION<0x28, IO_FLAG::RW, I2C_SLV_ADDR_DATA>;
using I2C_SLV1_REG = REGISTER_DESCRIPTION<0x29, IO_FLAG::RW>;
using I2C_SLV1_CTRL = REGISTER_DESCRIPTION<0x2A, IO_FLAG::RW, I2C_SLV_CTRL_DATA>;
using I2C_SLV2_ADDR = REGISTER_DESCRIPTION<0x2B, IO_FLAG::RW, I2C_SLV_ADDR_DATA>;
using I2C_SLV2_REG = REGISTER_DESCRIPTION<0x2C, IO_FLAG::RW>;
using I2C_SLV2_CTRL = REGISTER_DESCRIPTION<0x2D, IO_FLAG::RW, I2C_SLV_CTRL_DATA>;
using I2C_SLV3_ADDR = REGISTER_DESCRIPTION<0x2E, IO_FLAG::RW, I2C_SLV_ADDR_DATA>;
using I2C_SLV3_REG = REGISTER_DESCRIPTION<0x2F, IO_FLAG::RW>;
using I2C_SLV3_CTRL = REGISTER_DESCRIPTION<0x30, IO_FLAG::RW, I2C_SLV_CTRL_DATA>;
using I2C_SLV4_ADDR = REGISTER_DESCRIPTION<0x31, IO_FLAG::RW, I2C_SLV_ADDR_DATA>;
using I2C_SLV4_REG = REGISTER_DESCRIPTION<0x32, IO_FLAG::RW>;
using I2C_SLV4_DO = REGISTER_DESCRIPTION<0x33, IO_FLAG::RW>;
using I2C_SLV4_CTRL = REGISTER_DESCRIPTION<0x34, IO_FLAG::RW, I2C_SLV4_CTRL_DATA>;
using I2C_SLV4_DI = REGISTER_DESCRIPTION<0x35, IO_FLAG::R>;
using I2C_MST_STATUS = REGISTER_DESCRIPTION<0x36, IO_FLAG::R, I2C_MST_STATUS_DATA>;
using INT_PIN_CFG = REGISTER_DESCRIPTION<0x37, IO_FLAG::RW, INT_PIN_CFG_DATA>;
using INT_ENABLE = REGISTER_DESCRIPTION<0x38, IO_FLAG::RW, INT_ENABLE_DATA>;
using INT_STATUS = REGISTER_DESCRIPTION<0x3A, IO_FLAG::R, INT_STATUS_DATA>;
using ACCEL_XOUT_H = REGISTER_DESCRIPTION<0x3B, IO_FLAG::R>;
using ACCEL_XOUT_L = REGISTER_DESCRIPTION<0x3C, IO_FLAG::R>;
using ACCEL_YOUT_H = REGISTER_DESCRIPTION<0x3D, IO_FLAG::R>;
using ACCEL_YOUT_L = REGISTER_DESCRIPTION<0x3E, IO_FLAG::R>;
using ACCEL_ZOUT_H = REGISTER_DESCRIPTION<0x3F, IO_FLAG::R>;
using ACCEL_ZOUT_L = REGISTER_DESCRIPTION<0x40, IO_FLAG::R>;
using TEMP_OUT_H = REGISTER_DESCRIPTION<0x41, IO_FLAG::R>;
using TEMP_OUT_L = REGISTER_DESCRIPTION<0x42, IO_FLAG::R>;
using GYRO_XOUT_H = REGISTER_DESCRIPTION<0x43, IO_FLAG::R>;
using GYRO_XOUT_L = REGISTER_DESCRIPTION<0x44, IO_FLAG::R>;
using GYRO_YOUT_H = REGISTER_DESCRIPTION<0x45, IO_FLAG::R>;
using GYRO_YOUT_L = REGISTER_DESCRIPTION<0x46, IO_FLAG::R>;
using GYRO_ZOUT_H = REGISTER_DESCRIPTION<0x47, IO_FLAG::R>;
using GYRO_ZOUT_L = REGISTER_DESCRIPTION<0x48, IO_FLAG::R>;
using EXT_SENS_DATA_00 = REGISTER_DESCRIPTION<0x49, IO_FLAG::R>;
using EXT_SENS_DATA_01 = REGISTER_DESCRIPTION<0x4A, IO_FLAG::R>;
using EXT_SENS_DATA_02 = REGISTER_DESCRIPTION<0x4B, IO_FLAG::R>;
using EXT_SENS_DATA_03 = REGISTER_DESCRIPTION<0x4C, IO_FLAG::R>;
using EXT_SENS_DATA_04 = REGISTER_DESCRIPTION<0x4D, IO_FLAG::R>;
using EXT_SENS_DATA_05 = REGISTER_DESCRIPTION<0x4E, IO_FLAG::R>;
using EXT_SENS_DATA_06 = REGISTER_DESCRIPTION<0x4F, IO_FLAG::R>;
using EXT_SENS_DATA_07 = REGISTER_DESCRIPTION<0x50, IO_FLAG::R>;
using EXT_SENS_DATA_08 = REGISTER_DESCRIPTION<0x51, IO_FLAG::R>;
using EXT_SENS_DATA_09 = REGISTER_DESCRIPTION<0x52, IO_FLAG::R>;
using EXT_SENS_DATA_10 = REGISTER_DESCRIPTION<0x53, IO_FLAG::R>;
using EXT_SENS_DATA_11 = REGISTER_DESCRIPTION<0x54, IO_FLAG::R>;
using EXT_SENS_DATA_12 = REGISTER_DESCRIPTION<0x55, IO_FLAG::R>;
using EXT_SENS_DATA_13 = REGISTER_DESCRIPTION<0x56, IO_FLAG::R>;
using EXT_SENS_DATA_14 = REGISTER_DESCRIPTION<0x57, IO_FLAG::R>;
using EXT_SENS_DATA_15 = REGISTER_DESCRIPTION<0x58, IO_FLAG::R>;
using EXT_SENS_DATA_16 = REGISTER_DESCRIPTION<0x59, IO_FLAG::R>;
using EXT_SENS_DATA_17 = REGISTER_DESCRIPTION<0x5A, IO_FLAG::R>;
using EXT_SENS_DATA_18 = REGISTER_DESCRIPTION<0x5B, IO_FLAG::R>;
using EXT_SENS_DATA_19 = REGISTER_DESCRIPTION<0x5C, IO_FLAG::R>;
using EXT_SENS_DATA_20 = REGISTER_DESCRIPTION<0x5D, IO_FLAG::R>;
using EXT_SENS_DATA_21 = REGISTER_DESCRIPTION<0x5E, IO_FLAG::R>;
using EXT_SENS_DATA_22 = REGISTER_DESCRIPTION<0x5F, IO_FLAG::R>;
using EXT_SENS_DATA_23 = REGISTER_DESCRIPTION<0x60, IO_FLAG::R>;
using I2C_SLV0_DO = REGISTER_DESCRIPTION<0x63, IO_FLAG::RW>;
using I2C_SLV1_DO = REGISTER_DESCRIPTION<0x64, IO_FLAG::RW>;
using I2C_SLV2_DO = REGISTER_DESCRIPTION<0x65, IO_FLAG::RW>;
using I2C_SLV3_DO = REGISTER_DESCRIPTION<0x66, IO_FLAG::RW>;
using I2C_MST_DELAY_CTRL = REGISTER_DESCRIPTION<0x67, IO_FLAG::RW, I2C_MST_DELAY_CTRL_DATA>;
using SIGNAL_PATH_RESET = REGISTER_DESCRIPTION<0x68, IO_FLAG::RW, SIGNAL_PATH_RESET_DATA>;
using ACCEL_INTEL_CTRL = REGISTER_DESCRIPTION<0x69, IO_FLAG::RW, ACCEL_INTEL_CTRL_DATA>;
using USER_CTRL = REGISTER_DESCRIPTION<0x6A, IO_FLAG::RW, USER_CTRL_DATA>;
using PWR_MGMT1 = REGISTER_DESCRIPTION<0x6B, IO_FLAG::RW, PWR_MGMT1_DATA>;
using PWR_MGMT2 = REGISTER_DESCRIPTION<0x6C, IO_FLAG::RW, PWR_MGMT2_DATA>;
using FIFO_COUNTH = REGISTER_DESCRIPTION<0x72, IO_FLAG::RW, FIFO_COUNTH_DATA>;
using FIFO_COUNTL = REGISTER_DESCRIPTION<0x73, IO_FLAG::RW>;
using FIFO_R_W = REGISTER_DESCRIPTION<0x74, IO_FLAG::RW>;
using WHOAMI = REGISTER_DESCRIPTION<0x75, IO_FLAG::R>;
using XA_OFFS_H = REGISTER_DESCRIPTION<0x77, IO_FLAG::RW>;
using XA_OFFS_L = REGISTER_DESCRIPTION<0x78, IO_FLAG::RW, A_OFFS_L_DATA>;
using YA_OFFS_H = REGISTER_DESCRIPTION<0x7A, IO_FLAG::RW>;
using YA_OFFS_L = REGISTER_DESCRIPTION<0x7B, IO_FLAG::RW, A_OFFS_L_DATA>;
using ZA_OFFS_H = REGISTER_DESCRIPTION<0x7D, IO_FLAG::RW>;
using ZA_OFFS_L = REGISTER_DESCRIPTION<0x7E, IO_FLAG::RW, A_OFFS_L_DATA>;

using MAG_WIA = REGISTER_DESCRIPTION<0x00, IO_FLAG::R>;
using MAG_INFO = REGISTER_DESCRIPTION<0x01, IO_FLAG::R>;
using MAG_ST1 = REGISTER_DESCRIPTION<0x02, IO_FLAG::R, MAG_ST1_DATA>;
using MAG_XOUT_L = REGISTER_DESCRIPTION<0x03, IO_FLAG::R>;
using MAG_XOUT_H = REGISTER_DESCRIPTION<0x04, IO_FLAG::R>;
using MAG_YOUT_L = REGISTER_DESCRIPTION<0x05, IO_FLAG::R>;
using MAG_YOUT_H = REGISTER_DESCRIPTION<0x06, IO_FLAG::R>;
using MAG_ZOUT_L = REGISTER_DESCRIPTION<0x07, IO_FLAG::R>;
using MAG_ZOUT_H = REGISTER_DESCRIPTION<0x08, IO_FLAG::R>;
using MAG_ST2 = REGISTER_DESCRIPTION<0x09, IO_FLAG::R, MAG_ST2_DATA>;
using MAG_CNTL1 = REGISTER_DESCRIPTION<0x0A, IO_FLAG::RW, MAG_CNTL1_DATA>;
using MAG_CNTL2 = REGISTER_DESCRIPTION<0x0B, IO_FLAG::RW, MAG_CNTL2_DATA>;
using MAG_ASTC = REGISTER_DESCRIPTION<0x0C, IO_FLAG::RW, MAG_ASTC_DATA>;
using MAG_TS1 = REGISTER_DESCRIPTION<0x0D, IO_FLAG::RW>;
using MAG_TS2 = REGISTER_DESCRIPTION<0x0E, IO_FLAG::RW>;
using MAG_I2CDIS = REGISTER_DESCRIPTION<0x0F, IO_FLAG::RW>;
using MAG_ASAX = REGISTER_DESCRIPTION<0x10, IO_FLAG::R>;
using MAG_ASAY = REGISTER_DESCRIPTION<0x11, IO_FLAG::R>;
using MAG_ASAZ = REGISTER_DESCRIPTION<0x12, IO_FLAG::R>;


class MPU9250 {
    public:
        MPU9250();
        MPU9250(const char* i2cAddress);
        MPU9250(uint8_t mpuAddress);
        MPU9250(uint8_t mpuAddress, uint8_t magAddress);
        MPU9250(const char* i2cAddress, uint8_t mpuAddress, uint8_t magAddress);

        void initialize();

        uint8_t getAuxVDDIOLevel(void);        // -
        bool setAuxVDDIOLevel(const uint8_t level);        // -
        uint8_t getRate(void);        // -
        bool setRate(const uint8_t rate);        // -
        uint8_t getExternalFrameSync(void);        // -
        bool setExternalFrameSync(const uint8_t sync);        // -
        uint8_t getDLPFMode(void);        // -
        bool setDLPFMode(const uint8_t mode);        // -
        uint8_t getDHPFMode(void);        // -
        bool setDHPFMode(const uint8_t bandwidth);        // -
        uint8_t getFreefallDetectionThreshold(void);        // -
        bool setFreefallDetectionThreshold(const uint8_t threshold);        // -
        uint8_t getFreefallDetectionDuration(void);        // -
        bool setFreefallDetectionDuration(const uint8_t duration);        // -
        uint8_t getMotionDetectionThreshold(void);        // -
        bool setMotionDetectionThreshold(const uint8_t threshold);        // -
        uint8_t getMotionDetectionDuration(void);        // -
        bool setMotionDetectionDuration(const uint8_t duration);        // -
        uint8_t getZeroMotionDetectionThreshold(void);        // -
        bool setZeroMotionDetectionThreshold(const uint8_t threshold);        // -
        uint8_t getZeroMotionDetectionDuration(void);        // -
        bool setZeroMotionDetectionDuration(const uint8_t duration);        // -
        bool getSlave2FIFOEnabled(void);        // -
        bool setSlave2FIFOEnabled(const bool enabled);        // -
        bool getSlave1FIFOEnabled(void);        // -
        bool setSlave1FIFOEnabled(const bool enabled);        // -
        bool getSlave0FIFOEnabled(void);        // -
        bool setSlave0FIFOEnabled(const bool enabled);        // -
        bool getMultiMasterEnabled(void);        // -
        bool setMultiMasterEnabled(const bool enabled);        // -
        bool getWaitForExternalSensorEnabled(void);        // -
        bool setWaitForExternalSensorEnabled(const bool enabled);        // -
        bool getSlave3FIFOEnabled(void);        // -
        bool setSlave3FIFOEnabled(const bool enabled);        // -

        bool getSlaveReadWriteTransitionEnabled(void);        // -
        bool setSlaveReadWriteTransitionEnabled(const bool enabled);        // -
        uint8_t getMasterClockSpeed(void);        // -
        bool setMasterClockSpeed(const uint8_t speed);        // -
        uint8_t getSlaveAddress(uint8_t num);        // -
        bool setSlaveAddress(uint8_t num, uint8_t address);        // -
        uint8_t getSlaveRegister(uint8_t num);        // -
        bool setSlaveRegister(uint8_t num, uint8_t reg);        // -
        bool getSlaveEnabled(uint8_t num);        // -
        bool setSlaveEnabled(uint8_t num, const bool enabled);        // -
        bool getSlaveWordByteSwap(uint8_t num);        // -
        bool setSlaveWordByteSwap(uint8_t num, const bool enabled);        // -
        bool getSlaveWriteMode(uint8_t num);        // -
        bool setSlaveWriteMode(uint8_t num, const bool mode);        // -
        bool getSlaveWordGroupOffset(uint8_t num);        // -
        bool setSlaveWordGroupOffset(uint8_t num, const bool enabled);        // -
        uint8_t getSlaveDataLength(uint8_t num);        // -
        bool setSlaveDataLength(uint8_t num, const uint8_t length);        // -
        uint8_t getSlave4Address(void);        // -
        bool setSlave4Address(const uint8_t address);        // -
        uint8_t getSlave4Register(void);        // -
        bool setSlave4Register(const uint8_t reg);        // -
        bool setSlave4OutputByte(const uint8_t data);        // -
        bool getSlave4Enabled(void);        // -
        bool setSlave4Enabled(const bool enabled);        // -
        bool getSlave4InterruptEnabled(void);        // -
        bool setSlave4InterruptEnabled(const bool enabled);        // -
        bool getSlave4WriteMode(void);        // -
        bool setSlave4WriteMode(const bool mode);        // -
        uint8_t getSlave4MasterDelay(void);        // -
        bool setSlave4MasterDelay(const uint8_t delay);        // -
        uint8_t getSlate4InputByte(void);        // -
        bool getPassthroughStatus(void);        // -
        bool getSlave4IsDone(void);        // -
        bool getLostArbitration(void);        // -
        bool getSlave4Nack(void);        // -
        bool getSlave3Nack(void);        // -
        bool getSlave2Nack(void);        // -
        bool getSlave1Nack(void);        // -
        bool getSlave0Nack(void);        // -
        bool getInterruptMode(void);        // -
        bool setInterruptMode(const bool mode);        // -
        bool getInterruptDrive(void);        // -
        bool setInterruptDrive(const bool drive);        // -
        bool getInterruptLatch(void);        // -
        bool setInterruptLatch(const bool latch);        // -
        bool getInterruptLatchClear(void);        // -
        bool setInterruptLatchClear(const bool clear);        // -
        bool getFSyncInterruptLevel(void);        // -
        bool setFSyncInterruptLevel(const bool level);        // -
        bool getFSyncInterruptEnabled(void);        // -
        bool setFSyncInterruptEnabled(const bool enabled);        // -
        bool getI2CBypassEnabled(void);        // -
        bool setI2CBypassEnabled(const bool enabled);        // -
        bool getClockOutputEnabled(void);        // -
        bool setClockOutputEnabled(const bool enabled);        // -
        uint8_t getIntEnabled(void);        // -
        bool setIntEnabled(const uint8_t enabled);        // -
        bool getIntFreefallEnabled(void);        // -
        bool setIntFreefallEnabled(const bool enabled);        // -
        bool getIntMotionEnabled(void);        // -
        bool setIntMotionEnabled(const bool enabled);        // -
        bool getIntZeroMotionEnabled(void);        // -
        bool setIntZeroMotionEnabled(const bool enabled);        // -
        bool getIntFIFOBufferOverflowEnabled(void);        // -
        bool setIntFIFOBufferOverflowEnabled(const bool enabled);        // -
        bool getIntI2CMasterEnabled(void);        // -
        bool setIntI2CMasterEnabled(const bool enabled);        // -
        bool getIntDataReadyEnabled(void);        // -
        bool setIntDataReadyEnabled(const bool enabled);        // -
        uint8_t getIntStatus(void);        // -
        bool getIntFreefallStatus(void);        // -
        bool getIntMotionStatus(void);        // -
        bool getIntZeroMotionStatus(void);        // -
        bool getIntFIFOBufferOverflowStatus(void);        // -
        bool getIntI2CMasterStatus(void);        // -
        bool getIntDataReadyStatus(void);        // -


        uint8_t getExternalSensorByte(int position);        // -
        uint16_t getExternalSensorWord(int position);        // -
        uint32_t getExternalSensorDWord(int position);        // -
        bool getXNegMotionDetected(void);        // -
        bool getXPosMotionDetected(void);        // -
        bool getYNegMotionDetected(void);        // -
        bool getYPosMotionDetected(void);        // -
        bool getZNegMotionDetected(void);        // -
        bool getZPosMotionDetected(void);        // -
        bool getZeroMotionDetected(void);        // -
        bool setSlaveOutputByte(uint8_t num, const uint8_t data);        // -
        bool getExternalShadowDelayEnabled(void);        // -
        bool setExternalShadowDelayEnabled(const bool enabled);        // -
        bool getSlaveDelayEnabled(const uint8_t num);        // -
        bool setSlaveDelayEnabled(const uint8_t num, const bool enabled);        // -
        uint8_t getFreefallDetectionCounterDecrement(void);        // -
        bool setFreefallDetectionCounterDecrement(const uint8_t decrement);        // -
        uint8_t getMotionDetectionCounterDecrement(void);        // -
        void setMotionDetectionCounterDecrement(const uint8_t decrement);        // -
        bool getFIFOEnabled(void);        // -
        bool setFIFOEnabled(const bool enabled);        // -
        bool getI2CMasterModeEnabled(void);        // -
        bool setI2CMasterModeEnabled(const bool enabled);        // -
        bool switchSPIEnabled(const bool enabled);        // -
        bool resetFIFO(void);        // -
        bool resetI2CMaster(void);        // -
        bool resetSensors(void);        // -
        bool reset(void);        // -
        bool getSleepEnabled(bool& out);
        bool setSleepEnabled(bool enabled);
        bool getWakeCycleEnabled(void);        // -
        bool setWakeCycleEnabled(const bool enabled);        // -
        bool getClockSource(uint8_t& out);
        bool setClockSource(uint8_t source);
        uint8_t getWakeFrequency(void);        // -
        bool setWakeFrequency(const uint8_t frequency);        // -
        uint16_t getFIFOCount(void);        // -
        uint8_t getFIFOByte(void);        // -
        bool setFIFOByte(const uint8_t data);        // -



        uint8_t getOTPBankValid(void);        // -
        bool setOTPBankValid(const bool enabled);        // -
        int8_t getXFineGain(void);        // -
        bool setXFineGain(const int8_t gain);        // -
        int8_t getYFineGain(void);        // -
        bool setYFineGain(const int8_t gain);        // -
        int8_t getZFineGain(void);        // -
        bool setZFineGain(const int8_t gain);        // -
        bool getIntPLLReadyEnabled(void);        // -
        bool setIntPLLReadyEnabled(const bool enabled);        // -
        bool getIntDMPEnabled(void);        // -
        bool setIntDMPEnabled(const bool enabled);        // -
        bool getDMPInt5Status(void);        // -
        bool getDMPInt4Status(void);        // -
        bool getDMPInt3Status(void);        // -
        bool getDMPInt2Status(void);        // -
        bool getDMPInt1Status(void);        // -
        bool getDMPInt0Status(void);        // -
        bool getIntPLLReadyStatus(void);        // -
        bool getIntDMPStatus(void);        // -
        bool getDMPEnabled(void);        // -
        bool setDMPEnabled(const bool enabled);        // -
        bool resetDMP(void);        // -
        bool setMemoryBank(uint8_t bank, const bool prefetchEnabled, const bool userBank);        // -
        bool setMemoryStartAddress(const uint8_t address);        // -
        uint8_t readMemoryByte(void);        // -
        bool writeMemoryByte(uint8_t data);        // -
        void readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);        // -
        bool writeMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem);        // -
        bool writeProgMemoryBlock(const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify);        // -
        bool writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem);        // -
        bool writeProgDMPConfigurationSet(const uint8_t *data, const uint16_t dataSize);        // -
        uint8_t getDMPConfig1(void);        // -
        bool setDMPConfig1(uint8_t config);        // -
        uint8_t getDMPConfig2(void);        // -
        bool setDMPConfig2(uint8_t config);        // -

        uint8_t getDeviceID(void);        // -
        bool testConnection();        // -
        void selectClock(uint8_t clock_type);        // -

        //acceleration functions
        bool getAccelXSelfTest(void);        // -
        bool setAccelXSelfTest(const uint8_t enabled);        // -
        bool getAccelYSelfTest(void);        // -
        bool setAccelYSelfTest(const uint8_t enabled);        // -
        bool getAccelZSelfTest(void);        // -
        bool setAccelZSelfTest(const uint8_t enabled);        // -
        bool getFullScaleAccelRange(uint8_t& out);
        bool setFullScaleAccelRange(uint8_t range);

        bool getAccelFIFOEnabled(void);        // -
        bool setAccelFIFOEnabled(const bool enabled);        // -

        int16_t getAccelerationX(void);        // -
        int16_t getAccelerationY(void);        // -
        int16_t getAccelerationZ(void);        // -
        void getAcceleration(int16_t* ax, int16_t* ay, int16_t* az);        // -

        bool resetAccelerometerPath(void);        // -
        uint8_t getAccelerometerPowerOnDelay(void);        // -
        bool setAccelerometerPowerOnDelay(const uint8_t delay);        // -

        int16_t getXAccelOffset(void);        // -
        bool setXAccelOffset(const int16_t offset);        // -
        int16_t getYAccelOffset(void);        // -
        bool setYAccelOffset(const int16_t offset);        // -
        int16_t getZAccelOffset(void);        // -
        bool setZAccelOffset(const int16_t offset);        // -

        bool getStandbyXAccelEnabled(void);        // -
        bool setStandbyXAccelEnabled(const bool enabled);        // -
        bool getStandbyYAccelEnabled(void);        // -
        bool setStandbyYAccelEnabled(const bool enabled);        // -
        bool getStandbyZAccelEnabled(void);        // -
        bool setStandbyZAccelEnabled(const bool enabled);        // -

        bool enableAccelerometerAxis(const uint8_t axis);        // -
        bool disableAccelerometerAxis(const uint8_t axis);        // -
        bool enableAccelerometer(void);        // -
        bool disableAccelerometer(void);        // -

        void getAccelerometerTestData(uint8_t ax, uint8_t ay, uint8_t az);        // -

        bool accelerometerXIsEnabled(void);        // -
        bool accelerometerYIsEnabled(void);        // -
        bool accelerometerZIsEnabled(void);        // -

        // Gyroscope functions
        bool getFullScaleGyroRange(uint8_t& out);
        bool setFullScaleGyroRange(uint8_t range);

        bool getXGyroFIFOEnabled(void);        // -
        bool setXGyroFIFOEnabled(const bool enabled);        // -
        bool getYGyroFIFOEnabled(void);        // -
        bool setYGyroFIFOEnabled(const bool enabled);        // -
        bool getZGyroFIFOEnabled(void);        // -
        bool setZGyroFIFOEnabled(const bool enabled);        // -

        bool resetGyroscopePath(void);        // -

        bool getStandbyXGyroEnabled(void);        // -
        bool setStandbyXGyroEnabled(const bool enabled);        // -
        bool getStandbyYGyroEnabled(void);        // -
        bool setStandbyYGyroEnabled(const bool enabled);        // -
        bool getStandbyZGyroEnabled(void);        // -
        bool setStandbyZGyroEnabled(const bool enabled);        // -

        int16_t getXGyroOffsetUser(void);        // -
        void setXGyroOffsetUser(const int16_t offset);        // -
        int16_t getYGyroOffsetUser(void);        // -
        void setYGyroOffsetUser(const int16_t offset);        // -
        int16_t getZGyroOffsetUser(void);        // -
        void setZGyroOffsetUser(const int16_t offset);        // -

        int16_t getXGyroOffset(void);        // -
        bool setXGyroOffset(const int16_t offset);        // -
        int16_t getYGyroOffset(void);        // -
        bool setYGyroOffset(const int16_t offset);        // -
        int16_t getZGyroOffset(void);        // -
        bool setZGyroOffset(const int16_t offset);        // -

        int16_t getRotationX(void);        // -
        int16_t getRotationY(void);        // -
        int16_t getRotationZ(void);        // -
        void getRotation(int16_t* gx, int16_t* gy, int16_t* gz);        // -
        bool enableGyroAxis(const uint8_t axis);        // -
        bool disableGyroAxis(const uint8_t axis);        // -
        bool enableGyro(void);        // -
        bool disableGyro(void);        // -

        void getGyroTestData(uint8_t gx, uint8_t gy, uint8_t gz);        // -
        bool gyroXIsEnabled(void);        // -
        bool gyroYIsEnabled(void);        // -
        bool gyroZIsEnabled(void);        // -

        void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);        // -
        void getMotion9(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* mx, int16_t* my, int16_t* mz);        // -

        //temperature functions
        bool getTempFIFOEnabled(void);        // -
        bool setTempFIFOEnabled(const bool enabled);        // -

        bool enableTemperature(void);        // -
        bool disableTemperature(void);        // -
        bool temperatureIsEnabled(void);        // -
        int16_t getTemperature(void);        // -
        bool resetTemperaturePath(void);        // -

    protected:
        template<typename T>
        bool readRegister(T& t);

        template<typename T>
        bool writeRegister(T& t);

    private:
        template<typename T, typename F>
        bool updateRegister(F func);        // Captured lamdas are not functions =\

        template<typename T, typename F>
        bool parseRegister(F func);

        I2Cdev _i2c;
        uint8_t _mpuAddress;
        uint8_t _magAddress;
        uint8_t _whoami;
        int16_t _temperature;
};

#endif /* _MPU9250_H_ */
