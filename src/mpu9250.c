#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "sdk_errors.h"
#include "app_twi.h"
#include "nrf_delay.h"
#include "mpu9250.h"

/** #define MPU_DEBUG */
#define MPU9250_ADDR            (0x68)
#define MAGNET_ADDR             (0x0C)

/** MPU-9250 Register's value */
#define ACCEL_FCHOICE_B         (0x08)
#define FCHOICE_B               (0x00)  /** Inverted of FCHOICE */

#define A_DLPF_CFG_460          (0x00 | ACCEL_FCHOICE_B)
#define A_DLPF_CFG_184          (0x01 | ACCEL_FCHOICE_B)
#define A_DLPF_CFG_92           (0x02 | ACCEL_FCHOICE_B)
#define A_DLPF_CFG_41           (0x03 | ACCEL_FCHOICE_B)
#define A_DLPF_CFG_20           (0x04 | ACCEL_FCHOICE_B)
#define A_DLPF_CFG_10           (0x05 | ACCEL_FCHOICE_B)
#define A_DLPF_CFG_5            (0x06 | ACCEL_FCHOICE_B)
/** #define A_DLPF_CFG_460   (0x07 | ACCEL_FCHOICE_B) */

/** Works only if FCHOICE_B is set to 0x00 */
#define G_DLPF_CFG_250          (0x00) 
#define G_DLPF_CFG_184          (0x01)
#define G_DLPF_CFG_92           (0x02)
#define G_DLPF_CFG_41           (0x03)
#define G_DLPF_CFG_20           (0x04)
#define G_DLPF_CFG_10           (0x05)
#define G_DLPF_CFG_5            (0x06)
#define G_DLPF_CFG_3600         (0x07)

#define GYRO_FS_250             (0x00 | FCHOICE_B)
#define GYRO_FS_500             (0x08 | FCHOICE_B)
#define GYRO_FS_1000            (0x10 | FCHOICE_B)
#define GYRO_FS_2000            (0x18 | FCHOICE_B)

#define ACCEL_FS_2              (0x00)
#define ACCEL_FS_4              (0x08)
#define ACCEL_FS_8              (0x10)
#define ACCEL_FS_16             (0x18)

#define EN_ACCEL_GYRO           (0x00)

#define EN_INT_WOM              (0x40)
#define EN_INT_FIFOV            (0x10)
#define EN_INT_FSYNC            (0x08)
#define EN_INT_RAW_R            (0x01)

#define EN_ACCEL_INTEL          (0xC0)

#define EN_I2C_MST              (0x20)
#define EN_BYPASS               (0x02)

#define EN_LATCH_INT            (0x20)
#define CLEAR_INT_ANYRD         (0x10)

#define MPU_RESET               (0x80)
#define MPU_POWER               (0x00)

/** MPU-9250 Registers' address */
#define REG_CONFIG              (0x1A)
#define REG_GYRO_CFG            (0x1B)
#define REG_ACCEL_CFG           (0x1C)
#define REG_ACCEL_CFG2          (0x1D)
#define REG_WOM_THR             (0x1F)

#define REG_INT_PIN_CFG         (0x37)
#define REG_INT_ENABLE          (0x38)

#define REG_ACCEL_XOUT_H        (0x3B)
#define REG_ACCEL_XOUT_L        (0x3C)
#define REG_ACCEL_YOUT_H        (0x3D)
#define REG_ACCEL_YOUT_L        (0x3E)
#define REG_ACCEL_ZOUT_H        (0x3F)
#define REG_ACCEL_ZOUT_L        (0x40)

#define REG_GYRO_XOUT_H         (0x43)
#define REG_GYRO_XOUT_L         (0x44)
#define REG_GYRO_YOUT_H         (0x45)
#define REG_GYRO_YOUT_L         (0x46)
#define REG_GYRO_ZOUT_H         (0x47)
#define REG_GYRO_ZOUT_L         (0x48)

#define REG_USER_CTRL           (0x6A)

#define REG_MOT_DETECT_CTRL     (0x69)
#define REG_PWR_MGMT_1          (0x6B)
#define REG_PWR_MGMT_2          (0x6C)

/** Magnetometer Register's value */
#define MAG_WIA                 (0x48)
#define MAG_POWER_DOWN          (0x00)
#define MAG_SING_MEASURE        (0x01)
#define MAG_CONT_MEASURE_1      (0x02)
#define MAG_EXT_MEASURE         (0x04)
#define MAG_CONT_MEASURE_2      (0x06)
#define MAG_SELF_TEST           (0x08)
#define MAG_FUSE_ROM            (0x0F)

/** Magnetometer Registers' address */
#define REG_MAG_WIA             (0x00)
#define REG_MAG_CNTL            (0x0A)

#define REG_MAG_ST_1            (0x02)
#define REG_MAG_XOUT_L          (0x03)
#define REG_MAG_XOUT_H          (0x04)
#define REG_MAG_YOUT_L          (0x05)
#define REG_MAG_YOUT_H          (0x06)
#define REG_MAG_ZOUT_L          (0x07)
#define REG_MAG_ZOUT_H          (0x08)
#define REG_MAG_ST_2            (0x09)

static uint8_t buffer[10];
extern app_twi_t m_app_twi;

static bool regRead(uint8_t regAddr)
{
    uint8_t size;
    uint8_t errCode;

    buffer[0] = regAddr;

    app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(MPU9250_ADDR, buffer, 1, 0),
        APP_TWI_READ(MPU9250_ADDR, buffer, 1, 0)
    };

    size = sizeof(transfers) / sizeof(transfers[0]);

    if ((errCode = app_twi_perform(&m_app_twi, transfers, size, NULL)) != NRF_SUCCESS) {
#ifdef MPU_DEBUG
        printf("TWI Perform Error: %d\n", errCode);
#endif
        return false;
    }

    return true;
}

static bool regWrite(uint8_t regAddr, uint8_t data)
{
    uint8_t size;
    uint8_t errCode;

    buffer[0] = regAddr;
    buffer[1] = data;

    app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(MPU9250_ADDR, buffer, 2, 0)
    };

    size = sizeof(transfers) / sizeof(transfers[0]);

    if ((errCode = app_twi_perform(&m_app_twi, transfers, size, NULL)) != NRF_SUCCESS) {
#ifdef MPU_DEBUG
        printf("TWI Perform Error: %d\n", errCode);
#endif

        return false;
    }

    return true;
}

static bool magRegRead(uint8_t regAddr)
{
    uint8_t size;
    uint8_t errCode;

    buffer[0] = regAddr;

    app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(MAGNET_ADDR, buffer, 1, 0),
        APP_TWI_READ(MAGNET_ADDR, buffer, 1, 0)
    };

    size = sizeof(transfers) / sizeof(transfers[0]);

    if ((errCode = app_twi_perform(&m_app_twi, transfers, size, NULL)) != NRF_SUCCESS) {
#ifdef MPU_DEBUG
        printf("MagRead Error: %d\n", errCode);
#endif
        return false;
    }

    return true;
}

static bool magRegWrite(uint8_t regAddr, uint8_t data)
{
    uint8_t size;
    uint8_t errCode;

    buffer[0] = regAddr;
    buffer[1] = data;

    app_twi_transfer_t const transfers[] =
    {
        APP_TWI_WRITE(MAGNET_ADDR, buffer, 2, 0)
    };

    size = sizeof(transfers) / sizeof(transfers[0]);

    if ((errCode = app_twi_perform(&m_app_twi, transfers, size, NULL)) != NRF_SUCCESS) {
#ifdef MPU_DEBUG
        printf("TWI Perform Error: %d\n", errCode);
#endif

        return false;
    }

    return true;
}

uint8_t enableBypass(bool enable)
{
    if (!regRead(REG_USER_CTRL))
        return 1;

    if (enable)
        buffer[0] &= ~EN_I2C_MST;
    else
        buffer[0] |= EN_I2C_MST;

    if (!regWrite(REG_USER_CTRL, buffer[0]))
        return 2;

    nrf_delay_ms(3);

    if (!regRead(REG_INT_PIN_CFG))
        return 3;

    if (enable)
        buffer[0] |= EN_BYPASS;
    else {
        buffer[0] &= ~EN_BYPASS;
        buffer[0] |= EN_LATCH_INT | CLEAR_INT_ANYRD;
    }

    if (!regWrite(REG_INT_PIN_CFG, buffer[0]))
        return 4;

    return 0;
}

uint8_t mpuInit(void)
{
    /* Reset device. */
    if (!regWrite(REG_PWR_MGMT_1, MPU_RESET))
        return 1;

    /** delay_ms(100); */
    nrf_delay_ms(100);

    /* Wake up chip. */
    if (!regWrite(REG_PWR_MGMT_1, MPU_POWER))
        return 2;

     /** Enable accelerometer and gyroscope */
    if (!regWrite(REG_PWR_MGMT_2, EN_ACCEL_GYRO))
        return 3;

    /** Set accelerometer LPF 184Hz */
    if (!regWrite(REG_ACCEL_CFG2, A_DLPF_CFG_184))
        return 4;

     /** Set Gyroscope full-scale range 2000dps */
    if (!regWrite(REG_GYRO_CFG, GYRO_FS_2000))
        return 5;

     /** Set Accelerometer full-scale range 16g */
    if (!regWrite(REG_ACCEL_CFG, ACCEL_FS_16))
        return 6;

    /** Set gyroscope (and temperature) LPF 184Hz */
    if (!regWrite(REG_CONFIG, G_DLPF_CFG_184))
        return 7;

    /** if (!regWrite(REG_RATE_DIV, (1000 / (50 - 1)))) */
    /**     return 10; */

    if (!regWrite(REG_INT_ENABLE, EN_INT_WOM))
        return 9;

    if (!regWrite(REG_MOT_DETECT_CTRL, EN_ACCEL_INTEL))
        return 10;

    /** Wake-on-Motion Threshold 0-255 (0-1020mg) */
    if (!regWrite(REG_WOM_THR, 100))
        return 11;

    if (enableBypass(true) != 0)
        return 12;

    return 0;
}

static int16_t readAccelX(uint8_t *status)
{
    int16_t ret;

    if (!regRead(REG_ACCEL_XOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!regRead(REG_ACCEL_XOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;
}

static int16_t readAccelY(uint8_t *status)
{
    int16_t ret;

    if (!regRead(REG_ACCEL_YOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!regRead(REG_ACCEL_YOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;
}

static int16_t readAccelZ(uint8_t *status)
{
    int16_t ret;

    if (!regRead(REG_ACCEL_ZOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!regRead(REG_ACCEL_ZOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;
}

static int16_t readGyroX(uint8_t *status)
{
    int16_t ret;

    if (!regRead(REG_GYRO_XOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!regRead(REG_GYRO_XOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;
}

static int16_t readGyroY(uint8_t *status)
{
    int16_t ret;

    if (!regRead(REG_GYRO_YOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!regRead(REG_GYRO_YOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;
}

static int16_t readGyroZ(uint8_t *status)
{
    int16_t ret;

    if (!regRead(REG_GYRO_ZOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!regRead(REG_GYRO_ZOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;
}

static int16_t readMagnetX(uint8_t *status)
{
    int16_t ret;

    if (!magRegRead(REG_MAG_XOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!magRegRead(REG_MAG_XOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;

}

static int16_t readMagnetY(uint8_t *status)
{
    int16_t ret;

    if (!magRegRead(REG_MAG_YOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!magRegRead(REG_MAG_YOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;
}

static int16_t readMagnetZ(uint8_t *status)
{
    int16_t ret;

    if (!magRegRead(REG_MAG_ZOUT_H)) {
        (*status) += 1;
        return 0;
    }

    ret = ((int16_t)buffer[0]) << 8;
    
    if (!magRegRead(REG_MAG_ZOUT_L)) {
        (*status) += 1;
        return 0;
    }

    ret |= buffer[0];

    return ret;
}

bool mpuGetAccel(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t status = 0;

    (*x) = readAccelX(&status);
    (*y) = readAccelY(&status);
    (*z) = readAccelZ(&status);

#ifdef MPU_DEBUG
    printf("DEBUG ACCEL: %d\n", status);
#endif

    if (status > 0)
        return false;

    return true;
}
bool mpuGetGyro(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t status = 0;

    (*x) = readGyroX(&status);
    (*y) = readGyroY(&status);
    (*z) = readGyroZ(&status);

#ifdef MPU_DEBUG
    printf("DEBUG GYRO: %d\n", status);
#endif

    if (status > 0)
        return false;

    return true;
}

bool mpuGetMagnet(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t status = 0;

    (*x) = readMagnetX(&status);
    (*y) = readMagnetY(&status);
    (*z) = readMagnetZ(&status);

#ifdef MPU_DEBUG
    printf("DEBUG MAGNET: %d\n", status);
#endif

    if (status > 0)
        return false;

    if (!magRegWrite(REG_MAG_CNTL, MAG_SING_MEASURE))
        return false;

    return true;
}

