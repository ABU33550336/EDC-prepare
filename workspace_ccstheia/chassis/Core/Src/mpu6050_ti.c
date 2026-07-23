#include "mpu6050_ti.h"
#include "ti_msp_dl_config.h"
#include "hw_adapter.h"
#include <stddef.h>

#define MPU6050_ADDR       0x68
#define MPU6050_WHO_AM_I   0x75
#define MPU6050_PWR_MGMT   0x6B
#define MPU6050_SMPL_DIV   0x19
#define MPU6050_CONFIG     0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_GYRO_X_H   0x43

#define GYRO_SCALE_DPS     250.0f / 32768.0f

static uint8_t g_mpu_inited = 0;

static void I2C_WaitIdle(void)
{
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {}
}

static void I2C_WriteBytes(uint8_t *data, uint8_t len)
{
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, data, len);
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    I2C_WaitIdle();
}

static void I2C_ReadBytes(uint8_t *buf, uint8_t len)
{
    DL_I2C_startControllerTransfer(I2C_0_INST, MPU6050_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    for (uint8_t i = 0; i < len; i++) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {}
        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
}

static uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *val)
{
    I2C_WriteBytes(&reg, 1);
    I2C_ReadBytes(val, 1);
    return 0;
}

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    I2C_WriteBytes(buf, 2);
    return 0;
}

static uint8_t MPU6050_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    I2C_WriteBytes(&reg, 1);
    I2C_ReadBytes(buf, len);
    return 0;
}

uint8_t MPU6050_Init(void)
{
    if (g_mpu_inited) return 0;

    uint8_t who = 0;
    MPU6050_ReadReg(MPU6050_WHO_AM_I, &who);
    if (who != MPU6050_ADDR) return 1;

    MPU6050_WriteReg(MPU6050_PWR_MGMT, 0x80);
    HW_DelayMs(100);
    MPU6050_WriteReg(MPU6050_PWR_MGMT, 0x00);
    HW_DelayMs(10);
    MPU6050_WriteReg(MPU6050_SMPL_DIV, 0x07);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);

    g_mpu_inited = 1;
    return 0;
}

uint8_t MPU6050_ReadGyroDPS(float *x, float *y, float *z)
{
    if (!g_mpu_inited) {
        if (MPU6050_Init()) return 1;
    }
    uint8_t buf[6];
    MPU6050_ReadMulti(MPU6050_GYRO_X_H, buf, 6);
    int16_t rx = (int16_t)(buf[0] << 8 | buf[1]);
    int16_t ry = (int16_t)(buf[2] << 8 | buf[3]);
    int16_t rz = (int16_t)(buf[4] << 8 | buf[5]);
    if (x) *x = (float)rx * GYRO_SCALE_DPS;
    if (y) *y = (float)ry * GYRO_SCALE_DPS;
    if (z) *z = (float)rz * GYRO_SCALE_DPS;
    return 0;
}
