#include "mpu6500.h"
#include "abs_config.h"
#include <string.h>

/* ---- registers ---------------------------------------------------------- */

#define REG_SMPLRT_DIV     0x19u
#define REG_CONFIG         0x1Au
#define REG_GYRO_CONFIG    0x1Bu
#define REG_ACCEL_CONFIG   0x1Cu
#define REG_ACCEL_CONFIG2  0x1Du
#define REG_INT_PIN_CFG    0x37u
#define REG_ACCEL_XOUT_H   0x3Bu
#define REG_PWR_MGMT_1     0x6Bu
#define REG_PWR_MGMT_2     0x6Cu
#define REG_WHO_AM_I       0x75u

#define WHOAMI_MPU6500     0x70u
#define WHOAMI_MPU9250     0x71u   /* same core, an MPU-6500 with a compass */

#define I2C_TIMEOUT_MS     20u

/* Full-scale ranges.
 *
 * +/- 4 g: a small vehicle braking at 1.2 g with the odd sharp knock has
 * plenty of headroom, and the 8192 LSB/g sensitivity is twice what the 8 g
 * range would give. Going to 2 g would clip on any decent bump.
 *
 * +/- 500 deg/s: only pitch rate matters here and a body dives at a few tens
 * of degrees per second, so this is already generous. */
#define ACCEL_FS_4G        (1u << 3)   /* AFS_SEL = 1 */
#define GYRO_FS_500DPS     (1u << 3)   /* FS_SEL  = 1 */

#define ACCEL_LSB_PER_G    8192.0f
#define GYRO_LSB_PER_DPS   65.5f

#define DEG_TO_RAD         0.01745329252f

static HAL_StatusTypeDef write_i2c(mpu6500_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT,
                             &val, 1, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_i2c(mpu6500_t *dev, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT,
                            val, 1, I2C_TIMEOUT_MS);
}

mpu6500_status_t mpu6500_init(mpu6500_t *dev, I2C_HandleTypeDef *hi2c,
                              uint16_t addr_8bit)
{
    uint8_t who = 0;

    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;
    dev->addr = addr_8bit;

    dev->accel_scale = ABS_G_MPS2 / ACCEL_LSB_PER_G;
    dev->gyro_scale  = DEG_TO_RAD / GYRO_LSB_PER_DPS;

    if (HAL_I2C_IsDeviceReady(hi2c, addr_8bit, 3, I2C_TIMEOUT_MS) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }

    /* Full device reset, then wait for it to come back. The datasheet asks for
     * 100 ms; the register self-clears sooner but there is no reason to race
     * it during start-up. */
    if (write_i2c(dev, REG_PWR_MGMT_1, 0x80u) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }
    HAL_Delay(100);

    if (read_i2c(dev, REG_WHO_AM_I, &who) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }
    if (who != WHOAMI_MPU6500 && who != WHOAMI_MPU9250) {
        /* Almost always a genuine MPU-6050 sold as a 6500, which answers 0x68.
         * It is a different part with different registers, and silently
         * carrying on would produce plausible nonsense rather than an error. */
        return MPU6500_ERR_WHOAMI;
    }

    /* Clock from the gyro X PLL rather than the internal oscillator: it is the
     * more stable reference and it is what the sample timing derives from. */
    if (write_i2c(dev, REG_PWR_MGMT_1, 0x01u) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }
    HAL_Delay(10);

    if (write_i2c(dev, REG_PWR_MGMT_2, 0x00u) != HAL_OK) {   /* all axes on */
        return MPU6500_ERR_BUS;
    }

    /* Gyro DLPF at 92 Hz, 3.9 ms group delay.
     *
     * This is the trade that matters on this sensor. A wider filter passes
     * motor and gearbox vibration into the pitch rate; a narrower one delays
     * it. 3.9 ms of delay against ABS cycles of 100-200 ms is acceptable, and
     * the complementary filter is integrating this signal rather than
     * differentiating it, so a little smoothing costs nothing. */
    if (write_i2c(dev, REG_CONFIG, 0x02u) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }

    /* With DLPF enabled the internal rate is 1 kHz, so a divider of 0 gives
     * exactly the loop rate and every tick gets a fresh sample. */
    if (write_i2c(dev, REG_SMPLRT_DIV, 0x00u) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }

    if (write_i2c(dev, REG_GYRO_CONFIG, GYRO_FS_500DPS) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }
    if (write_i2c(dev, REG_ACCEL_CONFIG, ACCEL_FS_4G) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }
    /* Accelerometer DLPF at 99 Hz, 2.9 ms. */
    if (write_i2c(dev, REG_ACCEL_CONFIG2, 0x02u) != HAL_OK) {
        return MPU6500_ERR_BUS;
    }

    dev->busy  = 0;
    dev->fresh = 0;
    return MPU6500_OK;
}

mpu6500_status_t start_read(mpu6500_t *dev)
{
    if (dev->busy) {
        /* The previous transfer has not finished. Something is wrong with the
         * bus - a missing pull-up will do it - and starting another would just
         * queue errors. */
        return MPU6500_ERR_BUSY;
    }

    dev->busy = 1;
    if (HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->addr, REG_ACCEL_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, dev->raw,
                             sizeof(dev->raw)) != HAL_OK) {
        dev->busy = 0;
        dev->errors++;
        return MPU6500_ERR_BUS;
    }
    return MPU6500_OK;
}

static int16_t be16(const uint8_t *p)
{
    /* The device is big-endian and the STM32 is not. Assembling by hand rather
     * than casting avoids both the byte order bug and the alignment one. */
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

mpu6500_status_t get_dma(mpu6500_t *dev, imu_sample_t *out)
{
    uint8_t buf[14];

    if (!dev->fresh) {
        return MPU6500_ERR_STALE;
    }

    /* Copy before clearing the flag, so a DMA completion landing in the middle
     * of this cannot hand back half of one sample and half of the next.
     *
     * This is belt and braces: it also relies on the DMA and I2C interrupts
     * being configured BELOW the control tick in the NVIC, so a completion
     * cannot preempt this function at all. If those priorities are ever inverted, this copy is the only
     * thing standing between the fusion filter and a torn sample. */
    memcpy(buf, dev->raw, sizeof(buf));
    dev->fresh = 0;

    /* Layout: ax ay az temp gx gy gz, big-endian 16-bit each. Temperature at
     * bytes 6..7 is skipped. */
    out->ax = (float)be16(&buf[0])  * dev->accel_scale;
    out->ay = (float)be16(&buf[2])  * dev->accel_scale;
    out->az = (float)be16(&buf[4])  * dev->accel_scale;

    out->gx = (float)be16(&buf[8])  * dev->gyro_scale;
    out->gy = (float)be16(&buf[10]) * dev->gyro_scale;
    out->gz = (float)be16(&buf[12]) * dev->gyro_scale;

    return MPU6500_OK;
}

void dma_complete(mpu6500_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (hi2c != dev->hi2c) {
        return;
    }
    dev->busy  = 0;
    dev->fresh = 1;
    dev->samples++;
}

void i2c_error(mpu6500_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (hi2c != dev->hi2c) {
        return;
    }
    /* Clear busy so the next tick can try again. Deliberately not setting
     * fresh: a failed transfer leaves the buffer holding whatever arrived
     * before it died, and feeding that to the fusion filter as if it were a
     * measurement is worse than skipping a tick. */
    dev->busy = 0;
    dev->errors++;
}
