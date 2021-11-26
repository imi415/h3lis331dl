#include "h3lis331dl.h"

/* Register Map */
#define H3LIS_REG_WHO_AM_I             0x0F

#define H3LIS_REG_CTRL_REG1            0x20
#define H3LIS_REG_CTRL_REG1_PM_Pos     5UL
#define H3LIS_REG_CTRL_REG1_PM_Msk     (7UL << H3LIS_REG_CTRL_REG1_PM_Pos)

#define H3LIS_REG_CTRL_REG2            0x21
#define H3LIS_REG_CTRL_REG2_BOOT_Pos   7UL
#define H3LIS_REG_CTRL_REG2_BOOT_Msk   (1UL << H3LIS_REG_CTRL_REG2_BOOT_Pos)

#define H3LIS_REG_CTRL_REG3            0x22
#define H3LIS_REG_CTRL_REG3_I1_CFG_Pos 0UL
#define H3LIS_REG_CTRL_REG3_I1_CFG_Msk (3UL << H3LIS_REG_CTRL_REG3_I1_CFG_Pos)
#define H3LIS_REG_CTRL_REG3_I2_CFG_Pos 3UL
#define H3LIS_REG_CTRL_REG3_I2_CFG_Msk (3UL << H3LIS_REG_CTRL_REG3_I2_CFG_Pos)

#define H3LIS_REG_CTRL_REG4            0x23
#define H3LIS_REG_CTRL_REG5            0x24

#define H3LIS_REG_HP_FILTER_RESET      0x25
#define H3LIS_REG_REFERENCE            0x26
#define H3LIS_REG_STATUS_REG           0x27

#define H3LIS_REG_OUT_X_L              0x28
#define H3LIS_REG_OUT_X_H              0x29
#define H3LIS_REG_OUT_Y_L              0x2A
#define H3LIS_REG_OUT_Y_H              0x2B
#define H3LIS_REG_OUT_Z_L              0x2C
#define H3LIS_REG_OUT_Z_H              0x2D

#define H3LIS_REG_INT1_CFG             0x30
#define H3LIS_REG_INT1_SRC             0x31
#define H3LIS_REG_INT1_THS             0x32
#define H3LIS_REG_INT1_DURATION        0x33

#define H3LIS_REG_INT2_CFG             0x34
#define H3LIS_REG_INT2_SRC             0x35
#define H3LIS_REG_INT2_THS             0x36
#define H3LIS_REG_INT2_DURATION        0x37

/* Device ID stored in WHO_AM_I */
#define H3LIS_DEVICE_ID 0x32

/* IRQ pad num */
#define H3LIS_IRQ_PAD1 0x01
#define H3LIS_IRQ_PAD2 0x02

typedef enum {
    H3LIS_IRQF_IRQ          = 0U,
    H3LIS_IRQF_IRQ_COMBINED = 1U,
    H3LIS_IRQF_DATA_READY   = 2U,
    H3LIS_IRQF_BOOT_RUNNING = 3U,
} h3lis331dl_irq_function_t;

/* Mystery error check function */
#define H3LIS_ERROR_CHECK(x) \
    if (x != H3LIS331DL_OK) return H3LIS331DL_FAIL

/* Private functions */

/**
 * @brief Read H3LIS331DL register
 *
 * @param accel pointer to h3lis331dl_t
 * @param reg register or the first register to be read
 * @param[out] data register value buffer
 * @param len register length in bytes
 * @return h3lis331dl_ret_t H3LIS331DL_OK for success, H3LIS331DL_FAIL for error
 */
static h3lis331dl_ret_t h3lis331dl_read_register(h3lis331dl_t *accel, uint8_t reg, uint8_t *data, uint8_t len) {
    if(len > 1) reg |= 0x80; /* !! SUB(7) MUST BE 1 for continuous read */
    H3LIS_ERROR_CHECK(accel->cb.i2c_write_cb(accel->user_data, accel->slave_addr, &reg, 0x01));
    H3LIS_ERROR_CHECK(accel->cb.i2c_read_cb(accel->user_data, accel->slave_addr, data, len));

    return H3LIS331DL_OK;
}

/**
 * @brief Write H3LIS331DL register
 *
 * @param accel pointer to h3lis331dl_t
 * @param reg register or the first register to be written
 * @param[in] data register value buffer
 * @param len register length in bytes
 * @return h3lis331dl_ret_t H3LIS331DL_OK for success, H3LIS331DL_FAIL for error
 */
static h3lis331dl_ret_t h3lis331dl_write_register(h3lis331dl_t *accel, uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t tx_buf[16];

    tx_buf[0] = reg;
    for(uint8_t i = 0; i < len; i++) {
        tx_buf[i + 1] = data[i];
    }

    H3LIS_ERROR_CHECK(accel->cb.i2c_write_cb(accel->user_data, accel->slave_addr, tx_buf, len + 1));

    return H3LIS331DL_OK;
}

/**
 * @brief Configure INT1 and INT2 pad function
 *
 * @param accel pointer to h3lis331dl_t
 * @param pad_num H3LIS_IRQ_PAD1 or H3LIS_IRQ_PAD2 pad number
 * @param function one of the h3lis331dl_irq_function_t functions
 * @return h3lis331dl_ret_t H3LIS331DL_OK for success, H3LIS331DL_FAIL for error
 */
static h3lis331dl_ret_t h3lis331dl_irq_pad_function(h3lis331dl_t *accel, uint8_t pad_num,
                                                    h3lis331dl_irq_function_t function) {
    uint8_t reg_buf;

    H3LIS_ERROR_CHECK(h3lis331dl_read_register(accel, H3LIS_REG_CTRL_REG3, &reg_buf, 0x01));

    if (pad_num == H3LIS_IRQ_PAD1) {
        reg_buf &= ~H3LIS_REG_CTRL_REG3_I1_CFG_Msk;
        reg_buf |= (function << H3LIS_REG_CTRL_REG3_I1_CFG_Pos) & H3LIS_REG_CTRL_REG3_I1_CFG_Msk;
    } else {
        reg_buf &= ~H3LIS_REG_CTRL_REG3_I2_CFG_Msk;
        reg_buf |= (function << H3LIS_REG_CTRL_REG3_I2_CFG_Pos) & H3LIS_REG_CTRL_REG3_I2_CFG_Msk;
    }

    H3LIS_ERROR_CHECK(h3lis331dl_write_register(accel, H3LIS_REG_CTRL_REG3, &reg_buf, 0x01));

    return H3LIS331DL_OK;
}

static h3lis331dl_ret_t h3lis331dl_set_mode(h3lis331dl_t *accel, h3lis331dl_power_mode_t mode) {
    uint8_t reg_buf;

    H3LIS_ERROR_CHECK(h3lis331dl_read_register(accel, H3LIS_REG_CTRL_REG1, &reg_buf, 0x01));

    reg_buf &= ~H3LIS_REG_CTRL_REG1_PM_Msk;
    reg_buf |= (mode << H3LIS_REG_CTRL_REG1_PM_Pos) & H3LIS_REG_CTRL_REG1_PM_Msk;

    H3LIS_ERROR_CHECK(h3lis331dl_write_register(accel, H3LIS_REG_CTRL_REG1, &reg_buf, 0x01));

    return H3LIS331DL_OK;
}

/**
 * @brief Check communication parameters by reading WHO_AM_I register
 * and writing BOOT bit to soft-reset device
 * @param accel pointer to h3lis331dl_t
 * @return h3lis331dl_ret_t H3LIS331DL_OK for success, H3LIS331DL_FAIL for error
 */
static h3lis331dl_ret_t h3lis331dl_identify_and_reset(h3lis331dl_t *accel) {
    uint8_t reg_buf[7];

    /* Read WHO_AM_I and compare. */
    H3LIS_ERROR_CHECK(h3lis331dl_read_register(accel, H3LIS_REG_WHO_AM_I, reg_buf, 0x01));
    if (reg_buf[0] != H3LIS_DEVICE_ID) {
        return H3LIS331DL_FAIL;
    }

    /* Set BOOT bit */
    H3LIS_ERROR_CHECK(h3lis331dl_read_register(accel, H3LIS_REG_CTRL_REG2, reg_buf, 0x01));

    reg_buf[0] |= H3LIS_REG_CTRL_REG2_BOOT_Msk;
    H3LIS_ERROR_CHECK(h3lis331dl_write_register(accel, H3LIS_REG_CTRL_REG2, reg_buf, 0x01));

    /* Wait for initial calibration load complete. */
    do {
        H3LIS_ERROR_CHECK(h3lis331dl_read_register(accel, H3LIS_REG_CTRL_REG2, reg_buf, 0x01));
    } while (reg_buf[0] & H3LIS_REG_CTRL_REG2_BOOT_Msk);

    /* Power down sensor */
    H3LIS_ERROR_CHECK(h3lis331dl_set_mode(accel, H3LIS331DL_MODE_POWERDOWN));

    /* Read status register to clear DRDY flag */
    H3LIS_ERROR_CHECK(h3lis331dl_read_register(accel, H3LIS_REG_STATUS_REG, reg_buf, 0x07));

    return H3LIS331DL_OK;
}

/**
 * @brief Configure measurement parameters, including sample rate, filter and axis.
 *
 * @param accel pointer to h3lis331dl_t
 * @return h3lis331dl_ret_t H3LIS331DL_OK for success, H3LIS331DL_FAIL for error
 */
static h3lis331dl_ret_t h3lis331dl_config(h3lis331dl_t *accel) {
    //
    return H3LIS331DL_OK;
}

/**
 * @brief Initialize accelerometer driver.
 *
 * @param accel pointer to h3lis331dl_t
 * @return h3lis331dl_ret_t H3LIS331DL_OK for success, H3LIS331DL_FAIL for error
 */
h3lis331dl_ret_t h3lis331dl_init(h3lis331dl_t *accel) {
    /* Identify connected device */
    H3LIS_ERROR_CHECK(h3lis331dl_identify_and_reset(accel));

    /* INT1 for internal data ready use */
    H3LIS_ERROR_CHECK(h3lis331dl_irq_pad_function(accel, H3LIS_IRQ_PAD1, H3LIS_IRQF_DATA_READY));

    /* Set measure parameters */
    H3LIS_ERROR_CHECK(h3lis331dl_config(accel));

    return H3LIS331DL_OK;
}

h3lis331dl_ret_t h3lis331dl_measure(h3lis331dl_t *accel, h3lis331dl_result_t *results, uint32_t count) {
    uint8_t reg_buf[7];
    h3lis331dl_ret_t ret = H3LIS331DL_OK;

    // Bring the accelerometer out of Power down mode
    H3LIS_ERROR_CHECK(h3lis331dl_set_mode(accel, accel->config.sample_mode));

    for(uint32_t i = 0; i < count; i++) {
        ret = accel->cb.poll_irq_cb(accel->user_data, H3LIS331DL_INT1);
        if(ret != H3LIS331DL_OK) break;

        ret = h3lis331dl_read_register(accel, H3LIS_REG_STATUS_REG, reg_buf, 0x07);
        if(ret != H3LIS331DL_OK) break;

        results[i].accel_x = (reg_buf[2] << 8U) | reg_buf[1];
        results[i].accel_y = (reg_buf[4] << 8U) | reg_buf[3];
        results[i].accel_z = (reg_buf[6] << 8U) | reg_buf[5];
    }

    // Bring the accelerometer out of Power down mode
    H3LIS_ERROR_CHECK(h3lis331dl_set_mode(accel, H3LIS331DL_MODE_POWERDOWN));

    return ret;
}

h3lis331dl_ret_t h3lis331dl_measure_nonblocking(h3lis331dl_t *accel, h3lis331dl_result_available_cb cb) {
    //
    return H3LIS331DL_OK;
}
