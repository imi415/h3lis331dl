#ifndef H3LIS331DL_H
#define H3LIS331DL_H

#include <stdint.h>

typedef enum {
    H3LIS331DL_OK,
    H3LIS331DL_FAIL,
} h3lis331dl_ret_t;

typedef enum {
    H3LIS331DL_INT1 = 1U,
    H3LIS331DL_INT2 = 2U,
} h3lis331dl_int_t;

typedef enum {
    H3LIS331DL_MODE_POWERDOWN = 0U,
    H3LIS331DL_MODE_NORMAL    = 1U,
    H3LIS331DL_MODE_LP_0_5_HZ = 2U,
    H3LIS331DL_MODE_LP_1_HZ   = 3U,
    H3LIS331DL_MODE_LP_2_HZ   = 4U,
    H3LIS331DL_MODE_LP_5_HZ   = 5U,
    H3LIS331DL_MODE_LP_10_HZ  = 6U,
} h3lis331dl_power_mode_t;

typedef enum {
    H3LIS331DL_SR_50_HZ   = 0U,
    H3LIS331DL_SR_100_HZ  = 1U,
    H3LIS331DL_SR_400_HZ  = 2U,
    H3LIS331DL_SR_1000_HZ = 3U,
} h3lis331dl_sample_rate_t;

typedef enum {
    H3LIS331DL_RANGE_100G = 0U,
    H3LIS331DL_RANGE_200G = 1U,
    H3LIS331DL_RANGE_400G = 2U,
} h3lis331dl_range_t;

typedef enum {
    H3LIS331DL_HPF_MODE_NORMAL = 0U,
    H3LIS331DL_HPF_MODE_REF    = 1U,
} h3lis331dl_hpf_mode_t;

typedef enum {
    H3LIS331DL_HPCF_8  = 0U,
    H3LIS331DL_HPCF_16 = 1U,
    H3LIS331DL_HPCF_32 = 2U,
    H3LIS331DL_HPCF_64 = 3U,
} h3lis331dl_hpf_coeff_t;

typedef struct {
    h3lis331dl_ret_t (*i2c_read_cb)(void *handle, uint8_t addr, uint8_t *data, uint8_t len);
    h3lis331dl_ret_t (*i2c_write_cb)(void *handle, uint8_t addr, uint8_t *data, uint8_t len);
    h3lis331dl_ret_t (*poll_irq_cb)(void *handle, h3lis331dl_int_t irq);
} h3lis331dl_cb_t;
typedef struct {
    h3lis331dl_power_mode_t  sample_mode;
    h3lis331dl_sample_rate_t normal_mode_sample_rate;
    h3lis331dl_hpf_mode_t    hpf_mode;
    h3lis331dl_hpf_coeff_t   hpf_coeff;
    h3lis331dl_range_t       range;
} h3lis331dl_config_t;

typedef struct {
    void               *user_data;
    uint8_t             slave_addr;
    h3lis331dl_config_t config;
    h3lis331dl_cb_t     cb;
} h3lis331dl_t;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} h3lis331dl_result_t;

typedef h3lis331dl_ret_t (*h3lis331dl_result_available_cb)(void *handle, h3lis331dl_result_t result);

h3lis331dl_ret_t h3lis331dl_init(h3lis331dl_t *accel);
h3lis331dl_ret_t h3lis331dl_measure(h3lis331dl_t *accel, h3lis331dl_result_t *results, uint32_t count);
h3lis331dl_ret_t h3lis331dl_measure_nonblocking(h3lis331dl_t *accel, h3lis331dl_result_available_cb cb);

#endif