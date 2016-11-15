#include "error_event.h"

#define TWI_ERR_BIT     7
#define MPU_ERR_BIT     6
#define ACC_ERR_BIT     5
#define GYR_ERR_BIT     4
#define MAG_ERR_BIT     3
#define ADC_ERR_BIT     2
#define HRM_ERR_BIT     1
#define UART_ERR_BIT    0

#define ANOMALY_EVT_BIT (0 + 8)
#define WEARING_EVT_BIT (3 + 8)

static uint16_t evt_err = ~0;

void set_twi_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << TWI_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << TWI_ERR_BIT;

    set_mpu_error(1);
}

void set_mpu_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << MPU_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << MPU_ERR_BIT;

    set_acc_error(1);
    set_gyr_error(1);
    set_mag_error(1);
}

void set_acc_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << ACC_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << ACC_ERR_BIT;
}

void set_gyr_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << GYR_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << GYR_ERR_BIT;
}

void set_mag_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << MAG_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << MAG_ERR_BIT;
}

void set_adc_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << ADC_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << ADC_ERR_BIT;

    set_hrm_error(1);
}

void set_hrm_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << HRM_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << HRM_ERR_BIT;
}

void set_uart_error(uint8_t set)
{
    if (set)
        evt_err &= ~(((uint16_t)(1)) << UART_ERR_BIT);
    else
        evt_err |= ((uint16_t)(1)) << UART_ERR_BIT;
}

void set_anomaly_event(uint8_t evt)
{
    if (evt <= ANOMALY_UNKNOWN) {
        evt_err &= ~(((uint16_t)(0b111)) << ANOMALY_EVT_BIT);
        evt_err |= ((uint16_t)evt) << ANOMALY_EVT_BIT;
    }
}

void set_wearing_event(uint8_t evt)
{
    if (evt <= WEARING_UNKNOWN) {
        evt_err &= ~(((uint16_t)(0b11)) << WEARING_EVT_BIT);
        evt_err |= ((uint16_t)evt) << WEARING_EVT_BIT;
    }
}

uint16_t get_err_evt_status(void)
{
    return evt_err;
}
