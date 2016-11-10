#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include "nrf_drv_adc.h"

#define ADC_BUFFER_SIZE 1

uint8_t init_sensors(void);
uint8_t hrm_read(uint16_t *hrm);
uint8_t acc_read(int16_t *x, int16_t *y, int16_t *z);
uint8_t gyr_read(int16_t *x, int16_t *y, int16_t *z);
uint8_t mag_read(int16_t *x, int16_t *y, int16_t *z);
void adc_event_handler(nrf_drv_adc_evt_t const * p_event);
#endif /* ifndef SENSOR_H */
