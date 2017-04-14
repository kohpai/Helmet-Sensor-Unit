#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>

#include "nrf_drv_adc.h"
#include "app_twi.h"

#define ADC_BUFFER_SIZE 1

#define ACC_THR     (1.2)
#define ANG_THR     45
#define ANG_VEL_THR 100

uint8_t init_sensors(app_twi_t *twi_ins, uint8_t *ue);
uint8_t get_hrm(void);
uint8_t read_acc(int16_t *x, int16_t *y, int16_t *z);
uint8_t read_gyro(int16_t *x, int16_t *y, int16_t *z);
uint8_t read_mag(int16_t *x, int16_t *y, int16_t *z);
void adc_event_handler(nrf_drv_adc_evt_t const * p_event);
void sensor_routine(void);
#endif /* ifndef SENSOR_H */
