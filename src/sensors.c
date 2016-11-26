#include <stdio.h>
#include <math.h>

#include "nrf_delay.h"
#include "sensors.h"
#include "mpu9250.h"
#include "error_event.h"

#define MARK_UP   0
#define MARK_DOWN 1

static nrf_adc_value_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint8_t isSampleCmplt = false;
static uint8_t *uart_error;
static uint8_t hrm_state = MARK_UP;
static uint8_t tim_psd   = 0;
static uint8_t tim_mkd   = 0;
static uint8_t is_human  = 0;

uint8_t hrm_read(uint16_t *hrm)
{
    if (nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE) !=
            NRF_SUCCESS) {
        (*hrm) = 0;

        return 1;
    }

    nrf_drv_adc_sample();

    while (!isSampleCmplt);

    isSampleCmplt = false;
    (*hrm) = adc_buffer[0];

    return 0;
}

uint8_t acc_read(int16_t *x, int16_t *y, int16_t *z)
{
    if (!mpuGetAccel(x, y, z))
        return 1;

    return 0;
}

uint8_t gyr_read(int16_t *x, int16_t *y, int16_t *z)
{
    if (!mpuGetGyro(x, y, z))
        return 1;

    return 0;
}

uint8_t mag_read(int16_t *x, int16_t *y, int16_t *z)
{
    if (!mpuGetMagnet(x, y, z))
        return 1;

    return 0;
}

void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE) {
        isSampleCmplt = true;
        uint32_t i;
        for (i = 0; i < p_event->data.done.size; i++) {
            /** if (!(*uart_error)) */
            /**     printf("%d\n", p_event->data.done.p_buffer[i]); */

            adc_buffer[i] = p_event->data.done.p_buffer[i];
        }
    }
}

uint8_t init_sensors(app_twi_t *twi_ins, uint8_t *ue)
{
    uint8_t ret;

    uart_error = ue;

    nrf_delay_ms(1000);

    ret = mpuInit(twi_ins);

    if (ret != 0)
        return 1;

    return 0;
}

static uint8_t is_wearing(
        uint16_t amp,
        uint8_t *state,
        uint8_t *tp,
        uint8_t *tm,
        uint8_t *ih)
{
    uint8_t ret, tim_dis;

    ret = 0;

    switch (*state) {
        case MARK_UP:
            if (amp > 500) {
                tim_dis = (*tp) - (*tm);
                (*tm) = tim_dis + (*tm); // tim_mkd = time_psd;
                (*state) = MARK_DOWN;

                /** if (!(*uart_error)) */
                /**     printf("Marked tim_dis: %u\n\r", tim_dis); */

                if (tim_dis >= 7 && tim_dis < 14) {
                    ret = 1; // absolutely human
                    (*ih) = 1;
                } else {
                    (*ih) = 0;
                }
            } else if (*ih) {
                tim_dis = (*tp) - (*tm);

                if (tim_dis >= 0 && tim_dis < 14)
                    ret = 1;
            } else {
                (*ih) = 0;
            }
            break;
        case MARK_DOWN:
            if (amp < 500)
                (*state) = MARK_UP;

            ret = 2;
            break;
    }

    return ret;
}

static double raw_to_gforce(uint8_t range, int16_t val)
{
    double ret;
    int8_t sign;
    int16_t edge;

    if (range != 2 && range != 4 && range != 8 && range != 16)
        return 0.0;

    sign = (val < 0 ? -1 : 1);
    edge = (val < 0 ? 0x8000 : 0x7FFF);

    ret = (sign * range * ((double) val)) / edge;
    /** ret = (sign * range * ((double) val)); */
    /** ret = (sign * range); */
    /** ret = (sign); */
    /** ret = 1.0; */

    /** printf("ret = %f\n", ret); */

    return ret;
}

static double raw_to_angular_velocity(uint16_t range, int16_t val)
{
    double ret;
    int8_t sign;
    int16_t edge;

    if (range != 250 && range != 500 && range != 1000 && range != 2000)
        return 0.0;

    sign = (val < 0 ? -1 : 1);
    edge = (val < 0 ? 0x8000 : 0x7FFF);

    ret = (sign * range * ((double) val)) / edge;

    return ret;
}

void sensor_routine(void)
{
    uint16_t hrm;
    int16_t x, y, z;
    double xg, yg, zg, roll, pitch;

    ++tim_psd;

    if (hrm_read(&hrm) == 0) {
        switch (is_wearing(
                    hrm,
                    &hrm_state,
                    &tim_psd,
                    &tim_mkd,
                    &is_human)) {
            case 1:
                set_wearing_event(WEARING_YES);
                /** if (!(*uart_error)) */
                /**     printf("\thuman\n\r"); */
                break;
            case 0:
                set_wearing_event(WEARING_NO);
                /** if (!(*uart_error)) */
                /**     printf("\tmutant\n\r"); */
            default:
                break;
        }
    } else {
        set_wearing_event(WEARING_UNKNOWN);
    }

    if (!mpuGetAccel(&x, &y, &z)) {
        set_anomaly_event(ANOMALY_UNKNOWN);
        return;
    }

    xg = raw_to_gforce(16, x);
    yg = raw_to_gforce(16, y);
    zg = raw_to_gforce(16, z);

    roll    = (atan2(yg, zg) * 180.0) / M_PI;
    pitch   = (atan2(xg, sqrt((yg * yg) + (zg * zg))) * 180.0) / M_PI;

    if (xg > ACC_THR || xg < -ACC_THR ||
            yg > ACC_THR || yg < -ACC_THR ||
            zg > ACC_THR || zg < -ACC_THR) {
        /** if (!(*uart_error)) */
        /**     printf("ANOMALY_OVR_ACC\n"); */
        set_anomaly_event(ANOMALY_OVR_ACC);
    } else if (roll > ANG_THR || roll < -ANG_THR ||
            pitch > ANG_THR || pitch < -ANG_THR) {
        /** if (!(*uart_error)) */
        /**     printf("ANOMALY_OVR_ANG\n"); */
        set_anomaly_event(ANOMALY_OVR_ANG);
    } else {
        if (!mpuGetGyro(&x, &y, &z)) {
            set_anomaly_event(ANOMALY_UNKNOWN);
            return;
        } else {
            xg = raw_to_angular_velocity(2000, x);
            yg = raw_to_angular_velocity(2000, y);
            zg = raw_to_angular_velocity(2000, z);

            if (xg > ANG_VEL_THR || xg < -ANG_VEL_THR ||
                    yg > ANG_VEL_THR || yg < -ANG_VEL_THR ||
                    zg > ANG_VEL_THR || zg < -ANG_VEL_THR) {

                /** if (!(*uart_error)) */
                /**     printf("ANOMALY_OVR_ANG_VEL\n"); */
                set_anomaly_event(ANOMALY_OVR_ANG_VEL);
            }
            else {
                /** if (!(*uart_error)) */
                /**     printf("ANOMALY_FINE\n"); */
                set_anomaly_event(ANOMALY_FINE);
            }
        }

    }
}
