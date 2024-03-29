#include <stdio.h>
#include <math.h>

#include "nrf_drv_gpiote.h"
#include "time.h"
#include "nrf_delay.h"
#include "sensors.h"
#include "mpu9250.h"
#include "error_event.h"
#include "circular_queue.h"

#define WINDOW_SIZE 64
#define PEAK_SAMPLE_SIZE 5
#define PIN_IN 2
#define PIN_GND 3

enum WAVE_STATE {
    CLIMBING,
    SLIDING
};

static struct queue window, peaks;
static nrf_adc_value_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint8_t is_sample_cmplt = false;
static uint8_t *uart_error;
static uint8_t is_on_obj = false;
static uint8_t hrm = 0;

uint8_t get_hrm(void)
{
    return hrm;
}

/* Photoplethysmogram from Photoplethysmograph */
static uint8_t ppg_read(uint16_t *ppg)
{
    if (nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE) !=
            NRF_SUCCESS) {
        (*ppg) = 0;

        return 1;
    }

    nrf_drv_adc_sample();

    while (!is_sample_cmplt);

    is_sample_cmplt = false;
    (*ppg) = adc_buffer[0];

    return 0;
}

uint8_t read_acc(int16_t *x, int16_t *y, int16_t *z)
{
    if (!mpuGetAccel(x, y, z))
        return 1;

    return 0;
}

uint8_t read_gyro(int16_t *x, int16_t *y, int16_t *z)
{
    if (!mpuGetGyro(x, y, z))
        return 1;

    return 0;
}

uint8_t read_mag(int16_t *x, int16_t *y, int16_t *z)
{
    if (!mpuGetMagnet(x, y, z))
        return 1;

    return 0;
}

void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE) {
        is_sample_cmplt = true;
        uint32_t i;
        for (i = 0; i < p_event->data.done.size; i++) {
            /** if (!(*uart_error)) */
            /**     printf("%d\n", p_event->data.done.p_buffer[i]); */

            adc_buffer[i] = p_event->data.done.p_buffer[i];
        }
    }
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (nrf_gpio_pin_read(pin)) {
        /* if (!(*uart_error)) */
        /*     printf("Release!!\n\r"); */

        is_on_obj = false;
    } else {
        /* if (!(*uart_error)) */
        /*     printf("Press!!\n\r"); */

        is_on_obj = true;
    }
    /* nrf_drv_gpiote_out_toggle(PIN_OUT); */
}

static uint8_t gpiote_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();

    if (err_code != NRF_SUCCESS)
        return 1;

    /* nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false); */

    /* err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config); */
    /* APP_ERROR_CHECK(err_code); */

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);

    if (err_code != NRF_SUCCESS)
        return 2;

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);

    return 0;
}

uint8_t init_sensors(app_twi_t *twi_ins, uint8_t *ue)
{
    uint8_t ret;

    nrf_gpio_cfg_output(PIN_GND);
    nrf_gpio_pin_clear(PIN_GND);

    gpiote_init();

    init_queue(&window, WINDOW_SIZE);
    init_queue(&peaks, PEAK_SAMPLE_SIZE);

    uart_error = ue;

    nrf_delay_ms(1000);

    ret = mpuInit(twi_ins);

    if (ret != 0)
        return 1;

    return 0;
}

//Low pass butterworth filter order=1 alpha1=0.01
static uint16_t butterworth_lpf(uint16_t x)
{
    static float v[2] = {0};

    v[0] = v[1];
    v[1] = (3.046874709125380054e-2 * x) + (0.93906250581749239892 * v[0]);

    return (uint16_t)(v[0] + v[1]);
}

static uint16_t slope_sum(struct queue *q, uint16_t x)
{
    static uint32_t last_sum = 0;
    uint16_t ssf, garbage;
    int16_t lost, gain;

    garbage = enqueue(q, x);

    lost = q->tail->val - garbage;
    lost = (lost < 0 ? 0 : lost);

    gain = q->head->val - q->head->next->val;
    gain = (gain < 0 ? 0 : gain);

    ssf = last_sum - lost + gain;
    last_sum = ssf;

    return ssf;
}

/* expected that a caller of this procedure will call enqueue(&peaks, x) */
static uint8_t is_peak(uint16_t x, uint16_t last_peak)
{
    static uint16_t last_x = 0;
    static uint32_t t_last_peak = 0;
    static enum WAVE_STATE state = SLIDING;
    uint8_t peak = 0;
    int16_t diff_x = x - last_x;

    switch (state) {
        case CLIMBING:
            if (diff_x <= 0) {
                state = SLIDING;

                if (x > last_peak ||
                        last_peak - x <= 20 ||
                        get_millis() - t_last_peak > 3000) {
                    peak = 1;
                    t_last_peak = get_millis();
                    /** enqueue(&peaks, x); */
                }
            }
            break;

        case SLIDING:
            if (diff_x >= 0)
                state = CLIMBING;
            break;

        default:
            break;
    }

    last_x = x;

    return peak;
}

static uint16_t calculate_treshold(struct queue *q, uint16_t x)
{
    static float treshold = 0;

    if (is_peak(x, q->head->val)) {
        treshold *= PEAK_SAMPLE_SIZE;
        treshold += (int16_t)(x - enqueue(q, x));
        treshold /= PEAK_SAMPLE_SIZE;
    }

    return (uint16_t)((treshold/100) * 70);
}

static uint8_t calculate_bpm(uint16_t treshold, uint16_t x)
{
    static enum WAVE_STATE state = SLIDING;
    static uint32_t t_fst_peak = 0;
    static uint8_t count = 0, bpm = 0;

    switch (state) {
        case CLIMBING:
            if (x < treshold) {
                state = SLIDING;

                if (count == 0) {
                    t_fst_peak = get_millis();
                } else {
                    bpm = (uint8_t)(count /
                            ((get_millis() - t_fst_peak) / (1000 * 60.0)));
                    count = 0;
                    break;
                }
                ++count;
            }
            break;

        case SLIDING:
            if (x > treshold)
                state = CLIMBING;
            break;

        default:
            break;
    }

    if (get_millis() - t_fst_peak > 3000)
        bpm = 0;
    else
        hrm = bpm;

    return bpm;
}

static uint8_t is_wearing(uint8_t obj, uint8_t bpm)
{
    static uint8_t wearing = 0;

    if (obj) {
        if (bpm >= 60 && bpm <= 100) {
            wearing = 1;
        }
    } else {
        wearing = 0;
    }

    return wearing;
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
    uint8_t bpm;
    uint16_t ppg, ssf, treshold;
    int16_t x, y, z;
    double xg, yg, zg, roll, pitch;

    if (ppg_read(&ppg) == 0) {
        ssf = slope_sum(&window, butterworth_lpf(ppg));
        treshold = calculate_treshold(&peaks, ssf);
        bpm = calculate_bpm(treshold, ssf);

        switch (is_wearing(is_on_obj, bpm)) {
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

    roll    = (atan2(-yg, zg) * 180.0) / M_PI;
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
