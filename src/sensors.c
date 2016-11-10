#include <stdio.h>
#include "nrf_delay.h"

#include "sensors.h"
#include "mpu9250.h"

static nrf_adc_value_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint8_t isSampleCmplt = false;

uint8_t hrm_read(uint16_t *hrm)
{
    if (nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE) !=
            NRF_SUCCESS) {
        (*hrm) = 0;
    }
    else {
        nrf_drv_adc_sample();

        while (!isSampleCmplt);

        isSampleCmplt = false;
        (*hrm) = adc_buffer[0];
    }

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
            printf("%d\n", p_event->data.done.p_buffer[i]);
            adc_buffer[i] = p_event->data.done.p_buffer[i];
        }
    }
}

uint8_t init_sensors(void)
{
    uint8_t ret;

    nrf_delay_ms(1000);

    ret = mpuInit();

    if (ret != 0)
        return 1;

    return 0;
}
