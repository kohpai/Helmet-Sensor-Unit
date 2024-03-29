#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"

#include "ble.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"

#include "bsp.h"

#include "app_trace.h"
#include "app_uart.h"
#include "app_twi.h"
#include "app_error.h"
#include "app_util_platform.h"

#include "time.h"
#include "sensors.h"
#include "bluetooth.h"
#include "error_event.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT     1
#define CENTRAL_LINK_COUNT                  0
#define PERIPHERAL_LINK_COUNT               1

#define DEAD_BEEF                       0xDEADBEEF

#define MAX_PENDING_TRANSACTIONS    5
#define MAX_TEST_DATA_BYTES (15U)
#define UART_TX_BUF_SIZE    256
#define UART_RX_BUF_SIZE    1

#define PWR_PIN LED_0
#define ERR_PIN LED_1

static uint8_t  init_uart(void);
static uint8_t  init_adc(void);
static uint8_t  twi_config(app_twi_t *m_app_twi);
static void     power_manage(uint8_t uart_error);
static void     init_leds(void);

static nrf_drv_adc_channel_t m_channel_config =
    NRF_DRV_ADC_NATURAL_CHANNEL( NRF_ADC_CONFIG_INPUT_2);

int main(void)
{
    app_twi_t m_app_twi = APP_TWI_INSTANCE(0);
    uint8_t ret, uart_error;

    uart_error = 0;

    init_leds();
    nrf_gpio_pin_set(PWR_PIN);

    ret = init_uart();

    if (ret != 0) {
        nrf_gpio_pin_set(ERR_PIN);
        uart_error = 1;
        set_uart_error(1);
    }

    ret = init_adc();

    if (ret != 0) {
        if (!uart_error)
            printf("Cannot initialize ADC, Error: %u\n", ret);
    }

    ret = twi_config(&m_app_twi);

    if (ret != 0) {
        if (!uart_error)
            printf("Cannot initialize TWI, Error: %u\n", ret);

        set_twi_error(1);
    }

    ret = init_sensors(&m_app_twi, &uart_error);

    if (ret != 0) {
        if (!uart_error)
            printf("Cannot initialize sensors, Error: %u\n", ret);

        set_mpu_error(1);
    }

    ret = init_timers();

    while (ret != 0) {
        nrf_delay_ms(3000);

        if (!uart_error)
            printf("Trying to initialize timers\n");

        ret = init_timers();
    }

    ret = init_bluetooth(&uart_error);

    while (ret != 0) {
        nrf_delay_ms(3000);

        if (!uart_error)
            printf("Trying to initialize BLE\n");

        ret = init_bluetooth(&uart_error);
    }

    if (!uart_error)
        printf("Initialization's Done!!!!\n");

    while (true) {
        power_manage(uart_error);

        if (is_time()) {
            sensor_routine();
        }
    }
}

static void init_leds(void)
{
    nrf_gpio_cfg_output(PWR_PIN);
    nrf_gpio_cfg_output(ERR_PIN);
    nrf_gpio_cfg_output(CNTD_PIN);
    nrf_gpio_pins_clear(
            1 << PWR_PIN |
            1 << ERR_PIN |
            1 << CNTD_PIN);
}

static void uart_error_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type) {
        case APP_UART_COMMUNICATION_ERROR:
            LEDS_ON(BSP_LED_4_MASK);
            break;

        case APP_UART_FIFO_ERROR:
            LEDS_ON(BSP_LED_3_MASK);
            break;

        case APP_UART_DATA_READY:
        case APP_UART_TX_EMPTY:
        case APP_UART_DATA:
            break;
    }
}

/**
 * @brief UART initialization.
 */
static uint8_t init_uart(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

/**
 * @brief ADC initialization.
 */
static uint8_t init_adc(void)
{
    ret_code_t ret_code;
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;

    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    /** APP_ERROR_CHECK(ret_code); */
    if (ret_code != NRF_SUCCESS)
        return 1;

    nrf_drv_adc_channel_enable(&m_channel_config);

    return 0;
}

/**
 * @brief TWI initialization.
 */
static uint8_t twi_config(app_twi_t *m_app_twi)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    APP_TWI_INIT(m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product.
 *          You need to analyze how your product is supposed to react in
 *          case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void power_manage(uint8_t uart_error)
{
    uint32_t err_code = sd_app_evt_wait();

    if (err_code != NRF_SUCCESS)
        if (!uart_error)
            printf("power_manage: Cannot enter app event wait\n");
}

