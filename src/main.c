#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_adc.h"

#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_advertising.h"
#include "softdevice_handler.h"
#include "device_manager.h"
#include "pstorage.h"

#include "boards.h"
#include "bsp.h"

#include "app_timer.h"
#include "app_trace.h"
#include "app_uart.h"
#include "app_twi.h"
#include "app_error.h"
#include "app_util_platform.h"

#include "mpu9250.h"
#include "helmet_sensor_unit_service.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT     1
#define CENTRAL_LINK_COUNT                  0
#define PERIPHERAL_LINK_COUNT               1

/**
 * The advertising interval (in units of 0.625 ms. 
 * This value corresponds to 25 ms). 
**/
#define APP_ADV_INTERVAL                480

/** The advertising timeout in units of seconds. 0 means no timeout */
#define APP_ADV_TIMEOUT_IN_SECONDS      0

/** Value of the RTC1 PRESCALER register.  */
#define APP_TIMER_PRESCALER             0

/** Size of timer operation queues.  */
#define APP_TIMER_OP_QUEUE_SIZE         4                                          


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)           
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)           
#define SLAVE_LATENCY                   0                                          
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)            

/** 
 * Time from initiating event (connect or start of notification) to 
 * first time sd_ble_gap_conn_param_update is called (5 seconds). 
**/
#define FIRST_CONN_PARAMS_UPDATE_DELAY  \
    APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) 

/** 
 * Time between each call to sd_ble_gap_conn_param_update after the 
 * first call (30 seconds). 
**/
#define NEXT_CONN_PARAMS_UPDATE_DELAY   \
    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)

/** Number of attempts before giving up the connection parameter negotiation. */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                          

/** Perform bonding. */
#define SEC_PARAM_BOND                  1

/** Man In The Middle protection not required. */
#define SEC_PARAM_MITM                  0

/** LE Secure Connections not enabled. */
#define SEC_PARAM_LESC                  0

/** Keypress notifications not enabled. */
#define SEC_PARAM_KEYPRESS              0

/** No I/O capabilities. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE

/** Out Of Band data not available. */
#define SEC_PARAM_OOB                   0

/** Minimum encryption key size. */
#define SEC_PARAM_MIN_KEY_SIZE          7

/** Maximum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                         

/** 
 * Value used as error code on stack dump, 
 * can be used to identify stack location on stack unwind. 
**/
#define DEAD_BEEF                       0xDEADBEEF                                 

/** max number of transactions that can be in the queue. */
#define MAX_PENDING_TRANSACTIONS    5

/**max number of test bytes to be used for tx and rx. */
#define MAX_TEST_DATA_BYTES (15U)

/**UART TX buffer size. */
#define UART_TX_BUF_SIZE    256

/**UART RX buffer size. */
#define UART_RX_BUF_SIZE    1

/**Size of buffer for ADC samples. */
#define ADC_BUFFER_SIZE     1

#define DEV_CODE            { 0x53, 0x48, 0x41, 0x44, 0x44, 0x52 ,0x00, 0x00 }
#define TEAM_CODE           0x0059

app_twi_t m_app_twi = APP_TWI_INSTANCE(0);

static ble_hcus_t                   m_our_service;
static uint16_t                     m_conn_handle = BLE_CONN_HANDLE_INVALID;   
static nrf_adc_value_t              adc_buffer[ADC_BUFFER_SIZE];
static nrf_drv_adc_channel_t        m_channel_config = NRF_DRV_ADC_NATURAL_CHANNEL( NRF_ADC_CONFIG_INPUT_2); 
static volatile bool                isSampleCmplt = false;

static uint8_t uart_config(void);
static void timers_init(void);
static uint8_t ble_stack_init(void);
static uint8_t device_manager_init(dm_application_instance_t *m_app_handle,
        bool erase_bonds);
static uint8_t gap_params_init(void);
static uint8_t advertising_init(void);
static uint8_t conn_params_init(void);
static void power_manage(void);
static void adc_config(void);
static void twi_config(app_twi_t *m_app_twi);

int main(void)
{
    dm_application_instance_t    m_app_handle;                              

    uint32_t    err_code;
    uint8_t     ret;

    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    ret = uart_config();

    if (ret != 0) {
        LEDS_ON(BSP_LED_2_MASK);
        goto EXIT_PROGRAM;
    }

    adc_config();
    twi_config(&m_app_twi);

    ret = mpuInit();

    if (ret != 0) {
        printf("Cannot initialize MPU, Error: %u\n", ret);
        goto EXIT_PROGRAM;
    }

    timers_init();
    ret = ble_stack_init();

    if (ret != 0) {
        printf("Cannot initialize BLE stack, Error: %u.\n", ret);
        goto EXIT_PROGRAM;
    }

    ret = device_manager_init(&m_app_handle, true);

    if (ret != 0) {
        printf("Cannot initialize DeviceManager, Error: %u\n", ret);
        goto EXIT_PROGRAM;
    }

    ret = gap_params_init();

    if (ret != 0) {
        printf("Cannot initialize GAP, Error: %u\n", ret);
        goto EXIT_PROGRAM;
    }

    ret = init_hcu_service(&m_our_service);    

    if (ret != 0) {
        printf("Cannot initialize HCU service, Error: %u\n", ret);
        goto EXIT_PROGRAM;
    }

    ret = advertising_init();

    if (ret != 0) {
        printf("Cannot initialize advertising, Error: %u\n", ret);
        goto EXIT_PROGRAM;
    }

    ret = conn_params_init();

    if (ret != 0) {
        printf("Cannot initialize ConnParam, Error: %u\n", ret);
        goto EXIT_PROGRAM;
    }

    err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);

    if (err_code != NRF_SUCCESS) {
        printf("Cannot start advertising.\n");
        goto EXIT_PROGRAM;
    }
    else
        printf("Start advertising ...\n");

    while (true) {
        /** printf("isSampleCmplt = %s\n", (isSampleCmplt ? "true":"false")); */
        power_manage();
    }

EXIT_PROGRAM:
    while (true) {  }
}

/** 
 * @brief UART error handler
 */
void uart_error_handle(app_uart_evt_t * p_event)
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
static uint8_t uart_config(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_ENABLED,
          false,
          UART_BAUDRATE_BAUDRATE_Baud9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    /** APP_ERROR_CHECK(err_code); */
    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}
                                   
/**
 * @brief ADC interrupt handler.
 */
static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        isSampleCmplt = true;
        uint32_t i;
        for (i = 0; i < p_event->data.done.size; i++) {
            /** printf("%d\n", p_event->data.done.p_buffer[i]); */
            adc_buffer[i] = p_event->data.done.p_buffer[i];
        }
    }
}

/**
 * @brief ADC initialization.
 */
static void adc_config(void)
{
    ret_code_t ret_code;
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;

    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    /** APP_ERROR_CHECK(ret_code); */
    if (ret_code != NRF_SUCCESS) {
        printf("ADC is already initialized.\n");
        while (true);
    }

    nrf_drv_adc_channel_enable(&m_channel_config);
}

/**
 * @brief TWI initialization.
 */
static void twi_config(app_twi_t *m_app_twi)
{
    uint32_t err_code;

    nrf_drv_twi_config_t const config = {
       .scl                = TWI_SCL_PIN,
       .sda                = TWI_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    APP_TWI_INIT(m_app_twi, &config, MAX_PENDING_TRANSACTIONS, err_code);

    if (err_code != NRF_SUCCESS) {
        printf("TWI initialization failed.\n");
        while (true);
    }
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

static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
}

static uint8_t gap_params_init(void)
{
    uint32_t                err_code;

    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);

    if (err_code != NRF_SUCCESS)
        return 1;
                                          
    // Change ADDR_TYPE from RANDOM_STATIC to RANDOM_PRIVATE_RESOLVABLE
    /** ble_gap_addr_t gap_address; */
    /** gap_address.addr_type   = BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE; */
    /** err_code                = sd_ble_gap_address_set( */
    /**         BLE_GAP_ADDR_CYCLE_MODE_AUTO, &gap_address); */

    /** if (err_code != NRF_SUCCESS) */
    /**     printf("Cannot set GAP address.\n"); */

    return 0;
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        uint32_t err_code = sd_ble_gap_disconnect(m_conn_handle, 
                BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);

        if (err_code != NRF_SUCCESS)
            printf("Cannot disconnect GAP.\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static uint8_t conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

static void sleep_mode_enter(void)
{
    uint32_t err_code;
    /** err_code = bsp_indication_set(BSP_INDICATE_IDLE); */
    /** APP_ERROR_CHECK(err_code); */

    // Go to system-off mode (this function will not return; 
    // wakeup will cause a reset).
    err_code = sd_power_system_off();

    if (err_code != NRF_SUCCESS)
        printf("Cannot enter sleep mode.\n");
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_SLOW:
            /** err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING); */
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            /** err_code = bsp_indication_set(BSP_INDICATE_CONNECTED); */
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}

/** static void on_write(ble_hcus_t *p_hcus, ble_evt_t *p_ble_evt)
  * {
  *     uint8_t count;
  *     printf("Write!!!\n");
  * 
  *     ble_gatts_evt_write_t *p_evt_write = 
  *         &p_ble_evt->evt.gatts_evt.params.write;
  * 
  *     if (p_evt_write->handle == p_hcus->acc_char_handle.value_handle) {
  *         printf("ACC written: ");
  *     }
  *     else if (p_evt_write->handle == p_hcus->gyro_char_handle.value_handle) {
  *         printf("GYRO written: ");
  *     }
  *     else if (p_evt_write->handle == p_hcus->mag_char_handle.value_handle) {
  *         printf("MAG written: ");
  *     }
  *     else if (p_evt_write->handle == p_hcus->hrm_char_handle.value_handle) {
  *         printf("HRM written: ");
  *     }
  * 
  *     for (count = 0; count < p_evt_write->len; ++count)
  *         printf("%x ", p_evt_write->data[count]);
  * 
  *     printf("\n");
  * } */

static void on_rw_authorize_request(ble_hcus_t *p_hcus, ble_evt_t *p_ble_evt)
{
    ble_gatts_evt_read_t *p_evt_read = 
        &p_ble_evt->evt.gatts_evt.params.authorize_request.request.read;
    ble_gatts_rw_authorize_reply_params_t rw_authorize_reply_params;
    uint8_t value[6] = {0};
    int16_t x, y, z;

    memset( &rw_authorize_reply_params, 
            0, 
            sizeof(ble_gatts_rw_authorize_reply_params_t));
    
    rw_authorize_reply_params.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
    rw_authorize_reply_params.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS; 
    rw_authorize_reply_params.params.read.update = 1;

    if (p_evt_read->handle == p_hcus->acc_char_handle.value_handle) {
        printf("ACC read request\n");
        mpuGetAccel(&x, &y, &z);
        value[0] = (uint8_t) (x >> 8);
        value[1] = (uint8_t) x;
        value[2] = (uint8_t) (y >> 8);
        value[3] = (uint8_t) y;
        value[4] = (uint8_t) (z >> 8);
        value[5] = (uint8_t) z;
        rw_authorize_reply_params.params.read.len = 6;
        rw_authorize_reply_params.params.read.offset = 0;
        rw_authorize_reply_params.params.read.p_data = value;
    }
    else if (p_evt_read->handle == p_hcus->gyro_char_handle.value_handle) {
        printf("GYRO read request\n");
        mpuGetGyro(&x, &y, &z);
        value[0] = (uint8_t) (x >> 8);
        value[1] = (uint8_t) x;
        value[2] = (uint8_t) (y >> 8);
        value[3] = (uint8_t) y;
        value[4] = (uint8_t) (z >> 8);
        value[5] = (uint8_t) z;
        rw_authorize_reply_params.params.read.len = 6;
        rw_authorize_reply_params.params.read.offset = 0;
        rw_authorize_reply_params.params.read.p_data = value;
    }
    else if (p_evt_read->handle == p_hcus->mag_char_handle.value_handle) {
        printf("MAG read request\n");
        mpuGetMagnet(&x, &y, &z);
        value[0] = (uint8_t) (x >> 8);
        value[1] = (uint8_t) x;
        value[2] = (uint8_t) (y >> 8);
        value[3] = (uint8_t) y;
        value[4] = (uint8_t) (z >> 8);
        value[5] = (uint8_t) z;
        rw_authorize_reply_params.params.read.len = 6;
        rw_authorize_reply_params.params.read.offset = 0;
        rw_authorize_reply_params.params.read.p_data = value;
    }
    else if (p_evt_read->handle == p_hcus->hrm_char_handle.value_handle) {
        printf("HRM read request\n");

        if (nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE) != 
                NRF_SUCCESS) {
            value[0] = 0xFF;
            value[1] = 0xFF;
        }
        else {
            nrf_drv_adc_sample();

            while (!isSampleCmplt) {  }

            isSampleCmplt = false;
            value[0] = (uint8_t) (adc_buffer[0] >> 8);
            value[1] = (uint8_t) adc_buffer[0];
        }

        rw_authorize_reply_params.params.read.len = 2;
        rw_authorize_reply_params.params.read.offset = 0;
        rw_authorize_reply_params.params.read.p_data = value;
    }

    sd_ble_gatts_rw_authorize_reply(
            p_hcus->conn_handle,
            &rw_authorize_reply_params);
}

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    ble_hcu_service_on_ble_evt(
            &m_our_service, 
            p_ble_evt, 
            NULL, 
            on_rw_authorize_request);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}

static uint8_t ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    if (err_code != NRF_SUCCESS)
        return 1;
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    ble_enable_params.gatts_enable_params.service_changed = 
        IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = softdevice_enable(&ble_enable_params);

    if (err_code != NRF_SUCCESS)
        return 2;

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);

    if (err_code != NRF_SUCCESS)
        return 3;

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);

    if (err_code != NRF_SUCCESS)
        return 4;

    return 0;
}

static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    if (event_result != NRF_SUCCESS) {
        printf("Device Manager event_result error.\n");

        while (1) {  }  // Do nothing else.
    }

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}

static uint8_t device_manager_init(dm_application_instance_t *m_app_handle,
        bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();

    if (err_code != NRF_SUCCESS)
        return 1;

    err_code = dm_init(&init_param);

    if (err_code != NRF_SUCCESS)
        return 2;

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(m_app_handle, &register_param);

    if (err_code != NRF_SUCCESS)
        return 3;

    return 0;
}

static uint8_t advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_manuf_data_t manuf_data;

    // SHADDR00
    uint8_t dev_code[] = DEV_CODE;
    manuf_data.company_identifier = TEAM_CODE;
    manuf_data.data.p_data = dev_code;
    manuf_data.data.size = sizeof(dev_code);

    memset(&advdata, 0, sizeof(advdata));

    advdata.flags                   = 
        BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.p_manuf_specific_data   = &manuf_data;

    ble_adv_modes_config_t options  = {0};
    options.ble_adv_slow_enabled    = BLE_ADV_SLOW_ENABLED;
    options.ble_adv_slow_interval   = APP_ADV_INTERVAL;
    options.ble_adv_slow_timeout    = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    if (err_code != NRF_SUCCESS)
        printf("Cannot enter app event wait\n");
}

