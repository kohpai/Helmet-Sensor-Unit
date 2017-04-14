#include <stdio.h>

#include "boards.h"

#include "softdevice_handler.h"
#include "ble_conn_params.h"
#include "ble_advertising.h"
#include "ble_hci.h"

#include "device_manager.h"
#include "pstorage.h"
#include "pstorage_platform.h"

#include "app_timer.h"

#include "helmet_sensor_unit_service.h"
#include "bluetooth.h"
#include "sensors.h"
#include "error_event.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT     1
#define CENTRAL_LINK_COUNT                  0
#define PERIPHERAL_LINK_COUNT               1

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)
#define SLAVE_LATENCY                   0
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)

#define SEC_PARAM_BOND                  1
#define SEC_PARAM_MITM                  0
#define SEC_PARAM_LESC                  0
#define SEC_PARAM_KEYPRESS              0
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE
#define SEC_PARAM_OOB                   0
#define SEC_PARAM_MIN_KEY_SIZE          7
#define SEC_PARAM_MAX_KEY_SIZE          16

#define DEV_CODE            { 0x53, 0x48, 0x41, 0x44, 0x44, 0x52 ,0x00, 0x00 }
#define TEAM_CODE           0x0059

#define APP_ADV_INTERVAL                480
#define APP_ADV_TIMEOUT_IN_SECONDS      0

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static uint8_t *uart_error;
static ble_hcus_t m_our_service;

static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_set(CNTD_PIN);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_gpio_pin_clear(CNTD_PIN);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}

static void xyz_to_array(uint8_t *arr, int16_t *x, int16_t *y, int16_t *z)
{
    arr[0] = (uint8_t) ((*x) >> 8);
    arr[1] = (uint8_t) (*x);
    arr[2] = (uint8_t) ((*y) >> 8);
    arr[3] = (uint8_t) (*y);
    arr[4] = (uint8_t) ((*z) >> 8);
    arr[5] = (uint8_t) (*z);
}

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

    if (p_evt_read->handle == p_hcus->err_char_handle.value_handle) {
        uint16_t err_evt = get_err_evt_status();

        if (!(*uart_error))
            printf("on_rw_authorize_request: ERR read request\n");

        value[0] = (uint8_t) (err_evt >> 8);
        value[1] = (uint8_t) err_evt;

        rw_authorize_reply_params.params.read.len = 2;
        rw_authorize_reply_params.params.read.offset = 0;
        rw_authorize_reply_params.params.read.p_data = value;
    }
    else if (p_evt_read->handle == p_hcus->acc_char_handle.value_handle) {
        if (!(*uart_error))
            printf("on_rw_authorize_request: ACC read request\n");

        if (read_acc(&x, &y, &z) == 0) {
            xyz_to_array(value, &x, &y, &z);
            set_acc_error(0);
        }
        else {
            set_acc_error(1);
        }

        rw_authorize_reply_params.params.read.len = 6;
        rw_authorize_reply_params.params.read.offset = 0;
        rw_authorize_reply_params.params.read.p_data = value;
    }
    else if (p_evt_read->handle == p_hcus->gyro_char_handle.value_handle) {
        if (!(*uart_error))
            printf("on_rw_authorize_request: GYRO read request\n");

        if (read_gyro(&x, &y, &z) == 0) {
            xyz_to_array(value, &x, &y, &z);
            set_gyr_error(0);
        }
        else {
            set_gyr_error(1);
        }

        rw_authorize_reply_params.params.read.len = 6;
        rw_authorize_reply_params.params.read.offset = 0;
        rw_authorize_reply_params.params.read.p_data = value;
    }
    else if (p_evt_read->handle == p_hcus->mag_char_handle.value_handle) {
        if (!(*uart_error))
            printf("on_rw_authorize_request: MAG read request\n");

        if (read_mag(&x, &y, &z) == 0) {
            xyz_to_array(value, &x, &y, &z);
            set_mag_error(0);
        }
        else {
            set_mag_error(1);
        }

        rw_authorize_reply_params.params.read.len = 6;
        rw_authorize_reply_params.params.read.offset = 0;
        rw_authorize_reply_params.params.read.p_data = value;
    }
    else if (p_evt_read->handle == p_hcus->hrm_char_handle.value_handle) {
        uint8_t hrm;

        if (!(*uart_error))
            printf("on_rw_authorize_request: HRM read request\n");

        hrm = get_hrm();

        value[1] = hrm;
        set_hrm_error(0);

        // find some way to return error
        /* set_hrm_error(1); */

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

static uint32_t device_manager_evt_handler(
        dm_handle_t const * p_handle,
        dm_event_t const  * p_event,
        ret_code_t        event_result)
{
    if (event_result != NRF_SUCCESS) {
        if (!(*uart_error))
            printf( "device_manager_evt_handler: "
                    "Device Manager event_result error.\n");

        /** @TODO: Add restart BLE procedure */
        /** while (1) {  }  // Do nothing else. */
    }

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}

static uint8_t device_manager_init(
        dm_application_instance_t *m_app_handle,
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
    ble_gap_addr_t gap_address;
    gap_address.addr_type   = BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE;
    err_code                = sd_ble_gap_address_set(
            BLE_GAP_ADDR_CYCLE_MODE_AUTO, &gap_address);

    if (err_code != NRF_SUCCESS)
        return 2;

    return 0;
}

static void sleep_mode_enter(void)
{
    uint32_t err_code;

    // Go to system-off mode (this function will not return;
    // wakeup will cause a reset).
    err_code = sd_power_system_off();

    if (err_code != NRF_SUCCESS)
        if (!(*uart_error))
            printf("sleep_mode_enter: Cannot enter sleep mode.\n");
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_SLOW:
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
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

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        uint32_t err_code = sd_ble_gap_disconnect(m_conn_handle,
                BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);

        if (err_code != NRF_SUCCESS)
            if (!(*uart_error))
                printf("on_conn_params_evt: Cannot disconnect GAP.\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    /** APP_ERROR_HANDLER(nrf_error); */
    if (nrf_error != NRF_SUCCESS)
        if (!(*uart_error))
            printf("conn_params_error_handler: \n");
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

static uint8_t start_ble_adv(void)
{
    uint32_t err_code;

    err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);

    if (err_code != NRF_SUCCESS)
        return 1;

    return 0;
}

uint8_t init_bluetooth(uint8_t *ue)
{
    dm_application_instance_t m_app_handle;
    uint8_t ret;

    uart_error = ue;
    ret = ble_stack_init();

    if (ret != 0)
        return 1;

    ret = device_manager_init(&m_app_handle, true);

    if (ret != 0)
        return 2;

    ret = gap_params_init();

    if (ret != 0)
        return 3;

    ret = init_hcu_service(&m_our_service);

    if (ret != 0)
        return 4;

    ret = advertising_init();

    if (ret != 0)
        return 5;

    ret = conn_params_init();

    if (ret != 0)
        return 6;

    ret = start_ble_adv();

    if (ret != 0)
        return 7;

    return 0;
}

