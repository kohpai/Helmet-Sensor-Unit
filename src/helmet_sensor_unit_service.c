#include <stdint.h>
#include <string.h>

#include "nrf_gpio.h"
#include "ble_srv_common.h"
#include "ble_types.h"
#include "app_error.h"
#include "helmet_sensor_unit_service.h"

void ble_hcu_service_on_ble_evt(ble_hcus_t *p_hcus, 
        ble_evt_t *p_ble_evt,
        void (*on_write_handler) (ble_hcus_t*, ble_evt_t*),
        void (*on_read_request_handler) (ble_hcus_t*, ble_evt_t*))
{
    switch (p_ble_evt->header.evt_id) {
        case BLE_GAP_EVT_CONNECTED:
            p_hcus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_hcus->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        case BLE_GATTS_EVT_WRITE:
            if (on_write_handler != NULL)
                on_write_handler(p_hcus, p_ble_evt);
            break;
        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
            if (on_read_request_handler != NULL)
                on_read_request_handler(p_hcus, p_ble_evt);
            break;
        default:
            // No implementation needed.
            break;
    }
}

static uint8_t init_characteristics(ble_hcus_t * p_hcus)
{
    uint32_t err_code;
    uint8_t value[6] = {0};
    ble_uuid_t acc_char_uuid, gyro_char_uuid, mag_char_uuid, hrm_char_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t attr_md;
    ble_gatts_attr_t    acc_attr_char_value, 
                        gyro_attr_char_value, 
                        mag_attr_char_value, 
                        hrm_attr_char_value;


    ble_uuid128_t base_uuid = BLE_UUID_HCU_BASE_UUID;

    acc_char_uuid.uuid  = BLE_UUID_ACC_CHARACTERISTIC_UUID;
    gyro_char_uuid.uuid = BLE_UUID_GYRO_CHARACTERISTIC_UUID;
    mag_char_uuid.uuid  = BLE_UUID_MAG_CHARACTERISTIC_UUID;
    hrm_char_uuid.uuid  = BLE_UUID_HRM_CHARACTERISTIC_UUID;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &acc_char_uuid.type);

    if (err_code != NRF_SUCCESS)
        return 1;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &gyro_char_uuid.type);

    if (err_code != NRF_SUCCESS)
        return 2;

    err_code = sd_ble_uuid_vs_add(&base_uuid, &mag_char_uuid.type);

    if (err_code != NRF_SUCCESS)
        return 3;
    
    err_code = sd_ble_uuid_vs_add(&base_uuid, &hrm_char_uuid.type);

    if (err_code != NRF_SUCCESS)
        return 4;
    
    // OUR_JOB: Step 2.F Add read/write properties to our characteristic
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 0;

    // OUR_JOB: Step 2.B, Configure the attribute metadata
    memset(&attr_md, 0, sizeof(attr_md));  
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 1;

    // OUR_JOB: Step 2.G, Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    
    
    // OUR_JOB: Step 2.C, Configure the characteristic value attribute
    memset(&acc_attr_char_value, 0, sizeof(acc_attr_char_value));
    memset(&gyro_attr_char_value, 0, sizeof(gyro_attr_char_value));
    memset(&mag_attr_char_value, 0, sizeof(mag_attr_char_value));
    memset(&hrm_attr_char_value, 0, sizeof(hrm_attr_char_value));

    acc_attr_char_value.p_uuid  = &acc_char_uuid;
    gyro_attr_char_value.p_uuid = &gyro_char_uuid;
    mag_attr_char_value.p_uuid  = &mag_char_uuid;
    hrm_attr_char_value.p_uuid  = &hrm_char_uuid;

    acc_attr_char_value.p_attr_md = &attr_md;
    gyro_attr_char_value.p_attr_md = &attr_md;
    mag_attr_char_value.p_attr_md = &attr_md;
    hrm_attr_char_value.p_attr_md = &attr_md;
    
    // OUR_JOB: Step 2.H, Set characteristic length in number of bytes
    acc_attr_char_value.max_len = 
        gyro_attr_char_value.max_len = 
        mag_attr_char_value.max_len = 6;
    hrm_attr_char_value.max_len = 2;

    acc_attr_char_value.init_len = 
        gyro_attr_char_value.init_len = 
        mag_attr_char_value.init_len = 6;
    hrm_attr_char_value.init_len = 2;

    acc_attr_char_value.p_value = 
        gyro_attr_char_value.p_value = 
        mag_attr_char_value.p_value = 
        hrm_attr_char_value.p_value = value;

    // OUR_JOB: Step 2.E, Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_hcus->service_handle,
            &char_md,
            &acc_attr_char_value,
            &p_hcus->acc_char_handle);

    if (err_code != NRF_SUCCESS)
        return 5;

    err_code = sd_ble_gatts_characteristic_add(p_hcus->service_handle,
            &char_md,
            &gyro_attr_char_value,
            &p_hcus->gyro_char_handle);

    if (err_code != NRF_SUCCESS)
        return 6;

    err_code = sd_ble_gatts_characteristic_add(p_hcus->service_handle,
            &char_md,
            &mag_attr_char_value,
            &p_hcus->mag_char_handle);

    if (err_code != NRF_SUCCESS)
        return 7;

    err_code = sd_ble_gatts_characteristic_add(p_hcus->service_handle,
            &char_md,
            &hrm_attr_char_value,
            &p_hcus->hrm_char_handle);

    if (err_code != NRF_SUCCESS)
        return 8;

    return 0;
}

uint8_t init_hcu_service(ble_hcus_t * p_hcus)
{
    uint32_t    err_code;
    uint8_t     ret;

    ble_uuid_t        service_uuid;
    ble_uuid128_t     base_uuid = BLE_UUID_HCU_BASE_UUID;
    service_uuid.uuid = BLE_UUID_HCU_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);

    if (err_code != NRF_SUCCESS)
        return 1;
    
    p_hcus->conn_handle = BLE_CONN_HANDLE_INVALID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &service_uuid,
                                        &p_hcus->service_handle);
    
    if (err_code != NRF_SUCCESS)
        return 2;
    
    ret = init_characteristics(p_hcus);

    if (ret != 0)
        return 3;

    return 0;
}

