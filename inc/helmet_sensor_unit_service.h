
#ifndef HELMET_CONTROL_UNIT_SERVICE_H__
#define HELMET_CONTROL_UNIT_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"



#define BLE_UUID_HCU_BASE_UUID              {{ 0x63, 0xC3, 0xD4, 0x1D, \
                                               0x74, 0xCD, 0x42, 0x5B, \
                                               0x9B, 0xD6, 0x89, 0x10, \
                                               0x96, 0xA3, 0xC1, 0xEA }}

#define BLE_UUID_HCU_SERVICE_UUID           0x0001
#define BLE_UUID_ERR_CHARACTERISTIC_UUID    0x0101
#define BLE_UUID_ACC_CHARACTERISTIC_UUID    0x0102
#define BLE_UUID_GYRO_CHARACTERISTIC_UUID   0x0103
#define BLE_UUID_MAG_CHARACTERISTIC_UUID    0x0104
#define BLE_UUID_HRM_CHARACTERISTIC_UUID    0x0105

typedef struct
{
    uint16_t                    conn_handle;
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    err_char_handle;
    ble_gatts_char_handles_t    acc_char_handle;
    ble_gatts_char_handles_t    gyro_char_handle;
    ble_gatts_char_handles_t    mag_char_handle;
    ble_gatts_char_handles_t    hrm_char_handle;
}ble_hcus_t;

void ble_hcu_service_on_ble_evt(
        ble_hcus_t *p_hcus,
        ble_evt_t *p_ble_evt,
        void (*on_write_handler) (ble_hcus_t*, ble_evt_t*),
        void (*on_read_request_handler) (ble_hcus_t*, ble_evt_t*));

uint8_t init_hcu_service(ble_hcus_t * p_our_service);

#endif  /* _ HELMET_CONTROL_UNIT_H__ */
