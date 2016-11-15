#ifndef ERROR_EVENT_H
#define ERROR_EVENT_H

#include <stdint.h>

#define ANOMALY_FINE        0
#define ANOMALY_OVR_ANG     1
#define ANOMALY_OVR_ACC     2
#define ANOMALY_OVR_ANG_VEL 3
#define ANOMALY_UNKNOWN     4

#define WEARING_YES     0
#define WEARING_NO      1
#define WEARING_UNKNOWN 2

void set_twi_error(uint8_t set);
void set_mpu_error(uint8_t set);
void set_acc_error(uint8_t set);
void set_gyr_error(uint8_t set);
void set_mag_error(uint8_t set);
void set_adc_error(uint8_t set);
void set_hrm_error(uint8_t set);
void set_uart_error(uint8_t set);

void set_anomaly_event(uint8_t evt);
void set_wearing_event(uint8_t set);

uint16_t get_err_evt_status(void);
#endif /* ifndef ERROR_EVENT_H */
