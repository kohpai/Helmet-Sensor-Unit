#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>

#include "time.h"

#define FIRST_CONN_PARAMS_UPDATE_DELAY  \
    APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   \
    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

#define CNTD_PIN LED_2

uint8_t init_bluetooth(uint8_t *ue);
#endif /* ifndef BLUETOOTH_H */
