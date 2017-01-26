#ifndef TIME_H
#define TIME_H

#include <stdint.h>

#define APP_TIMER_PRESCALER             0
#define APP_TIMER_OP_QUEUE_SIZE         4

uint8_t init_timers(void);
uint64_t get_millis(void);
uint8_t is_time(void);
#endif /* ifndef TIME_H */
