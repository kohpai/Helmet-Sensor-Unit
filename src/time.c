#include "app_timer.h"
#include "time.h"

APP_TIMER_DEF(timer_id);
static uint64_t millis = 0;
static uint8_t is_time_var = 0;

uint8_t is_time(void)
{
    if (is_time_var) {
        is_time_var = 0;
        return 1;
    } else {
        return 0;
    }
}

uint64_t get_millis(void)
{
    return millis;
}

void timer_int_handler(void * p_context)
{
    // Do something
    /** printf("1 ms\r\n"); */
    is_time_var = ++millis % 2;
    /** ++millis; */
}

uint8_t init_timers(void)
{
    uint32_t err_code;
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers
    err_code = app_timer_create(&timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_int_handler);

    if (err_code != NRF_SUCCESS)
        return 1;

    err_code = app_timer_start(
            timer_id,
            APP_TIMER_TICKS(1, APP_TIMER_PRESCALER),
            NULL);

    if (err_code != NRF_SUCCESS)
        return 2;

    return 0;
}

