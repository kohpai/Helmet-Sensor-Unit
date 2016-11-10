#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>
#include <stdint.h>

uint8_t mpuInit(void);
bool mpuGetAccel(int16_t *x, int16_t *y, int16_t *z);
bool mpuGetGyro(int16_t *x, int16_t *y, int16_t *z);
bool mpuGetMagnet(int16_t *x, int16_t *y, int16_t *z);
#endif
