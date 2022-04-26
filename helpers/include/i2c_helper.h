#ifndef I2C_HELPER_H
#define I2C_HELPER_H

#include <stdint.h>

/************************************
 *
 *  I2C HELPER
 *
 ************************************/

class i2cHelper
{
    bool i2cError = false;
    int i2cfd;

public:
    i2cHelper(const char* path = "/dev/i2c-0");

    void TCA9548A_write(const uint8_t address, const uint16_t output);
    bool Geti2cError();
};

#endif
