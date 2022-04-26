#define MCP4725_CMD_WRITEDACEEPROM 0x60
#define TCAADDR 0x70

#include "i2c_helper.h"

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <fstream>
#include <iostream>

i2cHelper::i2cHelper(const char* path)
{
    std::cout << "Opening i2c : ";

    // open i2c-0 at given address as a file descriptor, set slave and enable checksums : https://xgoat.com/wp/2007/11/11/using-i2c-from-userspace-in-linux/
    i2cfd = open(path, O_RDWR);
    if (i2cfd < 0)
    {
        std::cout << "Failure\n";
        fprintf(stderr, "Failed to open i2c device: %m\n");
        i2cError = true;
        return;
    }
    if(!i2cError && ioctl(i2cfd, I2C_SLAVE_FORCE, TCAADDR) < 0)
    {
        std::cout << "Failure\n";
        fprintf(stderr, "Failed to set slave address: %m\n");
        i2cError = true;
        return;
    }
    if(!i2cError && ioctl(i2cfd, I2C_PEC, 1) < 0)
    {
        std::cout << "Failure\n";
        fprintf(stderr, "Failed to enable PEC\n");
        i2cError = true;
        return;
    }

    std::cout << "Success\n";
}

/*
 * Inspired by this code : https://github.com/mbin/i2c/blob/master/i2ctest/i2ctest/tca9548.c
 */
void i2cHelper::TCA9548A_write(const uint8_t address, const uint16_t output)
{
    if (i2cError)
        return;

    // select writing channel
    unsigned char val = (unsigned char) 1 << address;
    if (ioctl(i2cfd, I2C_SLAVE_FORCE, TCAADDR) < 0)
    {
        fprintf(stderr, "Failed to reach multiplexer address register: %m\n");
        i2cError = true;
        return;
    }
    if (write(i2cfd, &val, 1) < 0)
    {
        fprintf(stderr, "Failed to set multiplexer address register: %m\n");
        i2cError = true;
        return;
    }

    // select device address
    if(ioctl(i2cfd, I2C_SLAVE_FORCE, 0x60) < 0) 
    {
        fprintf(stderr, "Failed to reach distant sensor: %m\n");
        i2cError = true;
        return;
    }

    // write
    uint8_t packet[3];
    packet[0] = MCP4725_CMD_WRITEDACEEPROM;
    packet[1] = output / 16;        // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
    packet[2] = (output % 16) << 4; // Lower data bits (D3.D2.D1.D0.x.x.x.x)
    if(write(i2cfd, packet, 3) < 0)
    {
        fprintf(stderr, "Failed to write to I2C device: %m\n");
        i2cError = true;
        return;
    }

    // reset writing channel
    val = (unsigned char) 0x00;
    if (ioctl(i2cfd, I2C_SLAVE_FORCE, TCAADDR) < 0)
    {
        fprintf(stderr, "Failed to reach multiplexer address register: %m\n");
        i2cError = true;
        return;
    }
    if (write(i2cfd, &val, 1) < 0)
    {
        fprintf(stderr, "Failed to set multiplexer address register: %m\n");
        i2cError = true;
        return;
    }
}

bool i2cHelper::Geti2cError()
{
    return i2cError;
}

