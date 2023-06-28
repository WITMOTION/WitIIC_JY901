
#ifndef I2C_H
#define I2C_H

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/rtc.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

#define I2C_DEFAULT_TIMEOUT		1
#define I2C_DEFAULT_RETRY		3
typedef unsigned char           u8;
typedef unsigned short          u16;
typedef unsigned int            u32;
typedef unsigned long long      u64;
typedef signed char             s8;
typedef short                   s16;            
typedef int                     s32;
typedef long long               s64;

int
i2c_read_data(u8 addr, u8 reg, u8 *val, u32 len);

int
i2c_write_data(u8 addr, u8 reg, u8 *val, u32 len);

int
i2c_open(unsigned char* dev, unsigned int timeout, unsigned int retry);

#endif
