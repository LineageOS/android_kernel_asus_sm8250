#ifndef _EC_I2C_UPDATEFW_H_
#define _EC_I2C_UPDATEFW_H_

#include <linux/i2c.h>

struct ec_i2c_platform_data 
{
	struct i2c_client *client;
	dev_t devt;
};

#endif
