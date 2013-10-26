#ifndef __E2PROM_H__
#define __E2PROM_H__
#include "i2c.h"
rt_err_t e2prom_device_init(const char* e2_device_name, const char* i2c_bus_device_name);
#endif