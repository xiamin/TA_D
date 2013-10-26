
#include "tad_e2prom.h"
#include "rtdef.h"

struct e2prom_device
{
    /*inherit from rt_device*/
    struct rt_device parent;

    /*i2c mode*/
    struct rt_i2c_bus_device *i2c_bus_device;

    /*底层代码已经做好了读写保护*/
    //struct rt_mutex lock;
};

struct e2prom_device e2prom;

static rt_err_t e2prom_init(rt_device_t dev)
{
    return RT_EOK;
}

static rt_err_t e2prom_open(rt_device_t dev, rt_uint16_t flag)
{
    return RT_EOK;
}

static rt_size_t e2prom_read(rt_device_t dev,
                             rt_off_t pos,
                             void* buffer,
                             rt_size_t size)
{
    rt_uint16_t addr;
    rt_uint16_t flags;
    struct e2prom_device * e2 = (struct e2prom_device*)dev;
    struct rt_i2c_bus_device *bus = e2->i2c_bus_device;

    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    i2c_dbg("I2C bus dev reading %u bytes.\n", dev->parent.name, size);

    addr = pos & 0xffff;
    flags = (pos >> 16) & 0xffff;

    return rt_i2c_master_recv(bus, addr, flags, buffer, size);
    //return e2prom.i2c_bus_device.parent.read(dev, pos, buffer, size);
}

static rt_size_t e2prom_write(rt_device_t dev,
                              rt_off_t pos,
                              const void* buffer,
                              rt_size_t size)
{
    rt_uint16_t addr;
    rt_uint16_t flags;
    struct e2prom_device * e2 = (struct e2prom_device*)dev;
    struct rt_i2c_bus_device *bus = e2->i2c_bus_device;

    RT_ASSERT(bus != RT_NULL);
    RT_ASSERT(buffer != RT_NULL);

    i2c_dbg("I2C bus dev writing %u bytes.\n", dev->parent.name, size);

    addr = pos & 0xffff;
    flags = (pos >> 16) & 0xffff;

    return rt_i2c_master_send(bus, addr, flags, buffer, size);
    //return e2prom.i2c_bus_device.parent.write(dev, pos, buffer, size);
}

static rt_err_t e2prom_close(rt_device_t dev)
{
    return RT_EOK;
}
rt_err_t e2prom_device_init(const char* e2_device_name, const char* i2c_bus_device_name)
{
    struct rt_i2c_bus_device* i2c_device;

    /*initialize mutex*/
    /*if(rt_mutex_init(&e2prom.lock, e2_device_name, RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("init the e2prom mutex failed\n");
        return -RT_ENOSYS;
    }*/

    i2c_device  = rt_i2c_bus_device_find(i2c_bus_device_name);
    if(i2c_device == RT_NULL)
    {
        rt_kprintf("i2c bus device %s not found! \r\n", i2c_bus_device_name);
        return -RT_ENOSYS;
    }
    e2prom.i2c_bus_device = i2c_device;

    e2prom.parent.type = RT_Device_Class_Char;
    e2prom.parent.user_data = RT_NULL;
    e2prom.parent.init = e2prom_init;
    e2prom.parent.open = e2prom_open;
    e2prom.parent.write = e2prom_write;
    e2prom.parent.read = e2prom_read;
    e2prom.parent.close = e2prom_close;

    return rt_device_register(&e2prom.parent, e2_device_name, RT_DEVICE_FLAG_RDWR);
}
