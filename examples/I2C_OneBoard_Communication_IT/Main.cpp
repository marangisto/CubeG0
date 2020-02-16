#include <stdlib.h>
#include <cstring>
#include <usart.h>
#include <redirect.h>
#include <i2c.h>

using hal::sys_tick;
using hal::sys_clock;
using namespace hal::gpio;
using namespace hal::usart;
using namespace hal::i2c;

typedef usart_t<2, PA2, PA3> serial;
typedef output_t<PD9> probe;
typedef i2c_slave_t<1, PB8, PB9> i2c1;
typedef i2c_master_t<2, PB13, PB14> i2c2;

static uint8_t buf[128];

static uint8_t callback(uint8_t len)
{
    return 0;
}

extern "C" void aux_main()
{
    sys_clock::copy_system_core_clock();
    serial::setup<115200>();
    stdio_t::bind<serial>();
    probe::setup();
    printf("Welcome to the STM32G431!\n");
}

extern "C" void toggle_probe()
{
    probe::toggle();
}

extern "C" void write_probe(uint8_t x)
{
    probe::write(x != 0);
}

extern "C" void trace(const char *s, uint32_t x)
{
    printf("%s = %lx\n", s, x);
}

extern "C" void init_i2c1()
{
    i2c1::setup(90, callback, buf, sizeof(buf));
    hal::nvic<interrupt::I2C1>::enable();
}

extern "C" void init_i2c2()
{
    i2c2::setup();
}

extern "C" void write_i2c2(uint8_t addr, const uint8_t *buf, uint8_t nbytes)
{
    i2c2::write(addr, buf, nbytes);
}

extern "C" void read_i2c2(uint8_t addr, uint8_t *buf, uint8_t nbytes)
{
    i2c2::read(addr, buf, nbytes);
}

