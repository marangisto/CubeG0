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
typedef output_t<PA10> probe;
typedef i2c_t<2, PB13, PB14> i2c2;

extern "C" void aux_main()
{
    sys_clock::copy_system_core_clock();
    serial::setup<230400>();
    stdio_t::bind<serial>();
    probe::setup();
    printf("Welcome to the STM32G431!\n");
}

extern "C" void write_probe(uint8_t x)
{
    probe::write(x != 0);
}

extern "C" void trace(const char *s, uint32_t x)
{
    printf("%s = %lx\n", s, x);
}

extern "C" void init_i2c2()
{
    i2c2::setup();
}

extern "C" void write_i2c2(uint8_t addr, const uint8_t *buf, uint8_t nbytes)
{
    i2c2::write(addr, buf, nbytes);
}
