#include <stdlib.h>
#include <cstring>
#include <usart.h>
#include <redirect.h>
#include <dac.h>

using hal::sys_tick;
using hal::sys_clock;
using namespace hal::gpio;
using namespace hal::usart;
using namespace hal::dac;

typedef usart_t<2, PA2, PA3> serial;
typedef output_t<PA10> probe;
typedef hal::dac::dac_t<1> dac;

extern "C" void aux_main()
{
    sys_clock::copy_system_core_clock();
    serial::setup<230400>();
    stdio_t::bind<serial>();
    probe::setup();

    dac::setup();
    dac::enable<1>();
    dac::enable<2>();
    printf("Welcome to the STM32G431!\n");
}

extern "C" void write_probe(uint8_t x)
{
    probe::write(x != 0);
}

extern "C" void write_dac(uint16_t x)
{
    x &= 0x3ff;
    dac::write<1>(x);
    dac::write<2>(4095 - x);
}

extern "C" void trace(const char *s, uint32_t x)
{
    printf("%s = %lx\n", s, x);
}

