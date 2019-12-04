#include <stdlib.h>
#include <cstring>
#include <usart.h>
#include <redirect.h>
#include <timer.h>
#include <dac.h>
#include <adc.h>
#include <dma.h>

using hal::sys_tick;
using hal::sys_clock;
using namespace hal::gpio;
using namespace hal::usart;
using namespace hal::timer;
using namespace hal::dac;
using namespace hal::adc;
using namespace hal::dma;

typedef usart_t<2, PA2, PA3> serial;
typedef output_t<PA10> probe;
typedef hal::dac::dac_t<1> dac;
typedef hal::adc::adc_t<1> adc;
typedef hal::dma::dma_t<1> dma;
typedef hal::timer::timer_t<3> tim;

static const uint8_t adc_dma_ch = 1;

extern "C" void aux_main()
{
    sys_clock::copy_system_core_clock();
    serial::setup<230400>();
    stdio_t::bind<serial>();
    probe::setup();

    dma::setup();
    hal::nvic<interrupt::DMA_CHANNEL1>::enable();

    dac::setup();
    dac::enable<1>();
    dac::enable<2>();
    printf("Welcome to the STM32G431!\n");
}

extern "C" void tim_init()
{
    //tim::setup(244, 64);
    tim::setup(0, 64);
    tim::master_mode<tim::mm_update>();
}

extern "C" void adc_init(volatile uint16_t *dest, uint16_t nelem)
{
    adc::setup();
    adc::dma<dma, adc_dma_ch, uint16_t>(dest, nelem);
    adc::sequence<0>();
    adc::trigger<0x3>();
}

extern "C" void adc_activate()
{
    adc::enable();
}

extern "C" void write_probe(uint8_t x)
{
    probe::write(x != 0);
}

extern "C" void write_dac(uint16_t x)
{
    x &= 0xfff;
    dac::write<1>(x);
    dac::write<2>(4095 - x);
}

extern "C" void trace(const char *s, uint32_t x)
{
    printf("%s = %lx\n", s, x);
}

