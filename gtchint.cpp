#include <irsdefs.h>

#include <irsarchint.h>

#include <armcfg.h>

#include "defs.h"

#include "gtchint.h"

#include <irsfinal.h>

//----------------------------  RELAY PIN INTERRUPT  ---------------------------
gtch::pa3_relay_interrupt_generator_t::pa3_relay_interrupt_generator_t()
{
  RCC_APB2ENR_bit.SYSCFGEN = 1;
  irs::arm::port_clock_on(IRS_PORTA_BASE);
  irs::gpio_moder_input_enable(PA3);
  SYSCFG_EXTICR1_bit.EXTI3 = 0; // 0000: PA3 pin
  SETENA0_bit.SETENA_EXTI3 = 1;
  EXTI_IMR_bit.MR3 = 0; // Изначально прерывание на линии 3 отключено
  EXTI_RTSR_bit.TR3 = 1; // Включаем реакцию на передний фронт
  EXTI_FTSR_bit.TR3 = 1; // Включаем реакцию на задний фронт
}

void gtch::pa3_relay_interrupt_generator_t::add_event(mxfact_event_t *ap_event)
{
  irs::interrupt_array()->int_event_gen(irs::arm::exti3_int)->add(ap_event);
}

gtch::pa3_relay_interrupt_generator_t::~pa3_relay_interrupt_generator_t()
{
  EXTI_IMR_bit.MR3 = 0; // Отключаем линию 3
}

void gtch::pa3_relay_interrupt_generator_t::start()
{
  EXTI_IMR_bit.MR3 = 1; // Включаем линию 3
}

void gtch::pa3_relay_interrupt_generator_t::stop()
{
  EXTI_IMR_bit.MR3 = 0; // Отключаем линию 3
}
