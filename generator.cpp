#include <irsdefs.h>

#include <armioregs.h>
#include <armcfg.h>
#include <irsarchint.h>

#include "generator.h"

#include <irsfinal.h>

// class interrupt_generator_t
interrupt_generator_t::interrupt_generator_t(size_t a_timer_address):
  mp_tim(reinterpret_cast<tim_regs_t*>(a_timer_address))
{
  irs::clock_enable(a_timer_address);
  irs::reset_peripheral(a_timer_address);

  mp_tim->TIM_ARR = irs::cpu_traits_t::frequency()/
    (freq_50_hz*sin_period_point_count);
  mp_tim->TIM_CR1_bit.DIR = 0; // 0: Counter used as upcounter
  mp_tim->TIM_CR1_bit.CEN = 1; // 1: Counter enabled
  mp_tim->TIM_DIER_bit.UIE = 0; // 0: Update interrupt disabled
  //SETENA1_bit.SETENA_TIM8_UP_TIM13 = 1;
  irs::update_interrupt_enable(a_timer_address);
}

interrupt_generator_t::~interrupt_generator_t()
{
}

void interrupt_generator_t::interrupt_generator_t::add_event(
  mxfact_event_t *ap_event)
{
  switch (reinterpret_cast<size_t>(mp_tim)) {
    case IRS_TIM1_PWM1_BASE: {
      irs::interrupt_array()->int_event_gen(irs::arm::tim1_up_tim10_int)->
        add(ap_event);
    } break;
    case IRS_TIM8_PWM2_BASE: {
      irs::interrupt_array()->int_event_gen(irs::arm::tim8_up_tim13_int)->
        add(ap_event);
    } break;
    default: {
      IRS_LIB_ERROR(irs::ec_standard,
        "Для указанного таймера прерывание не определено");
    }
  }
}

void interrupt_generator_t::set_interval(counter_type a_interval)
{
  mp_tim->TIM_ARR = a_interval;
}

void interrupt_generator_t::start()
{
  mp_tim->TIM_DIER_bit.UIE = 1; // 1: Update interrupt enabled
}

void interrupt_generator_t::stop()
{
  mp_tim->TIM_DIER_bit.UIE = 0; // 0: Update interrupt disabled
}

void interrupt_generator_t::get_converting_koefs(calc_type *ap_num,
  calc_type *ap_denom) const
{
  *ap_num = irs::cpu_traits_t::timer_frequency(
    reinterpret_cast<size_t>(mp_tim));
  *ap_denom = 1;
}

