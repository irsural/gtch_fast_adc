#include <irspch.h>

#include "sinus_generator.h"

#include <irsfinal.h>

size_t shift_for_left_aligment_irs_i16(const irs_i16 a_value)
{
  irs_i16 value = a_value;
  const size_t max_shift = 14;
  size_t shift = 0;
  const irs_i16 mask = (1 << 14);
  while (shift < max_shift) {
    if (value & mask) {
      break;
    }
    value <<= 1;
    shift++;
  }
  return shift;
}

// class st_pwm_gen_adapter_t
st_pwm_gen_adapter_t::st_pwm_gen_adapter_t(
  irs::arm::st_pwm_gen_t* ap_st_pwm_gen,
  gpio_channel_t a_first_channel,
  gpio_channel_t a_second_channel
):
  mp_st_pwm_gen(ap_st_pwm_gen),
  m_first_channel(a_first_channel),
  m_second_channel(a_second_channel)
{
}

void st_pwm_gen_adapter_t::start()
{
  mp_st_pwm_gen->start();
}

void st_pwm_gen_adapter_t::stop()
{
  mp_st_pwm_gen->stop();
}

void st_pwm_gen_adapter_t::set_duty(irs_uarc a_duty)
{
  mp_st_pwm_gen->set_duty(m_first_channel, a_duty);
  mp_st_pwm_gen->set_duty(m_second_channel, get_max_duty() - a_duty);
}

void st_pwm_gen_adapter_t::set_duty(float a_duty)
{
  mp_st_pwm_gen->set_duty(m_first_channel, a_duty);
  mp_st_pwm_gen->set_duty(m_second_channel, 1 - a_duty);
}

irs::cpu_traits_t::frequency_type st_pwm_gen_adapter_t::set_frequency(
  irs::cpu_traits_t::frequency_type a_frequency)
{
  return mp_st_pwm_gen->set_frequency(a_frequency);
}

irs_uarc st_pwm_gen_adapter_t::get_max_duty()
{
  return mp_st_pwm_gen->get_max_duty();
}

irs::cpu_traits_t::frequency_type st_pwm_gen_adapter_t::get_max_frequency()
{
  return mp_st_pwm_gen->get_max_frequency();
}

irs::cpu_traits_t::frequency_type st_pwm_gen_adapter_t::get_timer_frequency()
{
  return mp_st_pwm_gen->get_timer_frequency();
}

// class sinus_generator_t
sinus_generator_t::point_interrupt_event_t::
  point_interrupt_event_t(sinus_generator_t* ap_owner):
  mp_owner(ap_owner),
  m_enabled(false)
{
}

void sinus_generator_t::point_interrupt_event_t::exec()
{
  if (m_enabled) {
    irs::event_t::exec();
    mp_owner->point_interrupt();
  }
}

void sinus_generator_t::point_interrupt_event_t::enable()
{
  m_enabled = true;
}

void sinus_generator_t::point_interrupt_event_t::disable()
{
  m_enabled = false;
}

sinus_generator_t::sinus_generator_t(
interrupt_generator_t* ap_interrupt_generator,
  size_type a_interrupt_freq_boost_factor,
  irs::pwm_gen_t* ap_pwm_gen,
  double a_pwm_frequency,
  double a_dead_time,
  size_type a_sinus_size
):
  m_max_value(0),
  m_min_amplitude(0.f),
  m_max_amplitude(1.f),
  m_min_frequency(1.f),
  m_max_frequency(10000.f),
  m_Kf(2.f * IRS_PI / double(a_sinus_size)),
  m_dead_time(a_dead_time),
  m_shift_alignment_irs_i16(shift_for_left_aligment_irs_i16(
    static_cast<irs_i16>(ap_pwm_gen->get_max_duty()/2 -
    m_dead_time*ap_pwm_gen->get_timer_frequency()))),
  m_max_sinus_value((ap_pwm_gen->get_max_duty()/2 -
    m_dead_time*ap_pwm_gen->get_timer_frequency())*
    (1 << m_shift_alignment_irs_i16)),
  m_started(false),
  m_amplitude_changed(false),
  mp_interrupt_generator(ap_interrupt_generator),
  m_interrupt_freq_boost_factor(a_interrupt_freq_boost_factor),
  mp_pwm_gen(ap_pwm_gen),
  m_pwm_frequency(a_pwm_frequency),
  m_point_interrupt_event(this),
  m_frequency(50.),
  m_amplitude(0.f),
  m_current_point(0),
  m_interrupt_count(0),
  m_floor_interval(0),
  m_sinus_size(a_sinus_size),
  m_ref_sinus(m_sinus_size, 0),
  m_sinus_array_1(m_sinus_size, 0),
  m_sinus_array_2(m_sinus_size, 0),
  mp_active_sinus_array(&m_sinus_array_1),
  mp_inactive_sinus_array(&m_sinus_array_2),
  m_freq_trans_koef(0.f),
  m_default_freq_trans_koef(0.f)
{
  for (size_type i = 0; i < m_ref_sinus.size(); i++) {
    m_ref_sinus[i] =
      (1. - dead_time_count*m_dead_time*m_pwm_frequency/2.)*sin(m_Kf * i);
  }
  calc_period_t num = 0;
  calc_period_t denom = 1;
  mp_interrupt_generator->get_converting_koefs(&num, &denom);
  m_freq_trans_koef = double(num)/double(denom);
  m_default_freq_trans_koef = m_freq_trans_koef;
  fill_sinus_array(&m_sinus_array_1);
  mp_active_sinus_array = &m_sinus_array_1;
  apply_frequency();
  mp_interrupt_generator->add_event(&m_point_interrupt_event);
  m_point_interrupt_event.enable();
}

sinus_generator_t::~sinus_generator_t()
{
  m_point_interrupt_event.disable();
  mp_interrupt_generator->erase_event(&m_point_interrupt_event);
  mp_pwm_gen->stop();
}

void sinus_generator_t::start()
{
  m_current_point = 0;
  set_duty_from_sinus(0);
  mp_pwm_gen->start();
  m_point_interrupt_event.enable();
  m_started = true;
}

void sinus_generator_t::set_duty_from_sinus(size_type a_sample_index)
{
  const irs_uarc duty = irs_uarc(irs_i16(mp_pwm_gen->get_max_duty()/2) +
    irs_i16((*mp_active_sinus_array)[a_sample_index]));
  mp_pwm_gen->set_duty(duty);
}

void sinus_generator_t::stop()
{
  mp_pwm_gen->stop();
  m_point_interrupt_event.disable();
  m_started = false;
}

bool sinus_generator_t::started() const
{
  return m_started;
}

void sinus_generator_t::set_value(unsigned int
  /*a_value*/)
{
}


void sinus_generator_t::set_amplitude(double a_amplitude)
{
  //
  //a_amplitude = 0.f;
  //
  if (a_amplitude > m_max_amplitude) {
    m_amplitude = m_max_amplitude;
  } else if (a_amplitude < m_min_amplitude) {
    m_amplitude = m_min_amplitude;
  } else {
    m_amplitude = a_amplitude;
  }
  fill_sinus_array(mp_inactive_sinus_array);
  m_amplitude_changed = true;
}

double sinus_generator_t::get_amplitude()
{
  return m_amplitude;
}

void sinus_generator_t::set_frequency(double a_frequency)
{
  if (a_frequency > m_max_frequency) {
    m_frequency = m_max_frequency;
  } else if (a_frequency < m_min_frequency) {
    m_frequency = m_min_frequency;
  } else {
    m_frequency = a_frequency;
  }
  apply_frequency();
}

unsigned int sinus_generator_t::get_max_value()
{
  return m_max_value;
}

double sinus_generator_t::get_max_amplitude()
{
  return m_max_amplitude;
}

double sinus_generator_t::get_max_frequency()
{
  return m_max_frequency;
}

void sinus_generator_t::set_correct_freq_koef(double a_correct_koef)
{
  m_freq_trans_koef = m_default_freq_trans_koef * a_correct_koef;
  apply_frequency();
}

void sinus_generator_t::point_interrupt()
{
  m_interrupt_count++;
  if (m_interrupt_count == m_interrupt_freq_boost_factor) {
    m_interrupt_count = 0;
    mp_interrupt_generator->set_interval(get_interval()-1);
    if (m_amplitude_changed &&
        ((m_current_point == 0) || (m_current_point == m_sinus_size/2))) {
      std::swap(mp_active_sinus_array, mp_inactive_sinus_array);

      m_amplitude_changed = false;
    }
    set_duty_from_sinus(m_current_point++);
    if (m_current_point >= m_sinus_size) {
      m_current_point = 0;
    }
  }
}

void sinus_generator_t::fill_sinus_array(std::vector<irs_i16>* ap_array)
{
  for (size_type i = 0; i < ap_array->size(); i++) {
    (*ap_array)[i] = static_cast<irs_i16>((mp_pwm_gen->get_max_duty()/2.)*
      m_amplitude*m_ref_sinus[i]);
  }
}

void sinus_generator_t::apply_frequency()
{
  const double float_floor_len = m_freq_trans_koef/m_frequency;
  double period = float_floor_len/double(m_sinus_size);
  period /= m_interrupt_freq_boost_factor;
  m_floor_interval = period_t(period);
}

// class dc_generator_t
dc_generator_t::dc_generator_t(
  interrupt_generator_t* ap_interrupt_generator,
  size_type a_interrupt_freq_boost_factor,
  irs::pwm_gen_t* ap_pwm_gen, double a_dead_time,
  size_type a_sinus_size
):
  m_max_value(0),
  m_min_amplitude(0.f),
  m_max_amplitude(1.f),
  m_dead_time(a_dead_time),
  m_started(false),
  mp_pwm_gen(ap_pwm_gen),
  m_amplitude(0.f),
  m_frequency(50.),
  m_floor_interval()
{
  calc_period_t num = 0;
  calc_period_t denom = 1;
  ap_interrupt_generator->get_converting_koefs(&num, &denom);
  double freq_trans_koef = double(num)/double(denom);

  const double float_floor_len = freq_trans_koef/m_frequency;
  double period = float_floor_len/double(a_sinus_size);
  period /= a_interrupt_freq_boost_factor;
  period_t floor_interval = period_t(period);
  ap_interrupt_generator->set_interval(floor_interval-1);

}

dc_generator_t::~dc_generator_t()
{
  mp_pwm_gen->stop();
}

void dc_generator_t::start()
{
  set_duty_from_amplitude(0);
  mp_pwm_gen->start();
  m_started = true;
}

void dc_generator_t::set_duty_from_amplitude(float a_amplitude)
{
  float duty = static_cast<float>(irs::bound(0.5 + 0.5*a_amplitude, 0.5, 1.));
  mp_pwm_gen->set_duty(duty);
}

void dc_generator_t::stop()
{
  mp_pwm_gen->stop();
  m_started = false;
}

bool dc_generator_t::started() const
{
  return m_started;
}

void dc_generator_t::set_value(unsigned int /*a_value*/)
{
}

void dc_generator_t::set_amplitude(double a_amplitude)
{
  m_amplitude = irs::bound(a_amplitude, m_min_amplitude, m_max_amplitude);
  set_duty_from_amplitude(m_amplitude);
}

double dc_generator_t::get_amplitude()
{
  return m_amplitude;
}

void dc_generator_t::set_frequency(double /*a_frequency*/)
{
}

unsigned int dc_generator_t::get_max_value()
{
  return m_max_value;
}

double dc_generator_t::get_max_amplitude()
{
  return m_max_amplitude;
}

double dc_generator_t::get_max_frequency()
{
  return 0;
}

void dc_generator_t::set_correct_freq_koef(double /*a_correct_koef*/)
{
}
