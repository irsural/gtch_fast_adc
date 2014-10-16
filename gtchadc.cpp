#include "gtchadc.h"
#include <armioregs.h>

// class adc_filter_t
gtch::adc_filter_t::adc_filter_t(irs::adc_t* ap_adc, size_type a_channel,
  float a_sampling_time, size_type a_num_of_points, shift_type a_shift
):
  mp_adc(ap_adc),
  m_channel(static_cast<irs_u8>(a_channel)),
  m_sample_timer(irs::make_cnt_s(a_sampling_time)),
  m_samples(a_num_of_points, 0),
  m_current_point(0),
  m_shift(a_shift),
  m_filled(false)
{
}

gtch::adc_filter_t::adc_data_t gtch::adc_filter_t::get_value()
{
  size_type value = 0;
  const size_type count = m_filled ? m_samples.size() : m_current_point;
  for (size_type i = 0; i < count; i++) {
    value += m_samples[i];
  }
  if (!m_samples.empty()) {
    value /= count;
  }
  if ((m_shift < 0) && (value < static_cast<size_type>(abs(m_shift)))) {
    value = 0;
  } else {
    value += m_shift;
  }
  return value;
}

void gtch::adc_filter_t::restart()
{
  m_current_point = 0;
  m_filled = false;
  m_sample_timer.start();
}

void gtch::adc_filter_t::tick()
{
  if (m_sample_timer.check()) {
    if (!m_samples.empty()) {
      m_samples[m_current_point] = mp_adc->get_u16_data(m_channel);
      m_current_point++;
      if (m_current_point >= m_samples.size()) {
        m_current_point = 0;
        m_filled = true;
      }
    }
  }
}

