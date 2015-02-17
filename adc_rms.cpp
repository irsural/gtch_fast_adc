#include <irspch.h>

#include <armcfg.h>

#include "adc_rms.h"

#include <irsfinal.h>

// class st_multi_adc_t
irs::arm::st_multi_adc_t::st_multi_adc_t(adc_channel_t a_first_adc_channel,
  adc_channel_t a_second_adc_channel,
  counter_t a_adc_interval
):
  mp_adc(reinterpret_cast<adc_regs_t*>(size_t(IRS_ADC1_BASE))),
  mp_adc2(reinterpret_cast<adc_regs_t*>(size_t(IRS_ADC2_BASE))),
  m_adc_timer(a_adc_interval)
{
  clock_enable(size_t(IRS_ADC1_BASE));
  clock_enable(size_t(IRS_ADC2_BASE));
  mp_adc->ADC_SMPR1 = 0xFFFFFFFF;
  mp_adc->ADC_SMPR2 = 0xFFFFFFFF;
  mp_adc2->ADC_SMPR1 = 0xFFFFFFFF;
  mp_adc2->ADC_SMPR2 = 0xFFFFFFFF;
  // 3: PCLK2 divided by 8
  ADC_CCR_bit.ADCPRE = 3;

  vector<pair<adc_channel_t, gpio_channel_t> > adc_gpio_pairs;
  adc_gpio_pairs.push_back(make_pair(ADC123_PA0_CH0, PA0));
  adc_gpio_pairs.push_back(make_pair(ADC123_PA1_CH1, PA1));
  adc_gpio_pairs.push_back(make_pair(ADC123_PA2_CH2, PA2));
  adc_gpio_pairs.push_back(make_pair(ADC123_PA3_CH3, PA3));
  adc_gpio_pairs.push_back(make_pair(ADC12_PA4_CH4, PA4));
  adc_gpio_pairs.push_back(make_pair(ADC3_PF6_CH4, PF6));
  adc_gpio_pairs.push_back(make_pair(ADC12_PA5_CH5, PA5));
  adc_gpio_pairs.push_back(make_pair(ADC3_PF7_CH5, PF7));
  adc_gpio_pairs.push_back(make_pair(ADC12_PA6_CH6, PA6));
  adc_gpio_pairs.push_back(make_pair(ADC3_PF8_CH6, PF8));
  adc_gpio_pairs.push_back(make_pair(ADC12_PA7_CH7, PA7));
  adc_gpio_pairs.push_back(make_pair(ADC3_PF9_CH7, PF9));
  adc_gpio_pairs.push_back(make_pair(ADC12_PB0_CH8, PB0));
  adc_gpio_pairs.push_back(make_pair(ADC3_PF10_CH8, PF10));
  adc_gpio_pairs.push_back(make_pair(ADC12_PB1_CH9, PB1));
  adc_gpio_pairs.push_back(make_pair(ADC3_PF3_CH9, PF3));
  adc_gpio_pairs.push_back(make_pair(ADC123_PC0_CH10, PC0));
  adc_gpio_pairs.push_back(make_pair(ADC123_PC1_CH11, PC1));
  adc_gpio_pairs.push_back(make_pair(ADC123_PC2_CH12, PC2));
  adc_gpio_pairs.push_back(make_pair(ADC123_PC3_CH13, PC3));
  adc_gpio_pairs.push_back(make_pair(ADC12_PC4_CH14, PC4));
  adc_gpio_pairs.push_back(make_pair(ADC3_PF4_CH14, PF4));
  adc_gpio_pairs.push_back(make_pair(ADC12_PC5_CH15, PC5));
  adc_gpio_pairs.push_back(make_pair(ADC3_PF5_CH15, PF5));

  const irs_u32 selected_channels = a_first_adc_channel|a_second_adc_channel;

  for (size_t i = 0; i < adc_gpio_pairs.size(); i++) {
    adc_channel_t adc_channel = adc_gpio_pairs[i].first;
    const select_channel_type adc_mask =
      (ADC1_MASK | ADC2_MASK | ADC3_MASK) & adc_channel;
    const select_channel_type channel_mask = static_cast<irs_u16>(adc_channel);
    if ((selected_channels & channel_mask) &&
      (selected_channels & adc_mask)) {
      clock_enable(adc_gpio_pairs[i].second);
      gpio_moder_analog_enable(adc_gpio_pairs[i].second);
    }
  }

  mp_adc->ADC_CR2_bit.ADON = 1;
  mp_adc2->ADC_CR2_bit.ADON = 1;

  mp_adc->ADC_SQR1_bit.L = 0;
  mp_adc2->ADC_SQR1_bit.L = 0;

  mp_adc->ADC_SQR3_bit.SQ1 =
    adc_channel_to_channel_index(a_first_adc_channel);
  mp_adc2->ADC_SQR3_bit.SQ1 =
    adc_channel_to_channel_index(a_second_adc_channel);

  ADC_CCR_bit.MULTI = 6; // 00110: Regular simultaneous mode only
  mp_adc->ADC_CR2_bit.SWSTART = 1;
}

irs_u32 irs::arm::st_multi_adc_t::adc_channel_to_channel_index(
  adc_channel_t a_adc_channel)
{
  irs_u16 adc_channel = static_cast<irs_u16>(a_adc_channel);
  for (size_t i = 0; i < reqular_channel_count; i++) {
    if (adc_channel & 0x1) {
      return i;
    }
    adc_channel >>= 1;
  }
  IRS_LIB_ERROR(ec_standard, "Канал не выбран");
  return 0;
}

irs::arm::st_multi_adc_t::~st_multi_adc_t()
{
  mp_adc->ADC_CR2_bit.ADON = 0;
  mp_adc2->ADC_CR2_bit.ADON = 0;
}

irs::arm::st_multi_adc_t::size_type
irs::arm::st_multi_adc_t::get_resulution() const
{
  return adc_resolution;
}

irs_u16 irs::arm::st_multi_adc_t::get_u16_minimum()
{
  return static_cast<irs_u16>(
    static_cast<irs_i16>((adc_max_value)*-1) << (16 - adc_resolution));
}

irs_u16 irs::arm::st_multi_adc_t::get_u16_maximum()
{
  return static_cast<irs_u16>(
    static_cast<irs_i16>(adc_max_value) << (16 - adc_resolution));
}

irs_u16 irs::arm::st_multi_adc_t::get_u16_data(irs_u8 /*a_channel*/)
{
  return static_cast<irs_u16>(m_value << (16 - adc_resolution));
}

irs_u32 irs::arm::st_multi_adc_t::get_u32_minimum()
{
  return static_cast<irs_u32>(
    static_cast<irs_i32>((adc_max_value)*-1) << (32 - adc_resolution));
}

irs_u32 irs::arm::st_multi_adc_t::get_u32_maximum()
{
  return static_cast<irs_u32>(
    static_cast<irs_i32>(adc_max_value) << (32 - adc_resolution));
}

irs_u32 irs::arm::st_multi_adc_t::get_u32_data(irs_u8 /*a_channel*/)
{
  return m_value << (32 - adc_resolution);
}

float irs::arm::st_multi_adc_t::get_float_minimum()
{
  return -1.f;
}

float irs::arm::st_multi_adc_t::get_float_maximum()
{
  return 1.f;
}

float irs::arm::st_multi_adc_t::get_float_data(irs_u8 /*a_channel*/)
{
  return static_cast<float>(m_value)/adc_max_value;
}

float irs::arm::st_multi_adc_t::get_temperature()
{
  return 0;
}

float irs::arm::st_multi_adc_t::get_v_battery()
{
  return 0;
}

float irs::arm::st_multi_adc_t::get_temperature_degree_celsius(
  const float /*a_vref*/)
{
  return 0;
}

void irs::arm::st_multi_adc_t::tick()
{
  const bool timer_check = m_adc_timer.check();
  if (timer_check) {
    bool eoc1 = ADC_CSR_bit.EOC1;
    if (eoc1 == 1) {
      mp_adc->ADC_SR_bit.EOC = 0;
      mp_adc2->ADC_SR_bit.EOC = 0;
      irs_i16 data1 = static_cast<irs_i16>(mp_adc->ADC_DR);
      irs_i16 data2 = static_cast<irs_i16>(mp_adc2->ADC_DR);
      m_value = static_cast<irs_u16>(data1 - data2);
      mp_adc->ADC_CR2_bit.SWSTART = 1;
    }
  }
}

// class adc_rms_t
gtch::adc_rms_t::adc_rms_t(
  irs::arm::arm_spi_t* ap_spi,
  irs::adc_t* ap_adc,
  irs_u8 a_adc_channel,
  interrupt_generator_t* ap_interrupt_generator,
  size_type a_downsampling_factor,
  size_type a_period_sample_count,
  const std::vector<size_type>& a_windows_sko_period_count,
  size_type a_average_period_count,
  size_type a_result_sko_point_count,
  size_type a_delta_point_count
):
  mp_spi(ap_spi),
  mp_adc(ap_adc),
  m_adc_channel(a_adc_channel),
  m_current_type(current_type_ac),
  m_downsampling_factor(a_downsampling_factor),
  m_period_sample_count(a_period_sample_count),
  m_windows_sko_period_count(a_windows_sko_period_count),
  m_average_period_count(a_average_period_count),
  m_result_sko_point_count(a_result_sko_point_count),
  m_delta_point_count(a_delta_point_count),
  m_conversion_factor(1),
  m_buf(m_period_sample_count*period_count, 0),
  m_interrupt_index(0),
  m_interrupt_count(0),
  m_processed_index(0),
  m_adc_read_event(this),
  mp_interrupt_generator(ap_interrupt_generator),
  m_sko_calc(
    make_windows_sko_size(m_period_sample_count, m_windows_sko_period_count),
    m_period_sample_count*m_windows_sko_period_count[adc_type_slow]),

  m_sko_timer(irs::make_cnt_ms(100)),
  m_delta_timer(irs::make_cnt_ms(100)),
  m_results(),
  m_result_fade(0),
  m_show_sinus(false)
{
  m_buf.resize(m_period_sample_count*period_count, 0);

  for (size_type i = 0; i < a_windows_sko_period_count.size(); i++) {
    result_t result;

    result.sko.resize(m_result_sko_point_count);
    result.sko.resize_average(m_result_sko_point_count);

    result.delta_calc.resize(m_delta_point_count);

    m_results.push_back(result);
  }

  set_current_type(m_current_type);

  mp_interrupt_generator->add_event(&m_adc_read_event);
  m_adc_read_event.enable();
}

gtch::adc_rms_t::~adc_rms_t()
{
  m_adc_read_event.disable();
  mp_interrupt_generator->erase_event(&m_adc_read_event);
}

std::vector<gtch::adc_rms_t::size_type>
gtch::adc_rms_t::default_windows_sko_period_count()
{
  std::vector<size_type> windows_sko_period_count;
  windows_sko_period_count.push_back(50);
  windows_sko_period_count.push_back(5);
  return windows_sko_period_count;
}

std::vector<gtch::adc_rms_t::size_type>
gtch::adc_rms_t::make_windows_sko_size(
  size_type a_period_sample_count,
  const std::vector<size_type>& a_windows_sko_period_count)
{
  std::vector<size_type> windows_size;
  for (size_type i = 0; i < a_windows_sko_period_count.size(); i++) {
    size_type size = a_period_sample_count*a_windows_sko_period_count[i];
    windows_size.push_back(size);
  }
  return windows_size;
}

void gtch::adc_rms_t::select_channels(irs_u32 a_selected_channels)
{
  m_adc_read_event.disable();
  mp_adc->select_channels(a_selected_channels);
  reset();
  m_adc_read_event.enable();
}

void gtch::adc_rms_t::set_current_type(current_type_t a_type)
{
  m_current_type = a_type;
  if (m_current_type == current_type_ac) {
    m_sko_calc.resize_average(m_period_sample_count*m_average_period_count);
  } else {
    m_sko_calc.resize_average(
      m_period_sample_count*m_windows_sko_period_count[adc_type_slow]);
  }
  reset();
}

double gtch::adc_rms_t::get(size_type a_window_index) const
{
  return m_results[a_window_index].value*m_conversion_factor;
}

double gtch::adc_rms_t::get_slow_adc() const
{
  return get(adc_type_slow);
}

double gtch::adc_rms_t::get_fast_adc() const
{
  return get(adc_type_fast);
}

double gtch::adc_rms_t::get_voltage_code(size_type a_window_index) const
{
  return m_results[a_window_index].value;
}

double gtch::adc_rms_t::get_max_voltage_code() const
{
  return std::numeric_limits<irs_u16>::max();
}

double gtch::adc_rms_t::get_slow_adc_voltage_code() const
{
  return m_results[adc_type_slow].value;
}

double gtch::adc_rms_t::get_fast_adc_voltage_code() const
{
  return m_results[adc_type_fast].value;
}

void gtch::adc_rms_t::set_convertion_factor(double a_value)
{
  m_conversion_factor = a_value;
  IRS_LIB_DBG_MSG("Коэф. преобр. АЦП = " << a_value);
}

double gtch::adc_rms_t::get_fade() const
{
  return m_result_fade*m_conversion_factor;
}

double gtch::adc_rms_t::get_fade_t() const
{
  return 0.;
}

void gtch::adc_rms_t::set_fade_t(double /*a_t*/)
{
}

double gtch::adc_rms_t::get_result_sko(size_type a_window_index) const
{
  return m_results[a_window_index].sko.relative();
}

double gtch::adc_rms_t::get_slow_result_sko() const
{
  return m_results[adc_type_slow].sko.relative();
}

double gtch::adc_rms_t::get_fast_result_sko() const
{
  return m_results[adc_type_fast].sko.relative();
}

double gtch::adc_rms_t::get_result_average(size_type a_window_index) const
{
  return m_results[a_window_index].sko.average()*m_conversion_factor;
}

double gtch::adc_rms_t::get_slow_result_average() const
{
  return m_results[adc_type_slow].sko.average()*m_conversion_factor;
}

double gtch::adc_rms_t::get_fast_result_average() const
{
  return m_results[adc_type_fast].sko.average()*m_conversion_factor;
}

double gtch::adc_rms_t::get_delta(size_type a_window_index) const
{
  return m_results[a_window_index].delta_calc.relative();
}

double gtch::adc_rms_t::get_slow_delta() const
{
  return m_results[adc_type_slow].delta_calc.relative();
}

double gtch::adc_rms_t::get_fast_delta() const
{
  return m_results[adc_type_fast].delta_calc.relative();
}

void gtch::adc_rms_t::set_sko_point_count(size_type a_window_index,
  size_type a_sko_point_count)
{
  for (size_type i = 0; i < m_results.size(); i++) {
    m_results[a_window_index].sko.resize(a_sko_point_count);
    m_results[a_window_index].sko.resize_average(a_sko_point_count);
  }
}

void gtch::adc_rms_t::set_delta_point_count(size_type a_window_index,
  size_type a_delta_point_count)
{
  for (size_type i = 0; i < m_results.size(); i++) {
    m_results[a_window_index].delta_calc.resize(a_delta_point_count);
  }
}

void gtch::adc_rms_t::set_sko_period_count(size_type a_window_index,
  size_type a_sko_period_count)
{
  m_sko_calc.resize(a_window_index, m_period_sample_count*a_sko_period_count);
}

void gtch::adc_rms_t::set_sko_period_count(size_type a_sko_period_count)
{
  set_slow_adc_period_count(a_sko_period_count);
}

void gtch::adc_rms_t::set_slow_adc_period_count(
  size_type a_sko_period_count)
{
  set_sko_period_count(adc_type_slow, a_sko_period_count);
}

void gtch::adc_rms_t::set_fast_adc_period_count(
  size_type a_sko_period_count)
{
  set_sko_period_count(adc_type_fast, a_sko_period_count);
}

void gtch::adc_rms_t::set_average_period_count(
  size_type a_average_period_count)
{
  m_average_period_count = a_average_period_count;
  m_sko_calc.resize_average(m_period_sample_count*m_average_period_count);
}

double gtch::adc_rms_t::get_sko(size_type a_window_index) const
{
  return m_sko_calc.get(a_window_index);
}

double gtch::adc_rms_t::get_average() const
{
  return m_sko_calc.average();
}

void gtch::adc_rms_t::show_sinus()
{
  m_show_sinus = true;
}

void gtch::adc_rms_t::reset()
{
  m_adc_read_event.disable();
  m_interrupt_index = 0;
  m_processed_index = 0;
  m_sko_calc.clear();
  for (size_type i = 0; i < m_results.size(); i++) {
    m_results[i].value = 0;
    m_results[i].sko.clear();
    m_results[i].delta_calc.clear();
  }
  m_adc_read_event.enable();
}

void gtch::adc_rms_t::preset()
{
  // Не дописан!!!!!
  const size_type sample_count = m_sko_calc.size(adc_type_slow);
  const size_type period_count = sample_count/m_period_sample_count;
  if (period_count > 0) {
    const size_type strict_sample_count = m_period_sample_count*period_count;
    m_sko_calc.preset(0, strict_sample_count);
  }
}

bool gtch::adc_rms_t::ready(size_type a_window_index) const
{
  const size_type index = a_window_index;
  return (m_sko_calc.size(index) == m_sko_calc.max_size(index)) &&
    (m_sko_calc.average_size() == m_sko_calc.average_max_size());
}

bool gtch::adc_rms_t::fast_adc_ready() const
{
  return ready(adc_type_fast);
}

bool gtch::adc_rms_t::ready() const
{
  for (size_type i = 0; i < m_results.size(); i++) {
    if (!ready(i)) {
      return false;
    }
  }
  return true;
}

void gtch::adc_rms_t::tick()
{
  bool period_complete = false;
  const size_type interrupt_index = m_interrupt_index;
  if (m_processed_index <= interrupt_index) {
    size_type count = interrupt_index - m_processed_index;
    if (count >= m_period_sample_count) {
      period_complete = true;
    }
  } else {
    IRS_LIB_ASSERT(m_period_sample_count == (m_buf.size() - m_processed_index));
    period_complete = true;
  }
  if (period_complete) {
    const size_type index_end = m_processed_index + m_period_sample_count;
    IRS_LIB_ASSERT(index_end <= m_buf.size());

    if (m_show_sinus) {
      //m_interrupt_generator.stop();
    }
    static int count = 0;
    const int max_count = 3;
    static std::vector<sample_type> buf(m_period_sample_count*max_count, 0);
    static int buf_index = 0;
    if (m_current_type == current_type_ac) {
      if (m_sko_calc.size() == 0) {
        for (size_type i = m_processed_index; i < index_end; i++) {
          const sample_type sample = m_buf[i];
          m_sko_calc.add_to_average(sample);
        }
      }
    }
    for (size_type i = m_processed_index; i < index_end; i++) {
      const sample_type sample = m_buf[i];
      m_sko_calc.add(sample);
      if (m_show_sinus) {
        buf.at(buf_index) = sample;
        buf_index++;
      }
    }

    if (m_show_sinus) {
      count++;
    }
    if (count >= max_count) {
      m_adc_read_event.disable();
      for (int i = 0; static_cast<size_type>(i) < buf.size(); i++) {
        irs::mlog() << buf[i] << endl;
      }

      buf_index = 0;
      m_adc_read_event.enable();
      irs::mlog() << "---------------------------------------" << endl;
      count = 0;
      m_show_sinus = false;
    }


    const bool sko_add = m_sko_timer.check();
    const bool delta_add = m_delta_timer.check();
    if (m_current_type == current_type_dc) {
      for (size_type i = 0; i < m_results.size(); i++) {
        m_results[i].value = m_sko_calc.average();
      }
    } else {
      for (size_type i = 0; i < m_results.size(); i++) {
        m_results[i].value = m_sko_calc.get(i);
      }
    }
    for (size_type i = 0; i < m_results.size(); i++) {
      if (sko_add) {
        m_results[i].sko.add(m_results[i].value);
      }
      if (delta_add) {
        m_results[i].delta_calc.add(m_results[i].value);
      }
    }

    m_processed_index += m_period_sample_count;
    if (m_processed_index >= m_buf.size()) {
      m_processed_index = 0;
    }
  }
}

gtch::adc_rms_t::adc_read_event_t::adc_read_event_t(
  adc_rms_t* ap_adc_rms
):
  mp_adc_rms(ap_adc_rms),
  m_enabled(false)
{
}

void gtch::adc_rms_t::adc_read_event_t::exec()
{
  if (m_enabled) {
    mp_adc_rms->m_interrupt_count++;
    if (mp_adc_rms->m_interrupt_count == mp_adc_rms->m_downsampling_factor) {
      mp_adc_rms->m_interrupt_count = 0;
      irs::event_t::exec();

      irs::timer_t timeout(irs::make_cnt_ms(1));

      sample_type value = static_cast<sample_type>(
        mp_adc_rms->mp_adc->get_u16_data(
        mp_adc_rms->m_adc_channel));

      mp_adc_rms->m_buf[mp_adc_rms->m_interrupt_index] = value;
       mp_adc_rms->m_interrupt_index++;
      if (mp_adc_rms->m_interrupt_index >= mp_adc_rms->m_buf.size()) {
        mp_adc_rms->m_interrupt_index = 0;
      }

      for (int i  = 0; i < 2; i++) {
        mp_adc_rms->mp_adc->tick();
      }
    } else {
      mp_adc_rms->mp_adc->tick();
    }
  }
}

void gtch::adc_rms_t::adc_read_event_t::enable()
{
  m_enabled = true;
}

void gtch::adc_rms_t::adc_read_event_t::disable()
{
  m_enabled = false;
}

// class fast_adc_rms_adapter_t
gtch::fast_adc_rms_adapter_t::fast_adc_rms_adapter_t(
  adc_rms_t* ap_adc_rms
):
  mp_adc_rms(ap_adc_rms)
{
}

gtch::fast_adc_rms_adapter_t::size_type
gtch::fast_adc_rms_adapter_t::get_resulution() const
{
  return adc_resolution;
}

irs_u32 gtch::fast_adc_rms_adapter_t::get_u32_data(irs_u8 /*a_channel*/)
{
  const irs_u16 code = static_cast<irs_u16>(
    irs::bound<double>(mp_adc_rms->get_slow_adc_voltage_code(), 0, 0xFFFF));
  return code << (32 - adc_resolution);
}

void gtch::fast_adc_rms_adapter_t::tick()
{
}

// class dc_adc_t
gtch::dc_adc_t::dc_adc_t(irs::adc_t* ap_adc,
  irs_u8 a_adc_channel):
  mp_adc(ap_adc),
  m_channel(a_adc_channel),
  m_conversion_factor(1)
{
}

double gtch::dc_adc_t::get() const
{
  return mp_adc->get_float_data(m_channel)*m_conversion_factor;
}

void gtch::dc_adc_t::set_convertion_factor(double a_value)
{
  m_conversion_factor = a_value;
}
