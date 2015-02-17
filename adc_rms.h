#ifndef adc_rmsH
#define adc_rmsH

#include <irsdefs.h>

#include <irsadc.h>
#include <irsalg.h>
#include <irsdsp.h>
#include <armspi.h>

#include "interrupt_generator.h"

#include <irsfinal.h>

namespace irs {

namespace arm {

class st_multi_adc_t: public adc_t
{
private:
  enum {
    ADC1_MASK = 1 << 30,
    ADC2_MASK = 1 << 29,
    ADC3_MASK = 1 << 28
  };
public:
  enum adc_channel_t {
    ADC123_PA0_CH0 = (1 << 0) | ADC1_MASK | ADC2_MASK | ADC3_MASK,
    ADC123_PA1_CH1 = (1 << 1) | ADC1_MASK | ADC2_MASK | ADC3_MASK,
    ADC123_PA2_CH2 = (1 << 2) | ADC1_MASK | ADC2_MASK | ADC3_MASK,
    ADC123_PA3_CH3 = (1 << 3) | ADC1_MASK | ADC2_MASK | ADC3_MASK,
    ADC12_PA4_CH4 = (1 << 4) | ADC1_MASK | ADC2_MASK,
    ADC3_PF6_CH4 = (1 << 4) | ADC3_MASK,
    ADC12_PA5_CH5 = (1 << 5) | ADC1_MASK | ADC2_MASK,
    ADC3_PF7_CH5 = (1 << 5) | ADC3_MASK,
    ADC12_PA6_CH6 = (1 << 6) | ADC1_MASK | ADC2_MASK,
    ADC3_PF8_CH6 = (1 << 6) | ADC3_MASK,
    ADC12_PA7_CH7 = (1 << 7) | ADC1_MASK | ADC2_MASK,
    ADC3_PF9_CH7 = (1 << 7) | ADC3_MASK,
    ADC12_PB0_CH8 = (1 << 8) | ADC1_MASK | ADC2_MASK,
    ADC3_PF10_CH8 = (1 << 8) | ADC3_MASK,
    ADC12_PB1_CH9 = (1 << 9) | ADC1_MASK | ADC2_MASK,
    ADC3_PF3_CH9 = (1 << 9) | ADC3_MASK,
    ADC123_PC0_CH10 = (1 << 10) | ADC1_MASK | ADC2_MASK | ADC3_MASK,
    ADC123_PC1_CH11 = (1 << 11) | ADC1_MASK | ADC2_MASK | ADC3_MASK,
    ADC123_PC2_CH12 = (1 << 12) | ADC1_MASK | ADC2_MASK | ADC3_MASK,
    ADC123_PC3_CH13 = (1 << 13) | ADC1_MASK | ADC2_MASK | ADC3_MASK,
    ADC12_PC4_CH14 = (1 << 14) | ADC1_MASK | ADC2_MASK,
    ADC3_PF4_CH14 = (1 << 14) | ADC3_MASK,
    ADC12_PC5_CH15 = (1 << 15) | ADC1_MASK | ADC2_MASK,
    ADC3_PF5_CH15 = (1 << 15) | ADC3_MASK,
    ADC1_TEMPERATURE = (1 << 16) | ADC1_MASK,
    ADC1_V_BATTERY = (1 << 18) | ADC1_MASK
  };
  st_multi_adc_t(adc_channel_t a_first_adc_channel,
    adc_channel_t a_second_adc_channel,
    counter_t a_adc_interval = make_cnt_ms(100));
  virtual ~st_multi_adc_t();
  virtual size_type get_resulution() const;
  virtual irs_u16 get_u16_minimum();
  virtual irs_u16 get_u16_maximum();
  virtual irs_u16 get_u16_data(irs_u8 a_channel);
  virtual irs_u32 get_u32_minimum();
  virtual irs_u32 get_u32_maximum();
  virtual irs_u32 get_u32_data(irs_u8 a_channel);
  virtual float get_float_minimum();
  virtual float get_float_maximum();
  virtual float get_float_data(irs_u8 a_channel);
  virtual float get_temperature();
  virtual float get_v_battery();
  virtual void tick();
  float get_temperature_degree_celsius(const float a_vref);
private:
  irs_u32 adc_channel_to_channel_index(adc_channel_t a_adc_channel);
  adc_regs_t* mp_adc;
  adc_regs_t* mp_adc2;
  irs::loop_timer_t m_adc_timer;
  enum { reqular_channel_count = 16 };
  enum { adc_resolution = 13 };
  enum { adc_max_value = 0xFFF };
  irs_u32 m_value;
};

}

}

namespace gtch {

class adc_rms_t
{
public:
  typedef irs_size_t size_type;
  enum adc_type_t {
    adc_type_slow = 0,
    adc_type_fast = 1
  };
  enum current_type_t {
    current_type_dc,
    current_type_ac
  };
  adc_rms_t(irs::arm::arm_spi_t* ap_spi,
    irs::adc_t* ap_adc,
    irs_u8 a_adc_channel,
    interrupt_generator_t* ap_interrupt_generator,
    //! \param[in] a_downsampling_factor - коэффициент децимации
    //!   Если a_downsampling_factor == 1, то децимация не происходит
    size_type a_downsampling_factor = 1,
    size_type a_period_sample_count = 128,
    const std::vector<size_type>& a_windows_sko_period_count =
      default_windows_sko_period_count(),
    size_type a_average_period_count = 4,
    size_type a_result_sko_point_count = 100,
    size_type a_delta_point_count = 100);
  ~adc_rms_t();
  void select_channels(irs_u32 a_selected_channels);
  void set_current_type(current_type_t a_type);
  double get(size_type a_window_index = adc_type_slow) const;
  double get_slow_adc() const;
  double get_fast_adc() const;
  double get_voltage_code(size_type a_window_index = 0) const;
  double get_max_voltage_code() const;
  double get_slow_adc_voltage_code() const;
  double get_fast_adc_voltage_code() const;
  void set_convertion_factor(double a_value);
  double get_fade() const;
  double get_fade_t() const;
  void set_fade_t(double a_t);
  double get_result_sko(size_type a_window_index) const;
  double get_slow_result_sko() const;
  double get_fast_result_sko() const;

  double get_result_average(size_type a_window_index) const;
  double get_slow_result_average() const;
  double get_fast_result_average() const;

  double get_delta(size_type a_window_index) const;
  double get_slow_delta() const;
  double get_fast_delta() const;
  void set_sko_point_count(size_type a_window_index,
    size_type a_sko_point_count);
  void set_delta_point_count(size_type a_window_index,
    size_type a_delta_point_count);
  void set_sko_period_count(size_type a_window_index,
    size_type a_sko_period_count);
  //! \brief Функция для обратной совместимости. В дальнейшем можно удалить
  void set_sko_period_count(size_type a_sko_period_count);
  void set_slow_adc_period_count(size_type a_sko_period_count);
  void set_fast_adc_period_count(size_type a_sko_period_count);
  void set_average_period_count(size_type a_average_period_count);
  double get_sko(size_type a_window_index) const;
  double get_average() const;
  void show_sinus();
  void reset();
  void preset();
  bool ready(size_type a_window_index) const;
  bool fast_adc_ready() const;
  bool ready() const;
  void tick();
private:
  static std::vector<size_type> default_windows_sko_period_count();
  std::vector<size_type> make_windows_sko_size(
    size_type a_period_sample_count,
    const std::vector<size_type>& a_windows_sko_period_count);
  class adc_read_event_t: public irs::event_t
  {
  public:
    adc_read_event_t(adc_rms_t* ap_adc_ad7686_rms);
    virtual void exec();
    void enable();
    void disable();
  private:
    adc_rms_t* mp_adc_rms;
    bool m_enabled;
  };
  friend class adc_read_event_t;
  enum { period_count = 4 };
  irs::arm::arm_spi_t* mp_spi;
  irs::adc_t* mp_adc;
  const irs_u8 m_adc_channel;
  current_type_t m_current_type;
  size_type m_downsampling_factor;
  size_type m_period_sample_count;
  std::vector<size_type> m_windows_sko_period_count;
  size_type m_average_period_count;
  size_type m_result_sko_point_count;
  size_type m_delta_point_count;
  double m_conversion_factor;
  typedef irs_i16 sample_type;
  std::vector<sample_type> m_buf;
  size_type m_interrupt_index;
  size_type m_interrupt_count;
  size_type m_processed_index;
  adc_read_event_t m_adc_read_event;
  interrupt_generator_t* mp_interrupt_generator;
  irs::fast_multi_sko_with_single_average_t<sample_type, double> m_sko_calc;
  irs::loop_timer_t m_sko_timer;
  irs::loop_timer_t m_delta_timer;
  struct result_t
  {
    double value;
    irs::fast_sko_t<double, double> sko;
    irs::delta_calc_t<double> delta_calc;
    result_t():
      value(0),
      sko(0, 0),
      delta_calc(0)
    {
    }
  };
  vector<result_t> m_results;
  double m_result_fade;
  bool m_show_sinus;
};

} // namespace calibrator

#endif // adc_rmsH
