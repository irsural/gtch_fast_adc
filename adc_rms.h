#ifndef adc_rmsH
#define adc_rmsH

#include <irsdefs.h>

#include <irsadc.h>
#include <irsalg.h>
#include <irsdsp.h>
#include <armspi.h>

#include "interrupt_generator.h"

#include <irsfinal.h>

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
  //void set_params(const std::vector<size_type>& a_windows_sko_period_count,
    //double a_fade_t);
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
  //size_type m_sko_period_count;
  std::vector<size_type> m_windows_sko_period_count;
  size_type m_average_period_count;
  size_type m_result_sko_point_count;
  size_type m_delta_point_count;
  double m_conversion_factor;
  std::vector<irs_u16> m_buf;
  size_type m_interrupt_index;
  size_type m_interrupt_count;
  size_type m_processed_index;
  adc_read_event_t m_adc_read_event;
  interrupt_generator_t* mp_interrupt_generator;
  irs::fast_multi_sko_with_single_average_t<irs_u16, double> m_sko_calc;
  //const size_type m_samples_max_size;
  //irs::deque_data_t<irs_u16> m_samples;
  //irs::delta_calc_t<double> m_delta_calc;
  //double m_result;
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
  //irs::fade_data_t m_adc_fade_data;
  double m_result_fade;
  bool m_show_sinus;
};

class fast_adc_rms_adapter_t: public irs::adc_t
{
public:
  enum { adc_resolution = 16 };
  fast_adc_rms_adapter_t(adc_rms_t* ap_adc_rms);
  virtual size_type get_resulution() const;
  virtual irs_u32 get_u32_data(irs_u8 a_channel);
  virtual void tick();
private:
  fast_adc_rms_adapter_t();
  adc_rms_t* mp_adc_rms;
};

class dc_adc_t
{
public:
  dc_adc_t(irs::adc_t* ap_adc,
    irs_u8 a_adc_channel);
  double get() const;
  void set_convertion_factor(double a_value);
private:
  irs::adc_t* mp_adc;
  irs_u8 m_channel;
  double m_conversion_factor;
};

} // namespace calibrator

#endif // adc_rmsH
