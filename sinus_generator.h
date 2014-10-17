#ifndef sinus_generatorH
#define sinus_generatorH

#include <irsdefs.h>

#include <irsdev.h>

#include "interrupt_generator.h"

#include <irsfinal.h>

size_t shift_for_left_aligment_irs_i16(const irs_i16 a_value);

class st_pwm_gen_adapter_t: public irs::pwm_gen_t
{
public:
  typedef irs_size_t size_type;
  st_pwm_gen_adapter_t(irs::arm::st_pwm_gen_t* ap_st_pwm_gen,
    gpio_channel_t a_first_channel,
    gpio_channel_t a_second_channel
  );
  virtual void start();
  virtual void stop();
  virtual void set_duty(irs_uarc a_duty);
  virtual void set_duty(float a_duty);
  virtual irs::cpu_traits_t::frequency_type set_frequency(
    irs::cpu_traits_t::frequency_type  a_frequency);
  virtual irs_uarc get_max_duty();
  virtual irs::cpu_traits_t::frequency_type get_max_frequency();
  virtual irs::cpu_traits_t::frequency_type get_timer_frequency();
private:
  irs::arm::st_pwm_gen_t* mp_st_pwm_gen;
  gpio_channel_t m_first_channel;
  gpio_channel_t m_second_channel;
};

class sinus_generator_t
{
public:
  typedef std::size_t size_type;
  typedef interrupt_generator_t::counter_type period_t;
  typedef interrupt_generator_t::calc_type calc_period_t;
  sinus_generator_t(interrupt_generator_t* ap_interrupt_generator,
    size_type a_interrupt_freq_boost_factor,
    irs::pwm_gen_t* ap_pwm_gen, double a_pwm_frequency, double a_dead_time,
    size_type a_sinus_size);
  ~sinus_generator_t();
  virtual void start();
  virtual void stop();
  virtual bool started() const;
  virtual void set_value(unsigned int a_value);
  virtual void set_amplitude(double a_amplitude);
  virtual double get_amplitude();
  virtual void set_frequency(double a_frequency);
  virtual unsigned int get_max_value();
  virtual double get_max_amplitude();
  virtual double get_max_frequency();
  void set_correct_freq_koef(double a_correct_koef);
private:
  // Множитель, чтобы взять dead time с запасом
  enum { dead_time_count = 3 };
  class point_interrupt_event_t: public irs::event_t
  {
  public:
    point_interrupt_event_t(sinus_generator_t* ap_owner);
    virtual void exec();
    void enable();
    void disable();
  private:
    sinus_generator_t* mp_owner;
    bool m_enabled;
  };
  friend class point_interrupt_event_t;

  const unsigned int m_max_value;
  const double m_min_amplitude;
  const double m_max_amplitude;
  const double m_min_frequency;
  const double m_max_frequency;
  const double m_Kf;
  const double m_dead_time;
  const size_t m_shift_alignment_irs_i16;
  const double m_max_sinus_value;
  enum { irs_u16_bits_count = 16 };
  bool m_started;
  bool m_amplitude_changed;
  interrupt_generator_t* mp_interrupt_generator;
  size_type m_interrupt_freq_boost_factor;
  irs::pwm_gen_t* mp_pwm_gen;
  double m_pwm_frequency;
  point_interrupt_event_t m_point_interrupt_event;

  double m_frequency;
  double m_amplitude;
  irs_u16 m_current_point;
  size_type m_interrupt_count;
  period_t m_floor_interval;
  //irs_u8 m_floor_len;
  size_type m_sinus_size;
  std::vector<double> m_ref_sinus;
  std::vector<irs_i16> m_sinus_array_1;
  std::vector<irs_i16> m_sinus_array_2;
  std::vector<irs_i16>* mp_active_sinus_array;
  std::vector<irs_i16>* mp_inactive_sinus_array;
  double m_freq_trans_koef;
  double m_default_freq_trans_koef;

  void point_interrupt();
  //void fill_sinus_array();
  void fill_sinus_array(std::vector<irs_i16>* ap_array);
  void apply_frequency();
  inline period_t get_interval()
  {
    return m_floor_interval;
  }
};

#endif // sinus_generatorH
