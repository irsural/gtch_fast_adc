#ifndef gtchcfgH
#define gtchcfgH

#include <irsdefs.h>

#include <irsgpio.h>
#include "defs.h"

#include <mxdata.h>

#include <armspi.h>
#include <irsadc.h>
#include <armadc.h>
#include <correct_alg.h>
#include <irsdev.h>
#include <mxdata.h>
#include <irsstd.h>
#include <irskbd.h>
#include <irsmem.h>

#include "interrupt_generator.h"
#include "gtchadc.h"
#include "gtchint.h"

#include <irsfinal.h>

namespace gtch
{

struct adc_settings_t
{
  typedef size_t size_type;
  typedef adc_filter_t::shift_type shift_type;
  float v_reference;
  float sampling_time;
  size_type filter_point_count;
  float filter_update_interval;
  shift_type shift;
  adc_settings_t(float a_v_reference, float a_sampling_time,
    size_type a_filter_point_count,
    float a_filter_update_interval,
    shift_type a_shift
  ):
    v_reference(a_v_reference),
    sampling_time(a_sampling_time),
    filter_point_count(a_filter_point_count),
    filter_update_interval(a_filter_update_interval),
    shift(a_shift)
  {
  }
};

class cfg_t
{
public:
  typedef size_t size_type;
  typedef interrupt_generator_t int_generator_t;
  typedef gtch::pa3_relay_interrupt_generator_t relay_interrupt_generator_t;
  enum { sinus_size = 256 };
  enum { sinus_pwm_frequency = 62500 };
  const double m_ir2183_dead_time;
private:
  #if GTCH_SK_STM32F217
  irs::arm::io_pin_t m_memory_chip_select_pin;
  #endif // GTCH_SK_STM32F217
  irs::arm::io_port_t m_lcd_port;
  irs::arm::io_pin_t m_lcd_rs_pin;
  irs::arm::io_pin_t m_lcd_e_pin;
  mxdisplay_drv_gen_t m_lcd_drv;
  vector<irs::handle_t<irs::gpio_pin_t> > m_key_drv_horizontal_pins;
  vector<irs::handle_t<irs::gpio_pin_t> > m_key_drv_vertical_pins;
  irs::mxkey_drv_mc_t m_keyboard_drv;
  const adc_settings_t m_adc_settings;
  irs::arm::st_adc_t m_adc;
  irs::blink_t m_debug_led;
  irs::pwm_pin_t m_buzzer_pin;
  irs::arm::io_pin_t m_relay_contact_pin;
  irs::arm::st_pwm_gen_t m_lcd_contrast_pwm;
  size_t m_sinus_pwm_timer;
  irs::arm::st_pwm_gen_t m_sinus_pwm;
  int_generator_t m_int_generator;
  enum  { spi_bitrate = 1000000 };
  irs::arm::arm_spi_t m_spi;
  irs::arm::io_pin_t m_eeprom_chip_select_pin;
  irs::eeprom_at25_t m_page_mem;
  irs::mem_data_t m_mem_data;
  //  Еепром параметров
  const irs_uarc m_eeprom_start_position;
  const irs_uarc m_eeprom_size;
  irs::mxdata_comm_t m_eeprom;
  const irs_uarc m_eeprom_correct_start_position;
  const irs_uarc m_eeprom_correct_size;
  irs::mxdata_comm_t m_eeprom_correct;
  irs::arm::st_dac_t m_current_limit_code;
  relay_interrupt_generator_t m_relay_interrupt_generator;
  //  Термодатчик
  const counter_t m_termo_interval;
  irs::arm::io_pin_t m_termometr_chip_select_pin;
  irs::th_lm95071_t m_termometr; // Перенести в app_t
  irs::arm::st_window_watchdog_t m_window_watchdog;
  irs::arm::st_independent_watchdog_t m_independent_watchdog;
public:
  irs::blink_t* get_debug_led();
  irs::gpio_pin_t* get_buzzer_pin();
  irs::gpio_pin_t* get_relay_contact_pin();
  irs::pwm_gen_t* get_lcd_contrast_pwm();
  irs::pwm_gen_t& get_sinus_pwm();
  int_generator_t& get_int_generator();
  mxdisplay_drv_t* get_lcd_drv();
  mxkey_drv_t& get_keyboard_drv();
  const adc_settings_t& get_adc_settings() const;
  irs::adc_t* get_adc();
  bool nonvolatile_memory_error();
  bool nonvolatile_memory_partial_error();
  irs::mxdata_t* get_nonvolatile_memory();
  irs::mxdata_t* get_correct_nonvolatile_memory();
  irs::dac_t* get_current_limit_code();
  relay_interrupt_generator_t* get_relay_interrupt_generator();
  float get_themperature_conv_koef();
  irs::mxdata_t& get_th_data();
  irs::watchdog_t* get_window_watchdog();
  irs::watchdog_t* get_independent_watchdog();
  bool overcurrent_dectected();
  cfg_t();
  ~cfg_t();
  void tick();
};

} //  gtch

#endif  //  gtchcfgH
