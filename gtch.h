#ifndef gtchH
#define gtchH

//#define FLASH_SINUS

#include <irsdefs.h>

#include <timer.h>
#include <irsmenu.h>
#include <irsdsp.h>
#include <irsalg.h>
#include <mxdata.h>

#include "gtchcfg.h"
#include "defs.h"
#include "sinus_generator.h"
#include "adc_rms.h"

#include <irsfinal.h>

namespace gtch {

enum {
  num_of_points = 256
};

#ifdef FLASH_SINUS
typedef const __flash irs_i16 ref_sinus_sample_t;
#else //FLASH_SINUS
typedef irs_i16 ref_sinus_sample_t;
#endif //FLASH_SINUS
typedef ref_sinus_sample_t* ref_sinus_sample_ponter_t;
#ifdef FLASH_SINUS
ref_sinus_sample_ponter_t get_ref_sinus();
#endif //FLASH_SINUS

const char voltage_setup_str_1[] = "Установка";
const char voltage_setup_str_2[] = "напряжения";
const char pause_str[] = "Пауза";
const char freq_up_str[] = "F^ ";
const char freq_down_str[] = "F~ ";
const char time_up_str[] = "T^ ";
const char time_down_str[] = "T~ ";
const char relay_no_action_str[] = "-----";

struct nonvolatile_data_t
{
  irs::conn_data_t<float> ac_k;
  irs::conn_data_t<float> ac_ki;
  irs::conn_data_t<float> ac_kd;
  irs::conn_data_t<float> dc_k;
  irs::conn_data_t<float> dc_ki;
  irs::conn_data_t<float> dc_kd;
  irs::conn_data_t<float> contrast;
  irs::conn_data_t<float> current_limit;
  irs::conn_data_t<float> termo_limit;
  irs::conn_data_t<float> temp_2;
  irs::conn_data_t<float> temp_3;
  irs::conn_data_t<float> freq_begin;
  irs::conn_data_t<float> freq_end;
  irs::conn_data_t<float> speed_freq;
  irs::conn_data_t<float> ac_voltage_ref;
  irs::conn_data_t<float> dc_voltage_ref;
  irs::conn_data_t<bool> mode_dc_enabled;
  irs::conn_data_t<bool> go_to_end;
  irs::conn_data_t<bool> need_meas_time;
  irs::conn_data_t<bool> correct_freq_by_time;
  irs::conn_data_t<bool> tuda_obratno;
  irs::conn_data_t<bool> ac_need_meas_koef;
  irs::conn_data_t<bool> dc_need_meas_koef;
  irs::conn_data_t<bool> reset_nonvolatile_memory;
  irs::conn_data_t<bool> reset_correct_nonvolatile_memory;
  irs::conn_data_t<bool> voltage_correct_enable;
  irs::conn_data_t<bool> dc_voltage_correct_enable;
  irs::conn_data_t<float> dc_voltage_correct_koef;
  irs::conn_data_t<bool> freq_correct_enable;
  irs::conn_data_t<float> freq_correct_koef;
  irs::conn_data_t<float> ac_t_adc;
  irs::conn_data_t<float> ac_iso_k;
  irs::conn_data_t<float> ac_iso_t;
  irs::conn_data_t<float> ac_dead_band;
  irs::conn_data_t<float> dc_t_adc;
  irs::conn_data_t<float> dc_iso_k;
  irs::conn_data_t<float> dc_iso_t;
  irs::conn_data_t<float> dc_dead_band;
  irs::conn_data_t<irs_u16> id;
  nonvolatile_data_t(irs::mxdata_t *ap_data = 0, irs_uarc a_start_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    if (ap_data) {
      irs_uarc size = connect(ap_data, a_start_index);
      if (ap_size != IRS_NULL) {
        *ap_size = size;
      }
    }
  };
  irs_uarc connect(irs::mxdata_t *ap_data = 0, irs_uarc a_start_index = 0)
  {
    irs_uarc index = a_start_index;
    index = ac_k.connect(ap_data, index);
    index = ac_ki.connect(ap_data, index);
    index = ac_kd.connect(ap_data, index);
    index = dc_k.connect(ap_data, index);
    index = dc_ki.connect(ap_data, index);
    index = dc_kd.connect(ap_data, index);
    index = contrast.connect(ap_data, index);
    index = current_limit.connect(ap_data, index);
    index = termo_limit.connect(ap_data, index);
    index = temp_2.connect(ap_data, index);
    index = temp_3.connect(ap_data, index);
    index = freq_begin.connect(ap_data, index);
    index = freq_end.connect(ap_data, index);
    index = speed_freq.connect(ap_data, index);
    index = ac_voltage_ref.connect(ap_data, index);
    index = dc_voltage_ref.connect(ap_data, index);
    index = mode_dc_enabled.connect(ap_data, index);
    index = go_to_end.connect(ap_data, index);
    index = need_meas_time.connect(ap_data, index);
    index = correct_freq_by_time.connect(ap_data, index);
    index = tuda_obratno.connect(ap_data, index);
    index = ac_need_meas_koef.connect(ap_data, index);
    index = dc_need_meas_koef.connect(ap_data, index);
    index = reset_nonvolatile_memory.connect(ap_data, index);
    index = reset_correct_nonvolatile_memory.connect(ap_data, index);
    index = voltage_correct_enable.connect(ap_data, index);
    index = dc_voltage_correct_enable.connect(ap_data, index);
    index = dc_voltage_correct_koef.connect(ap_data, index);
    index = freq_correct_enable.connect(ap_data, index);
    index = freq_correct_koef.connect(ap_data, index);
    index = ac_t_adc.connect(ap_data, index);
    index = ac_iso_k.connect(ap_data, index);
    index = ac_iso_t.connect(ap_data, index);
    index = ac_dead_band.connect(ap_data, index);
    index = dc_t_adc.connect(ap_data, index);
    index = dc_iso_k.connect(ap_data, index);
    index = dc_iso_t.connect(ap_data, index);
    index = dc_dead_band.connect(ap_data, index);
    index = id.connect(ap_data, index);
    return index;
  };
};

class hold_key_wait_t {
private:
  enum mode_t { mode_key_down, mode_key_up };

  mxkey_drv_t& m_key_drv;
  irs::timer_t m_hold_key_timer;
  mode_t m_mode;
  irskey_t m_key;
public:
  hold_key_wait_t(mxkey_drv_t& a_key_drv, counter_t a_hold_key_time,
    irskey_t a_key
  ):
    m_key_drv(a_key_drv),
    m_hold_key_timer(a_hold_key_time),
    m_mode(mode_key_down),
    m_key(a_key)
  {
  }
  bool check(bool a_is_main_screen_cur)
  {
    irskey_t key = m_key_drv();
    bool check = false;
    switch (m_mode) {
      case mode_key_down: {
        if (a_is_main_screen_cur && (key == m_key)) {
          m_hold_key_timer.start();
          m_mode = mode_key_up;
        }
      } break;
      case mode_key_up: {
        if (m_hold_key_timer.check()) {
          check = true;
          m_mode = mode_key_down;
        } else {
          if (key != m_key) {
            m_mode = mode_key_down;
          }
        }
      } break;
    }
    return check;
  }
};

//------------------------------------------------------------------------------

class gtch_reg_t
{
public:
  gtch_reg_t(float a_reg_interval, float a_meas_interval);
  ~gtch_reg_t();
  void set_voltage_ref(float a_ref);
  void set_k(float a_k);
  void set_ki(float a_ki);
  void set_kd(float a_kd);
  bool ready();
  void start();
  void stop();
  void tick(float a_voltage);
};

//------------------------------------------------------------------------------

class buzzer_t
{
public:
  buzzer_t(irs::gpio_pin_t* ap_buzzer_pin);
  ~buzzer_t();
  void bzzz();
  void bzz();
  void tick();
private:
  const counter_t m_bzz_interval;
  const counter_t m_bzzz_interval;
  bool m_buzzed;
  irs::timer_t m_timer;
  irs::gpio_pin_t* mp_pin;
};

//------------------------------------------------------------------------------

template <class T>
class bounded_user_input_t
{
public:
  bounded_user_input_t(T& a_var, T a_step, T a_step_max,
    const T& a_min, const T& a_max);
  ~bounded_user_input_t();
  void add_change_event(mxfact_event_t *ap_event);
  inline void operator ++ (int) { inc(); }
  inline void operator -- (int) { dec(); }
  void process(irskey_t a_key);
  void inc();
  void dec();
  void set_keys(const irskey_t a_key_inc, const irskey_t a_key_dec);
  void reset_step();
  void set_step(T a_step, T a_step_max);
private:
  enum process_t {
    INC,
    DEC,
    NONE
  };
  enum {
    m_step_count_before_step_increment = 9
  };
  T &m_var;
  T m_step;
  T m_step_max;
  const T& m_min;
  const T& m_max;
  mxfact_event_t *mp_event;
  T m_current_step;
  irs_u8 m_step_count;
  process_t m_process;
  irskey_t m_key_inc;
  irskey_t m_key_dec;
};

template <class T>
bounded_user_input_t<T>::bounded_user_input_t(T& a_var, T a_step,
  T a_step_max, const T& a_min, const T& a_max
):
  m_var(a_var),
  m_step(a_step),
  m_step_max(a_step_max),
  m_min(a_min),
  m_max(a_max),
  mp_event(0),
  m_current_step(a_step),
  m_step_count(0),
  m_process(NONE),
  m_key_inc(irskey_none),
  m_key_dec(irskey_none)
{
}

template <class T>
bounded_user_input_t<T>::~bounded_user_input_t()
{
}

template <class T>
void bounded_user_input_t<T>::add_change_event(mxfact_event_t *ap_event)
{
  ap_event->connect(mp_event);
}

template <class T>
void bounded_user_input_t<T>::process(irskey_t a_key)
{
  if ((m_key_inc == irskey_none) && (m_key_dec == irskey_none)) {
    return;
  }

  if (a_key == m_key_inc) {
    if (m_process == DEC) {
      m_current_step = m_step;
      m_step_count = 0;
    }
    m_process = INC;
    if (m_step_count >= m_step_count_before_step_increment) {
      m_current_step = min(m_current_step*10, m_step_max);
      m_step_count = 0;
    } else {
      m_step_count++;
    }
    inc();
  } else if (a_key == m_key_dec) {
    if (m_process == INC) {
      m_current_step = m_step;
      m_step_count = 0;
    }
    m_process = DEC;
    if (m_step_count >= m_step_count_before_step_increment) {
      m_current_step = min(m_current_step*10, m_step_max);
      m_step_count = 0;
    } else {
      m_step_count++;
    }
    dec();
  }
}

template <class T>
void bounded_user_input_t<T>::inc()
{
  irs_i64 int_var = static_cast<irs_i64>((m_var/m_current_step) +
    static_cast<T>(0.05));
  m_var = static_cast<T>(int_var + 1);
  m_var *= m_current_step;
  if (m_var > m_max) {
    m_var = m_max;
  }
  if (mp_event) {
    mp_event->exec();
  }
}

template <class T>
void bounded_user_input_t<T>::dec()
{
  irs_i64 int_var = static_cast<irs_i64>((m_var/m_current_step) +
    static_cast<T>(0.95));
  m_var = static_cast<T>(int_var - 1);
  m_var *= m_current_step;
  if (m_var < m_min) {
    m_var = m_min;
  }
  if (mp_event) {
    mp_event->exec();
  }
}

template <class T>
void bounded_user_input_t<T>::set_keys(const irskey_t a_key_inc,
  const irskey_t a_key_dec)
{
  m_key_inc = a_key_inc;
  m_key_dec = a_key_dec;
}

template <class T>
void bounded_user_input_t<T>::reset_step()
{
  m_current_step = m_step;
}

template <class T>
void bounded_user_input_t<T>::set_step(T a_step, T a_step_max)
{
  m_step = a_step;
  m_step_max = a_step_max;
  reset_step();
}

//------------------------------------------------------------------------------

struct reg_options_t
{
  typedef size_t size_type;
  enum {
    CAN_WRITE = true,
    CANNOT_WRITE = false
  };
  //  К
  double k;
  irs_menu_double_item_t k_item;
  //  Кi
  double ki;
  irs_menu_double_item_t ki_item;
  //  Кd
  double kd;
  irs_menu_double_item_t kd_item;
  //
  bool can_edit;
  //  Переизмерение коэффициентов
  bool need_meas_koef;
  irs_menu_bool_item_t need_meas_koef_item;
  //  Постоянная фильтра АЦП
  double t_adc;
  irs_menu_double_item_t t_adc_item;
  bool voltage_filter_on;
  //  Изодромное звено: К
  double iso_k;
  irs_menu_double_item_t iso_k_item;
  //  Изодромное звено: Т
  double iso_t;
  irs_menu_double_item_t iso_t_item;
  // Зона нечувствительности
  double dead_band;
  irs_menu_double_item_t dead_band_item;
  reg_options_t(char* ap_user_str, char* ap_exit_msg,
    size_type a_menu_item_width, size_type a_menu_item_prec);
private:
  reg_options_t();
};

irs::string_t align_center(const irs::string_t a_str, const size_t a_width,
  const irs::char_t a_fill = irst(' '));

template <class CFG>
class app_t
{
  typedef size_t size_type;
  static const irs_u16 m_num_of_points = num_of_points;
  typedef typename CFG::int_generator_t int_generator_t;

  typedef double adc_data_t;
  typedef typename CFG::relay_interrupt_generator_t relay_interrupt_generator_t;

  enum mode_dc_ac_t {
    MODE_DC,
    MODE_AC
  };
  enum status_t {
    SPLASH,
    STOP,
    OVERHEAT,
    SETUP_MEAS,
    SETUP,
    SETUP_READY,
    SCAN,
    PAUSE,
    MESSAGE,
    CORRECT_ACV_SETUP_MEAS,
    CORRECT_ACV_PAUSE,
    CORRECT_DCV,
    CORRECT_F
  };
  enum scan_status_t {
    SCAN_STOP,

    SCAN_FORWARD_SETUP,
    SCAN_FORWARD,
    SCAN_RETURN_SETUP,
    SCAN_RETURN,

    SCAN_FORWARD_JUMP_SETUP,
    SCAN_FORWARD_JUMP,
    SCAN_RETURN_JUMP_SETUP,
    SCAN_RETURN_JUMP
  };
  enum scan_mode_t {
    SM_TUDA,
    SM_TUDA_WITH_JUMP,
    SM_TUDA_OBRATNO,
    SM_TUDA_OBRATNO_WITH_JUMPS
  };
  enum direction_t {
    UP,
    DOWN
  };
  enum interface_mode_t {
    DEBUG,
    RELEASE
  };
  enum {
    CAN_WRITE = true,
    CANNOT_WRITE = false
  };
  enum {
    SHOW_ITEM = true,
    HIDE_ITEM = false
  };

  //  Горячие клавиши
  enum {
    m_key_Fn_up = irskey_7,
    m_key_Fn_down = irskey_4,
    m_key_Fk_up = irskey_8,
    m_key_Fk_down = irskey_5,
    m_key_S_up = irskey_9,
    m_key_S_down = irskey_6,
    m_key_V_up = irskey_up,
    m_key_V_down = irskey_down,
    m_key_start = irskey_1,
    m_key_pause = irskey_1,
    m_key_stop = irskey_2,
    m_key_esc = irskey_escape
  };
  // Пароль id
  enum { m_id_pass_src = 74 };

  CFG &m_cfg;
  irs::blink_t* mp_debug_led;
  irs::gpio_pin_t *mp_relay_contact_pin;
  irs::pwm_gen_t* mp_lcd_contrast_pwm;
  irs::pwm_gen_t &m_sinus_pwm;
  bool m_overcurrent_detection_enabled;
  int_generator_t &m_interrupt_generator;

  // Количество точек на период синусоиды
  size_type m_period_sample_count;
  // Количество периодов синусоиды, которое участвует в усреднении
  size_type m_sko_period_count;
  size_type m_short_sko_period_count;
  // Количество периодов синусоиды, которое участвует в расчете среднего
  size_type m_average_period_count;
  // Количество точек для вычисления СКО результата
  size_type m_result_sko_point_count;
  size_type m_delta_point_count;
  // Во сколько раз частота считывания точек с АЦП меньше частоты прерываний
  size_type m_downsampling_factor;
  gtch::adc_rms_t m_fast_adc_rms;
  enum { interrupt_freq_boost_factor = 2 };
  irs::handle_t<generator_t> mp_generator;
  mxdisplay_drv_t* mp_lcd_drv;
  mxdisplay_drv_service_t m_lcd_drv_service;
  mxkey_event_t m_buzzer_kb_event;
  mxkey_event_t m_hot_kb_event;
  mxkey_event_t m_menu_kb_event;
  mxkey_event_gen_t m_keyboard_event_gen;
  mxkey_drv_t &m_keyboard_drv;
  const float m_filter_update_interval;
  irs::loop_timer_t m_filter_update_interval_timer;

  adc_settings_t m_adc_settings;
  irs::fade_data_t m_adc_fade_data;
  //  Non-volatile memory
  irs_uarc m_nonvolatile_data_size;
  nonvolatile_data_t m_nonvolatile_data;
  const float m_dac_v_reference;
  irs::dac_t* mp_current_limit_code;

  //  Прерывание от контактов реле
  class gtch_relay_int_t : public mxfact_event_t
  {
    app_t<CFG> &m_outer;
  public:
    gtch_relay_int_t(app_t<CFG> &a_outer);
    virtual void exec();
  };
  friend class gtch_relay_int_t;
  relay_interrupt_generator_t* mp_relay_interrupt_generator;
  gtch_relay_int_t m_gtch_relay_interrupt;
  //  Регулятор
  const counter_t m_meas_interval;
  const counter_t m_reg_interval;
  const float m_meas_gen_value;
  const float m_operating_duty_deviation;
  // Если true, то прибор в рабочем режиме
  bool m_operating_duty;
  irs::timer_t m_reg_timer;
  irs::pid_data_t m_reg_pd;
  irs::rate_limiter_t<double> m_rate_data;
  float m_meas_k;
  const float m_ac_meas_k_default;
  const float m_dc_meas_k_default;
  const float m_min_meas_k_voltage;
  bool m_reg_koefs_changed;
  bool m_can_reg;
  //
  mode_dc_ac_t m_mode_dc_ac;
  status_t m_status;
  irs::timer_t m_splash_timer;
  //  Отладочный режим
  interface_mode_t m_interface_mode;
  enum { key_array_size = 5 };
  irskey_t m_current_interface_mode_key_array[key_array_size];
  irskey_t m_ref_interface_mode_key_array[key_array_size];
  irs_u8 m_current_interface_mode_key;
  size_t m_temperature_item_pos;
  //  Menu
  irs::loop_timer_t m_menu_timer;
  buzzer_t m_buzzer;
  //  Строки и бегущая строка
  static const irs_u8 m_user_str_len = 30;
  char mp_user_str[m_user_str_len + 1];

  static const irs_u8 m_creep_len = 100;
  static const irs_u8 m_creep_static_path_len = 20;
  static const irs_u8 m_creep_dynamic_path_len = 40;
  static const irs_u8 m_creep_message = m_creep_len - m_creep_static_path_len -
    m_creep_dynamic_path_len;
  static const irs_u8 m_creep_time_num = 1;
  static const irs_u8 m_creep_time_denom = 5;
  char mp_enter_msg[m_creep_message];
  char mp_exit_msg[m_creep_message];
  char mp_creep_buffer[m_creep_len+1];
  irs_menu_creep_t m_main_creep;

  enum {
    m_half_screen = 10,
    m_menu_item_width = 5,
    m_menu_reg_options_item_width = 6,
    m_menu_item_prec = 2,
    m_time_item_width = 5,
    m_time_item_prec = 4,
    m_termo_item_prec = 2,
    m_adc_item_width = 5,
    m_adc_item_prec = 0,
    m_reg_item_width = 7,
    m_reg_item_prec = 5,
    m_meas_k_item_width = 5,
    m_meas_k_item_prec = 4,
    m_volt_item_width = 5,
    m_volt_item_prec = 0,
    m_volt_item_width_debug = 6,
    m_volt_item_prec_debug = 2,
    m_v_correct_item_width = 6,
    m_v_correct_item_prec = 2,
    m_dcv_correct_item_width = 6,
    m_dcv_correct_item_prec = 2,
    m_f_correct_item_width = 6,
    m_f_correct_item_prec = 3,
    m_id_item_width = 5,
    m_id_item_prec = 0
  };
  //  Основной экран
  irs_advanced_tablo_t m_main_screen;
  //  Главное меню
  irs_advanced_menu_t m_main_menu;
  //  Меню параметров ac регулятора
  irs_advanced_menu_t m_ac_reg_menu;
  //  Меню параметров dc регулятора
  irs_advanced_menu_t m_dc_reg_menu;
  //  Меню параметров защит
  irs_advanced_menu_t m_limit_menu;
  //  Частота
  const double m_min_freq;
  const double m_max_freq;
  const double m_freq_step;
  const double m_freq_step_max;
  double m_freq;
  irs_menu_simply_item_t<double> m_freq_item;
  //  Скорость изменения частоты
  const double m_min_speed;
  const double m_max_speed;
  const double m_speed_step;
  const double m_steed_step_max;
  double m_speed_freq_saved_copy;
  double m_speed_freq;
  irs_menu_simply_item_t<double> m_speed_freq_item;
  //  Начальная частота
  double m_freq_begin_saved_copy;
  double m_freq_begin;
  irs_menu_simply_item_t<double> m_freq_begin_item;
  //  Конечная частота
  double m_freq_end_saved_copy;
  double m_freq_end;
  irs_menu_simply_item_t<double> m_freq_end_item;
  //  Уставка напряжения
  const double m_min_ac_voltage_debug;
  const double m_max_ac_voltage_debug;
  const double m_min_ac_voltage_release;
  const double m_max_ac_voltage_release;
  const double m_min_dc_voltage_debug;
  const double m_max_dc_voltage_debug;
  const double m_min_dc_voltage_release;
  const double m_max_dc_voltage_release;
  double m_min_voltage;
  double m_max_voltage;
  const double m_voltage_step_release;
  const double m_voltage_step_max_release;
  const double m_voltage_step_debug;
  const double m_voltage_step_max_debug;  
  double m_voltage_ref;
  irs_menu_simply_item_t<double> m_voltage_ref_item;
  //  Напряжение
  double m_voltage;
  double m_voltage_display;
  irs_menu_simply_item_t<double> m_voltage_item;
  //  Уставка защиты по току
  const double m_trans_koef;
  double m_current_limit;
  irs_menu_double_item_t m_current_limit_item;
  //  Температура
  irs::th_lm95071_data_t m_temperature_conn_data;
  irs::watchdog_t* mp_window_watchdog;
  irs::watchdog_t* mp_independent_watchdog;
  double m_temperature;
  irs_menu_simply_item_t<double> m_temperature_item;
  //  Уставка защиты по температуре
  const double m_termo_hist;
  double m_termo_limit;
  irs_menu_double_item_t m_termo_limit_item;
  //  Контрастность
  double m_contrast;
  irs_menu_double_item_t m_contrast_item;

  irs::handle_t<reg_options_t> mp_ac_reg_options;
  irs::handle_t<reg_options_t> mp_dc_reg_options;

  bool m_can_edit;
  bool m_voltage_filter_on;
  //  Изодромное звено
  irs::isodr_data_t m_iso_data;
  //  АЦП
  adc_data_t m_adc_value;
  irs_menu_simply_item_t<adc_data_t> m_adc_item;
  //  ШИМ
  double m_pwm_out;
  irs_menu_simply_item_t<double> m_pwm_out_item;
  //  Коэффициент регулятора
  irs_menu_simply_item_t<float> m_meas_k_item;
  //  Пункты меню сканирования
  irs_advanced_menu_t m_scan_menu;
  //  Режим постоянного тока
  bool m_mode_dc_enabled_previous;
  bool m_mode_dc_enabled;
  irs_menu_bool_item_t m_mode_dc_item;
  //  Идти до конца
  bool m_scan_go_to_end;
  irs_menu_bool_item_t m_go_to_end_item;
  //  Измерять время срабатывания
  bool m_scan_need_meas_time;
  irs_menu_bool_item_t m_need_meas_time_item;
  //  Коррекция частоты срабатывания по времени
  bool m_scan_correct_freq_by_time;
  irs_menu_bool_item_t m_correct_freq_by_time_item;
  //  Туда и обратно
  bool m_scan_tuda_obratno;
  irs_menu_bool_item_t m_tuda_obratno_item;
  //  Частота срабатывания реле
  //  Нижняя строка главного экрана
  irs_menu_string_item_t m_relay_string_item_1;
  irs_menu_string_item_t m_relay_string_item_2;
  char *mp_relay_string_1;
  char *mp_relay_string_2;
  char *mp_message_string;
  irs_u8 m_relay_str_len;
  float m_freq_forward;
  float m_freq_return;
  float m_time_forward;
  float m_time_return;
  //  Время срабатывания
  counter_t m_relay_ref_time_cnt;
  counter_t m_relay_meas_time_cnt;
  //  Сканирование
  scan_status_t m_scan_status;
  scan_mode_t m_scan_mode;
  counter_t m_scan_interval;
  counter_t m_scan_pause_interval;
  counter_t m_scan_jump_interval;
  counter_t m_scan_jump_wait_interval;
  irs::loop_timer_t m_scan_timer;
  const float m_scan_freq_step;
  bool m_relay_contact_prev;
  bool m_relay_was_forward_action;
  bool m_relay_was_return_action;
  bool m_relay_was_time_forward_action;
  bool m_relay_was_time_return_action;
  bool m_relay_was_forward_correct;
  bool m_relay_was_return_correct;
  direction_t m_scan_direction;
  //  Коррекция
  irs_advanced_menu_t m_correct_menu;
  //  Разрешение коррекции переменного напряжения
  bool m_ac_voltage_correct_enable;
  irs_menu_bool_item_t m_ac_voltage_correct_enable_item;
  //  Коррекция напряжения
  const bool m_ac_voltage_correct_use_b_koefs;
  const irs_u8 m_ac_voltage_correct_start_pos;
  irs::mxdata_t *mp_acv_correct_nonvolatile_memory;
  irs::correct_map_t<double, double, double, double> m_ac_voltage_correct_map;
  irs::correct_t<double, double, double, double> m_ac_voltage_correct;
  irs_menu_2param_master_item_t m_ac_voltage_correct_item;

  //  Разрешение коррекции постоянного напряжения
  const double m_dc_voltage_ref_correct;
  bool m_dc_voltage_correct_enable;
  irs_menu_bool_item_t m_dc_voltage_correct_enable_item;
  //  Коррекция постоянного напряжения
  double m_dc_voltage_correct;
  irs_menu_double_item_t m_dc_voltage_correct_item;

  //  Разрешение коррекции частоты
  bool m_freq_correct_enable;
  irs_menu_bool_item_t m_freq_correct_enable_item;
  //  Коррекция частоты
  const double m_freq_correct_set;
  double m_freq_correct_result;
  irs_menu_double_item_t m_freq_correct_item;
  //  Ввод параметров стрелочками с ограничениями
  bounded_user_input_t<double> m_freq_begin_input;
  bounded_user_input_t<double> m_freq_end_input;
  bounded_user_input_t<double> m_speed_freq_input;
  bounded_user_input_t<double> m_voltage_ref_input;
  irs::loop_timer_t m_nonvolatile_update_timer;
  //  Функции передачи

  mxfact_event_t m_trans_ac_k_nonv_event;
  mxfact_event_t m_trans_ac_ki_event;
  mxfact_event_t m_trans_ac_ki_nonv_event;
  mxfact_event_t m_trans_ac_kd_event;
  mxfact_event_t m_trans_ac_kd_nonv_event;
  mxfact_event_t m_trans_dc_k_nonv_event;
  mxfact_event_t m_trans_dc_ki_event;
  mxfact_event_t m_trans_dc_ki_nonv_event;
  mxfact_event_t m_trans_dc_kd_event;
  mxfact_event_t m_trans_dc_kd_nonv_event;
  mxfact_event_t m_trans_contrast_event;
  mxfact_event_t m_trans_contrast_nonv_event;
  mxfact_event_t m_trans_current_limit_event;
  mxfact_event_t m_trans_current_limit_nonv_event;
  mxfact_event_t m_trans_termo_limit_nonv_event;
  mxfact_event_t m_trans_freq_begin_event;
  mxfact_event_t m_trans_freq_begin_nonv_event;
  mxfact_event_t m_trans_freq_end_nonv_event;
  mxfact_event_t m_trans_speed_freq_nonv_event;
  mxfact_event_t m_trans_voltage_ref_nonv_event;
  mxfact_event_t m_trans_mode_dc_event;
  mxfact_event_t m_trans_mode_dc_nonv_event;
  mxfact_event_t m_trans_go_to_end_nonv_event;
  mxfact_event_t m_trans_need_meas_time_nonv_event;
  mxfact_event_t m_trans_correct_freq_by_time_nonv_event;
  mxfact_event_t m_trans_tuda_obratno_nonv_event;
  mxfact_event_t m_trans_ac_need_meas_koef_nonv_event;
  mxfact_event_t m_trans_dc_need_meas_koef_nonv_event;
  mxfact_event_t m_trans_ac_voltage_correct_enable_nonv_event;
  mxfact_event_t m_trans_dc_voltage_correct_enable_nonv_event;
  mxfact_event_t m_trans_dc_voltage_correct_koef_event;
  mxfact_event_t m_trans_freq_correct_enable_nonv_event;
  mxfact_event_t m_trans_freq_correct_koef_event;
  mxfact_event_t m_trans_ac_t_adc_event;
  mxfact_event_t m_trans_ac_t_adc_nonv_event;
  mxfact_event_t m_trans_ac_iso_k_event;
  mxfact_event_t m_trans_ac_iso_k_nonv_event;
  mxfact_event_t m_trans_ac_iso_t_event;
  mxfact_event_t m_trans_ac_iso_t_nonv_event;
  mxfact_event_t m_trans_ac_dead_band_nonv_event;
  mxfact_event_t m_trans_dc_t_adc_event;
  mxfact_event_t m_trans_dc_t_adc_nonv_event;
  mxfact_event_t m_trans_dc_iso_k_event;
  mxfact_event_t m_trans_dc_iso_k_nonv_event;
  mxfact_event_t m_trans_dc_iso_t_event;
  mxfact_event_t m_trans_dc_iso_t_nonv_event;
  mxfact_event_t m_trans_dc_dead_band_nonv_event;
  mxfact_event_t m_id_pass_event;
  mxfact_event_t m_id_nonv_event;
  mxfact_event_t m_reset_nonvolatile_memory_nonv_event;
  mxfact_event_t m_reset_correct_nonvolatile_memory_nonv_event;
  //
  irs_menu_base_t *mp_cur_menu;
  //
  hold_key_wait_t m_contrast_hold_key;
  double m_id;
  irs_menu_double_item_t m_id_item;
  double m_id_pass;
  irs_menu_double_item_t m_id_pass_item;

  bool m_reset_nonvolatile_memory;
  irs_menu_bool_item_t m_reset_nonvolatile_memory_item;
  bool m_reset_acv_correct_nonvolatile_memory;
  irs_menu_bool_item_t m_reset_acv_correct_nonvolatile_memory_item;

  void set_mode_ac();
  void set_mode_dc();
  void update_configuration_for_mode_dc_ac();
  void update_config_voltage_range();
  reg_options_t* get_actual_reg_options();
  double get_meas_k_default() const;
  std::vector<size_type> make_adc_channels_sko_period_count();
  float adc_to_voltage(adc_data_t a_adc);
  void to_stop();
  void to_start();
  void to_overheat();
  void to_pause();
  void to_message();
  void to_correct_pause();
  void create_time_string(direction_t a_direction, float *ap_time,
    bool a_action, char *ap_str);
  void create_freq_string(direction_t a_direction, float *ap_freq,
    bool a_action, char *ap_str);
  void space_fill_message_string();
  bool operating_duty();
  bool reg_on();
  bool test_freq_limit(double &a_freq, double &a_freq_limit);
  void switch_key(irskey_t a_key);
  void put_scan_mode_symbol();
  void reg();
  void reg_with_meas();
  bool test_switch_interface_mode(irskey_t a_key);
  void voltage_filter_on();
  void voltage_filter_off();
  void id(irs_u16 a_id);
  irs_u16 id();
  void reset_nonvolatile_memory();
  irs_uarc reset_correct_nonvolatile_memory();
  //void set_precision_voltage_menu_items(size_type a_precision);
public:
  app_t(CFG &a_cfg, size_t a_revision);
  ~app_t();
  void in_tick();
  void tick();
  void out_tick();
};

template <class CFG>
app_t<CFG>::app_t(CFG &a_cfg, size_t a_revision):
  m_cfg(a_cfg),
  mp_debug_led(m_cfg.get_debug_led()),
  mp_relay_contact_pin(m_cfg.get_relay_contact_pin()),
  mp_lcd_contrast_pwm(m_cfg.get_lcd_contrast_pwm()),
  m_sinus_pwm(m_cfg.get_sinus_pwm()),
  m_overcurrent_detection_enabled(false),
  m_interrupt_generator(m_cfg.get_int_generator()),

  m_period_sample_count(128), //100
  m_sko_period_count(10),//50
  m_short_sko_period_count(10),
  m_average_period_count(1),
  m_result_sko_point_count(60),
  m_delta_point_count(60),
  m_downsampling_factor(interrupt_freq_boost_factor*CFG::sinus_size/
    m_period_sample_count),
  m_fast_adc_rms(NULL/*m_cfg.get_fast_adc_spi()*/, m_cfg.get_adc(), 0,
    &m_interrupt_generator, m_downsampling_factor, m_period_sample_count,
    make_adc_channels_sko_period_count(),
    m_average_period_count, m_result_sko_point_count, m_delta_point_count),

  mp_generator(new sinus_generator_t(&m_interrupt_generator,
    interrupt_freq_boost_factor, &m_sinus_pwm,
    CFG::sinus_pwm_frequency, m_cfg.m_ir2183_dead_time, CFG::sinus_size)),
  mp_lcd_drv(m_cfg.get_lcd_drv()),
  m_lcd_drv_service(),
  m_buzzer_kb_event(),
  m_hot_kb_event(),
  m_menu_kb_event(),
  m_keyboard_event_gen(),
  m_keyboard_drv(m_cfg.get_keyboard_drv()),
  m_filter_update_interval(0.1),
  m_filter_update_interval_timer(irs::make_cnt_s(m_filter_update_interval)),
  m_adc_settings(m_cfg.get_adc_settings()),
  m_adc_fade_data(),
  //  Nonvolatile memory


  m_nonvolatile_data_size(0),
  m_nonvolatile_data(m_cfg.get_nonvolatile_memory(), 0,
    &m_nonvolatile_data_size),
  m_dac_v_reference(3.3),
  mp_current_limit_code(m_cfg.get_current_limit_code()),
  mp_relay_interrupt_generator(m_cfg.get_relay_interrupt_generator()),
  m_gtch_relay_interrupt(*this),
  // Регульятор
  m_meas_interval(irs::make_cnt_ms(1000)),
  m_reg_interval(irs::make_cnt_ms(100)),
  m_meas_gen_value(0.001f), //  Было 0.1f
  m_operating_duty_deviation(0.03f),
  m_operating_duty(false),
  m_reg_timer(m_meas_interval),
  m_rate_data(),
  m_meas_k(0.f),
  m_ac_meas_k_default(1./220.),
  m_dc_meas_k_default(1./300.),
  m_min_meas_k_voltage(5.f),
  m_reg_koefs_changed(false),
  m_can_reg(true),
  m_mode_dc_ac(MODE_AC),
  m_status(SPLASH),
  m_splash_timer(irs::make_cnt_s(3)),
  //  Отладочный режим
  m_interface_mode(RELEASE),
  m_current_interface_mode_key(0),
  m_menu_timer(irs::make_cnt_ms(10)),
  m_buzzer(m_cfg.get_buzzer_pin()),
  m_main_creep(
    mp_creep_buffer,
    m_creep_len,
    mp_lcd_drv->get_width(),
    m_creep_static_path_len,
    m_creep_message,
    m_creep_time_num,
    m_creep_time_denom),
  //  Основной экран
  m_main_screen(),
  //  Главное меню
  m_main_menu(),
  //  Меню параметров регулятора
  m_ac_reg_menu(),
  m_dc_reg_menu(),
  //  Меню параметров защит
  m_limit_menu(),
  //  Частота
  m_min_freq(45.0),
  m_max_freq(55.0),
  m_freq_step(0.01),
  m_freq_step_max(1),
  m_freq(m_min_freq),
  m_freq_item(&m_freq),
  //  Скорость изменения частоты
  m_min_speed(0.01),
  m_max_speed(1.0),
  m_speed_step(0.01),
  m_steed_step_max(1),
  m_speed_freq_saved_copy(m_min_speed),
  m_speed_freq(m_min_speed),
  m_speed_freq_item(&m_speed_freq),
  //  Начальная частота
  m_freq_begin_saved_copy(m_min_freq),
  m_freq_begin(m_min_freq),
  m_freq_begin_item(&m_freq_begin),
  //  Конечная частота
  m_freq_end_saved_copy(m_max_freq),
  m_freq_end(m_max_freq),
  m_freq_end_item(&m_freq_end),
  //  Уставка напряжения
  m_min_ac_voltage_debug(0.001),
  m_max_ac_voltage_debug(300.),
  m_min_ac_voltage_release(1.),
  m_max_ac_voltage_release(220.),
  m_min_dc_voltage_debug(0.001),
  m_max_dc_voltage_debug(400.),
  m_min_dc_voltage_release(1.),
  m_max_dc_voltage_release(300.),
  m_min_voltage(m_min_ac_voltage_release),
  m_max_voltage(m_max_ac_voltage_release), 
  m_voltage_step_release(1.0),
  m_voltage_step_max_release(10.),
  m_voltage_step_debug(0.01),
  m_voltage_step_max_debug(10.),
  m_voltage_ref(m_min_voltage),
  m_voltage_ref_item(&m_voltage_ref),
  //  Напряжение
  m_voltage(0.f),
  m_voltage_display(0.f),
  m_voltage_item(&m_voltage_display),
  //  Уставка защиты по току
  m_trans_koef(1.5*mp_current_limit_code->get_u32_maximum()/m_dac_v_reference),
  m_current_limit(1.5),
  m_current_limit_item(&m_current_limit, CAN_WRITE),
  //  Температура
  m_temperature_conn_data(&m_cfg.get_th_data()),
  mp_window_watchdog(m_cfg.get_window_watchdog()),
  mp_independent_watchdog(m_cfg.get_independent_watchdog()),
  m_temperature(0.),
  m_temperature_item(&m_temperature),
  //  Уставка защиты по температуре
  m_termo_hist(5.),
  m_termo_limit(61.),
  m_termo_limit_item(&m_termo_limit, CAN_WRITE),
  //  Контрастность
  m_contrast(0.f),
  m_contrast_item(&m_contrast, CAN_WRITE),

  mp_ac_reg_options(),
  mp_dc_reg_options(),

  m_can_edit(true),
  m_voltage_filter_on(false),
  //  Изодромное звено
  m_iso_data(),
  //  АЦП
  m_adc_value(0),
  m_adc_item(&m_adc_value),
  //  ШИМ
  m_pwm_out(0.),
  m_pwm_out_item(&m_pwm_out),
  //
  m_meas_k_item(&m_meas_k),
  //  Пункты меню сканирования
  m_scan_menu(),
  //  Режим постоянного тока
  m_mode_dc_enabled_previous(false),
  m_mode_dc_enabled(false),
  m_mode_dc_item((irs_bool*)&m_mode_dc_enabled, m_can_edit),
  //  Идти до конца
  m_scan_go_to_end(true),
  m_go_to_end_item((irs_bool*)&m_scan_go_to_end, m_can_edit),
  //  Измерять время срабатывания
  m_scan_need_meas_time(true),
  m_need_meas_time_item((irs_bool*)&m_scan_need_meas_time, m_can_edit),
  //  Коррекция частоты срабатывания по измеренному времени срабатывания
  m_scan_correct_freq_by_time(true),
  m_correct_freq_by_time_item((irs_bool*)&m_scan_correct_freq_by_time,
    m_can_edit),
  //  Туда и обратно
  m_scan_tuda_obratno(true),
  m_tuda_obratno_item((irs_bool*)&m_scan_tuda_obratno, m_can_edit),
  //
  m_relay_string_item_1(),
  m_relay_string_item_2(),
  mp_relay_string_1(0),
  mp_relay_string_2(0),
  mp_message_string(0),
  m_relay_str_len(m_half_screen),
  //  Частота срабатывания реле
  m_freq_forward(m_min_freq),
  m_freq_return(m_min_freq),
  m_time_forward(0.f),
  m_time_return(0.f),
  //  Время срабатывания
  m_relay_ref_time_cnt(0),
  m_relay_meas_time_cnt(0),
  //  Сканирование
  m_scan_status(SCAN_STOP),
  m_scan_mode(SM_TUDA),
  m_scan_interval(irs::make_cnt_ms(10)),
  m_scan_pause_interval(irs::make_cnt_ms(2000)),
  m_scan_jump_interval(irs::make_cnt_ms(3000)),
  m_scan_jump_wait_interval(irs::make_cnt_s(60)),
  m_scan_timer(m_scan_interval),
  m_scan_freq_step(0.01f),
  m_relay_contact_prev(false),
  m_relay_was_forward_action(false),
  m_relay_was_return_action(false),
  m_relay_was_time_forward_action(false),
  m_relay_was_time_return_action(false),
  m_relay_was_forward_correct(false),
  m_relay_was_return_correct(false),
  m_scan_direction(UP),
  //  Коррекция
  m_correct_menu(),
  //  Разрешение коррекции напряжения
  m_ac_voltage_correct_enable(false),
  m_ac_voltage_correct_enable_item((irs_bool*)&m_ac_voltage_correct_enable,
    m_can_edit),
  //  Коррекция напряжения
  m_ac_voltage_correct_use_b_koefs(false),
  m_ac_voltage_correct_start_pos(0),
  mp_acv_correct_nonvolatile_memory(m_cfg.get_correct_nonvolatile_memory()),
  m_ac_voltage_correct_map(),
  m_ac_voltage_correct(mp_acv_correct_nonvolatile_memory,
    m_ac_voltage_correct_start_pos, m_ac_voltage_correct_use_b_koefs),
  m_ac_voltage_correct_item(m_freq, m_voltage_ref, m_voltage_display),
  //  Разрешение коррекции постоянного напряжения
  m_dc_voltage_ref_correct(200),
  m_dc_voltage_correct_enable(false),
  m_dc_voltage_correct_enable_item((irs_bool*)&m_dc_voltage_correct_enable,
    m_can_edit),
  //  Коррекция постоянного напряжения
  m_dc_voltage_correct(1.),
  m_dc_voltage_correct_item(&m_dc_voltage_correct, CAN_WRITE),
  //  Разрешение коррекции частоты
  m_freq_correct_enable(false),
  m_freq_correct_enable_item((irs_bool*)&m_freq_correct_enable,
    m_can_edit),
  //  Коррекция частоты
  m_freq_correct_set(45.0),
  m_freq_correct_result(1.),
  m_freq_correct_item(&m_freq_correct_result, CAN_WRITE),

  // Ввод параметров стрелочками с ограничениями
  m_freq_begin_input(m_freq_begin, m_freq_step, m_freq_step_max, m_min_freq,
    m_max_freq),
  m_freq_end_input(m_freq_end, m_freq_step, m_freq_step_max, m_min_freq,
    m_max_freq),
  m_speed_freq_input(m_speed_freq, m_speed_step, m_steed_step_max, m_min_speed,
    m_max_speed),
  m_voltage_ref_input(m_voltage_ref, m_voltage_step_release, 
    m_voltage_step_max_release, m_min_voltage, m_max_voltage),
  m_nonvolatile_update_timer(irs::make_cnt_ms(300)),
  //
  m_trans_ac_k_nonv_event(),
  m_trans_ac_ki_event(),
  m_trans_ac_ki_nonv_event(),
  m_trans_ac_kd_event(),
  m_trans_ac_kd_nonv_event(),
  m_trans_dc_k_nonv_event(),
  m_trans_dc_ki_event(),
  m_trans_dc_ki_nonv_event(),
  m_trans_dc_kd_event(),
  m_trans_dc_kd_nonv_event(),
  m_trans_contrast_event(),
  m_trans_contrast_nonv_event(),
  m_trans_current_limit_event(),
  m_trans_current_limit_nonv_event(),
  m_trans_termo_limit_nonv_event(),
  m_trans_freq_begin_event(),
  m_trans_freq_begin_nonv_event(),
  m_trans_freq_end_nonv_event(),
  m_trans_speed_freq_nonv_event(),
  m_trans_voltage_ref_nonv_event(),
  m_trans_mode_dc_event(),
  m_trans_mode_dc_nonv_event(),
  m_trans_go_to_end_nonv_event(),
  m_trans_need_meas_time_nonv_event(),
  m_trans_correct_freq_by_time_nonv_event(),
  m_trans_tuda_obratno_nonv_event(),
  m_trans_ac_need_meas_koef_nonv_event(),
  m_trans_dc_need_meas_koef_nonv_event(),
  m_trans_ac_voltage_correct_enable_nonv_event(),
  m_trans_dc_voltage_correct_enable_nonv_event(),
  m_trans_dc_voltage_correct_koef_event(),
  m_trans_freq_correct_enable_nonv_event(),
  m_trans_freq_correct_koef_event(),
  m_trans_ac_t_adc_event(),
  m_trans_ac_t_adc_nonv_event(),
  m_trans_ac_iso_k_event(),
  m_trans_ac_iso_k_nonv_event(),
  m_trans_ac_iso_t_event(),
  m_trans_ac_iso_t_nonv_event(),
  m_trans_ac_dead_band_nonv_event(),
  m_trans_dc_t_adc_event(),
  m_trans_dc_t_adc_nonv_event(),
  m_trans_dc_iso_k_event(),
  m_trans_dc_iso_k_nonv_event(),
  m_trans_dc_iso_t_event(),
  m_trans_dc_iso_t_nonv_event(),
  m_trans_dc_dead_band_nonv_event(),
  m_id_pass_event(),
  m_id_nonv_event(),
  m_reset_nonvolatile_memory_nonv_event(),
  m_reset_correct_nonvolatile_memory_nonv_event(),
  mp_cur_menu(&m_main_screen),
  m_contrast_hold_key(m_keyboard_drv, irs::make_cnt_s(5),
    static_cast<irskey_t>(m_key_stop)),
  m_id(0),
  m_id_item(&m_id, CAN_WRITE),
  m_id_pass(0),
  m_id_pass_item(&m_id_pass, CAN_WRITE),
  m_reset_nonvolatile_memory(false),
  m_reset_nonvolatile_memory_item(
    (irs_bool*)&m_reset_nonvolatile_memory, m_can_edit),
  m_reset_acv_correct_nonvolatile_memory(false),
  m_reset_acv_correct_nonvolatile_memory_item(
    (irs_bool*)&m_reset_acv_correct_nonvolatile_memory, m_can_edit)
{
  mp_relay_interrupt_generator->add_event(&m_gtch_relay_interrupt);

  irs::mlog() << irsm("Размер EEPROM в байтах: ");
  irs::mlog() << m_cfg.get_nonvolatile_memory()->size() << endl;
  irs::mlog() << irsm("Размер параметров в байтах: ");
  irs::mlog() << m_nonvolatile_data_size << endl;
  if (m_nonvolatile_data_size > m_cfg.get_nonvolatile_memory()->size()) {
    GTCH_DBG_MSG(irsm("Размер EEPROM недостаточен для хранения"));
    GTCH_DBG_MSG(irsm(" всех перменных") << endl);
    for (;;);
  }
  //  Input
  m_freq_begin_input.set_keys(static_cast<irskey_t>(m_key_Fn_up),
    static_cast<irskey_t>(m_key_Fn_down));
  m_freq_end_input.set_keys(static_cast<irskey_t>(m_key_Fk_up),
    static_cast<irskey_t>(m_key_Fk_down));
  m_speed_freq_input.set_keys(static_cast<irskey_t>(m_key_S_up),
    static_cast<irskey_t>(m_key_S_down));
  m_voltage_ref_input.set_keys(static_cast<irskey_t>(m_key_V_up),
    static_cast<irskey_t>(m_key_V_down));
  //  Вспомогательные ШИМы
  mp_lcd_contrast_pwm->start();

  //  ЖКИ и клавиатура
  m_lcd_drv_service.connect(mp_lcd_drv);
  m_keyboard_event_gen.connect(&m_keyboard_drv);
  m_keyboard_event_gen.add_event(&m_menu_kb_event);
  m_keyboard_event_gen.add_event(&m_buzzer_kb_event);
  m_keyboard_event_gen.add_event(&m_hot_kb_event);

  m_keyboard_event_gen.set_antibounce_time(irs::make_cnt_ms(60));
  m_keyboard_event_gen.set_defence_time(irs::make_cnt_ms(500));
  m_keyboard_event_gen.set_rep_time(irs::make_cnt_ms(200));

  m_splash_timer.start();
  ostringstream revision_ostr;
  revision_ostr << "Версия " <<  a_revision << "F";
  irs::string_t revision_str = align_center(revision_ostr.str(),
    m_lcd_drv_service.get_width());
  m_lcd_drv_service.outtextpos(0, 1, "      ГТЧ-03М       ");
  m_lcd_drv_service.outtextpos(0, 2, revision_str.c_str());
  m_lcd_drv_service.outtextpos(0, 3, "  ООО 'РЭС' 2017 г. ");
  m_lcd_drv_service.outtextpos(0, 0, "   www.irsural.ru   ");

  {
    irs::timer_t splash_out_timer(irs::make_cnt_ms(200));
    splash_out_timer.start();
    while (!splash_out_timer.check()) {
      m_cfg.tick();
    }
  }

  //  Меню
  memset(mp_user_str, ' ', m_user_str_len);
  mp_user_str[m_user_str_len] = '\0';
  m_main_creep.change_static("Режим: СТОП");
  strcpy(mp_enter_msg, "Для входа в меню нажмите '€'");
  strcpy(mp_exit_msg, "Для выхода из меню нажмите 'esc'");

  mp_ac_reg_options.reset(new reg_options_t(mp_user_str, mp_exit_msg,
    m_menu_reg_options_item_width, m_menu_item_prec));
  mp_dc_reg_options.reset(new reg_options_t(mp_user_str, mp_exit_msg,
    m_menu_reg_options_item_width, m_menu_item_prec));


  // Nonvolatile memory
  const bool nonvolatile_memory_error = m_cfg.nonvolatile_memory_error();
  const bool need_reset_correct_nonvolatile_memory =
    m_nonvolatile_data.reset_correct_nonvolatile_memory;
  if (nonvolatile_memory_error || m_nonvolatile_data.reset_nonvolatile_memory) {
    if (nonvolatile_memory_error) {
      GTCH_DBG_MSG(irsm("eeprom error") << endl);
    }
    reset_nonvolatile_memory();
  }
  if (m_cfg.nonvolatile_memory_partial_error()) {
    GTCH_DBG_MSG(irsm("partial eeprom error") << endl);
  }

  mp_ac_reg_options->k = m_nonvolatile_data.ac_k;
  mp_ac_reg_options->ki = m_nonvolatile_data.ac_ki;
  mp_ac_reg_options->kd = m_nonvolatile_data.ac_kd;
  mp_dc_reg_options->k = m_nonvolatile_data.dc_k;
  mp_dc_reg_options->ki = m_nonvolatile_data.dc_ki;
  mp_dc_reg_options->kd = m_nonvolatile_data.dc_kd;
  m_contrast = m_nonvolatile_data.contrast;
  mp_lcd_contrast_pwm->set_duty(float(m_contrast));
  m_current_limit = m_nonvolatile_data.current_limit;
  m_termo_limit = m_nonvolatile_data.termo_limit;
  mp_current_limit_code->set_u32_data(0,
    static_cast<irs_u32>(m_current_limit*m_trans_koef));
  m_freq_begin = m_nonvolatile_data.freq_begin;
  m_freq_end = m_nonvolatile_data.freq_end;
  m_speed_freq = m_nonvolatile_data.speed_freq;
  m_voltage_ref = m_nonvolatile_data.ac_voltage_ref;
  m_mode_dc_enabled = m_nonvolatile_data.mode_dc_enabled;
  m_scan_go_to_end = m_nonvolatile_data.go_to_end;
  m_scan_need_meas_time = m_nonvolatile_data.need_meas_time;
  m_scan_correct_freq_by_time = m_nonvolatile_data.correct_freq_by_time;
  m_scan_tuda_obratno = m_nonvolatile_data.tuda_obratno;
  mp_ac_reg_options->need_meas_koef = m_nonvolatile_data.ac_need_meas_koef;
  mp_dc_reg_options->need_meas_koef = m_nonvolatile_data.dc_need_meas_koef;
  m_ac_voltage_correct_enable = m_nonvolatile_data.voltage_correct_enable;

  m_dc_voltage_correct_enable = m_nonvolatile_data.dc_voltage_correct_enable;
  m_dc_voltage_correct = m_dc_voltage_ref_correct*
    m_nonvolatile_data.dc_voltage_correct_koef;

  m_freq_correct_enable = m_nonvolatile_data.freq_correct_enable;
  m_freq_correct_result =
    m_freq_correct_set * m_nonvolatile_data.freq_correct_koef;
  mp_ac_reg_options->t_adc = m_nonvolatile_data.ac_t_adc;
  mp_ac_reg_options->iso_k = m_nonvolatile_data.ac_iso_k;
  mp_ac_reg_options->iso_t = m_nonvolatile_data.ac_iso_t;
  mp_ac_reg_options->dead_band = m_nonvolatile_data.ac_dead_band;
  mp_dc_reg_options->t_adc = m_nonvolatile_data.dc_t_adc;
  mp_dc_reg_options->iso_k = m_nonvolatile_data.dc_iso_k;
  mp_dc_reg_options->iso_t = m_nonvolatile_data.dc_iso_t;
  mp_dc_reg_options->dead_band = m_nonvolatile_data.dc_dead_band;
  id(m_nonvolatile_data.id);

  //!!! Не забыть включить обратно
  if (id() == 0) {
    m_buzzer.bzzz();
  }

  // Меню
  //  Основной экран
  m_main_screen.set_key_event(&m_menu_kb_event);
  m_main_screen.set_message(mp_enter_msg);
  m_main_screen.set_creep(&m_main_creep);
  m_main_screen.set_disp_drv(&m_lcd_drv_service);
  m_main_screen.set_cursor_symbol(0x01);
  m_main_screen.set_slave_menu(&m_main_menu);

  m_main_screen.add(&m_freq_begin_item, 0, 0, IMM_WITHOUT_SUFFIX);
  m_main_screen.add(&m_freq_end_item, 0, 1, IMM_WITHOUT_SUFFIX);
  m_main_screen.add(&m_freq_item, 0, 2, IMM_WITHOUT_SUFFIX);
  m_main_screen.add(&m_speed_freq_item, m_half_screen, 0, IMM_WITHOUT_SUFFIX);
  m_main_screen.add(&m_voltage_ref_item, m_half_screen, 1, IMM_WITHOUT_SUFFIX);
  m_main_screen.add(&m_voltage_item, m_half_screen, 2, IMM_WITHOUT_SUFFIX);
  m_main_screen.add(&m_relay_string_item_1, 0, 3, IMM_WITHOUT_SUFFIX);
  m_main_screen.add(&m_relay_string_item_2, m_half_screen, 3,
    IMM_WITHOUT_SUFFIX);
  m_main_screen.add(&m_temperature_item, 0, 3, IMM_WITHOUT_SUFFIX);
  m_main_screen.hide_item(&m_temperature_item);
  m_main_screen.add(&m_adc_item, 0, 0, IMM_WITHOUT_SUFFIX);
  m_main_screen.hide_item(&m_adc_item);
  m_main_screen.add(&m_pwm_out_item, 0, 1, IMM_WITHOUT_SUFFIX);
  m_main_screen.hide_item(&m_pwm_out_item);
  m_main_screen.add(&m_meas_k_item, m_half_screen, 0, IMM_WITHOUT_SUFFIX);
  m_main_screen.hide_item(&m_meas_k_item);
  //  Основное меню
  m_main_menu.set_header("Настройки");
  m_main_menu.add(&m_ac_reg_menu, HIDE_ITEM);
  m_main_menu.add(&m_dc_reg_menu, HIDE_ITEM);
  m_main_menu.add(&m_limit_menu, HIDE_ITEM);
  m_main_menu.add(&m_scan_menu, HIDE_ITEM);
  m_main_menu.add(&m_correct_menu, HIDE_ITEM);
  m_main_menu.add(&m_mode_dc_item, SHOW_ITEM);
  m_main_menu.add(&m_go_to_end_item, SHOW_ITEM);
  m_main_menu.add(&m_need_meas_time_item, SHOW_ITEM);
  m_main_menu.add(&m_correct_freq_by_time_item, SHOW_ITEM);
  m_main_menu.add(&m_tuda_obratno_item, SHOW_ITEM);

  //  Меню регулятора
  m_ac_reg_menu.set_header("Меню AC регулят.");
  m_ac_reg_menu.add(&mp_ac_reg_options->k_item);
  m_ac_reg_menu.add(&mp_ac_reg_options->ki_item);
  m_ac_reg_menu.add(&mp_ac_reg_options->kd_item);
  m_ac_reg_menu.add(&mp_ac_reg_options->need_meas_koef_item);
  m_ac_reg_menu.add(&mp_ac_reg_options->t_adc_item);
  m_ac_reg_menu.add(&mp_ac_reg_options->iso_k_item);
  m_ac_reg_menu.add(&mp_ac_reg_options->iso_t_item);
  m_ac_reg_menu.add(&mp_ac_reg_options->dead_band_item);

  //  Меню регулятора
  m_dc_reg_menu.set_header("Меню DC регулят.");
  m_dc_reg_menu.add(&mp_dc_reg_options->k_item);
  m_dc_reg_menu.add(&mp_dc_reg_options->ki_item);
  m_dc_reg_menu.add(&mp_dc_reg_options->kd_item);
  m_dc_reg_menu.add(&mp_dc_reg_options->need_meas_koef_item);
  m_dc_reg_menu.add(&mp_dc_reg_options->t_adc_item);
  m_dc_reg_menu.add(&mp_dc_reg_options->iso_k_item);
  m_dc_reg_menu.add(&mp_dc_reg_options->iso_t_item);
  m_dc_reg_menu.add(&mp_dc_reg_options->dead_band_item);

  //  Меню защит
  m_limit_menu.set_header("Меню защит");
  m_limit_menu.add(&m_current_limit_item);
  m_limit_menu.add(&m_termo_limit_item);
  //  Частота
  m_freq_item.set_str(mp_user_str, "F ", "Гц",
    m_menu_item_width, m_menu_item_prec);
  //  Начальная частота
  m_freq_begin_item.set_str(mp_user_str, "Fн", "",
    m_menu_item_width, m_menu_item_prec);
  //  Конечная частота
  m_freq_end_item.set_str(mp_user_str, "Fк", "",
    m_menu_item_width, m_menu_item_prec);
  //  Уставка напряжения
  m_voltage_ref_item.set_str(mp_user_str, "Uo", "",
    m_volt_item_width, m_volt_item_prec);
  //  Напряжение
  m_voltage_item.set_str(mp_user_str, "Uн", "В",
    m_volt_item_width, m_volt_item_prec);
  //  Уставка защиты по току
  m_current_limit_item.set_header("Уставка защиты");
  m_current_limit_item.set_message(mp_exit_msg);
  m_current_limit_item.set_str(mp_user_str, "Imax", "А",
    m_menu_item_width, m_menu_item_prec);
  m_current_limit_item.set_max_value(
    mp_current_limit_code->get_u32_maximum() / m_trans_koef);
  m_current_limit_item.set_min_value(0.05);
  m_current_limit_item.add_change_event(&m_trans_current_limit_event);
  m_current_limit_item.add_change_event(&m_trans_current_limit_nonv_event);
  m_current_limit_item.set_key_type(IMK_ARROWS);
  m_current_limit_item.set_change_step(0.1f);
  //  Температура
  m_temperature_item.set_header("Температура");
  m_temperature_item.set_message(mp_exit_msg);
  m_temperature_item.set_str(mp_user_str, "T°", "°C",
    m_menu_item_width, m_termo_item_prec);
  //  Уставка защиты по температуре
  m_termo_limit_item.set_header("Макс. темп.");
  m_termo_limit_item.set_message(mp_exit_msg);
  m_termo_limit_item.set_str(mp_user_str, "Tmax", "°C",
    m_menu_item_width, m_termo_item_prec);
  m_termo_limit_item.set_max_value(85.);
  m_termo_limit_item.set_min_value(30.);
  m_termo_limit_item.add_change_event(&m_trans_termo_limit_nonv_event);
  m_termo_limit_item.set_key_type(IMK_ARROWS);
  m_termo_limit_item.set_change_step(1.f);
  //  Контрастность
  m_contrast_item.set_header("Контраст");
  m_contrast_item.set_message(mp_exit_msg);
  m_contrast_item.set_str(mp_user_str, "К", "",
    m_menu_item_width, m_menu_item_prec);
  m_contrast_item.set_max_value(1.0);
  m_contrast_item.set_min_value(0.0);
  m_contrast_item.add_change_event(&m_trans_contrast_event);
  m_contrast_item.add_change_event(&m_trans_contrast_nonv_event);
  m_contrast_item.set_key_type(IMK_ARROWS);
  m_contrast_item.set_change_step(0.01f);
  m_contrast_item.set_apply_immediately(true);
  m_main_menu.add(&m_contrast_item, HIDE_ITEM);
  // Контрасность - дополнительно
  m_contrast_item.set_master_menu(&m_main_screen);
  m_contrast_item.set_key_event(&m_menu_kb_event);
  m_contrast_item.set_creep(&m_main_creep);
  m_contrast_item.set_disp_drv(&m_lcd_drv_service);
  m_contrast_item.set_cursor_symbol(0x01);

  mp_ac_reg_options->k_item.add_change_event(&m_trans_ac_k_nonv_event);
  mp_ac_reg_options->ki_item.add_change_event(&m_trans_ac_ki_event);
  mp_ac_reg_options->ki_item.add_change_event(&m_trans_ac_ki_nonv_event);
  mp_ac_reg_options->kd_item.add_change_event(&m_trans_ac_kd_event);
  mp_ac_reg_options->kd_item.add_change_event(&m_trans_ac_kd_nonv_event);
  mp_ac_reg_options->need_meas_koef_item.add_change_event(
    &m_trans_ac_need_meas_koef_nonv_event);
  mp_ac_reg_options->t_adc_item.add_change_event(&m_trans_ac_t_adc_event);
  mp_ac_reg_options->t_adc_item.add_change_event(&m_trans_ac_t_adc_nonv_event);
  mp_ac_reg_options->iso_k_item.add_change_event(&m_trans_ac_iso_k_event);
  mp_ac_reg_options->iso_k_item.add_change_event(&m_trans_ac_iso_k_nonv_event);
  mp_ac_reg_options->iso_t_item.add_change_event(&m_trans_ac_iso_t_event);
  mp_ac_reg_options->iso_t_item.add_change_event(&m_trans_ac_iso_t_nonv_event);
  mp_ac_reg_options->dead_band_item.add_change_event(
    &m_trans_ac_dead_band_nonv_event);

  mp_dc_reg_options->k_item.add_change_event(&m_trans_dc_k_nonv_event);
  mp_dc_reg_options->ki_item.add_change_event(&m_trans_dc_ki_event);
  mp_dc_reg_options->ki_item.add_change_event(&m_trans_dc_ki_nonv_event);
  mp_dc_reg_options->kd_item.add_change_event(&m_trans_dc_kd_event);
  mp_dc_reg_options->kd_item.add_change_event(&m_trans_dc_kd_nonv_event);
  mp_dc_reg_options->need_meas_koef_item.add_change_event(
    &m_trans_dc_need_meas_koef_nonv_event);
  mp_dc_reg_options->t_adc_item.add_change_event(&m_trans_dc_t_adc_event);
  mp_dc_reg_options->t_adc_item.add_change_event(&m_trans_dc_t_adc_nonv_event);
  mp_dc_reg_options->iso_k_item.add_change_event(&m_trans_dc_iso_k_event);
  mp_dc_reg_options->iso_k_item.add_change_event(&m_trans_dc_iso_k_nonv_event);
  mp_dc_reg_options->iso_t_item.add_change_event(&m_trans_dc_iso_t_event);
  mp_dc_reg_options->iso_t_item.add_change_event(&m_trans_dc_iso_t_nonv_event);
  mp_dc_reg_options->dead_band_item.add_change_event(
    &m_trans_dc_dead_band_nonv_event);

  //  АЦП
  m_adc_item.set_str(mp_user_str, "АЦП", "", m_adc_item_width, m_adc_item_prec);
  //  ШИМ
  m_pwm_out_item.set_str(mp_user_str, "Ш", "", m_reg_item_width,
    m_reg_item_prec);
  //  K
  m_meas_k_item.set_str(mp_user_str, "K", "", m_meas_k_item_width,
    m_meas_k_item_prec);
  //
  m_scan_menu.set_header("Меню сканирования");
  m_scan_menu.add(&m_go_to_end_item);
  m_scan_menu.add(&m_need_meas_time_item);
  m_scan_menu.add(&m_correct_freq_by_time_item);
  m_scan_menu.add(&m_tuda_obratno_item);
  //  Скорость изменения частоты
  m_speed_freq_item.set_str(mp_user_str, "S ", "",
    m_menu_item_width, m_menu_item_prec);

  m_mode_dc_item.set_header("Режим пост. тока");
  m_mode_dc_item.set_str("Включено", "Выключено");

  m_mode_dc_item.add_change_event(&m_trans_mode_dc_event);
  m_mode_dc_item.add_change_event(&m_trans_mode_dc_nonv_event);

  m_go_to_end_item.set_header("Остан. при сраб.");
  m_go_to_end_item.set_str("Выключено", "Включено");
  m_go_to_end_item.add_change_event(&m_trans_go_to_end_nonv_event);

  m_need_meas_time_item.set_header("Измерять вр.сраб.");
  m_need_meas_time_item.set_str("Включено", "Выключено");
  m_need_meas_time_item.add_change_event(&m_trans_need_meas_time_nonv_event);

  m_correct_freq_by_time_item.set_header("Корр. частоты");
  m_correct_freq_by_time_item.set_str("Включено", "Выключено");
  m_correct_freq_by_time_item.add_change_event(
    &m_trans_correct_freq_by_time_nonv_event);

  m_tuda_obratno_item.set_header("Скан. в две стор.");
  m_tuda_obratno_item.set_str("Включено", "Выключено");
  m_tuda_obratno_item.add_change_event(&m_trans_tuda_obratno_nonv_event);

  //  Частота срабатывания реле и нижняя строка экрана
  mp_relay_string_1 = new char[m_relay_str_len + 1];
  mp_relay_string_2 = new char[m_relay_str_len + 1];
  mp_message_string = new char[mp_lcd_drv->get_width() + 1];
  memset(mp_relay_string_1, ' ', m_relay_str_len);
  memset(mp_relay_string_2, ' ', m_relay_str_len);
  memset(mp_message_string, ' ', mp_lcd_drv->get_width());
  mp_relay_string_1[m_relay_str_len] = 0;
  mp_relay_string_2[m_relay_str_len] = 0;
  mp_message_string[mp_lcd_drv->get_width()] = 0;
  m_relay_string_item_1.set_parametr_string(mp_relay_string_1);
  m_relay_string_item_2.set_parametr_string(mp_relay_string_2);
  m_relay_contact_prev = mp_relay_contact_pin->pin();
  //  Коррекция
  m_correct_menu.set_header("Коррекция");
  m_correct_menu.add(&m_ac_voltage_correct_enable_item);
  m_correct_menu.add(&m_ac_voltage_correct_item);
  m_correct_menu.add(&m_dc_voltage_correct_enable_item);
  m_correct_menu.add(&m_dc_voltage_correct_item);
  m_correct_menu.add(&m_freq_correct_enable_item);
  m_correct_menu.add(&m_freq_correct_item);
  //  Разрешение коррекции напряжения
  m_ac_voltage_correct_enable_item.set_header("Исп.коррекцию ACV");
  m_ac_voltage_correct_enable_item.set_str("Использовать", "Не использовать");
  m_ac_voltage_correct_enable_item.add_change_event(
    &m_trans_ac_voltage_correct_enable_nonv_event);
  //  Коррекция напряжения
  m_ac_voltage_correct_item.set_header("Коррекция ACV");
  m_ac_voltage_correct_item.set_str(mp_user_str,
    m_v_correct_item_width, m_v_correct_item_prec);
  m_ac_voltage_correct_item.set_param_1_str("F", "Гц", IMM_WITHOUT_SUFFIX);
  m_ac_voltage_correct_item.set_param_2_str("Uo", "В", IMM_WITHOUT_SUFFIX);
  m_ac_voltage_correct_item.set_param_out_str("Uн", "В", IMM_FULL);
  m_ac_voltage_correct_item.set_limit_values(m_min_ac_voltage_release - 10.,
    m_max_ac_voltage_release + 10.);
  m_ac_voltage_correct_item.set_step(0.01);
  m_ac_voltage_correct_item.set_start_key(static_cast<irskey_t>(m_key_start));
  m_ac_voltage_correct_item.set_stop_key(static_cast<irskey_t>(m_key_stop));

  const irs_uarc correct_size =
    m_ac_voltage_correct_map.connect(mp_acv_correct_nonvolatile_memory,
    m_ac_voltage_correct_start_pos, m_ac_voltage_correct_use_b_koefs);

  if (nonvolatile_memory_error || need_reset_correct_nonvolatile_memory) {
    if (nonvolatile_memory_error) {
      GTCH_DBG_MSG(irsm("correct eeprom error") << endl);
    }
    reset_correct_nonvolatile_memory();
    m_nonvolatile_data.reset_correct_nonvolatile_memory = false;
  }

  irs::mlog() << irsm("Размер EEPROM коррекции в байтах: ");
  irs::mlog() << mp_acv_correct_nonvolatile_memory->size() << endl;
  irs::mlog() << irsm("Размер параметров коррекции в байтах: ");
  irs::mlog() << correct_size << endl;
  if (correct_size > mp_acv_correct_nonvolatile_memory->size()) {
    GTCH_DBG_MSG(irsm("Размер EEPROM коррекции недостаточен для хранения"));
    GTCH_DBG_MSG(irsm(" всех перменных") << endl);
    for (;;);
  }

  m_ac_voltage_correct_item.reserve(m_ac_voltage_correct_map.m_y_count*
    m_ac_voltage_correct_map.m_x_count);
  for (irs_u8 u = 1; u < m_ac_voltage_correct_map.m_y_count; u++) {
    for (irs_u8 f = 0; f < m_ac_voltage_correct_map.m_x_count; f++) {
      double f_value = m_ac_voltage_correct_map.mp_x_points[f];
      double u_value = m_ac_voltage_correct_map.mp_y_points[u];
      double k = m_ac_voltage_correct_map.mp_k_array[
        u * m_ac_voltage_correct_map.m_x_count + f];
      double out_value = u_value * k;
      m_ac_voltage_correct_item.add_point(f_value, u_value, out_value);
    }
  }
  //  Разрешение коррекции постоянного напряжения
  m_dc_voltage_correct_enable_item.set_header("Исп.коррекцию DCV");
  m_dc_voltage_correct_enable_item.set_str("Использовать", "Не использовать");
  m_dc_voltage_correct_enable_item.add_change_event(
    &m_trans_dc_voltage_correct_enable_nonv_event);
  //  Коррекция постоянного напряжения
  m_dc_voltage_correct_item.set_header("Коррекция DCV");
  m_dc_voltage_correct_item.set_message(mp_exit_msg);
  m_dc_voltage_correct_item.set_str(mp_user_str, "U", "В",
    m_dcv_correct_item_width, m_dcv_correct_item_prec);
  m_dc_voltage_correct_item.set_max_value(m_dc_voltage_ref_correct + 10.0);
  m_dc_voltage_correct_item.set_min_value(m_dc_voltage_ref_correct - 10.0);
  m_dc_voltage_correct_item.add_change_event(
    &m_trans_dc_voltage_correct_koef_event);
  m_dc_voltage_correct_item.set_key_type(IMK_ARROWS);
  m_dc_voltage_correct_item.set_change_step(0.01f);
  m_dc_voltage_correct_item.set_change_step_max(1.f);
  m_dc_voltage_correct_item.progressive_change_enabled(true);

  //  Разрешение коррекции частоты
  m_freq_correct_enable_item.set_header("Исп.коррекцию F");
  m_freq_correct_enable_item.set_str("Использовать", "Не использовать");
  m_freq_correct_enable_item.add_change_event(
    &m_trans_freq_correct_enable_nonv_event);
  //  Коррекция частоты
  m_freq_correct_item.set_header("Коррекция F");
  m_freq_correct_item.set_message(mp_exit_msg);
  m_freq_correct_item.set_str(mp_user_str, "F", "Гц",
    m_f_correct_item_width, m_f_correct_item_prec);
  m_freq_correct_item.set_max_value(m_max_freq + 5.0);
  m_freq_correct_item.set_min_value(m_min_freq - 5.0);
  m_freq_correct_item.add_change_event(&m_trans_freq_correct_koef_event);
  m_freq_correct_item.set_key_type(IMK_ARROWS);
  m_freq_correct_item.set_change_step(0.001f);
  m_freq_correct_item.set_change_step_max(1.f);
  m_freq_correct_item.progressive_change_enabled(true);
  
  //  Подстройка параметров стрелочками
  m_freq_begin_input.add_change_event(&m_trans_freq_begin_event);
  m_freq_begin_input.add_change_event(&m_trans_freq_begin_nonv_event);
  m_freq_end_input.add_change_event(&m_trans_freq_end_nonv_event);
  m_speed_freq_input.add_change_event(&m_trans_speed_freq_nonv_event);
  m_voltage_ref_input.add_change_event(&m_trans_voltage_ref_nonv_event);

  if (m_freq_correct_enable) {
    mp_generator->set_correct_freq_koef(
      m_nonvolatile_data.freq_correct_koef);
  } else {
    mp_generator->set_correct_freq_koef(1.0);
  }

  //  Отладочный режим
  memset((void*)m_current_interface_mode_key_array, 0,
    key_array_size * sizeof(irskey_t));
  m_ref_interface_mode_key_array[0] = static_cast<irskey_t>(m_key_esc);
  m_ref_interface_mode_key_array[1] = static_cast<irskey_t>(m_key_stop);
  m_ref_interface_mode_key_array[2] = static_cast<irskey_t>(m_key_stop);
  m_ref_interface_mode_key_array[3] = static_cast<irskey_t>(m_key_esc);
  m_ref_interface_mode_key_array[4] = static_cast<irskey_t>(m_key_stop);

  // Идентификатор
  m_id_item.set_header("Идентификатор");
  m_id_item.set_message(mp_exit_msg);
  m_id_item.set_str(mp_user_str, "", "",
    m_id_item_width, m_id_item_prec);
  m_id_item.set_max_value(INT_MAX);
  m_id_item.set_min_value(0);
  m_id_item.add_change_event(&m_id_nonv_event);
  m_id_item.set_key_type(IMK_ARROWS);
  m_id_item.set_change_step(1);
  m_main_menu.add(&m_id_item);
  // Пароль идентификатора
  m_id_pass_item.set_header("Пароль сброса ид.");
  m_id_pass_item.set_message(mp_exit_msg);
  m_id_pass_item.set_str(mp_user_str, "", "",
    m_id_item_width, m_id_item_prec);
  m_id_pass_item.set_max_value(200);
  m_id_pass_item.set_min_value(0);
  m_id_pass_item.add_change_event(&m_id_pass_event);
  m_id_pass_item.set_key_type(IMK_ARROWS);
  m_id_pass_item.set_change_step(1);
  m_main_menu.add(&m_id_pass_item);
  m_main_menu.hide_item(&m_id_pass_item);
  // Сброс EEPROM
  m_reset_nonvolatile_memory_item.set_header("Сбросить настройки");
  m_reset_nonvolatile_memory_item.set_str("Да", "Нет");
  m_reset_nonvolatile_memory_item.add_change_event(
    &m_reset_nonvolatile_memory_nonv_event);
  m_main_menu.add(&m_reset_nonvolatile_memory_item);
  m_main_menu.hide_item(&m_reset_nonvolatile_memory_item);
  m_reset_acv_correct_nonvolatile_memory_item.set_header("Сбросить корр. ACV");
  m_reset_acv_correct_nonvolatile_memory_item.set_str("Да", "Нет");
  m_reset_acv_correct_nonvolatile_memory_item.add_change_event(
    &m_reset_correct_nonvolatile_memory_nonv_event);
  m_main_menu.add(&m_reset_acv_correct_nonvolatile_memory_item);
  m_main_menu.hide_item(&m_reset_acv_correct_nonvolatile_memory_item);

  //!!! Не забыть включить обратно
  mp_window_watchdog->start();
  mp_independent_watchdog->start();


  m_freq_begin_saved_copy = m_freq_begin;
  m_freq_end_saved_copy = m_freq_end;
  m_speed_freq_saved_copy = m_speed_freq;
  m_fast_adc_rms.set_current_type(gtch::adc_rms_t::current_type_ac);

  update_configuration_for_mode_dc_ac();

  //  АЦП
  const reg_options_t* reg_opt = get_actual_reg_options();
  m_adc_fade_data.x1 = 0.;
  m_adc_fade_data.y1 = 0.;
  m_adc_fade_data.t = reg_opt->t_adc /m_filter_update_interval;
  //  Регульятор
  m_reg_pd.k = reg_opt->k;
  m_reg_pd.ki = reg_opt->ki * CNT_TO_DBLTIME(m_reg_interval);
  m_reg_pd.kd = reg_opt->kd / CNT_TO_DBLTIME(m_reg_interval);
  m_reg_pd.min = m_meas_gen_value;
  m_reg_pd.max = mp_generator->get_max_amplitude();
  m_reg_pd.prev_e = 0.;
  m_reg_pd.pp_e = 0.;
  m_reg_pd.prev_out = 0.;
  m_reg_pd.block_int = 0;
  m_reg_pd.block_int_ext = 0;
  m_reg_pd.int_val = 0.;
  m_reg_pd.k_d_pid = 0.1;

  //  Изодромное звено
  m_iso_data.k = reg_opt->iso_k;
  m_iso_data.fd.x1 = 0.;
  m_iso_data.fd.y1 = 0.;
  m_iso_data.fd.t = reg_opt->iso_t / CNT_TO_DBLTIME(m_reg_interval);

  m_rate_data.cur = 0;
  m_rate_data.dt = CNT_TO_DBLTIME(m_reg_interval);
  m_rate_data.slope = 300;
  m_rate_data.recalc = 0;
  m_rate_data.dl = 0;
  m_rate_data.slope_prev = 0;
  m_rate_data.dt_prev = 0;

  mp_generator->stop();
  mp_generator->set_amplitude(0.1f);
  mp_generator->set_frequency(m_freq);
  m_interrupt_generator.start();
}
template <class CFG>
app_t<CFG>::~app_t()
{
  delete []mp_relay_string_1;
  delete []mp_relay_string_2;
}

//  Прерывание от контактов реле
template <class CFG>
app_t<CFG>::gtch_relay_int_t::gtch_relay_int_t(app_t<CFG> &a_outer):
  mxfact_event_t(),
  m_outer(a_outer)
{
}

template <class CFG>
void app_t<CFG>::gtch_relay_int_t::exec()
{
  m_outer.mp_relay_interrupt_generator->stop();
  m_outer.m_relay_meas_time_cnt = counter_get();
  if (m_outer.m_scan_status == SCAN_FORWARD_JUMP) {
    m_outer.m_relay_was_time_forward_action = true;
  } else if (m_outer.m_scan_status == SCAN_RETURN_JUMP) {
    m_outer.m_relay_was_time_return_action = true;
  }
  m_outer.m_buzzer.bzzz();
  mxfact_event_t::exec();
}

template <class CFG>
void app_t<CFG>::set_mode_ac()
{
  m_mode_dc_enabled = false;
  update_configuration_for_mode_dc_ac();
}

template <class CFG>
void app_t<CFG>::set_mode_dc()
{
  m_mode_dc_enabled = true;
  update_configuration_for_mode_dc_ac();
}

template <class CFG>
void app_t<CFG>::update_configuration_for_mode_dc_ac()
{
  if (m_mode_dc_enabled == m_mode_dc_enabled_previous) {
    return;
  }

  if (m_mode_dc_enabled) {
    m_voltage_ref = m_nonvolatile_data.dc_voltage_ref;
    m_freq_begin = 0;
    m_freq_end = 0;
    m_speed_freq = 0;

    m_fast_adc_rms.set_current_type(gtch::adc_rms_t::current_type_dc);
    mp_generator.reset();
    mp_generator.reset(new dc_generator_t(&m_interrupt_generator,
      interrupt_freq_boost_factor, &m_sinus_pwm,
      m_cfg.m_ir2183_dead_time, CFG::sinus_size));

    m_freq_begin_input.set_keys(irskey_none, irskey_none);
    m_freq_end_input.set_keys(irskey_none, irskey_none);
    m_speed_freq_input.set_keys(irskey_none, irskey_none);
  } else {
    m_voltage_ref = m_nonvolatile_data.ac_voltage_ref;
    m_freq_begin = m_nonvolatile_data.freq_begin;
    m_freq_end = m_nonvolatile_data.freq_end;
    m_speed_freq = m_nonvolatile_data.speed_freq;
    m_fast_adc_rms.set_current_type(gtch::adc_rms_t::current_type_ac);
    mp_generator.reset();
    mp_generator.reset(new sinus_generator_t(&m_interrupt_generator,
      interrupt_freq_boost_factor, &m_sinus_pwm,
      CFG::sinus_pwm_frequency, m_cfg.m_ir2183_dead_time, CFG::sinus_size));

    if (m_freq_correct_enable) {
      mp_generator->set_correct_freq_koef(
        m_nonvolatile_data.freq_correct_koef);
    } else {
      mp_generator->set_correct_freq_koef(1.0);
    }


    m_freq_begin_input.set_keys(static_cast<irskey_t>(m_key_Fn_up),
      static_cast<irskey_t>(m_key_Fn_down));
    m_freq_end_input.set_keys(static_cast<irskey_t>(m_key_Fk_up),
      static_cast<irskey_t>(m_key_Fk_down));
    m_speed_freq_input.set_keys(static_cast<irskey_t>(m_key_S_up),
      static_cast<irskey_t>(m_key_S_down));
  }
  update_config_voltage_range();

  m_mode_dc_enabled_previous = m_mode_dc_enabled;
}

template <class CFG>
void app_t<CFG>::update_config_voltage_range()
{
  if (m_mode_dc_enabled) {
    if (m_interface_mode == RELEASE) {
      m_min_voltage = m_min_dc_voltage_release;
      m_max_voltage = m_max_dc_voltage_release;
    } else {
      m_min_voltage = m_min_dc_voltage_debug;
      m_max_voltage = m_max_dc_voltage_debug;
    }
  } else {
    if (m_interface_mode == RELEASE) {
      m_min_voltage = m_min_ac_voltage_release;
      m_max_voltage = m_max_ac_voltage_release;
    } else {
      m_min_voltage = m_min_ac_voltage_debug;
      m_max_voltage = m_max_ac_voltage_debug;
    }
  }

  m_voltage_ref = irs::bound(m_voltage_ref, m_min_voltage, m_max_voltage);

  m_voltage_ref_input.reset_step();
}

template <class CFG>
reg_options_t* app_t<CFG>::get_actual_reg_options()
{
  if (m_mode_dc_enabled) {
    return mp_dc_reg_options.get();
  }
  return mp_ac_reg_options.get();
}

template <class CFG>
double app_t<CFG>::get_meas_k_default() const
{
  if (m_mode_dc_enabled) {
    return m_dc_meas_k_default;
  }
  return m_ac_meas_k_default;
}

template <class CFG>
std::vector<typename app_t<CFG>::size_type>
app_t<CFG>::make_adc_channels_sko_period_count()
{
  std::vector<size_type> sizes;
  sizes.push_back(m_sko_period_count);
  sizes.push_back(m_short_sko_period_count);
  return sizes;
}

template <class CFG>
float app_t<CFG>::adc_to_voltage(adc_data_t a_adc)
{
  #if GTCH_SK_STM32F217
  const float k = m_adc_settings.v_reference/
    m_cfg.get_adc()->get_u16_maximum()*100;
  #else // Плата ГТЧ
  // Коэффициент подобран экспериментально при U = 51.4 В
  const float experimental_factor = 122.995831;
  const float k = experimental_factor*
    (m_adc_settings.v_reference/
     static_cast<irs_i16>(m_cfg.get_adc()->get_u16_maximum()));
  #endif // Плата ГТЧ
  float voltage = k*a_adc;
  if (m_mode_dc_enabled) {
    if (m_dc_voltage_correct_enable &&
        (mp_cur_menu != &m_dc_voltage_correct_item)) {
      voltage = voltage*m_nonvolatile_data.dc_voltage_correct_koef;
    }
  } else {
    if (m_ac_voltage_correct_enable && (mp_cur_menu != &m_ac_voltage_correct_item)) {
      voltage = m_ac_voltage_correct.apply(m_freq, voltage, voltage);
    }
  }
  return voltage;
}

template <class CFG>
void app_t<CFG>::to_stop()
{
  m_overcurrent_detection_enabled = false;
  mp_generator->stop();
  mp_generator->set_amplitude(0.f);
  m_reg_timer.stop();
  m_main_creep.change_static("Режим: СТОП");
  voltage_filter_off();
  m_status = STOP;
  if (m_interface_mode == RELEASE) {
    m_main_screen.creep_start();
  } else {
    strcpy(mp_relay_string_2, "<СТОП>");
  }
  mp_relay_interrupt_generator->stop();
  m_freq = m_freq_begin;
  m_operating_duty = false;
  if (mp_cur_menu == &m_ac_voltage_correct_item) {
    m_ac_voltage_correct_item.external_process_stop();
  }
}

template <class CFG>
void app_t<CFG>::to_start()
{
  m_rate_data.cur = m_voltage;

  const reg_options_t* reg_opt = get_actual_reg_options();

  m_meas_k = get_meas_k_default();
  m_reg_pd.k = reg_opt->k * m_meas_k;
  m_reg_pd.ki = reg_opt->ki * CNT_TO_DBLTIME(m_reg_interval);
  m_reg_pd.kd = reg_opt->kd / CNT_TO_DBLTIME(m_reg_interval);
  m_reg_timer.set(m_reg_interval);

  m_iso_data.k = reg_opt->iso_k;
  m_iso_data.fd.t = reg_opt->iso_t / CNT_TO_DBLTIME(m_reg_interval);
  m_rate_data.dt = CNT_TO_DBLTIME(m_reg_interval);

  const double cur_voltage_ref =
    m_voltage_ref;//irs::rate_limiter(&m_rate_data, m_voltage_ref);
  float e = cur_voltage_ref - m_voltage;
  pid_reg_sync(&m_reg_pd, e, m_meas_gen_value);
  mp_generator->set_amplitude(m_meas_gen_value);
  mp_generator->start();
  m_overcurrent_detection_enabled = true;

  m_reg_timer.start();
}

template <class CFG>
void app_t<CFG>::to_overheat()
{
  mp_generator->stop();
  mp_generator->set_amplitude(0.f);
  m_reg_timer.stop();
  m_main_creep.change_static("< T° >");
  m_status = OVERHEAT;
  if (m_interface_mode == RELEASE) {
    m_main_screen.creep_start();
  } else {
    strcpy(mp_relay_string_2, "< T° >");
  }
  mp_relay_interrupt_generator->stop();
  m_freq = m_freq_begin;
  m_buzzer.bzzz();
  if (mp_cur_menu == &m_ac_voltage_correct_item) {
    m_ac_voltage_correct_item.external_process_stop();
  }
}

template <class CFG>
void app_t<CFG>::to_pause()
{
  m_status = PAUSE;
  m_scan_status = SCAN_STOP;
  if (m_interface_mode == RELEASE) {
    strcpy(mp_relay_string_1, pause_str);
    strcpy(mp_relay_string_2, "");
  } else {
    strcpy(mp_relay_string_2, "<ПАУЗА>");
  }
  voltage_filter_on();
  mp_relay_interrupt_generator->stop();
}

template <class CFG>
void app_t<CFG>::to_message()
{
  m_lcd_drv_service.clear();
  m_lcd_drv_service.outtextpos(0, 0, "Результаты:");

  direction_t dir = UP;
  if (m_freq_begin > m_freq_end) {
    dir = DOWN;
  }

  create_freq_string(dir, &m_freq_forward, m_relay_was_forward_action,
    mp_message_string);

  if (m_scan_correct_freq_by_time && m_relay_was_time_forward_action) {
    strcat(mp_message_string, "*");
  }
  space_fill_message_string();

  if (m_scan_need_meas_time) {
    create_time_string(dir, &m_time_forward, m_relay_was_time_forward_action,
      &mp_message_string[m_half_screen]);
  }
  bool place_second_message_on_top = false;
  if (m_scan_tuda_obratno && dir == DOWN) {
    place_second_message_on_top = true;
    m_lcd_drv_service.outtextpos(0, 2, mp_message_string);
  }
  else m_lcd_drv_service.outtextpos(0, 1, mp_message_string);
  if (m_scan_tuda_obratno) {
    if (dir == UP) dir = DOWN;
    else dir = UP;

    create_freq_string(dir, &m_freq_return, m_relay_was_return_action,
      mp_message_string);
    if (m_scan_correct_freq_by_time && m_relay_was_time_return_action) {
      strcat(mp_message_string, "*");
    }

    space_fill_message_string();

    if (m_scan_need_meas_time) {
      create_time_string(dir, &m_time_return, m_relay_was_time_return_action,
        &mp_message_string[m_half_screen]);
    }
    if (place_second_message_on_top) {
      m_lcd_drv_service.outtextpos(0, 1, mp_message_string);
    } else {
      m_lcd_drv_service.outtextpos(0, 2, mp_message_string);
    }
  }

  if (m_scan_tuda_obratno) {
    if (!m_relay_was_forward_action &&
      !m_relay_was_return_action &&
      !m_relay_was_time_forward_action &&
      !m_relay_was_time_return_action) {
      m_lcd_drv_service.outtextpos(0, 3, "Нет срабатываний");
    } else if (!m_relay_was_forward_action ||
      !m_relay_was_return_action ||
      !m_relay_was_time_forward_action ||
      !m_relay_was_time_return_action) {
      m_lcd_drv_service.outtextpos(0, 3, "Не все срабатывания");
    } else if (m_relay_was_forward_action &&
      m_relay_was_return_action &&
      m_relay_was_time_forward_action &&
      m_relay_was_time_return_action) {
      m_lcd_drv_service.outtextpos(0, 3, "Все срабатывания");
    }
  } else {
    if (!m_relay_was_forward_action &&
      !m_relay_was_time_forward_action) {
      m_lcd_drv_service.outtextpos(0, 3, "Нет срабатываний");
    } else if (!m_relay_was_forward_action ||
      !m_relay_was_time_forward_action) {
      m_lcd_drv_service.outtextpos(0, 3, "Не все срабатывания");
    } else if (m_relay_was_forward_action &&
      m_relay_was_time_forward_action) {
      m_lcd_drv_service.outtextpos(0, 3, "Все срабатывания");
    }
  }

  m_status = MESSAGE;
  m_scan_status = SCAN_STOP;
}

template <class CFG>
void app_t<CFG>::to_correct_pause()
{
  m_status = CORRECT_ACV_PAUSE;
  m_scan_status = SCAN_STOP;
  voltage_filter_on();
  mp_relay_interrupt_generator->stop();
}

template <class CFG>
void app_t<CFG>::create_time_string(direction_t a_direction, float *ap_time,
  bool a_action, char *ap_str)
{
  char const *time_str = time_up_str;
  if (a_direction == DOWN) {
    time_str = time_down_str;
  }
  strcpy(ap_str, time_str);
  if (a_action) {
    afloat_to_str(&ap_str[strlen(time_str)], *ap_time,
      m_time_item_width, m_time_item_prec);
  }
  else strcat(&ap_str[strlen(time_str)], relay_no_action_str);
}

template <class CFG>
void app_t<CFG>::create_freq_string(direction_t a_direction, float *ap_freq,
  bool a_action, char *ap_str)
{
  char const *freq_str = freq_up_str;
  if (a_direction == DOWN) {
    freq_str = freq_down_str;
  }
  strcpy(ap_str, freq_str);
  if (a_action) {
    afloat_to_str(&ap_str[strlen(freq_str)], *ap_freq,
      m_menu_item_width, m_menu_item_prec);
  } else {
    strcat(&ap_str[strlen(freq_str)], relay_no_action_str);
  }
}

template <class CFG>
void app_t<CFG>::space_fill_message_string()
{
  for (size_t i = strlen(mp_message_string); i < mp_lcd_drv->get_width(); i++) {
    mp_message_string[i] = ' ';
  }
  mp_message_string[mp_lcd_drv->get_width()] = 0;
}

template <class CFG>
bool app_t<CFG>::operating_duty()
{
  if (m_voltage_ref == 0.f) {
    return false;
  }
  if (abs(1.f - m_voltage_display / m_voltage_ref) <
    m_operating_duty_deviation) {
    return true;
  } else {
    return false;
  }
}

template <class CFG>
bool app_t<CFG>::reg_on()
{
  if (m_status == SCAN) {
    return true;
  }
  if (m_status == PAUSE) {
    return true;
  }
  if (m_status == SETUP_MEAS) {
    return true;
  }
  if (m_status == SETUP) {
    return true;
  }
  if (m_status == SETUP_READY) {
    return true;
  }
  return false;
}

template <class CFG>
bool app_t<CFG>::test_freq_limit(double & a_freq, double &a_freq_limit)
{
  if ((m_scan_direction == UP) &&
    (a_freq >= (a_freq_limit - 0.5f * m_scan_freq_step))) {
    return true;
  }
  if ((m_scan_direction == DOWN) &&
    (a_freq <= (a_freq_limit + 0.5f * m_scan_freq_step))) {
    return true;
  }
  return false;
}

template <class CFG>
void app_t<CFG>::in_tick()
{
  if (m_status != OVERHEAT) {
    if (m_temperature > m_termo_limit) {
      to_overheat();
      if (mp_cur_menu == &m_ac_voltage_correct_item) {
        m_ac_voltage_correct_item.external_process_stop();
      }
    }
  }

  //  Реакция на события без записи в ЕЕПРОМ
  if (m_trans_contrast_event.check()) {
    mp_lcd_contrast_pwm->set_duty(float(m_contrast));
  }

  if (m_trans_mode_dc_event.check()) {
    update_configuration_for_mode_dc_ac();
  }

  if (m_trans_freq_begin_event.check()) {
    m_freq = m_freq_begin;
    if (m_status == PAUSE) mp_generator->set_frequency(m_freq);
  }

  if (m_trans_current_limit_event.check()) {
    mp_current_limit_code->set_u32_data(0,
      static_cast<irs_u32>(m_current_limit*m_trans_koef));
  }
  if (m_trans_ac_ki_event.check()) {
    if (!m_mode_dc_enabled) {
      m_reg_pd.ki = mp_ac_reg_options->ki * CNT_TO_DBLTIME(m_reg_interval);
    }
  }
  if (m_trans_ac_kd_event.check()) {
    if (!m_mode_dc_enabled) {
      m_reg_pd.kd = mp_ac_reg_options->kd / CNT_TO_DBLTIME(m_reg_interval);
    }
  }
  if (m_trans_ac_t_adc_event.check()) {
    if (!m_mode_dc_enabled) {
      m_adc_fade_data.t = mp_ac_reg_options->t_adc/m_filter_update_interval;
    }
  }

  if (m_trans_ac_iso_k_event.check()) {
    if (!m_mode_dc_enabled) {
      m_iso_data.k = mp_ac_reg_options->iso_k;
    }
  }
  if (m_trans_ac_iso_t_event.check()) {
    if (!m_mode_dc_enabled) {
      m_iso_data.fd.t = mp_ac_reg_options->iso_t / CNT_TO_DBLTIME(m_reg_interval);
    }
  }

  if (m_trans_dc_ki_event.check()) {
    if (m_mode_dc_enabled) {
      m_reg_pd.ki = mp_dc_reg_options->ki * CNT_TO_DBLTIME(m_reg_interval);
    }
  }
  if (m_trans_dc_kd_event.check()) {
    if (m_mode_dc_enabled) {
      m_reg_pd.kd = mp_dc_reg_options->kd / CNT_TO_DBLTIME(m_reg_interval);
    }
  }
  if (m_trans_dc_t_adc_event.check()) {
    if (m_mode_dc_enabled) {
      m_adc_fade_data.t = mp_dc_reg_options->t_adc/m_filter_update_interval;
    }
  }

  if (m_trans_dc_iso_k_event.check()) {
    if (m_mode_dc_enabled) {
      m_iso_data.k = mp_dc_reg_options->iso_k;
    }
  }
  if (m_trans_dc_iso_t_event.check()) {
    if (m_mode_dc_enabled) {
      m_iso_data.fd.t = mp_dc_reg_options->iso_t /
        CNT_TO_DBLTIME(m_reg_interval);
    }
  }

  if (m_id_pass_event.check()) {
    irs_u16 id_pass = static_cast<irs_u16>(m_id_pass);
    if (id_pass == m_id_pass_src) {
      id(0);
      m_id_pass = 0;
    }
  }

  //  Запись в ЕЕПРОМ
  if (m_nonvolatile_update_timer.check()) {
    if (m_trans_contrast_nonv_event.check()) {
      m_nonvolatile_data.contrast = m_contrast;
    }
    if (m_trans_ac_k_nonv_event.check()) {
      m_nonvolatile_data.ac_k = mp_ac_reg_options->k;
    }
    if (m_trans_ac_ki_nonv_event.check()) {
      m_nonvolatile_data.ac_ki = mp_ac_reg_options->ki;
    }
    if (m_trans_ac_kd_nonv_event.check()) {
      m_nonvolatile_data.ac_kd = mp_ac_reg_options->kd;
    }
    if (m_trans_dc_k_nonv_event.check()) {
      m_nonvolatile_data.dc_k = mp_dc_reg_options->k;
    }
    if (m_trans_dc_ki_nonv_event.check()) {
      m_nonvolatile_data.dc_ki = mp_dc_reg_options->ki;
    }
    if (m_trans_dc_kd_nonv_event.check()) {
      m_nonvolatile_data.dc_kd = mp_dc_reg_options->kd;
    }
    if (m_trans_current_limit_nonv_event.check()) {
      m_nonvolatile_data.current_limit = m_current_limit;
    }
    if (m_trans_termo_limit_nonv_event.check()) {
      m_nonvolatile_data.termo_limit = m_termo_limit;
    }
    if (m_trans_freq_begin_nonv_event.check()) {
      if (!m_mode_dc_enabled) {
        m_nonvolatile_data.freq_begin = m_freq_begin;
      }
    }
    if (m_trans_freq_end_nonv_event.check()) {
      if (!m_mode_dc_enabled) {
        m_nonvolatile_data.freq_end = m_freq_end;
      }
    }
    if (m_trans_speed_freq_nonv_event.check()) {
      if (!m_mode_dc_enabled) {
        m_nonvolatile_data.speed_freq = m_speed_freq;
      }
    }
    if (m_trans_voltage_ref_nonv_event.check()) {
      if (m_interface_mode == RELEASE) {
        if (m_mode_dc_enabled) {
          m_nonvolatile_data.dc_voltage_ref = m_voltage_ref;
        } else {
          m_nonvolatile_data.ac_voltage_ref = m_voltage_ref;
        }
      }
    }
    if (m_trans_mode_dc_nonv_event.check()) {
      m_nonvolatile_data.mode_dc_enabled = m_mode_dc_enabled;
    }
    if (m_trans_go_to_end_nonv_event.check()) {
      m_nonvolatile_data.go_to_end = m_scan_go_to_end;
    }
    if (m_trans_need_meas_time_nonv_event.check()) {
      m_nonvolatile_data.need_meas_time = m_scan_need_meas_time;
    }
    if (m_trans_correct_freq_by_time_nonv_event.check()) {
      m_nonvolatile_data.correct_freq_by_time = m_scan_correct_freq_by_time;
    }
    if (m_trans_tuda_obratno_nonv_event.check()) {
      m_nonvolatile_data.tuda_obratno = m_scan_tuda_obratno;
    }
    if (m_trans_ac_need_meas_koef_nonv_event.check()) {
      m_nonvolatile_data.ac_need_meas_koef = mp_ac_reg_options->need_meas_koef;
    }
    if (m_trans_dc_need_meas_koef_nonv_event.check()) {
      m_nonvolatile_data.dc_need_meas_koef = mp_dc_reg_options->need_meas_koef;
    }
    if (m_reset_nonvolatile_memory_nonv_event.check()) {
      m_nonvolatile_data.reset_nonvolatile_memory = m_reset_nonvolatile_memory;
    }
    if (m_reset_correct_nonvolatile_memory_nonv_event.check()) {
      m_nonvolatile_data.reset_correct_nonvolatile_memory =
        m_reset_acv_correct_nonvolatile_memory;
    }
    if (m_trans_ac_voltage_correct_enable_nonv_event.check()) {
      m_nonvolatile_data.voltage_correct_enable = m_ac_voltage_correct_enable;
    }
    if (m_trans_dc_voltage_correct_koef_event.check()) {
      const float koef = m_dc_voltage_correct / m_dc_voltage_ref_correct;
      m_nonvolatile_data.dc_voltage_correct_koef = koef;
    }
    if (m_trans_dc_voltage_correct_enable_nonv_event.check()) {
      m_nonvolatile_data.dc_voltage_correct_enable =
        m_dc_voltage_correct_enable;
    }
    if (m_trans_freq_correct_enable_nonv_event.check()) {
      m_nonvolatile_data.freq_correct_enable = m_freq_correct_enable;
      if (m_freq_correct_enable) {
        mp_generator->set_correct_freq_koef(
          m_nonvolatile_data.freq_correct_koef);
      } else {
        mp_generator->set_correct_freq_koef(1.0);
      }
    }
    if (m_trans_freq_correct_koef_event.check()) {
      float koef = m_freq_correct_result / m_freq_correct_set;
      m_nonvolatile_data.freq_correct_koef = koef;

      if (m_freq_correct_enable) {
        mp_generator->set_correct_freq_koef(koef);
      } else {
        mp_generator->set_correct_freq_koef(1.0);
      }
    }
    if (m_trans_ac_t_adc_nonv_event.check()) {
      m_nonvolatile_data.ac_t_adc = mp_ac_reg_options->t_adc;
    }
    if (m_trans_ac_iso_k_nonv_event.check()) {
      m_nonvolatile_data.ac_iso_k = mp_ac_reg_options->iso_k;
    }
    if (m_trans_ac_iso_t_nonv_event.check()) {
      m_nonvolatile_data.ac_iso_t = mp_ac_reg_options->iso_t;
    }
    if (m_trans_ac_dead_band_nonv_event.check()) {
      m_nonvolatile_data.ac_dead_band = mp_ac_reg_options->dead_band;
    }


    if (m_trans_dc_t_adc_nonv_event.check()) {
      m_nonvolatile_data.dc_t_adc = mp_dc_reg_options->t_adc;
    }
    if (m_trans_dc_iso_k_nonv_event.check()) {
      m_nonvolatile_data.dc_iso_k = mp_dc_reg_options->iso_k;
    }
    if (m_trans_dc_iso_t_nonv_event.check()) {
      m_nonvolatile_data.dc_iso_t = mp_dc_reg_options->iso_t;
    }
    if (m_trans_dc_dead_band_nonv_event.check()) {
      m_nonvolatile_data.dc_dead_band = mp_dc_reg_options->dead_band;
    }
    if (m_id_nonv_event.check()) {
      // Блокировка изменения идентификатора
      id(id());
      // Запись идентификатора в EEPROM
      m_nonvolatile_data.id = id();
    }
  }
  if (m_buzzer_kb_event.check()) {
    m_buzzer.bzz();
  }



  //  Коррекция - еепром
  if (mp_cur_menu == &m_ac_voltage_correct_item) {
    if (m_ac_voltage_correct_item.check_out_param_change()) {
      size_t point = 0;
      double user_value = 0.;
      m_ac_voltage_correct_item.get_last_point(point, user_value);
      irs_u8 u = point / m_ac_voltage_correct_map.m_x_count;
      double u_value = m_ac_voltage_correct_map.mp_y_points[u+1];
      m_ac_voltage_correct_map.mp_k_array[point + m_ac_voltage_correct_map.m_x_count]
        = user_value / u_value;
    }
  }

  const irskey_t key = m_hot_kb_event.check();

  if (test_switch_interface_mode(key)) {
    switch (m_interface_mode) {
      case DEBUG: {
        m_voltage_ref_input.set_step(m_voltage_step_release, 
          m_voltage_step_max_release);
        
        m_voltage_ref_item.set_length(m_volt_item_width);
        m_voltage_ref_item.set_accuracy(m_volt_item_prec);
        m_voltage_item.set_length(m_volt_item_width);
        m_voltage_item.set_accuracy(m_volt_item_prec);

        m_main_screen.creep_start();
        m_main_screen.hide_item(&m_temperature_item);
        m_main_screen.hide_item(&m_adc_item);
        m_main_screen.hide_item(&m_pwm_out_item);
        m_main_screen.hide_item(&m_meas_k_item);
        m_main_screen.show_item(&m_relay_string_item_1);

        m_main_menu.hide_item(&m_ac_reg_menu);
        m_main_menu.hide_item(&m_dc_reg_menu);
        m_main_menu.hide_item(&m_limit_menu);
        m_main_menu.hide_item(&m_scan_menu);
        m_main_menu.hide_item(&m_correct_menu);
        m_main_menu.hide_item(&m_id_pass_item);
        m_main_menu.hide_item(&m_reset_nonvolatile_memory_item);
        m_main_menu.hide_item(&m_reset_acv_correct_nonvolatile_memory_item);
        m_main_menu.hide_item(&m_contrast_item);
        m_main_menu.show_item(&m_go_to_end_item);
        m_main_menu.show_item(&m_need_meas_time_item);
        m_main_menu.show_item(&m_correct_freq_by_time_item);
        m_main_menu.show_item(&m_tuda_obratno_item);
        strcpy(mp_relay_string_2, "");
        m_can_reg = true;
        m_interface_mode = RELEASE;
        break;
      }
      case RELEASE: {

        m_voltage_ref_input.set_step(m_voltage_step_debug, 
          m_voltage_step_max_debug);
        
        m_voltage_ref_item.set_length(m_volt_item_width_debug);
        m_voltage_ref_item.set_accuracy(m_volt_item_prec_debug);
        m_voltage_item.set_length(m_volt_item_width_debug);
        m_voltage_item.set_accuracy(m_volt_item_prec_debug);

        m_main_screen.creep_stop();
        m_main_screen.hide_item(&m_relay_string_item_1);
        m_main_screen.show_item(&m_temperature_item);
        m_main_screen.show_item(&m_adc_item);
        m_main_screen.show_item(&m_pwm_out_item);
        m_main_screen.show_item(&m_meas_k_item);

        m_main_menu.show_item(&m_ac_reg_menu);
        m_main_menu.show_item(&m_dc_reg_menu);
        m_main_menu.show_item(&m_limit_menu);
        m_main_menu.show_item(&m_scan_menu);
        m_main_menu.show_item(&m_correct_menu);
        m_main_menu.show_item(&m_id_pass_item);
        m_main_menu.show_item(&m_reset_nonvolatile_memory_item);
        m_main_menu.show_item(&m_reset_acv_correct_nonvolatile_memory_item);
        m_main_menu.show_item(&m_contrast_item);
        m_main_menu.hide_item(&m_go_to_end_item);
        m_main_menu.hide_item(&m_need_meas_time_item);
        m_main_menu.hide_item(&m_correct_freq_by_time_item);
        m_main_menu.hide_item(&m_tuda_obratno_item);
        strcpy(mp_relay_string_2, "<СТОП>");
        m_interface_mode = DEBUG;
        break;
      }
    }
    update_config_voltage_range();
  }

  switch (m_status) {
    case SPLASH: {
      if (m_splash_timer.check())
      {
        m_status = STOP;
        m_splash_timer.stop();
      }
      break;
    }
    case STOP: {
      if (mp_cur_menu == &m_main_screen) {
        if ((key == m_key_Fn_up) && m_mode_dc_enabled) {
          m_mode_dc_enabled = false;
          m_trans_mode_dc_nonv_event.exec();
          update_configuration_for_mode_dc_ac();
        } else if ((key == m_key_Fn_down) && (m_freq_begin == m_min_freq)) {
          if (!m_mode_dc_enabled) {
            m_mode_dc_enabled = true;
            m_trans_mode_dc_nonv_event.exec();
            update_configuration_for_mode_dc_ac();
          }
        } else {
          switch_key(key);
        }

        if (m_freq != m_freq_begin) {
          m_freq = m_freq_begin;
        }
        if (key == m_key_start) {
          m_status = SETUP_MEAS;
          if (m_interface_mode == RELEASE) {
            m_main_screen.creep_stop();
            strcpy(mp_relay_string_1, voltage_setup_str_1);
            strcpy(mp_relay_string_2, voltage_setup_str_2);
          } else {
            strcpy(mp_relay_string_2, "<УСТ>");
          }
          m_can_reg = true;
          m_freq = m_freq_begin;
          mp_generator->set_frequency(m_freq);
          to_start();
        }
      } else if (mp_cur_menu == &m_ac_voltage_correct_item) {
        if (m_ac_voltage_correct_item.check_process_start())
        {
          set_mode_ac();
          m_status = CORRECT_ACV_SETUP_MEAS;
          mp_generator->set_frequency(m_freq);
          to_start();
        }
        if (key == m_key_Fn_up) {
          m_ac_voltage_correct_item.reset_cur_param_to(
            irs_menu_2param_master_item_t::value_second);
        }
      } else if (mp_cur_menu == &m_dc_voltage_correct_item) {
        if (key == m_key_start) {
          set_mode_dc();
          m_voltage_ref = m_dc_voltage_ref_correct;
          m_status = CORRECT_DCV;
          to_start();
        }
      } else if (mp_cur_menu == &m_freq_correct_item) {
        if (key == m_key_start) {
          set_mode_ac();
          mp_generator->set_correct_freq_koef(1.0);
          m_status = CORRECT_F;
          m_freq = m_freq_correct_set;
          mp_generator->set_frequency(m_freq);
          to_start();
        }
      }
      break;
    }
    case CORRECT_ACV_SETUP_MEAS: {
      if (m_ac_voltage_correct_item.check_process_stop())
      {
        m_ac_voltage_correct_item.set_operating_duty(false);
        to_stop();
      } else {
        reg_with_meas();
        if (m_operating_duty) {
          m_ac_voltage_correct_item.set_operating_duty(true);
          to_correct_pause();
        }
        if (m_ac_voltage_correct_item.check_process_change_param()) {
          mp_generator->set_frequency(m_freq);
        }
      }
      break;
    }
    case CORRECT_ACV_PAUSE: {
      if (m_ac_voltage_correct_item.check_process_stop()) {
        m_ac_voltage_correct_item.set_operating_duty(false);
        to_stop();
      } else {
        reg();
        m_ac_voltage_correct_item.set_operating_duty(m_operating_duty);
        if (m_ac_voltage_correct_item.check_process_change_param()) {
          mp_generator->set_frequency(m_freq);
        }
      }
      break;
    }
    case CORRECT_DCV: {
      if ((mp_cur_menu != &m_dc_voltage_correct_item) || (key == m_key_stop)) {
        if (m_freq_correct_enable) {
          mp_generator->set_correct_freq_koef(
            m_nonvolatile_data.freq_correct_koef);
        }
        to_stop();
        m_voltage_ref = m_nonvolatile_data.dc_voltage_ref;
      } else {
        if (!m_operating_duty) {
          reg_with_meas();
        } else {
          reg();
        }
      }
      break;
    }
    case CORRECT_F: {
      if (mp_cur_menu != &m_freq_correct_item || key == m_key_stop) {
        if (m_freq_correct_enable) {
          mp_generator->set_correct_freq_koef(
            m_nonvolatile_data.freq_correct_koef);
        }
        to_stop();
      } else {
        if (!m_operating_duty) {
          reg_with_meas();
        } else {
          reg();
        }
      }
      break;
    }
    case OVERHEAT: {
      if (m_temperature < (m_termo_limit - m_termo_hist)) {
        to_stop();
      }
      if (mp_cur_menu == &m_main_screen) {
        switch_key(key);
      }
      if (mp_cur_menu == &m_ac_voltage_correct_item) {
        if (m_ac_voltage_correct_item.check_process_start()) {
          m_ac_voltage_correct_item.external_process_stop();
        }
      }
      break;
    }
    case SETUP_MEAS: {
      m_menu_kb_event.check();
      
      if (key == m_key_stop) {
        to_stop();
      } else {
        reg_with_meas();
        if (m_operating_duty) {
          to_pause();
        } else {
          switch_key(key);
        }
        
        if (key == m_key_pause) {
          if (m_interface_mode == DEBUG) {
            m_can_reg = !m_can_reg;
          }
        }       
      }
      break;
    }
    case PAUSE: {
      m_menu_kb_event.check();

      if (key == m_key_stop) {
        to_stop();
      } else {
        if ((key == m_key_start) && (m_interface_mode == RELEASE)) {
          if (m_freq_begin != m_freq_end) {
            m_status = SCAN;
            m_scan_interval = irs::make_cnt_s(m_scan_freq_step / m_speed_freq);

            if (m_scan_need_meas_time) {
              m_scan_status = SCAN_FORWARD_JUMP_SETUP;
              strcpy(mp_relay_string_1, "Измерение ");
              if (m_scan_direction == UP) {
                strcpy(mp_relay_string_2, "T^");
              } else {
                strcpy(mp_relay_string_2, "T~");
              }
            } else {
              m_scan_status = SCAN_FORWARD_SETUP;
            }
            if (m_freq_begin < m_freq_end) {
              m_scan_direction = UP;
            } else {
              m_scan_direction = DOWN;
            }

            m_scan_timer.set(m_scan_jump_interval);
            m_scan_timer.start();

            m_freq = m_freq_begin;

            mp_generator->set_frequency(m_freq);

          }
        } else {
          switch_key(key);
        }

        if (m_interface_mode == DEBUG) {
          if (key == m_key_pause) {
            m_can_reg = !m_can_reg;
            if (m_can_reg) {
              m_lcd_drv_service.outtextpos(19, 3, "R");
            } else {
              m_lcd_drv_service.outtextpos(19, 3, " ");
            }
          }
        }

        reg();
      }
      break;
    }
    case SCAN: {
      m_menu_kb_event.check();
      switch (key) {
        case m_key_stop:  to_stop();  break;
        case m_key_pause: to_pause(); break;
        default:
        {
          switch (m_scan_status)
          {
            case SCAN_STOP:
            {
              //  Не бывает
              to_pause();
              break;
            }
            case SCAN_FORWARD_JUMP_SETUP:
            {
              if (m_scan_timer.check())
              {
                m_scan_timer.set(m_scan_jump_wait_interval);//  1 min for wait
                m_scan_timer.start();

                m_freq = m_freq_end;
                mp_generator->set_frequency(m_freq);
                mp_relay_interrupt_generator->start();
                m_relay_ref_time_cnt = counter_get();
                m_relay_was_time_forward_action = false;

                m_scan_status = SCAN_FORWARD_JUMP;
              }
              break;
            }
            case SCAN_FORWARD_JUMP:
            {
              if (m_relay_was_time_forward_action || m_scan_timer.check()) {
                mp_relay_interrupt_generator->stop();
                if (m_relay_was_time_forward_action) {
                  m_time_forward = CNT_TO_DBLTIME(m_relay_meas_time_cnt
                    - m_relay_ref_time_cnt);
                  m_scan_timer.set(m_scan_pause_interval);
                  m_scan_timer.start();
                } else {
                  m_scan_timer.set(m_scan_interval);
                  m_scan_timer.start();
                }
                //  Показать сообщение
                create_time_string(m_scan_direction, &m_time_forward,
                  m_relay_was_time_forward_action, mp_message_string);
                strcpy(mp_relay_string_1, mp_message_string);
                strcpy(mp_relay_string_2, "");

                if (m_scan_tuda_obratno) {
                  //  Прыгнуть обратно
                  m_scan_status = SCAN_RETURN_JUMP_SETUP;
                } else {
                  //  Переход к сканированию
                  m_scan_status = SCAN_FORWARD_SETUP;
                  m_scan_timer.set(m_scan_jump_interval);
                  m_scan_timer.start();
                  m_freq = m_freq_begin;
                  mp_generator->set_frequency(m_freq);
                }
              }
              break;
            }
            case SCAN_RETURN_JUMP_SETUP: {
              if (m_scan_timer.check()) {
                m_scan_timer.set(m_scan_jump_wait_interval);
                m_scan_timer.start();

                m_freq = m_freq_begin;
                mp_generator->set_frequency(m_freq);
                mp_relay_interrupt_generator->start();
                m_relay_ref_time_cnt = counter_get();
                m_relay_was_time_return_action = false;

                strcpy(mp_relay_string_1, "Измерение ");
                //  Тут наоборот
                if (m_scan_direction == UP) strcpy(mp_relay_string_2, "T~");
                else strcpy(mp_relay_string_2, "T^");

                m_scan_status = SCAN_RETURN_JUMP;
              }
              break;
            }
            case SCAN_RETURN_JUMP: {
              if (m_relay_was_time_return_action || m_scan_timer.check()) {
                mp_relay_interrupt_generator->stop();
                if (m_relay_was_time_return_action) {
                  m_time_return = CNT_TO_DBLTIME(m_relay_meas_time_cnt
                    - m_relay_ref_time_cnt);
                  m_scan_timer.set(m_scan_pause_interval);
                  m_scan_timer.start();
                } else {
                  m_scan_timer.set(m_scan_interval);
                  m_scan_timer.start();
                }
                //  Показать сообщение
                direction_t return_dir = DOWN;
                if (m_scan_direction == DOWN) return_dir = UP;
                create_time_string(return_dir, &m_time_return,
                  m_relay_was_time_return_action, mp_message_string);
                strcpy(mp_relay_string_1, mp_message_string);
                strcpy(mp_relay_string_2, "");
                //  Переход к сканированию
                m_scan_status = SCAN_FORWARD_SETUP;
              }
              break;
            }
            case SCAN_FORWARD_SETUP: {
              if (m_scan_timer.check()) {
                m_scan_timer.set(m_scan_interval);
                m_scan_timer.start();
                m_relay_was_forward_action = false;
                m_relay_was_forward_correct = false;

                strcpy(mp_relay_string_1, "Сканирование");
                strcpy(mp_relay_string_2, "");

                m_relay_contact_prev = mp_relay_contact_pin->pin();
                m_scan_status = SCAN_FORWARD;
              }
              break;
            }
            case SCAN_FORWARD: {
              if (m_scan_timer.check()) {
                if (!test_freq_limit(m_freq, m_freq_end)) {
                  if (m_relay_contact_prev != mp_relay_contact_pin->pin()) {
                    m_relay_contact_prev = mp_relay_contact_pin->pin();
                    m_relay_was_forward_action = true;
                    m_freq_forward = m_freq;
                    m_buzzer.bzzz();

                    if (m_scan_correct_freq_by_time) {
                      if (m_scan_need_meas_time) {
                        if (m_relay_was_time_forward_action) {
                          m_relay_was_forward_correct = true;
                          float df = m_time_forward * m_speed_freq;
                          if (m_scan_direction == UP) m_freq_forward -= df;
                          else m_freq_forward += df;
                        }
                      }
                    }

                    if (!m_scan_tuda_obratno && !m_scan_go_to_end) {
                      to_message();
                    }
                    if (m_scan_tuda_obratno || m_scan_go_to_end) {
                      //  Показать сообщение
                      create_freq_string(m_scan_direction, &m_freq_forward,
                        m_relay_was_forward_action, mp_message_string);
                      if (m_relay_was_forward_correct)
                        strcat(mp_message_string, "*");
                      strcpy(mp_relay_string_1, mp_message_string);
                      strcpy(mp_relay_string_2, "");
                    }
                  }

                  if (m_scan_status == SCAN_FORWARD) {
                    if (m_scan_direction == UP) m_freq += m_scan_freq_step;
                    else m_freq -= m_scan_freq_step;
                    mp_generator->set_frequency(m_freq);
                  }
                } else {
                  m_freq = m_freq_end;
                  mp_generator->set_frequency(m_freq);
                  if (m_scan_tuda_obratno) {
                    m_scan_status = SCAN_RETURN_SETUP;

                    //  Показать сообщение
                    if (!m_relay_was_forward_action) {
                      create_freq_string(m_scan_direction, &m_freq_forward,
                        m_relay_was_forward_action, mp_message_string);
                      strcpy(mp_relay_string_1, mp_message_string);
                      strcpy(mp_relay_string_2, "");
                    }

                    if (m_freq_begin < m_freq_end) m_scan_direction = DOWN;
                    else m_scan_direction = UP;

                    m_scan_timer.set(m_scan_pause_interval);
                    m_scan_timer.start();
                  }
                  else to_message();
                }
              }
              break;
            }
            case SCAN_RETURN_SETUP: {
              if (m_scan_timer.check()) {
                m_scan_timer.set(m_scan_interval);
                m_scan_timer.start();
                m_relay_was_return_action = false;
                m_relay_was_return_correct = false;
                m_relay_contact_prev = mp_relay_contact_pin->pin();
                m_scan_status = SCAN_RETURN;
              }
              break;
            }
            case SCAN_RETURN: {
              if (m_scan_timer.check()) {
                if (!test_freq_limit(m_freq, m_freq_begin)) {
                  if (m_relay_contact_prev != mp_relay_contact_pin->pin()) {
                    m_relay_contact_prev = mp_relay_contact_pin->pin();
                    m_relay_was_return_action = true;
                    m_freq_return = m_freq;
                    m_buzzer.bzzz();

                    if (m_scan_correct_freq_by_time) {
                      if (m_scan_need_meas_time) {
                        if (m_relay_was_time_return_action) {
                          m_relay_was_return_correct = true;
                          float df = m_time_return * m_speed_freq;
                          if (m_scan_direction == UP) m_freq_return -= df;
                          else m_freq_return += df;
                        }
                      }
                    }

                    if (m_scan_go_to_end) {
                      //  Показать сообщение
                      create_freq_string(m_scan_direction, &m_freq_return,
                        m_relay_was_return_action, mp_message_string);
                      if (m_relay_was_return_correct)
                        strcat(mp_message_string, "*");
                      strcpy(mp_relay_string_2, mp_message_string);
                    }
                    else to_message();
                  }

                  if (m_scan_status == SCAN_RETURN) {
                    if (m_scan_direction == UP) m_freq += m_scan_freq_step;
                    else m_freq -= m_scan_freq_step;
                    mp_generator->set_frequency(m_freq);
                  }
                } else {
                  m_freq = m_freq_begin;
                  mp_generator->set_frequency(m_freq);
                  to_message();
                }
              }
              break;
            }
          }
        }
        break;
      }

      reg();
      break;
    }
    case MESSAGE: {
      m_menu_kb_event.check();
      switch (key) {
        case  m_key_stop:  to_stop();  break;
        case  m_key_pause: to_pause(); break;
      }
      reg();
      break;
    }
  }
  if (m_cfg.overcurrent_dectected()) {
    if (m_overcurrent_detection_enabled) {
      to_stop();
      m_buzzer.bzzz();
    }
  }
}

template <class CFG>
void app_t<CFG>::tick()
{
  m_keyboard_event_gen.tick();
  (*mp_debug_led)();
  m_cfg.tick();
  m_fast_adc_rms.tick();
  m_buzzer.tick();
  bool is_main_srceen_cur = (mp_cur_menu == &m_main_screen);
  if (m_contrast_hold_key.check(is_main_srceen_cur)) {
    m_contrast = 0.05f;
    mp_lcd_contrast_pwm->set_duty(float(m_contrast));
    mp_cur_menu = &m_contrast_item;
  }
  mp_window_watchdog->restart();
  mp_independent_watchdog->restart();
}

template <class CFG>
void app_t<CFG>::out_tick()
{
  if (m_menu_timer.check()) {
    if (m_status != MESSAGE && m_status != SPLASH) {
      mp_cur_menu->draw(&mp_cur_menu);
    }
    put_scan_mode_symbol();
    m_temperature = m_temperature_conn_data.temperature_code*
      m_cfg.get_themperature_conv_koef();
  }

  if (m_filter_update_interval_timer.check()) {
    m_adc_value = fabs(m_fast_adc_rms.get_slow_adc_voltage_code());
    m_voltage = adc_to_voltage(m_adc_value);
    if (m_voltage_filter_on) {
      m_voltage_display = fade(&m_adc_fade_data, m_voltage);
    } else {
      m_voltage_display = m_voltage;
    }

    if (m_voltage < 0.f) {
      m_voltage = 0.f;
    }
    if (m_voltage_display < 0.f) {
      m_voltage_display = 0.f;
    }

    if (m_interface_mode == DEBUG) {
      m_pwm_out = mp_generator->get_amplitude();
    }
  }
}

template <class CFG>
void app_t<CFG>::switch_key(irskey_t a_key)
{
  m_freq_begin_input.process(a_key);
  m_freq_end_input.process(a_key);
  m_speed_freq_input.process(a_key);
  m_voltage_ref_input.process(a_key);

  switch (a_key) {
    case m_key_Fn_up: {
      if (m_status == PAUSE) mp_generator->set_frequency(m_freq);
      break;
    }
    case m_key_Fn_down: {
      if (m_status == PAUSE) mp_generator->set_frequency(m_freq);
      break;
    }
    case m_key_S_up: {
      break;
    }
    case m_key_S_down: {
      break;
    }
    case m_key_V_up: {

      break;
    }
  }
}

template <class CFG>
void app_t<CFG>::put_scan_mode_symbol()
{
  static mxdisp_pos_t symbol_pos = 0;
  static const irs_u8 x = mp_lcd_drv->get_width() - 1;
  static irs::loop_timer_t symbol_timer(irs::make_cnt_ms(500));
  if (m_interface_mode == RELEASE) {
    switch (m_status) {
      case STOP: {
        break;
      }
      case SETUP: {
        break;
      }
      case PAUSE: {
        if (abs(m_freq_begin - m_freq_end) < 0.5f * m_scan_freq_step) return;

        if (!m_scan_tuda_obratno) {
          if (m_scan_need_meas_time) {
            if (m_scan_direction == UP) {
              m_lcd_drv_service.outtextpos(x, 0, "^");
              m_lcd_drv_service.outtextpos(x, 1, "T");
            } else {
              m_lcd_drv_service.outtextpos(x, 0, "T");
              m_lcd_drv_service.outtextpos(x, 1, "~");
            }
          } else {
            if (m_scan_direction == UP) m_lcd_drv_service.outtextpos(x, 0, "^");
            else m_lcd_drv_service.outtextpos(x, 0, "~");
          }
        } else {
          m_lcd_drv_service.outtextpos(x, 0, "^");
          if (m_scan_need_meas_time) {
            m_lcd_drv_service.outtextpos(x, 1, "T");
            m_lcd_drv_service.outtextpos(x, 2, "~");
          } else {
            m_lcd_drv_service.outtextpos(x, 1, "~");
          }
        }
        break;
      }
      case SCAN: {
        if (m_scan_status == SCAN_FORWARD_SETUP ||
            m_scan_status == SCAN_FORWARD ||
            m_scan_status == SCAN_RETURN_SETUP ||
            m_scan_status == SCAN_RETURN) {
          switch (m_scan_direction) {
            case UP: {
              m_lcd_drv_service.outtextpos(x, symbol_pos, "^");
              if (symbol_timer.check()) {
                if (symbol_pos > 0) {
                  symbol_pos--;
                } else if (mp_lcd_drv->get_height() > 0) {
                  symbol_pos = mp_lcd_drv->get_height() - 1;
                }
              }
              break;
            }
            case DOWN: {
              m_lcd_drv_service.outtextpos(x, symbol_pos, "~");
              if (symbol_timer.check()) {
                symbol_pos++;
                if (symbol_pos >= mp_lcd_drv->get_height()) {
                  symbol_pos = 0;
                }
              }
              break;
            }
          }
        }
        else if (m_scan_status == SCAN_FORWARD_JUMP_SETUP ||
          m_scan_status == SCAN_FORWARD_JUMP) {
          switch (m_scan_direction) {
            case UP: {
              m_lcd_drv_service.outtextpos(x, 0, "^");
              m_lcd_drv_service.outtextpos(x, 1, "T");
              break;
            }
            case DOWN: {
              m_lcd_drv_service.outtextpos(x, 0, "T");
              m_lcd_drv_service.outtextpos(x, 1, "~");
              break;
            }
          }
        }
        else if (m_scan_status == SCAN_RETURN_JUMP_SETUP ||
          m_scan_status == SCAN_RETURN_JUMP) {
          switch (m_scan_direction) {
            case UP: {
              m_lcd_drv_service.outtextpos(x, 0, "T");
              m_lcd_drv_service.outtextpos(x, 1, "~");
              break;
            }
            case DOWN: {
              m_lcd_drv_service.outtextpos(x, 0, "^");
              m_lcd_drv_service.outtextpos(x, 1, "T");
              break;
            }
          }
        }
        break;
      }
    }
  } else {
    if (m_can_reg && m_status != STOP) {
      static bool blink = true;
      if (symbol_timer.check()) {
        if (blink) {
          blink = false;
        } else {
          blink = true;
        }
      }
      if (blink) {
        m_lcd_drv_service.outtextpos(19, 3, "R");
      } else {
        m_lcd_drv_service.outtextpos(19, 3, " ");
      }
    } else {
      m_lcd_drv_service.outtextpos(19, 3, " ");
    }
  }
}

template <class CFG>
void app_t<CFG>::reg()
{
  if (m_reg_timer.check()) {
    m_reg_timer.start();
    if (m_can_reg) {
      const double cur_voltage_ref =
        m_voltage_ref;//irs::rate_limiter(&m_rate_data, m_voltage_ref);

      const float e = cur_voltage_ref - irs::isodr(&m_iso_data, m_voltage);
      bool exec_reg = true;
      const double dead_band = get_actual_reg_options()->dead_band;
      if (m_interface_mode == DEBUG) {
        exec_reg = (m_voltage_ref != 0);
      } else {
        exec_reg = (m_voltage_ref != 0) &&
          (abs(e/m_voltage_ref) > (dead_band/100));
      }
      if (exec_reg) {
        const double pid_reg_out = pid_reg(&m_reg_pd, e);
        mp_generator->set_amplitude(pid_reg_out);
      } else {
        pid_reg_sync(&m_reg_pd, e, mp_generator->get_amplitude());
      }
    }
    m_operating_duty = operating_duty();
  }
}

template <class CFG>
void app_t<CFG>::reg_with_meas()
{
  if (m_reg_timer.check()) {
    m_reg_timer.start();
    const bool need_meas_koef = get_actual_reg_options()->need_meas_koef;
    if (!need_meas_koef || m_voltage < m_min_meas_k_voltage) {
      m_meas_k = get_meas_k_default();
    } else {
      m_meas_k = m_reg_pd.prev_out / m_voltage;
    }

    const double k = get_actual_reg_options()->k;
    if (m_reg_pd.k != k * m_meas_k) {
      m_reg_pd.k = k * m_meas_k;
      pid_reg_sync(&m_reg_pd);
    }
    if (m_can_reg) {
      const double cur_voltage_ref =
        m_voltage_ref;//irs::rate_limiter(&m_rate_data, m_voltage_ref);
      const float e = cur_voltage_ref - irs::isodr(&m_iso_data, m_voltage);
      const double pid_reg_out = pid_reg(&m_reg_pd, e);
      mp_generator->set_amplitude(pid_reg_out);
    }

    m_operating_duty = operating_duty();
  }
}
template <class CFG>
bool app_t<CFG>::test_switch_interface_mode(irskey_t a_key)
{
  if (a_key != irskey_none) {
    m_current_interface_mode_key_array[m_current_interface_mode_key] = a_key;
    if (m_current_interface_mode_key_array[m_current_interface_mode_key]
      == m_ref_interface_mode_key_array[m_current_interface_mode_key]) {
      m_current_interface_mode_key++;
      if (m_current_interface_mode_key >= key_array_size) {
        m_current_interface_mode_key = 0;
        return true;
      }
      return false;
    } else {
      m_current_interface_mode_key = 0;
    }
  }
  return false;
}

template <class CFG>
void app_t<CFG>::voltage_filter_on()
{
  m_adc_fade_data.x1 = m_voltage;
  m_adc_fade_data.y1 = m_voltage;
  m_voltage_filter_on = true;
}

template <class CFG>
void app_t<CFG>::voltage_filter_off()
{
  m_voltage_filter_on = false;
}

template <class CFG>
void app_t<CFG>::id(irs_u16 a_id)
{
  m_id = a_id;
  if (m_id == 0) {
    m_id_item.set_can_edit(irs_true);
  } else {
    m_id_item.set_can_edit(irs_false);
  }
}

template <class CFG>
irs_u16 app_t<CFG>::id()
{
  return static_cast<irs_u16>(m_id);
}

template <class CFG>
void app_t<CFG>::reset_nonvolatile_memory()
{
  m_nonvolatile_data.ac_k = 0.55f;
  m_nonvolatile_data.ac_ki = 5.f;
  m_nonvolatile_data.ac_kd = 0.f;
  m_nonvolatile_data.dc_k = 0.5f;
  m_nonvolatile_data.dc_ki = 4.99f;
  m_nonvolatile_data.dc_kd = 0.f;
  m_nonvolatile_data.contrast = 0.15f;
  m_nonvolatile_data.current_limit = 0.8f;
  m_nonvolatile_data.termo_limit = 61.f;
  m_nonvolatile_data.temp_2 = 2.f;
  m_nonvolatile_data.temp_3 = 3.f;
  m_nonvolatile_data.freq_begin = 45.f;
  m_nonvolatile_data.freq_end = 55.f;
  m_nonvolatile_data.speed_freq = 1.f;
  m_nonvolatile_data.dc_voltage_ref = 30.f;
  m_nonvolatile_data.ac_voltage_ref = 30.f;
  m_nonvolatile_data.mode_dc_enabled = false;
  m_nonvolatile_data.go_to_end = true;
  m_nonvolatile_data.need_meas_time = true;
  m_nonvolatile_data.correct_freq_by_time = true;
  m_nonvolatile_data.tuda_obratno = true;
  m_nonvolatile_data.ac_need_meas_koef = false;
  m_nonvolatile_data.dc_need_meas_koef = false;
  m_nonvolatile_data.reset_nonvolatile_memory = false;
  m_nonvolatile_data.reset_correct_nonvolatile_memory = false;
  m_nonvolatile_data.voltage_correct_enable = true;
  m_nonvolatile_data.dc_voltage_correct_enable = true;
  m_nonvolatile_data.dc_voltage_correct_koef = 1.f;
  m_nonvolatile_data.freq_correct_enable = true;
  m_nonvolatile_data.freq_correct_koef = 1.0002f;//16009408.f/16000000.f;
  m_nonvolatile_data.ac_t_adc = 0.1f;
  m_nonvolatile_data.ac_iso_k = 1.f;
  m_nonvolatile_data.ac_iso_t = 1.f;
  m_nonvolatile_data.ac_dead_band = 0.5f;
  m_nonvolatile_data.dc_t_adc = 0.1f;
  m_nonvolatile_data.dc_iso_k = 1.f;
  m_nonvolatile_data.dc_iso_t = 1.f;
  m_nonvolatile_data.dc_dead_band = 0.5f;
  m_nonvolatile_data.id = 0;
}

template <class CFG>
irs_uarc app_t<CFG>::reset_correct_nonvolatile_memory()
{
  char id[4];
  id[0] = 'G';
  id[1] = 'T';
  id[2] = 'C';
  id[3] = 'H';
  m_ac_voltage_correct_map.m_map_id = *(irs_u32*)id;
  m_ac_voltage_correct_map.m_x_count = 2;
  m_ac_voltage_correct_map.m_y_count = 3;

  const irs_uarc correct_size =
    m_ac_voltage_correct_map.connect(mp_acv_correct_nonvolatile_memory,
    m_ac_voltage_correct_start_pos, m_ac_voltage_correct_use_b_koefs);

  m_ac_voltage_correct_map.mp_x_points[0] = 45.00;
  m_ac_voltage_correct_map.mp_x_points[1] = 55.00;
  m_ac_voltage_correct_map.mp_y_points[0] = 0.000;
  m_ac_voltage_correct_map.mp_y_points[1] = 30.00;
  m_ac_voltage_correct_map.mp_y_points[2] = m_max_ac_voltage_release;

  irs_u8 koefs_count =
    m_ac_voltage_correct_map.m_x_count * m_ac_voltage_correct_map.m_y_count;
  for (irs_u8 i = 0; i < koefs_count; i++) {
    m_ac_voltage_correct_map.mp_k_array[i] = 1;
  }
  if (m_ac_voltage_correct_use_b_koefs) {
    for (irs_u8 i = 0; i < koefs_count; i++) {
      m_ac_voltage_correct_map.mp_b_array[i] = 0;
    }
  }
  return correct_size;
}

/*template <class CFG>
void irs_uarc app_t<CFG>::set_precision_voltage_menu_items(size_type a_precision)
{
  m_voltage_ref_item.set_str(mp_user_str, "Uo", "",
    m_volt_item_width, a_precision);
  m_voltage_item.set_str(mp_user_str, "Uн", "В",
    m_volt_item_width, a_precision);
}*/

/*
1. Режим СТОП
  Меняются все параметры, в т.ч. через меню. При нажатии кнопки "старт" проис-
  ходит переход в режим СЕТУП и регулятор начинает выставлять заданное
  напряжение. Частота сигнала равна Fн.
2. Режим СЕТУП
  Никакие параметры изменить нельзя. Реакция - только на кнопку "стоп", при
  нажатии на которую прибор переходит в режим СТОП. При достижении заданного
  напряжения происходит переход в режим ПАУЗА.
3. Режим ПАУЗА
  Меняются все параметры также, как в режиме СТОП, но вход в меню не разрешён.
  При изменении Fн меняется частота выходного сигнала. Регулятор поддерживает
  заданное выходное напряжение и реагирует на смену уставки. При нажатии на
  кнопку "старт" прибор переходит в режим СКАН с заданными параметрами. Если
  Fн = Fк, то никто никуда не переходит.
4. Режим СКАН
  Реакция - только на кнопки "стоп" и "пауза". Во время плавного изменения
  частоты регистрируется положение контактов реле. При первом изменении
  положения текущее значение частоты сохраняется в памяти и отображается на
  экране. Если положение контактов реле изменяется ещё раз при изменении
  частоты в ту же сторону, прибор переходит в режим ПАУЗА и на экране
  показывается сообщение об ошибке. Если нажать "ввод", сканирование продолжится
  далее, в качестве точки срабатывания реле останется второе значение. Если
  нажать "старт" - сканирование продолжится с запоминанием предыдущей точки.
  При достижении частотой сигнала значения Fк произойдет следующее:
    - Если был выбран режим SM_TUDA: будет выведен отчёт со значением
      запомненного значения частоты срабатывания реле. После нажатия клавиши
      "старт" прибор перейдет в режим "пауза".
    - Если был выбран режим SM_TUDA_WITH_JUMP: последует выдержака в 2 с,
      затем частота будет резко изменена на Fн, опять выдержана пауза 2 с, и
      частота будет изменена обратно на Fк. При этом будет измерено время
      срабатывания реле и выведен отчёт со значениями частоты срабатывания и
      времени срабатывания реле.
*/

} //  gtch

#endif  //  gtchH
