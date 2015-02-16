#include <irsdefs.h>

#include "gtch.h"

#include <irsfinal.h>

gtch::buzzer_t::buzzer_t(irs::gpio_pin_t* ap_buzzer_pin):
  m_bzz_interval(irs::make_cnt_ms(10)),
  m_bzzz_interval(irs::make_cnt_ms(500)),
  m_buzzed(false),
  m_timer(),
  mp_pin(ap_buzzer_pin)
{
  mp_pin->clear();
  m_timer.stop();
}

gtch::buzzer_t::~buzzer_t()
{
  mp_pin->clear();
}

void gtch::buzzer_t::bzzz()
{
  m_timer.set(m_bzzz_interval);
  m_timer.start();
  m_buzzed = true;
  mp_pin->set();
}

void gtch::buzzer_t::bzz()
{
  m_timer.set(m_bzz_interval);
  m_timer.start();
  m_buzzed = true;
  mp_pin->set();
}

void gtch::buzzer_t::tick()
{
  if (m_timer.check())
  {
    m_timer.stop();
    m_buzzed = false;
    mp_pin->clear();
  }
}

#ifdef FLASH_SINUS
gtch::ref_sinus_sample_ponter_t gtch::get_ref_sinus()
{
  // Массив синуса, длинной 256 точек
  static ref_sinus_sample_t ref_sinus[num_of_points] = {
    0, 785, 1570, 2354, 3137, 3917, 4695, 5471, 6243, 7011, 7775, 8535,
    9289, 10038, 10780, 11517, 12246, 12968, 13682, 14388, 15085, 15773,
    16451, 17120, 17778, 18426, 19062, 19687, 20301, 20902, 21490, 22065,
    22627, 23176, 23710, 24231, 24736, 25227, 25703, 26163, 26607, 27035,
    27447, 27843, 28221, 28583, 28928, 29255, 29564, 29856, 30129, 30385,
    30622, 30841, 31041, 31222, 31385, 31529, 31654, 31759, 31846, 31913,
    31961, 31990, 32000, 31990, 31961, 31913, 31846, 31759, 31654, 31529,
    31385, 31222, 31041, 30841, 30622, 30385, 30129, 29856, 29564, 29255,
    28928, 28583, 28221, 27843, 27447, 27035, 26607, 26163, 25703, 25227,
    24736, 24231, 23710, 23176, 22627, 22065, 21490, 20902, 20301, 19687,
    19062, 18426, 17778, 17120, 16451, 15773, 15085, 14388, 13682, 12968,
    12246, 11517, 10780, 10038, 9289, 8535, 7775, 7011, 6243, 5471, 4695,
    3917, 3137, 2354, 1570, 785, 0, -785, -1570, -2354, -3137, -3917,
    -4695, -5471, -6243, -7011, -7775, -8535, -9289, -10038, -10780,
    -11517, -12246, -12968, -13682, -14388, -15085, -15773, -16451,
    -17120, -17778, -18426, -19062, -19687, -20301, -20902, -21490,
    -22065, -22627, -23176, -23710, -24231, -24736, -25227, -25703,
    -26163, -26607, -27035, -27447, -27843, -28221, -28583, -28928,
    -29255, -29564, -29856, -30129, -30385, -30622, -30841, -31041,
    -31222, -31385, -31529, -31654, -31759, -31846, -31913, -31961,
    -31990, -32000, -31990, -31961, -31913, -31846, -31759, -31654,
    -31529, -31385, -31222, -31041, -30841, -30622, -30385, -30129,
    -29856, -29564, -29255, -28928, -28583, -28221, -27843, -27447,
    -27035, -26607, -26163, -25703, -25227, -24736, -24231, -23710,
    -23176, -22627, -22065, -21490, -20902, -20301, -19687, -19062,
    -18426, -17778, -17120, -16451, -15773, -15085, -14388, -13682,
    -12968, -12246, -11517, -10780, -10038, -9289, -8535, -7775, -7011,
    -6243, -5471, -4695, -3917, -3137, -2354, -1570, -785
  };
  return ref_sinus;
}
#endif //FLASH_SINUS
#ifdef NOP
size_t gtch::shift_for_left_aligment_irs_i16(const irs_i16 a_value)
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
#endif // NOP
irs::string_t gtch::align_center(const irs::string_t a_str,
  const size_t a_width, const irs::char_t a_fill)
{
  irs::string_t str(a_width, a_fill);
  if (a_str.size() == a_width) {
    str = a_str;
  } else if (a_str.size() > a_width) {
    const size_t shift = (a_str.size() - a_width)/2;
    for (size_t i = 0; i < a_width; i++) {
      str[i] = a_str[i + shift];
    }
  } else {
    const size_t shift = (a_width - a_str.size())/2;
    for (size_t i = shift; i < (a_width - shift); i++) {
      str[i] = a_str[i - shift];
    }
  }
  return str;
}

// reg_options_t
gtch::reg_options_t::reg_options_t(char* ap_user_str, char* ap_exit_msg,
  size_type a_menu_item_width, size_type a_menu_item_prec
):
  k(0.f),
  k_item(&k, CAN_WRITE),
  //  Кi
  ki(0.f),
  ki_item(&ki, CAN_WRITE),
  //  Кd
  kd(0.f),
  kd_item(&kd, CAN_WRITE),
  //
  can_edit(true),
  //  Переизмерение коэффициентов
  need_meas_koef(false),
  need_meas_koef_item((irs_bool*)&need_meas_koef, can_edit),
  //  Постоянная фильтра АЦП
  t_adc(0.),
  t_adc_item(&t_adc, CAN_WRITE),
  voltage_filter_on(false),
  //  Изодромное звено: К
  iso_k(0.),
  iso_k_item(&iso_k, CAN_WRITE),
  //  Изодромное звено: Т
  iso_t(0.),
  iso_t_item(&iso_t, CAN_WRITE),
  // Зона нечувствительности
  dead_band(0.),
  dead_band_item(&dead_band, CAN_WRITE)
{
  //  К
  k_item.set_header("Проп. коэф.");
  k_item.set_message(ap_exit_msg);
  k_item.set_str(ap_user_str, "Кп", "",
    a_menu_item_width, a_menu_item_prec);
  k_item.set_max_value(1000.0);
  k_item.set_min_value(0.0);
  //k_item.add_change_event(&m_trans_k_nonv_event);
  k_item.set_key_type(IMK_ARROWS);
  k_item.set_change_step(0.01f);
  k_item.set_change_step_max(100.f);
  k_item.progressive_change_enabled(true);
  //  Кi
  ki_item.set_header("Инт. коэф.");
  ki_item.set_message(ap_exit_msg);
  ki_item.set_str(ap_user_str, "Кi", "c-1",
    a_menu_item_width, a_menu_item_prec);
  ki_item.set_max_value(1000.0);
  ki_item.set_min_value(0.0);
  //ki_item.add_change_event(&m_trans_ki_event);
  //ki_item.add_change_event(&m_trans_ki_nonv_event);
  ki_item.set_key_type(IMK_ARROWS);
  ki_item.set_change_step(0.01f);
  ki_item.set_change_step_max(100.f);
  ki_item.progressive_change_enabled(true);
  //  Кd
  kd_item.set_header("Диф. коэф");
  kd_item.set_message(ap_exit_msg);
  kd_item.set_str(ap_user_str, "Кd", "c",
    a_menu_item_width, a_menu_item_prec);
  kd_item.set_max_value(1000.0);
  kd_item.set_min_value(0.0);
  //kd_item.add_change_event(&m_trans_kd_event);
  //kd_item.add_change_event(&m_trans_kd_nonv_event);
  kd_item.set_key_type(IMK_ARROWS);
  kd_item.set_change_step(0.01f);
  kd_item.set_change_step_max(100.f);
  kd_item.progressive_change_enabled(true);
  //  Переизмерение коэффициентов
  need_meas_koef_item.set_header("Изм. коэф.");
  need_meas_koef_item.set_str("Включено", "Выключено");
  //need_meas_koef_item.add_change_event(&m_trans_need_meas_koef_nonv_event);
  //  Постоянная фильтра АЦП
  t_adc_item.set_header("Пост. фильтра напр.");
  t_adc_item.set_message(ap_exit_msg);
  t_adc_item.set_str(ap_user_str, "T ", "с",
    a_menu_item_width, a_menu_item_prec);
  t_adc_item.set_max_value(20.0);
  t_adc_item.set_min_value(0.0);
  t_adc_item.set_key_type(IMK_ARROWS);
  t_adc_item.set_change_step(0.01f);
  t_adc_item.set_change_step_max(1.f);
  t_adc_item.progressive_change_enabled(true);
  //t_adc_item.add_change_event(&m_trans_t_adc_event);
  //t_adc_item.add_change_event(&m_trans_t_adc_nonv_event);

  //  Изодромное звено: К
  iso_k_item.set_header("Изодр. коэф.");
  iso_k_item.set_message(ap_exit_msg);
  iso_k_item.set_str(ap_user_str, "Киз", "",
    a_menu_item_width, a_menu_item_prec);
  iso_k_item.set_max_value(1000.0);
  iso_k_item.set_min_value(0.);
  iso_k_item.set_key_type(IMK_ARROWS);
  iso_k_item.set_change_step(0.01f);
  iso_k_item.set_change_step_max(100.f);
  iso_k_item.progressive_change_enabled(true);
  //iso_k_item.add_change_event(&m_trans_iso_k_event);
  //iso_k_item.add_change_event(&m_trans_iso_k_nonv_event);
  //  Изодромное звено: Т
  iso_t_item.set_header("Изодр. пост. вр.");
  iso_t_item.set_message(ap_exit_msg);
  iso_t_item.set_str(ap_user_str, "Тиз", "с",
    a_menu_item_width, a_menu_item_prec);
  iso_t_item.set_max_value(1000.0);
  iso_t_item.set_min_value(0.0);
  iso_t_item.set_key_type(IMK_ARROWS);
  iso_t_item.set_change_step(0.01f);
  iso_t_item.set_change_step_max(100.f);
  iso_t_item.progressive_change_enabled(true);
  //iso_t_item.add_change_event(&m_trans_iso_t_event);
  //iso_t_item.add_change_event(&m_trans_iso_t_nonv_event);
  // Зона нечувствительности
  dead_band_item.set_header("Мертвая зона");
  dead_band_item.set_message(ap_exit_msg);
  dead_band_item.set_str(ap_user_str, "V", "%",
    a_menu_item_width, a_menu_item_prec);
  dead_band_item.set_max_value(10.0);
  dead_band_item.set_min_value(0.0);
  dead_band_item.set_key_type(IMK_ARROWS);
  dead_band_item.set_change_step(0.01f);
  dead_band_item.set_change_step_max(1.f);
  dead_band_item.progressive_change_enabled(true);
  //dead_band_item.add_change_event(&m_dead_band_nonv_event);
}


