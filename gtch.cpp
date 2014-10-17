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
