#ifndef gtchadcH
#define gtchadcH

#include <timer.h>
#include <irsstd.h>
#include <irsadcabs.h>
#include "defs.h"
namespace gtch {

class adc_filter_t
{
public:
  typedef size_t size_type;
  typedef irs_u32 adc_data_t;
  typedef irs_i32 shift_type;
public:
  adc_filter_t(irs::adc_t* ap_adc, size_type a_channel,
    float a_sampling_time = 0.01, size_type a_num_of_points = 10,
    shift_type a_shift = 0);
  //~adc_filter_t();
  adc_data_t get_value();
  void restart();
  void tick();
private:
  irs::adc_t* mp_adc;
  irs_u8 m_channel;
  bool m_continious;
  irs::loop_timer_t m_sample_timer;
  vector<adc_data_t> m_samples;
  size_type m_current_point;
  shift_type m_shift;
  bool m_filled;
};

} // namespace gtch

#endif  //  gtchadcH
