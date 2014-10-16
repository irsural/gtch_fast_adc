#ifndef generatorH
#define generatorH

#include <irsdefs.h>

#include <irsstd.h>

#include "defs.h"

#include <irsfinal.h>

class interrupt_generator_t
{
public:
  typedef irs_u16 counter_type;
  typedef irs_u32 calc_type;
  interrupt_generator_t(size_t a_timer_address);
  ~interrupt_generator_t();
  void add_event(mxfact_event_t *ap_event);
  void set_interval(counter_type a_interval);
  void start();
  void stop();
  void get_converting_koefs(calc_type *ap_num, calc_type *ap_denom) const;
private:
  enum { freq_50_hz = 50 };
  enum { sin_period_point_count = 256 };
  void apply_koefs();
  tim_regs_t* mp_tim;
  bool m_started;
};

#endif  //  generatorH
