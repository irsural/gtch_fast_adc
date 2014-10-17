#ifndef interrupt_generatorH
#define interrupt_generatorH

#include <irsdefs.h>

#include <irsadc.h>
#include <irsalg.h>
#include <irsdsp.h>
#include <armspi.h>
#include <irsint.h>

#include <irsfinal.h>

class interrupt_generator_t
{
public:
  typedef irs_u32 counter_type;
  typedef irs_u32 calc_type;
  interrupt_generator_t(size_t a_timer_address);
  ~interrupt_generator_t();
  void add_event(irs::event_t* ap_event);
  void erase_event(irs::event_t* ap_event);
  void set_interval(counter_type a_interval);
  void start();
  void stop();
  void get_converting_koefs(calc_type *ap_num, calc_type *ap_denom) const;
private:
  enum { freq_50_hz = 50 };
  void apply_koefs();
  tim_regs_t* mp_tim;
  bool m_started;
};

#endif // interrupt_generatorH
