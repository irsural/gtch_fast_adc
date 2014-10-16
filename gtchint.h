#ifndef gtchintH
#define gtchintH

#include <irsdefs.h>

#include <timer.h>
#include <irsstd.h>

#include "defs.h"

#include <irsfinal.h>

namespace gtch {

class pa3_relay_interrupt_generator_t
{
public:
  pa3_relay_interrupt_generator_t();
  ~pa3_relay_interrupt_generator_t();
  void add_event(mxfact_event_t *ap_event);
  void start();
  void stop();
  bool stopped();
private:
};

} // namespace gtch

#endif  //  gtchintH
