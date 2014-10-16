#ifndef DEFSH
#define DEFSH

#include <irsdefs.h>

#include <irserror.h>
#include <irsdbgutil.h>

#include <irsfinal.h>

#define GTCH_DEBUG
#define GTCH_SK_STM32F217 0
#ifdef GTCH_DEBUG

#define GTCH_DBG_MSG(msg) IRS_LIB_DBG_RAW_MSG(msg)
#define GTCH_MEMCHECK()\
  {\
    static const size_t heap_size = 1400;\
    static const size_t call_stack_size = 900;\
    static const size_t return_stack_size = 50;\
    irs::avr::memcheck<heap_size, call_stack_size, return_stack_size>(\
      &irs::mlog());\
  }
#else //GTCH_DEBUG
#define GTCH_DBG_MSG(msg)
#define GTCH_MEMCHECK()
#endif //GTCH_DEBUG


#endif //DEFSH
