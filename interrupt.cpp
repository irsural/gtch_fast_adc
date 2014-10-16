#include <irsdefs.h>

#include <irsconfig.h>
#include <armioregs.h>

#include "gtch.h"
#include "interrupt.h"

#include <irsfinal.h>

#ifdef NOP

// Возвращает массив прерываний ARM
irs::interrupt_array_base_t* gtch::interrupt_array()
{
  enum
  {
    default_interrupt_count = 5
  };
  static auto_ptr<interrupt_array_t>
    p_interrupt_array(new interrupt_array_t(interrupt_count,
    default_interrupt_count));
  return p_interrupt_array.get();
}

extern "C"
{
  void NMI_Handler( void );
  void HardFault_Handler( void );
  void MemManage_Handler( void );
  void BusFault_Handler( void );
  void UsageFault_Handler( void );
  void SVC_Handler( void );
  void DebugMon_Handler( void );
  void PendSV_Handler( void );
  void SysTick_Handler( void );
}

//  Прерывания ядра
void NMI_Handler()
{
}

void HardFault_Handler()
{
}
void MemManage_Handler()
{
}
void BusFault_Handler()
{
}
void UsageFault_Handler()
{
}
void SVC_Handler()
{
}
void DebugMon_Handler()
{
}
void PendSV_Handler()
{
}
void SysTick_Handler()
{
}
//  Прерывания периферии
void irs_arm_tim3_func()
{
}

void irs_arm_default_int_func()
{
  while (true);
}

typedef void( *intfunc )( void );

#pragma location = ".periph_intvec"
__root const intfunc __int_vector_table[] =
{
  //irs_arm_default_int_func,  // -1
  irs_arm_default_int_func,  // 0
  irs_arm_default_int_func,  // 1
  irs_arm_default_int_func,  // 2
  irs_arm_default_int_func,  // 3
  irs_arm_default_int_func,  // 4
  irs_arm_default_int_func,  // 5
  irs_arm_default_int_func,  // 6
  irs_arm_default_int_func,  // 7
  irs_arm_default_int_func,  // 8
  irs_arm_default_int_func,  // 9
  irs_arm_default_int_func,  // 10
  irs_arm_default_int_func,  // 11
  irs_arm_default_int_func,  // 12
  irs_arm_default_int_func,  // 13
  irs_arm_default_int_func,  // 14
  irs_arm_default_int_func,  // 15
  irs_arm_default_int_func,  // 16
  irs_arm_default_int_func,  // 17
  irs_arm_default_int_func,  // 18
  irs_arm_default_int_func,  // 19
  irs_arm_default_int_func,  // 20
  irs_arm_default_int_func,  // 21
  irs_arm_default_int_func,  // 22
  irs_arm_default_int_func,  // 23
  irs_arm_default_int_func,  // 24
  irs_arm_default_int_func,  // 25
  irs_arm_default_int_func,  // 26
  irs_arm_default_int_func,  // 27
  irs_arm_default_int_func,  // 28
  irs_arm_tim3_func,         // 29
  irs_arm_default_int_func,  // 30
  irs_arm_default_int_func,  // 31
  irs_arm_default_int_func,  // 32
  irs_arm_default_int_func,  // 33
  irs_arm_default_int_func,  // 34
  irs_arm_default_int_func,  // 35
  irs_arm_default_int_func,  // 36
  irs_arm_default_int_func,  // 37
  irs_arm_default_int_func,  // 38
  irs_arm_default_int_func,  // 39
  irs_arm_default_int_func,  // 40
  irs_arm_default_int_func,  // 41
  irs_arm_default_int_func,  // 42
  irs_arm_default_int_func,  // 43
  tim8_up_tim13_int,         // 44
  irs_arm_default_int_func,  // 45
  irs_arm_default_int_func,  // 46
  irs_arm_default_int_func,  // 47
  irs_arm_default_int_func,  // 48
  irs_arm_default_int_func,  // 49
  irs_arm_default_int_func,  // 50
  irs_arm_default_int_func,  // 51
  irs_arm_default_int_func,  // 52
  irs_arm_default_int_func,  // 53
  irs_arm_default_int_func,  // 54
  irs_arm_default_int_func,  // 55
  irs_arm_default_int_func,  // 56
  irs_arm_default_int_func,  // 57
  irs_arm_default_int_func,  // 58
  irs_arm_default_int_func,  // 59
  irs_arm_default_int_func,  // 60
  irs_arm_default_int_func,  // 61
  irs_arm_default_int_func,  // 62
  irs_arm_default_int_func,  // 63
  irs_arm_default_int_func,  // 64
  irs_arm_default_int_func,  // 65
  irs_arm_default_int_func,  // 66
  irs_arm_default_int_func,  // 67
  irs_arm_default_int_func,  // 68
  irs_arm_default_int_func,  // 69
  irs_arm_default_int_func,  // 70
  irs_arm_default_int_func,  // 71
  irs_arm_default_int_func,  // 72
  irs_arm_default_int_func,  // 73
  irs_arm_default_int_func,  // 74
  irs_arm_default_int_func,  // 75
  irs_arm_default_int_func,  // 76
  irs_arm_default_int_func,  // 77
  irs_arm_default_int_func,  // 78
  irs_arm_default_int_func,  // 79
  irs_arm_default_int_func   // 80
};

#endif // NOP
