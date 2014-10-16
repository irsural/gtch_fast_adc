// 03.09.2014 �������������: gtch_rev = 32
enum { gtch_rev = 32, mxsrclib_rev = 1088, extern_libs_rev = 3 };
//enum { gtch_rev = 31, mxsrclib_rev = 1088, extern_libs_rev = 3 };
// 01.04.2011 �������������
// - ������� ��� � �������� �� 2011
// - �������� ����� ���������� � EEPROM (��������� ��������� � ���������)
// - ���������� ������ ������ � ������������� ��� ������������� ������������
// - �������� ������ EEPROM ��������� 400 ���� ������� 100
// - ��������� ������ ������ ��������� �� ���������� � ������� �����
// - ���� � ���� ������������ � ����� ��������� ���������
// - ��������� �� _pos-���������� � ��������� 28 ���� ���
//enum { gtch_rev = 16, mxsrclib_rev = 737 };
//enum { gtch_rev = 14, mxsrclib_rev = 624 };
//enum { gtch_rev = 12, mxsrclib_rev = 624 };

#include <irspch.h>

#include <armioregs.h>

#include <irserror.h>
#include <irsstrm.h>
#include <irsdbgutil.h>
#include <timer.h>
#include <armadc.h>

#include "defs.h"
#include "gtch.h"
#include <armcfg.h>

#include <irsfinal.h>

// ESC STOP STOP ESC STOP - ���� � ���� � ����� �� ���� ������������
// ����������� STOP 5 ������ - ����������� ������������
// "F� �����" - ����� ��������� �� ���������� � ������� �����
// ������ ������ �������������� - 74

#ifdef GTCH_DEBUG
#define SHOW_TICK_COUNT_ENABLED 0
#else // !GTCH_DEBUG
#define SHOW_TICK_COUNT_ENABLED 0
#endif // !GTCH_DEBUG

class interrupt_event_t: public mxfact_event_t
{
public:
  interrupt_event_t():
    m_measure(),
    m_counter(0),
    m_first(0),
    m_second(0)
  {
  }
  virtual void exec()
  {
    m_counter = TIM8_CNT;
    mxfact_event_t::exec();
    m_second = m_first;
    m_first = counter_get();
  }
  irs_u32 get_counter()
  {
    return m_counter;
  }
  double get_delta()
  {
    return CNT_TO_DBLTIME(m_first - m_second);
  }
private:
  irs::measure_time_t m_measure;
  irs_u32 m_counter;
  counter_t m_first;
  counter_t m_second;
};

int main()
{
  pll_on();
  static irs::arm::com_buf log_buf(1, 10, 1000000);
  irs::mlog().rdbuf(&log_buf);

  #ifdef GTCH_DEBUG

  #if GTCH_SK_STM32F217
  irs::arm::io_pin_t green_led(GPIO_PORTD, 1, irs::io_t::dir_out);
  #endif // GTCH_SK_STM32F217

  irs::loop_timer_t timer(irs::make_cnt_ms(500));

  #if GTCH_SK_STM32F217
  static irs::mc_error_handler_t error_handler(GPIO_PORTD, 0, &irs::mlog());
  #else // ����� ���
  static irs::mc_error_handler_t error_handler(GPIO_PORTC, 10, &irs::mlog());
  #endif // ����� ���

  #endif // GTCH_DEBUG

  irs::mlog() << irsm("\n------------- START 70 -------------") << endl;
  irs::mlog() << irsm("gtch rev. ") << gtch_rev << irsm(", ");
  irs::mlog() << irsm("mxsrclib rev. ") << mxsrclib_rev << endl;
  #if GTCH_SK_STM32F217
  irs::arm::io_pin_t m_memory_chip_select_pin(GPIO_PORTD, 7, irs::io_t::dir_out,
    irs::io_pin_on);
  irs::arm::io_pin_t m_termometr_chip_select_pin(GPIO_PORTG, 10,
    irs::io_t::dir_out, irs::io_pin_on);
  irs::arm::io_pin_t m_eeprom_chip_select_pin(GPIO_PORTD, 2, irs::io_t::dir_out,
    irs::io_pin_on);
  #endif // GTCH_SK_STM32F217

  static gtch::cfg_t cfg;
  static gtch::app_t<gtch::cfg_t> app(cfg, gtch_rev);
  while (true)
  {
    #if SHOW_TICK_COUNT_ENABLED
    static const counter_t period = irs::make_cnt_s(5);
    static irs::measure_time_t period_update_measure_time;
    static double tick_time_max_period = 0;
    static double tick_time_min_period = numeric_limits<irs_u32>::max();
    static double tick_time_max = 0;
    static double tick_time_min = numeric_limits<irs_u32>::max();
    static int count = 0;
    irs::measure_time_t tick_measure_time;
    tick_measure_time.start();
    #endif // SHOW_TICK_COUNT_ENABLED

    #if GTCH_SK_STM32F217 && defined(GTCH_DEBUG)
    if (timer.check()) {
      if (green_led.pin()) {
        green_led.clear();
      } else {
        green_led.set();
      }
    }
    #endif // GTCH_SK_STM32F217 && defined(GTCH_DEBUG)

    app.in_tick();
    app.tick();
    app.out_tick();

    #if SHOW_TICK_COUNT_ENABLED
    count++;
    tick_time_max_period = max (tick_time_max_period, tick_measure_time.get());
    tick_time_max = max(tick_time_max, tick_time_max_period);
    tick_time_min_period = min(tick_time_min_period, tick_measure_time.get());
    tick_time_min = min(tick_time_min, tick_time_min_period);
    if (period_update_measure_time.get_cnt() >= period) {
      const int cell_count = 5;
      const int cell_width = 20;
      IRS_DBG_RAW_MSG(scientific << setprecision(5) <<
        setw((cell_width + 3)*cell_count + 1) << setfill('-') << "-" <<
        setfill(' ') << endl <<
        "| " << setw(cell_width) << "����. ��. �� ������" <<
        " | " << setw(cell_width) << "���. ��. �� ������" <<
        " | " << setw(cell_width) << "����. ��." <<
        " | " << setw(cell_width) << "���. ��." <<
        " | " << setw(cell_width) << "���������� �����" <<
        " |" << endl <<
        "| " << setw(cell_width) << tick_time_max_period <<
        " | " << setw(cell_width) << tick_time_min_period <<
        " | " << setw(cell_width) << tick_time_max <<
        " | " << setw(cell_width) << tick_time_min <<
        " | " << setw(cell_width) << count/period_update_measure_time.get() <<
        " |" << endl <<
        setw((cell_width + 3)*cell_count + 1) << setfill('-') << "-" <<
        setfill(' ') << endl);
      tick_time_max_period = 0;
      tick_time_min_period = numeric_limits<irs_u32>::max();
      count = 0;
      period_update_measure_time.start();
    }
    #endif // SHOW_TICK_COUNT_ENABLED
  }
}
