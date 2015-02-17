#include <irspch.h>

#include "gtchcfg.h"

#include <irsfinal.h>

gtch::cfg_t::cfg_t():
  m_ir2183_dead_time(500e-9),
  #if GTCH_SK_STM32F217
  m_memory_chip_select_pin(GPIO_PORTD, 7, irs::io_t::dir_out, irs::io_pin_on),
  #endif // GTCH_SK_STM32F217
  #if GTCH_SK_STM32F217
  m_lcd_port(GPIO_PORTE, 0xFF, irs::io_t::dir_open_drain, 8),
  m_lcd_rs_pin(GPIO_PORTF, 11, irs::io_t::dir_open_drain),
  m_lcd_e_pin(GPIO_PORTB, 1, irs::io_t::dir_open_drain),
  #else // !GTCH_SK_STM32F217
  m_lcd_port(GPIO_PORTB, 0xFF, irs::io_t::dir_open_drain, 2),
  m_lcd_rs_pin(GPIO_PORTB, 1, irs::io_t::dir_open_drain),
  m_lcd_e_pin(GPIO_PORTB, 0, irs::io_t::dir_open_drain),
  #endif // !GTCH_SK_STM32F217
  m_lcd_drv(irslcd_4x20, m_lcd_port, m_lcd_rs_pin, m_lcd_e_pin),
  m_key_drv_horizontal_pins(),
  m_key_drv_vertical_pins(),
  m_keyboard_drv(),
  m_adc_settings(3.3, 10e-6, 10, 0.1),
  #if GTCH_SK_STM32F217
  m_adc(ADC3_BASE, irs::arm::st_adc_t::ADC3_PF8_CH6/*ADC1_BASE, irs::arm::st_adc_t::ADC123_PC0_CH10*/,
    irs::make_cnt_s(m_adc_settings.sampling_time)),
  #else // Плата ГТЧ
  /*m_adc(IRS_ADC1_BASE, irs::arm::st_adc_t::ADC123_PA2_CH2,
    irs::make_cnt_s(m_adc_settings.sampling_time)),*/
  m_adc(irs::arm::st_multi_adc_t::ADC123_PA1_CH1,
    irs::arm::st_multi_adc_t::ADC123_PA2_CH2,
    irs::make_cnt_s(m_adc_settings.sampling_time)),
  #endif // Плата ГТЧ
  #if GTCH_SK_STM32F217
  m_debug_led(GPIO_PORTD, 0, irs::make_cnt_ms(1000)),
  m_buzzer_pin(irs::handle_t<irs::pwm_gen_t>(
    new irs::arm::st_pwm_gen_t(PB0, TIM3_BASE, 4000, 0.5))),
  #else // Плата ГТЧ
  m_debug_led(GPIO_PORTC, 10, irs::make_cnt_ms(1000)),
  m_buzzer_pin(irs::handle_t<irs::pwm_gen_t>(
    new irs::arm::st_pwm_gen_t(PB10, IRS_TIM2_BASE, 4000, 0.5))),
  #endif // Плата ГТЧ
  #if GTCH_SK_STM32F217
  m_relay_contact_pin(GPIO_PORTF, 9, irs::io_t::dir_in),
  #else // Плата ГТЧ
  m_relay_contact_pin(GPIO_PORTA, 3, irs::io_t::dir_in),
  #endif // Плата ГТЧ
  #if GTCH_SK_STM32F217
  m_lcd_contrast_pwm(PE6, TIM9_BASE, 4000, 0.5),
  m_sinus_pwm_timer(TIM8_PWM2_BASE),
  m_sinus_pwm(PC6, m_sinus_pwm_timer, 62500, 0.5),
  #else // !GTCH_SK_STM32F217
  m_lcd_contrast_pwm(PB11, IRS_TIM2_BASE, 4000, 0.5),
  m_sinus_pwm_timer(IRS_TIM1_PWM1_BASE),
  m_sinus_pwm(PA8, m_sinus_pwm_timer, 62500, 0.5),
  #endif // !GTCH_SK_STM32F217
  #if GTCH_SK_STM32F217
  m_int_generator(TIM1_PWM1_BASE),
  m_spi(SPI3_I2S3_BASE, spi_bitrate, PC10, PC11, PC12),
  m_eeprom_chip_select_pin(GPIO_PORTD, 2, irs::io_t::dir_out, irs::io_pin_on),
  #else // Плата ГТЧ
  m_int_generator(IRS_TIM8_PWM2_BASE),
  m_spi(IRS_SPI2_I2S2_BASE, spi_bitrate, PB13, PB14, PB15),
  m_eeprom_chip_select_pin(GPIO_PORTB, 12, irs::io_t::dir_out, irs::io_pin_on),
  #endif // Плата ГТЧ
  m_page_mem(&m_spi, &m_eeprom_chip_select_pin, irs::at25128),
  m_mem_data(&m_page_mem),
  m_eeprom_start_position(0),
  m_eeprom_size(200),
  m_eeprom(&m_mem_data, m_eeprom_start_position, m_eeprom_size),
  m_eeprom_correct_start_position(1500),
  m_eeprom_correct_size(400),
  m_eeprom_correct(&m_mem_data, m_eeprom_correct_start_position,
    m_eeprom_correct_size),
  #if GTCH_SK_STM32F217
  m_current_limit_code(irs::arm::st_dac_t::DAC_PA4_CH0),
  #else // Плата ГТЧ
  m_current_limit_code(irs::arm::st_dac_t::DAC_PA5_CH1),
  #endif // Плата ГТЧ
  m_relay_interrupt_generator(),
  //  Термодатчик
  m_termo_interval(irs::make_cnt_ms(300)),
  #if GTCH_SK_STM32F217
  m_termometr_chip_select_pin(GPIO_PORTG, 10, irs::io_t::dir_out,
    irs::io_pin_on),
  #else // Плата ГТЧ
  m_termometr_chip_select_pin(GPIO_PORTC, 6, irs::io_t::dir_out,
    irs::io_pin_on),
  #endif // Плата ГТЧ
  m_termometr(&m_spi, &m_termometr_chip_select_pin, m_termo_interval),
  m_window_watchdog(0, 69e-3),
  m_independent_watchdog((69e-3)*4)
{
  #if GTCH_SK_STM32F217
  m_sinus_pwm.complementary_channel_enable(PA5);
  #else // !GTCH_SK_STM32F217
  m_sinus_pwm.complementary_channel_enable(PA7);
  #endif // !GTCH_SK_STM32F217
  m_sinus_pwm.break_enable(PA6, irs::arm::break_polarity_active_low);

  #if GTCH_SK_STM32F217
  m_key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTD, 10, irs::io_t::dir_in)));
  m_key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTC, 7, irs::io_t::dir_in)));
  m_key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTD, 15, irs::io_t::dir_in)));
  m_key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTB, 2, irs::io_t::dir_open_drain)));
  m_key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTB, 10, irs::io_t::dir_open_drain)));
  m_key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTD, 9, irs::io_t::dir_open_drain)));
  m_key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTD, 8, irs::io_t::dir_open_drain)));
  #else // Плата ГТЧ
  m_key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTC, 13, irs::io_t::dir_in)));
  m_key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTC, 15, irs::io_t::dir_in)));
  m_key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTC, 14, irs::io_t::dir_in)));
  m_key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTC, 0, irs::io_t::dir_open_drain)));
  m_key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTC, 1, irs::io_t::dir_open_drain)));
  m_key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTC, 2, irs::io_t::dir_open_drain)));
  m_key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTC, 3, irs::io_t::dir_open_drain)));
  #endif // Плата ГТЧ

  m_keyboard_drv.add_horizontal_pins(&m_key_drv_horizontal_pins);
  m_keyboard_drv.add_vertical_pins(&m_key_drv_vertical_pins);

  m_keyboard_drv.add_key(irskey_7);
  m_keyboard_drv.add_key(irskey_8);
  m_keyboard_drv.add_key(irskey_9);
  m_keyboard_drv.add_key(irskey_up );
  m_keyboard_drv.add_key(irskey_4);
  m_keyboard_drv.add_key(irskey_5);
  m_keyboard_drv.add_key(irskey_6);
  m_keyboard_drv.add_key(irskey_down);
  m_keyboard_drv.add_key(irskey_1);
  m_keyboard_drv.add_key(irskey_2);
  m_keyboard_drv.add_key(irskey_escape);
  m_keyboard_drv.add_key(irskey_enter);

  init_to_cnt();
}

gtch::cfg_t::~cfg_t()
{
  deinit_to_cnt();
}

irs::blink_t* gtch::cfg_t::get_debug_led()
{
  return &m_debug_led;
}

irs::gpio_pin_t* gtch::cfg_t::get_buzzer_pin()
{
  return &m_buzzer_pin;
}

irs::gpio_pin_t* gtch::cfg_t::get_relay_contact_pin()
{
  return &m_relay_contact_pin;
}

irs::pwm_gen_t* gtch::cfg_t::get_lcd_contrast_pwm()
{
  return &m_lcd_contrast_pwm;
}

irs::pwm_gen_t& gtch::cfg_t::get_sinus_pwm()
{
  return m_sinus_pwm;
}

gtch::cfg_t::int_generator_t& gtch::cfg_t::get_int_generator()
{
  return m_int_generator;
}

mxdisplay_drv_t* gtch::cfg_t::get_lcd_drv()
{
  return &m_lcd_drv;
}

mxkey_drv_t& gtch::cfg_t::get_keyboard_drv()
{
  return m_keyboard_drv;
}

const gtch::adc_settings_t& gtch::cfg_t::get_adc_settings() const
{
  return m_adc_settings;
}

irs::adc_t* gtch::cfg_t::get_adc()
{
  return &m_adc;
}

bool gtch::cfg_t::nonvolatile_memory_error()
{
  return m_mem_data.error();
}

bool gtch::cfg_t::nonvolatile_memory_partial_error()
{
  return false;
}

irs::mxdata_t* gtch::cfg_t::get_nonvolatile_memory()
{
  return &m_eeprom;
}

irs::mxdata_t* gtch::cfg_t::get_correct_nonvolatile_memory()
{
  return &m_eeprom_correct;
}

irs::dac_t* gtch::cfg_t::get_current_limit_code()
{
  return &m_current_limit_code;
}

gtch::cfg_t::relay_interrupt_generator_t*
gtch::cfg_t::get_relay_interrupt_generator()
{
  return &m_relay_interrupt_generator;
}

irs::mxdata_t& gtch::cfg_t::get_th_data()
{
  return m_termometr;
}

irs::watchdog_t* gtch::cfg_t::get_window_watchdog()
{
  return &m_window_watchdog;
}

irs::watchdog_t* gtch::cfg_t::get_independent_watchdog()
{
  return &m_independent_watchdog;
}

bool gtch::cfg_t::overcurrent_dectected()
{
  tim_regs_t* tim = reinterpret_cast<tim_regs_t*>(m_sinus_pwm_timer);
  if (tim->TIM_SR_bit.BIF == 1) {
    tim->TIM_SR_bit.BIF = 0;
    return true;
  } else {
    return false;
  }
}

float gtch::cfg_t::get_themperature_conv_koef()
{
  return m_termometr.get_conv_koef();
}
void gtch::cfg_t::tick()
{
  m_page_mem.tick();
  m_mem_data.tick();
  m_eeprom.tick();
  m_eeprom_correct.tick();
  m_lcd_drv.tick();
  m_termometr.tick();
  //m_adc.tick();
}
