#include <irspch.h>

#include "cfg.h"

#include <irsfinal.h>

hrm::cfg_t::cfg_t():
  m_local_mac(irs::make_mxmac(0, 0, IP_0, IP_1, IP_2, IP_3)),
  m_config(),
  m_arm_eth(1500, m_local_mac, m_config),
  m_local_ip(mxip_t::zero_ip()),
  m_local_port(5005),
  m_dest_ip(irs::make_mxip(IP_0, IP_1, IP_2, IP_3)),
  m_dest_port(5005),
  m_tcpip(&m_arm_eth, m_local_ip, m_dest_ip, 10),
  simple_hardflow(&m_tcpip, m_local_ip, m_local_port,
    m_dest_ip, m_dest_port, 10),

  m_spi_bitrate(1000),
  spi(IRS_SPI1_BASE, m_spi_bitrate, PA5, PA6, PB5,
    irs::arm::arm_spi_t::gpio_speed_25mhz),
  spi_th(IRS_SPI3_I2S3_BASE, m_spi_bitrate, PC10, PC11, PC12),

  vben(GPIO_PORTG, 4, irs::io_t::dir_out, irs::io_pin_on),
  ee_cs(GPIO_PORTC, 13, irs::io_t::dir_out, irs::io_pin_on),
  lcd_port(GPIO_PORTE, 0xFF, irs::io_t::dir_open_drain, 0),
  lcd_rs_pin(GPIO_PORTD, 14, irs::io_t::dir_open_drain),
  lcd_e_pin(GPIO_PORTD, 15, irs::io_t::dir_open_drain),
  key_drv_horizontal_pins(),
  key_drv_vertical_pins(),
  encoder_gpio_channel_1(PC6),
  encoder_gpio_channel_2(PC7),
  encoder_timer_address(IRS_TIM3_BASE),
  key_encoder(GPIO_PORTD, 10, irs::io_t::dir_in),

  adc_cs(GPIO_PORTG, 7, irs::io_t::dir_out, irs::io_pin_on),
  dac_cs(GPIO_PORTG, 8, irs::io_t::dir_out, irs::io_pin_on),
  dac_ldac(GPIO_PORTG, 6, irs::io_t::dir_out, irs::io_pin_off),
  dac_clr(GPIO_PORTF, 8, irs::io_t::dir_out, irs::io_pin_off),
  dac_reset(GPIO_PORTG, 5, irs::io_t::dir_out, irs::io_pin_off),
  dac_ti_cs(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_on),
  th_cs(GPIO_PORTD, 2, irs::io_t::dir_out, irs::io_pin_on),
  relay_bridge_pos_on(GPIO_PORTB, 2, irs::io_t::dir_out, irs::io_pin_off),
  relay_bridge_pos_off(GPIO_PORTD, 8, irs::io_t::dir_out, irs::io_pin_off),
  relay_bridge_neg_on(GPIO_PORTE, 8, irs::io_t::dir_out, irs::io_pin_off),
  relay_bridge_neg_off(GPIO_PORTF, 11, irs::io_t::dir_out, irs::io_pin_off),
  relay_gain_high(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_off),
  relay_gain_low(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_off),
  relay_voltage_high(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_off),
  relay_voltage_low(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_off),
  relay_prot(GPIO_PORTD, 9, irs::io_t::dir_out, irs::io_pin_on),
  led_blink(GPIO_PORTB, 9, irs::io_t::dir_out, irs::io_pin_off),
  led_hf(GPIO_PORTF, 6, irs::io_t::dir_out, irs::io_pin_off),
  led_pon(GPIO_PORTB, 8, irs::io_t::dir_out, irs::io_pin_off),
  buzzer(irs::handle_t<irs::pwm_gen_t>(
    new irs::arm::st_pwm_gen_t(PB6, IRS_TIM4_BASE, 4000, 0.5))),

  adc_exti()
{
  key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTG, 15, irs::io_t::dir_in)));
  key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTG, 12, irs::io_t::dir_in)));
  key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTA, 4, irs::io_t::dir_in)));
  key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTG, 11, irs::io_t::dir_in)));

  key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTG, 10, irs::io_t::dir_open_drain)));
  key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTD, 6, irs::io_t::dir_open_drain)));
  key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTD, 3, irs::io_t::dir_open_drain)));
  key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTG, 4, irs::io_t::dir_open_drain)));

  irs::pause(irs::make_cnt_ms(100));
}

/*irs::hardflow::simple_udp_flow_t* hrm::cfg_t::hardflow()
{
  return &simple_hardflow;
}

irs::spi_t* hrm::cfg_t::spi()
{
  return &m_spi;
}

irs::spi_t* hrm::cfg_t::spi_th()
{
  return &spi_th;
}

hrm::cfg_t::pins_t* hrm::cfg_t::pins()
{
  return &m_pins;
}*/

void hrm::cfg_t::tick()
{
  m_arm_eth.tick();
  m_tcpip.tick();
  simple_hardflow.tick();
}
