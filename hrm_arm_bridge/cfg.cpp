#include <irspch.h>

#include "cfg.h"

#include <irsfinal.h>

hrm::cfg_t::pins_t::pins_t(
  irs::gpio_pin_t* ap_vben,
  irs::gpio_pin_t* ap_adc_cs,
  irs::gpio_pin_t* ap_dac_cs,
  irs::gpio_pin_t* ap_dac_ldac,
  irs::gpio_pin_t* ap_dac_clr,
  irs::gpio_pin_t* ap_dac_reset,
  irs::gpio_pin_t* ap_relay_100k,
  irs::gpio_pin_t* ap_relay_1m,
  irs::gpio_pin_t* ap_relay_10m,
  irs::gpio_pin_t* ap_relay_100m,
  irs::gpio_pin_t* ap_relay_1g,
  irs::gpio_pin_t* ap_relay_eton,
  irs::gpio_pin_t* ap_relay_chon,
  irs::gpio_pin_t* ap_relay_zero_on,
  irs::gpio_pin_t* ap_relay_zero_off,
  irs::gpio_pin_t* ap_relay_etpol_on,
  irs::gpio_pin_t* ap_relay_etpol_off,
  irs::gpio_pin_t* ap_led_blink,
  irs::gpio_pin_t* ap_led_hf,
  irs::gpio_pin_t* ap_led_pon):
  
  p_vben(ap_vben),
  p_adc_cs(ap_adc_cs),
  p_dac_cs(ap_dac_cs),
  p_dac_ldac(ap_dac_ldac),
  p_dac_clr(ap_dac_clr),
  p_dac_reset(ap_dac_reset),
  p_relay_100k(ap_relay_100k),
  p_relay_1m(ap_relay_1m),
  p_relay_10m(ap_relay_10m),
  p_relay_100m(ap_relay_100m),
  p_relay_1g(ap_relay_1g),
  p_relay_eton(ap_relay_eton),
  p_relay_chon(ap_relay_chon),
  p_relay_zero_on(ap_relay_zero_on),
  p_relay_zero_off(ap_relay_zero_off),
  p_relay_etpol_on(ap_relay_etpol_on),
  p_relay_etpol_off(ap_relay_etpol_off),
  p_led_blink(ap_led_blink),
  p_led_hf(ap_led_hf),
  p_led_pon(ap_led_pon)
{
}

hrm::cfg_t::cfg_t():
  m_local_mac(irs::make_mxmac(0, 0, IP_0, IP_1, IP_2, IP_3)),
  m_config(),
  m_arm_eth(1500, m_local_mac, m_config),
  m_local_ip(mxip_t::zero_ip()),
  m_local_port(5005),
  m_dest_ip(irs::make_mxip(IP_0, IP_1, IP_2, IP_3)),
  m_dest_port(5005),
  m_tcpip(&m_arm_eth, m_local_ip, m_dest_ip, 10),
  m_simple_hardflow(&m_tcpip, m_local_ip, m_local_port,
    m_dest_ip, m_dest_port, 10),
  
  m_spi_bitrate(10000),
  m_spi(IRS_SPI3_I2S3_BASE, m_spi_bitrate, PC10, PC11, PC12),
  
  m_vben(GPIO_PORTG, 4, irs::io_t::dir_out, irs::io_pin_off),
  m_adc_cs(GPIO_PORTE, 0, irs::io_t::dir_out, irs::io_pin_on),
  m_dac_cs(GPIO_PORTA, 6, irs::io_t::dir_out, irs::io_pin_on),
  m_dac_ldac(GPIO_PORTD, 3, irs::io_t::dir_out, irs::io_pin_on),
  m_dac_clr(GPIO_PORTC, 6, irs::io_t::dir_out, irs::io_pin_on),
  m_dac_reset(GPIO_PORTC, 7, irs::io_t::dir_out, irs::io_pin_on),
  m_relay_100k(GPIO_PORTE, 6, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_1m(GPIO_PORTE, 5, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_10m(GPIO_PORTD, 2, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_100m(GPIO_PORTD, 6, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_1g(GPIO_PORTG, 15, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_eton(GPIO_PORTG, 12, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_chon(GPIO_PORTB, 6, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_zero_on(GPIO_PORTG, 11, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_zero_off(GPIO_PORTA, 4, irs::io_t::dir_out, irs::io_pin_on),
  m_relay_etpol_on(GPIO_PORTE, 1, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_etpol_off(GPIO_PORTE, 4, irs::io_t::dir_out, irs::io_pin_on),
  m_led_blink(GPIO_PORTB, 9, irs::io_t::dir_out, irs::io_pin_off),
  m_led_hf(GPIO_PORTF, 6, irs::io_t::dir_out, irs::io_pin_off),
  m_led_pon(GPIO_PORTB, 8, irs::io_t::dir_out, irs::io_pin_off),
  
  m_pins(
    &m_vben,
    &m_adc_cs,
    &m_dac_cs,
    &m_dac_ldac,
    &m_dac_clr,
    &m_dac_reset,
    &m_relay_100k,
    &m_relay_1m,
    &m_relay_10m,
    &m_relay_100m,
    &m_relay_1g,
    &m_relay_eton,
    &m_relay_chon,
    &m_relay_zero_on,
    &m_relay_zero_off,
    &m_relay_etpol_on,
    &m_relay_etpol_off,
    &m_led_blink,
    &m_led_hf,
    &m_led_pon)
{
}

irs::hardflow::simple_udp_flow_t* hrm::cfg_t::hardflow()
{
  return &m_simple_hardflow;
}

irs::arm::arm_spi_t* hrm::cfg_t::spi()
{
  return &m_spi;
}

hrm::cfg_t::pins_t* hrm::cfg_t::pins()
{
  return &m_pins;
}

void hrm::cfg_t::tick()
{
  m_arm_eth.tick();
  m_tcpip.tick();
  m_simple_hardflow.tick();  
}
