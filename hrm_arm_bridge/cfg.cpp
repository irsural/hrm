#include <irspch.h>

#include "cfg.h"

#include <irsfinal.h>

hrm::cfg_t::pins_t::pins_t(
  irs::gpio_pin_t* ap_vben,
  irs::gpio_pin_t* ap_ee_cs,
  irs::gpio_pin_t* ap_adc_cs,
  irs::gpio_pin_t* ap_dac_cs,
  irs::gpio_pin_t* ap_dac_ldac,
  irs::gpio_pin_t* ap_dac_clr,
  irs::gpio_pin_t* ap_dac_reset,
  irs::gpio_pin_t* ap_dac_ti_cs,
  irs::gpio_pin_t* ap_relay_bridge_pos_on,
  irs::gpio_pin_t* ap_relay_bridge_pos_off,
  irs::gpio_pin_t* ap_relay_bridge_neg_on,
  irs::gpio_pin_t* ap_relay_bridge_neg_off,
  irs::gpio_pin_t* ap_relay_gain_high,
  irs::gpio_pin_t* ap_relay_gain_low,
  irs::gpio_pin_t* ap_relay_voltage_high,
  irs::gpio_pin_t* ap_relay_voltage_low,
  irs::gpio_pin_t* ap_relay_prot,
  irs::gpio_pin_t* ap_led_blink,
  irs::gpio_pin_t* ap_led_hf,
  irs::gpio_pin_t* ap_led_pon,
  irs::gpio_pin_t* ap_buzzer):
  
  p_vben(ap_vben),
  p_ee_cs(ap_ee_cs),
  p_adc_cs(ap_adc_cs),
  p_dac_cs(ap_dac_cs),
  p_dac_ldac(ap_dac_ldac),
  p_dac_clr(ap_dac_clr),
  p_dac_reset(ap_dac_reset),
  p_dac_ti_cs(ap_dac_ti_cs),
  p_relay_bridge_pos_on(ap_relay_bridge_pos_on),
  p_relay_bridge_pos_off(ap_relay_bridge_pos_off),
  p_relay_bridge_neg_on(ap_relay_bridge_neg_on),
  p_relay_bridge_neg_off(ap_relay_bridge_neg_off),
  p_relay_gain_high(ap_relay_gain_high),
  p_relay_gain_low(ap_relay_gain_low),
  p_relay_voltage_high(ap_relay_voltage_high),
  p_relay_voltage_low(ap_relay_voltage_low),
  p_relay_prot(ap_relay_prot),
  p_led_blink(ap_led_blink),
  p_led_hf(ap_led_hf),
  p_led_pon(ap_led_pon),
  p_buzzer(ap_buzzer)
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
  
  m_spi_bitrate(1000),
  m_spi(IRS_SPI1_BASE, m_spi_bitrate, PA5, PA6, PB5, 
    irs::arm::arm_spi_t::gpio_speed_25mhz),
  m_spi_2(IRS_SPI2_I2S2_BASE, m_spi_bitrate, PB10, PC2, PC3),
  
  m_vben(GPIO_PORTG, 4, irs::io_t::dir_out, irs::io_pin_on),
  m_ee_cs(GPIO_PORTC, 13, irs::io_t::dir_out, irs::io_pin_on),
  m_adc_cs(GPIO_PORTG, 7, irs::io_t::dir_out, irs::io_pin_on),
  m_dac_cs(GPIO_PORTG, 8, irs::io_t::dir_out, irs::io_pin_on),
  m_dac_ldac(GPIO_PORTG, 6, irs::io_t::dir_out, irs::io_pin_off),
  m_dac_clr(GPIO_PORTF, 8, irs::io_t::dir_out, irs::io_pin_off),
  m_dac_reset(GPIO_PORTG, 5, irs::io_t::dir_out, irs::io_pin_off),
  m_dac_ti_cs(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_on),
  m_relay_bridge_pos_on(GPIO_PORTB, 2, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_bridge_pos_off(GPIO_PORTD, 8, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_bridge_neg_on(GPIO_PORTE, 8, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_bridge_neg_off(GPIO_PORTF, 11, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_gain_high(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_gain_low(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_voltage_high(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_voltage_low(GPIO_PORTF, 9, irs::io_t::dir_out, irs::io_pin_off),
  m_relay_prot(GPIO_PORTD, 9, irs::io_t::dir_out, irs::io_pin_on),
  m_led_blink(GPIO_PORTB, 9, irs::io_t::dir_out, irs::io_pin_off),
  m_led_hf(GPIO_PORTF, 6, irs::io_t::dir_out, irs::io_pin_off),
  m_led_pon(GPIO_PORTB, 8, irs::io_t::dir_out, irs::io_pin_off),
  m_buzzer(irs::handle_t<irs::pwm_gen_t>(
    new irs::arm::st_pwm_gen_t(PB6, IRS_TIM4_BASE, 4000, 0.5))),
  
  m_pins(
    &m_vben,
    &m_ee_cs,
    &m_adc_cs,
    &m_dac_cs,
    &m_dac_ldac,
    &m_dac_clr,
    &m_dac_reset,
    &m_dac_ti_cs,
    &m_relay_bridge_pos_on,
    &m_relay_bridge_pos_off,
    &m_relay_bridge_neg_on,
    &m_relay_bridge_neg_off,
    &m_relay_gain_high,
    &m_relay_gain_low,
    &m_relay_voltage_high,
    &m_relay_voltage_low,
    &m_relay_prot,
    &m_led_blink,
    &m_led_hf,
    &m_led_pon,
    &m_buzzer),
  m_adc_exti()  
{
  irs::pause(irs::make_cnt_ms(100));
}

irs::hardflow::simple_udp_flow_t* hrm::cfg_t::hardflow()
{
  return &m_simple_hardflow;
}

irs::spi_t* hrm::cfg_t::spi()
{
  return &m_spi;
}

irs::spi_t* hrm::cfg_t::spi_2()
{
  return &m_spi_2;
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
