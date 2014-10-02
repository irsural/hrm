#include <irspch.h>

#include "cfg.h"

#include <irsfinal.h>

// class lwip_config_t
hrm::network_config_t::network_config_t(
  irs::arm::st_ethernet_t* ap_arm_eth,
  irs::handle_t<irs::lwip::ethernet_t>* ap_ethernet,
  irs::handle_t<irs::hardflow::lwip::udp_t>* ap_udp_client,
  irs::hardflow::connector_t *ap_connector_hardflow
):
  mp_arm_eth(ap_arm_eth),
  mp_ethernet(ap_ethernet),
  mp_udp_client(ap_udp_client),
  mp_connector_hardflow(ap_connector_hardflow),
  m_config()
{

}

void hrm::network_config_t::set_ip(mxip_t a_ip)
{
  m_config.ip = a_ip;
  reset();
}

void hrm::network_config_t::set_dhcp(bool a_dhcp)
{
  m_config.dhcp_enabled = a_dhcp;
  reset();
}

void hrm::network_config_t::set_mask(mxip_t a_mask)
{
  m_config.netmask = a_mask;
  reset();
}

void hrm::network_config_t::set_gateway(mxip_t a_gateway)
{
  m_config.gateway = a_gateway;
  reset();
}

void hrm::network_config_t::get(
  mxip_t* ap_ip, mxip_t* ap_mask, mxip_t* ap_gateway, bool* ap_dhcp_enabled)
{
  *ap_ip = (*mp_ethernet)->get_ip();
  *ap_mask = (*mp_ethernet)->get_netmask();
  *ap_gateway = (*mp_ethernet)->get_gateway();
  *ap_dhcp_enabled = m_config.dhcp_enabled;
}

void hrm::network_config_t::set(
  mxip_t a_ip, mxip_t a_mask, mxip_t a_gateway, bool a_dhcp_enabled)
{
  m_config.ip = a_ip;
  m_config.netmask = a_mask;
  m_config.dhcp_enabled = a_dhcp_enabled;
  m_config.gateway = a_gateway;
  reset();
}

void hrm::network_config_t::reset()
{
  mp_udp_client->reset();

  irs::lwip::ethernet_t::configuration_t eth_config = m_config;
  if (eth_config.dhcp_enabled) {
    eth_config.ip = mxip_t::any_ip();
  }
  const mxip_t local_ip = eth_config.ip;
  const mxip_t remote_ip = mxip_t::any_ip();

  mp_ethernet->reset(new irs::lwip::ethernet_t(mp_arm_eth, eth_config));

  irs::hardflow::lwip::udp_t::configuration_t configuration;
  mp_udp_client->reset(new irs::hardflow::lwip::udp_t(
    local_ip, 5005, remote_ip, 0, channel_max_count, configuration));
  (*mp_udp_client)->
    set_channel_switching_mode(irs::hardflow_t::csm_ready_for_reading);

  mp_connector_hardflow->hardflow(mp_udp_client->get());
}

hrm::cfg_t::cfg_t():
  m_local_mac(irs::arm::st_generate_mac(irs::device_code_hrm)),
  m_config(),
  m_arm_eth(1500, m_local_mac, m_config),
  lwip_ethernet(),
  udp_client(),
  connector_hardflow(NULL),
  network_config(&m_arm_eth, &lwip_ethernet, &udp_client, &connector_hardflow),

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

void hrm::cfg_t::tick()
{
  m_arm_eth.tick();
  IRS_LIB_ASSERT(!lwip_ethernet.is_empty());
  IRS_LIB_ASSERT(!udp_client.is_empty());
  lwip_ethernet->tick();
  udp_client->tick();
  connector_hardflow.tick();
}
