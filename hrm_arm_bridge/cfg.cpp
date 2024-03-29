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

void hrm::network_config_t::get_mac(mxmac_t* ap_mac)
{
  *ap_mac = mp_arm_eth->get_local_mac();
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
  //configuration.connections_mode = irs::hardflow::udplc_mode_limited;
  //configuration.limit_downtime_enabled = true;
  //configuration.max_downtime_sec = 5;
  mp_udp_client->reset(new irs::hardflow::lwip::udp_t(
    local_ip, 5005, remote_ip, 0, channel_max_count, configuration));
  (*mp_udp_client)->
    set_channel_switching_mode(irs::hardflow_t::csm_ready_for_reading);

  mp_connector_hardflow->hardflow(mp_udp_client->get());
}

hrm::cfg_t::cfg_t():
  m_local_mac(irs::arm::st_generate_mac(irs::device_code_hrm)),
  m_config(phy_config()),
  m_arm_eth(1500, m_local_mac, m_config),
  lwip_ethernet(),
  udp_client(),
  connector_hardflow(NULL),
  network_config(&m_arm_eth, &lwip_ethernet, &udp_client, &connector_hardflow),
  //  AUX
  led_blink(GPIO_PORTD, 8, irs::io_t::dir_out, irs::io_pin_off),
  ee_cs(GPIO_PORTE, 15, irs::io_t::dir_out, irs::io_pin_on),
  mezzo3(GPIO_PORTB, 0, irs::io_t::dir_out, irs::io_pin_off),
  mezzo4(GPIO_PORTB, 1, irs::io_t::dir_out, irs::io_pin_off),
  //  UI
  lcd_port(GPIO_PORTG, 0xFF, irs::io_t::dir_open_drain, 8),
  lcd_rs_pin(GPIO_PORTG, 7, irs::io_t::dir_open_drain),
  lcd_e_pin(GPIO_PORTG, 6, irs::io_t::dir_open_drain),
  key_drv_horizontal_pins(),
  key_drv_vertical_pins(),
  buzzer(irs::handle_t<irs::pwm_gen_t>(
    new irs::arm::st_pwm_gen_t(PF6, IRS_TIM10_BASE, 4000, 0.05))),
  //  AD4630
  ad4630_rst(GPIO_PORTC, 14, irs::io_t::dir_out, irs::io_pin_off),
  ad4630_cs(GPIO_PORTF, 5, irs::io_t::dir_out, irs::io_pin_on),
  ad4630_busy(GPIO_PORTF, 4, irs::io_t::dir_in),
  ad4630_pwr(GPIO_PORTC, 15, irs::io_t::dir_in),
  ad4630_point_control(GPIO_PORTC, 12, irs::io_t::dir_out),
  pulse_gen(),
  //  Bridge voltage DAC
  bridge_voltage_dac_cs(GPIO_PORTE, 2, irs::io_t::dir_out, irs::io_pin_on),
  //  Relays
  relay_bridge_pos_on(GPIO_PORTD, 0, irs::io_t::dir_out, irs::io_pin_off),
  relay_bridge_pos_off(GPIO_PORTD, 3, irs::io_t::dir_out, irs::io_pin_off),
  relay_bridge_neg_on(GPIO_PORTD, 2, irs::io_t::dir_out, irs::io_pin_off),
  relay_bridge_neg_off(GPIO_PORTD, 1, irs::io_t::dir_out, irs::io_pin_off),
  relay_prot(GPIO_PORTF, 3, irs::io_t::dir_out, irs::io_pin_on),
  //  Relays AD4630
  relay_bridge_divp_on(GPIO_PORTD, 4, irs::io_t::dir_out, irs::io_pin_off),
  relay_bridge_divp_off(GPIO_PORTD, 5, irs::io_t::dir_out, irs::io_pin_off),
  //relay_bridge_divn_on(GPIO_PORTD, 6, irs::io_t::dir_out, irs::io_pin_off),
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  ��-�� ��������� PD6
  relay_bridge_divn_on(GPIO_PORTA, 11, irs::io_t::dir_out, irs::io_pin_off),
  relay_bridge_divn_off(GPIO_PORTD, 7, irs::io_t::dir_out, irs::io_pin_off),
  //  Relay DIVSW, ON and OFF was swaped
  relay_divsw_on(GPIO_PORTF, 1, irs::io_t::dir_out, irs::io_pin_off),
  relay_divsw_off(GPIO_PORTF, 0, irs::io_t::dir_out, irs::io_pin_off),
  led_divsw(GPIO_PORTE, 1, irs::io_t::dir_out, irs::io_pin_off),
  //  SPI
  m_spi_bitrate(50000),
  spi_dac(IRS_SPI1_BASE, 37500000/*m_spi_bitrate*/, PA5, PB4, PB5,
    irs::arm::arm_spi_t::gpio_speed_100mhz),
  spi_aux(IRS_SPI2_I2S2_BASE, m_spi_bitrate, PB10, PC2, PC3,
    irs::arm::arm_spi_t::gpio_speed_25mhz),
  //  Climate
  peltier_pwm1_gen(peltier_pwm1_channel, IRS_TIM9_BASE, 25000, 0.5),
  peltier_pwm2_gen(peltier_pwm2_channel, IRS_TIM9_BASE, 25000, 0.5),
  peltier_pol1(GPIO_PORTD, 10, irs::io_t::dir_out, irs::io_pin_off),
  peltier_pol2(GPIO_PORTB, 15, irs::io_t::dir_out, irs::io_pin_off),
  fan_ac_on(GPIO_PORTG, 5, irs::io_t::dir_out, irs::io_pin_off),
  fan_dc_ls(GPIO_PORTD, 12, irs::io_t::dir_out, irs::io_pin_off),
  fan_dc_hs(GPIO_PORTD, 13, irs::io_t::dir_out, irs::io_pin_off),
  fan_dc_sen(GPIO_PORTE, 14, irs::io_t::dir_in)
{
  key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTF, 13, irs::io_t::dir_in)));
  key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTF, 14, irs::io_t::dir_in)));
  key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTF, 15, irs::io_t::dir_in)));
  key_drv_horizontal_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTG, 0, irs::io_t::dir_in)));

  key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTE, 9, irs::io_t::dir_open_drain)));
  key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTE, 8, irs::io_t::dir_open_drain)));  
  key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTE, 7, irs::io_t::dir_open_drain)));  
  key_drv_vertical_pins.push_back(irs::handle_t<irs::gpio_pin_t>(
    new irs::arm::io_pin_t(GPIO_PORTG, 1, irs::io_t::dir_open_drain)));
  
  irs::pause(irs::make_cnt_ms(100));
}

irs::arm::st_ethernet_t::config_t hrm::cfg_t::phy_config()
{
  irs::arm::st_ethernet_t::config_t cfg;
  cfg.mii_mode = irs::arm::st_ethernet_t::reduced_mii_mode;
  cfg.rx_clk_or_ref_clk = PA1;
  cfg.txd[0] = PB12;
  cfg.txd[1] = PB13;
  cfg.tx_en = PB11;
  cfg.rxd[0] = PC4;
  cfg.rxd[1] = PC5;
  cfg.rx_dv_or_crs_dv = PA7;
  cfg.mdc = PC1;
  cfg.mdio = PA2;
  cfg.phy_reset = PB14;
  cfg.phy_address = 0;
  return cfg;
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
