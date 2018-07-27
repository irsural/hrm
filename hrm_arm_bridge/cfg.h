#ifndef cfgh
#define cfgh

#include <irsdefs.h>

#include <armcfg.h>
#include <armspi.h>
#include <armgpio.h>
#include <armeth.h>
#include <armadc.h>
#include <irskbd.h>
#include <irserror.h>
#include <irsdev.h>
#include <mxdata.h>

#include <irsconfig.h>
//#include <irstcpip.h>
#include <irslwip.h>
#include <hardflowg.h>

#include "privatecfg.h"

#include <irsfinal.h>

namespace hrm {

//#define HRM_DEBUG

#ifdef HRM_DEBUG
#define HRM_ASSERT(assert_expr) IRS_ASSERT(assert_expr)
#define HRM_ASSERT_MSG(msg) IRS_ASSERT_MSG(msg)
#define HRM_NEW_ASSERT(new_expr, file_idx)\
  IRS_NEW_ASSERT(new_expr, file_idx)
#define HRM_ERROR(error_code, msg) IRS_ERROR(error_code, msg)
#define HRM_ERROR_IF_NOT(assert_expr, error_code, msg)\
  IRS_ERROR_IF_NOT(assert_expr, error_code, msg)
#define HRM_DBG_RAW_MSG(msg) IRS_DBG_RAW_MSG(msg)
#define HRM_DBG_MSG(msg) IRS_LIB_DBG_MSG(msg)
#else //HRM_DEBUG
#define HRM_ASSERT(assert_expr)
#define HRM_ASSERT_MSG(msg)
#define HRM_NEW_ASSERT(new_expr, file_idx) (new new_expr)
#define HRM_ERROR(error_code, msg)
#define HRM_ERROR_IF_NOT(assert_expr, error_code, msg)
#define HRM_DBG_RAW_MSG(msg)
#define HRM_DBG_MSG(msg)
#endif //HRM_DEBUG

#define HRM_DELETE_ASSERT(expr) IRS_DELETE_ASSERT(expr)

class adc_exti_t
{
public:
//  inline adc_exti_t()
//  {
//    irs::clock_enable(IRS_PORTF_BASE);
//    irs::gpio_moder_input_enable(PF1);
//    SETENA0_bit.SETENA_EXTI1 = 1;     //  exti1 ����� 1
//    SYSCFG_EXTICR1_bit.EXTI1 = 5;     //  PORT F
//    EXTI_IMR_bit.MR1 = 0;   // ���������� ���������� �� ����� 1 ���������
//    EXTI_FTSR_bit.TR1 = 1;  // �������� ������� �� ������ �����
//  }
//  inline ~adc_exti_t()
//  {
//    SETENA0_bit.SETENA_EXTI1 = 0;
//  };
//  inline void add_event(mxfact_event_t *ap_event)
//  {
//    irs::interrupt_array()->int_event_gen(irs::arm::exti1_int)->add(ap_event);
//  }
//  inline void start()   { EXTI_IMR_bit.MR1 = 1; }
//  inline void stop()    { EXTI_IMR_bit.MR1 = 0; }
//  inline bool stopped() { return EXTI_IMR_bit.MR1; }
  inline adc_exti_t()
  {
    irs::clock_enable(IRS_PORTF_BASE);
    irs::gpio_moder_input_enable(PF0);
    SETENA0_bit.SETENA_EXTI0 = 1;     //  exti1 ����� 0
    SYSCFG_EXTICR1_bit.EXTI0 = 5;     //  PORT F
    EXTI_IMR_bit.MR0 = 0;   // ���������� ���������� �� ����� 0 ���������
    EXTI_FTSR_bit.TR0 = 1;  // �������� ������� �� ������ �����
  }
  inline ~adc_exti_t()
  {
    SETENA0_bit.SETENA_EXTI0 = 0;
  };
  inline void add_event(mxfact_event_t *ap_event)
  {
    irs::interrupt_array()->int_event_gen(irs::arm::exti0_int)->add(ap_event);
  }
  inline void start()   { EXTI_IMR_bit.MR0 = 1; }
  inline void stop()    { EXTI_IMR_bit.MR0 = 0; }
  inline bool stopped() { return EXTI_IMR_bit.MR0; }
};

class network_config_t
{
public:
  network_config_t(
    irs::arm::st_ethernet_t* ap_arm_eth,
    irs::handle_t<irs::lwip::ethernet_t>* ap_ethernet,
    irs::handle_t<irs::hardflow::lwip::udp_t>* ap_udp_client,
    irs::hardflow::connector_t* ap_connector_hardflow);
  void set_ip(mxip_t a_ip);
  void set_dhcp(bool a_dhcp);
  void set_mask(mxip_t a_mask);
  void set_gateway(mxip_t a_gateway);
  void get(mxip_t* ap_ip, mxip_t* ap_mask, mxip_t* ap_gateway,
    bool* ap_dhcp_enabled);
  void set(mxip_t a_ip, mxip_t a_mask, mxip_t a_gateway, bool a_dhcp_enabled);
  void get_mac(mxmac_t* ap_mac);
private:
  void reset();
  network_config_t();
  //enum { channel_max_count = 10 };
  enum { channel_max_count = 3 };
  irs::arm::st_ethernet_t* mp_arm_eth;
  irs::handle_t<irs::lwip::ethernet_t>* mp_ethernet;
  irs::handle_t<irs::hardflow::lwip::udp_t>* mp_udp_client;
  irs::hardflow::connector_t* mp_connector_hardflow;
  irs::lwip::ethernet_t::configuration_t m_config;
};

class cfg_t
{
public:
  cfg_t();
  void tick();

private:
  mxmac_t m_local_mac;
  irs::arm::st_ethernet_t::config_t m_config;
  irs::arm::st_ethernet_t::config_t phy_config();
  irs::arm::st_ethernet_t m_arm_eth;
  irs::handle_t<irs::lwip::ethernet_t> lwip_ethernet;
  irs::handle_t<irs::hardflow::lwip::udp_t> udp_client;
public:
  //  Ethernet
  irs::hardflow::connector_t connector_hardflow;
  network_config_t network_config;
  //  AUX
  irs::arm::io_pin_t led_blink;
  irs::arm::io_pin_t ee_cs;
  irs::arm::io_pin_t aux1;
  irs::arm::io_pin_t aux2;
  irs::arm::io_pin_t aux3;
  irs::arm::io_pin_t mezzo1;
  irs::arm::io_pin_t mezzo2;
  irs::arm::io_pin_t mezzo3;
  irs::arm::io_pin_t mezzo4;
  //  UI
  irs::arm::io_port_t lcd_port;
  irs::arm::io_pin_t lcd_rs_pin;
  irs::arm::io_pin_t lcd_e_pin;
  vector<irs::handle_t<irs::gpio_pin_t> > key_drv_horizontal_pins;
  vector<irs::handle_t<irs::gpio_pin_t> > key_drv_vertical_pins;
  gpio_channel_t enc_a;
  gpio_channel_t enc_b;
  size_t encoder_timer_address;
  irs::arm::io_pin_t enc_sw;
  irs::pwm_pin_t buzzer;
  //  ADC
  irs::arm::io_pin_t adc_cs;
  irs::arm::io_pin_t adc_reset;
  irs::arm::io_pin_t adc_start;
  irs::arm::io_pin_t adc_clk;
  irs::arm::io_pin_t adc_en;
  //  DAC
  irs::arm::io_pin_t dac_cs;
  irs::arm::io_pin_t dac_ldac;
  irs::arm::io_pin_t dac_clr;
  irs::arm::io_pin_t dac_reset;
  irs::arm::io_pin_t dac_en;
  irs::arm::io_pin_t dac_enctrl;
  //  Relays
  irs::arm::io_pin_t relay_bridge_pos_on;
  irs::arm::io_pin_t relay_bridge_pos_off;
  irs::arm::io_pin_t relay_bridge_neg_on;
  irs::arm::io_pin_t relay_bridge_neg_off;
  irs::arm::io_pin_t relay_prot;
  //  SPI
  irs_u32 m_spi_bitrate;
  irs::arm::arm_spi_t spi_adc;
  irs::arm::arm_spi_t spi_dac;
  irs::arm::arm_spi_t spi_aux;
  //  External interrupt
  adc_exti_t adc_exti;
  //  Climate
  static const gpio_channel_t peltier_pwm1_channel = PE6;
  irs::arm::st_pwm_gen_t peltier_pwm1_gen;
  //irs::pwm_pin_t peltier_pwm1;
  static const gpio_channel_t peltier_pwm2_channel = PE5;
  irs::arm::st_pwm_gen_t peltier_pwm2_gen;
  //irs::pwm_pin_t peltier_pwm2;
  irs::arm::io_pin_t peltier_pol1;
  irs::arm::io_pin_t peltier_pol2;
  irs::arm::io_pin_t fan_ac_on;
  irs::arm::io_pin_t fan_dc_ls;
  irs::arm::io_pin_t fan_dc_hs;
  irs::arm::io_pin_t fan_dc_sen;
};

} //  hrm

#endif // cfgh
