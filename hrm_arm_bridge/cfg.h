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
#include <irstcpip.h>
#include <irslwip.h>
#include <hardflowg.h>

#include "privatecfg.h"

#include <irsfinal.h>

#define LWIP

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
  inline adc_exti_t()
  {
    irs::clock_enable(IRS_PORTG_BASE);
    irs::gpio_moder_input_enable(PG3);
    SETENA0_bit.SETENA_EXTI3 = 1; //  exti3 линия 3
    SYSCFG_EXTICR1_bit.EXTI3 = 6;     //  PORT G
    EXTI_IMR_bit.MR3 = 0; // Изначально прерывание на линии 3 отключено
    EXTI_FTSR_bit.TR3 = 1; // Включаем реакцию на задний фронт
  }
  inline ~adc_exti_t()
  {
    SETENA0_bit.SETENA_EXTI3 = 0;
  };
  inline void add_event(mxfact_event_t *ap_event)
  {
    irs::interrupt_array()->int_event_gen(irs::arm::exti3_int)->add(ap_event);
  }
  inline void start()   { EXTI_IMR_bit.MR3 = 1; }
  inline void stop()    { EXTI_IMR_bit.MR3 = 0; }
  inline bool stopped() { return EXTI_IMR_bit.MR3; }
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
private:
  void reset();
  network_config_t();
  enum { channel_max_count = 3 };
  irs::arm::st_ethernet_t* mp_arm_eth;
  irs::handle_t<irs::lwip::ethernet_t>* mp_ethernet;
  irs::handle_t<irs::hardflow::lwip::udp_t>* mp_udp_client;
  irs::hardflow::connector_t* mp_connector_hardflow;
  irs::lwip::ethernet_t::configuration_t m_config;
  /*mxip_t m_ip;
  bool m_dhcp;
  mxip_t m_mask;
  mxip_t m_gateway;*/
};

class cfg_t
{
public:
  cfg_t();
  void tick();

private:
  mxmac_t m_local_mac;
  irs::arm::st_ethernet_t::config_t m_config;
  irs::arm::st_ethernet_t m_arm_eth;
  #ifndef LWIP
  mxip_t m_local_ip;
  irs_u16 m_local_port;
  mxip_t m_dest_ip;
  irs_u16 m_dest_port;
  irs::simple_tcpip_t m_tcpip;
  #endif // !LWIP
  #ifdef LWIP
  irs::handle_t<irs::lwip::ethernet_t> lwip_ethernet;
  irs::handle_t<irs::hardflow::lwip::udp_t> udp_client;
  #endif // LWIP
public:
  #ifdef LWIP
  irs::hardflow::connector_t connector_hardflow;
  network_config_t network_config;
  #else // !LWIP
  irs::hardflow::simple_udp_flow_t simple_hardflow;
  #endif // !LWIP

  irs_u32 m_spi_bitrate;
  irs::arm::arm_spi_t spi;
  irs::arm::arm_spi_t spi_th;

  irs::arm::io_pin_t vben;
  irs::arm::io_pin_t ee_cs;
  irs::arm::io_port_t lcd_port;
  irs::arm::io_pin_t lcd_rs_pin;
  irs::arm::io_pin_t lcd_e_pin;
  vector<irs::handle_t<irs::gpio_pin_t> > key_drv_horizontal_pins;
  vector<irs::handle_t<irs::gpio_pin_t> > key_drv_vertical_pins;
  gpio_channel_t encoder_gpio_channel_1;
  gpio_channel_t encoder_gpio_channel_2;
  size_t encoder_timer_address;
  irs::arm::io_pin_t key_encoder;
  irs::arm::io_pin_t adc_cs;
  irs::arm::io_pin_t dac_cs;
  irs::arm::io_pin_t dac_ldac;
  irs::arm::io_pin_t dac_clr;
  irs::arm::io_pin_t dac_reset;
  irs::arm::io_pin_t dac_ti_cs;
  irs::arm::io_pin_t th_cs;
  irs::arm::io_pin_t relay_bridge_pos_on;
  irs::arm::io_pin_t relay_bridge_pos_off;
  irs::arm::io_pin_t relay_bridge_neg_on;
  irs::arm::io_pin_t relay_bridge_neg_off;
  irs::arm::io_pin_t relay_gain_high;
  irs::arm::io_pin_t relay_gain_low;
  irs::arm::io_pin_t relay_voltage_high;
  irs::arm::io_pin_t relay_voltage_low;
  irs::arm::io_pin_t relay_prot;
  irs::arm::io_pin_t led_blink;
  irs::arm::io_pin_t led_hf;
  irs::arm::io_pin_t led_pon;
  irs::pwm_pin_t buzzer;

  adc_exti_t adc_exti;
};

} //  hrm

#endif // cfgh
