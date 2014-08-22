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

#include <irsconfig.h>
#include <irstcpip.h>
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
#else //HRM_DEBUG
#define HRM_ASSERT(assert_expr)
#define HRM_ASSERT_MSG(msg)
#define HRM_NEW_ASSERT(new_expr, file_idx) (new new_expr)
#define HRM_ERROR(error_code, msg)
#define HRM_ERROR_IF_NOT(assert_expr, error_code, msg)
#define HRM_DBG_RAW_MSG(msg)
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
  
class cfg_t
{
public:
  struct pins_t {
    pins_t(
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
      irs::gpio_pin_t* ap_buzzer);
    
    irs::gpio_pin_t* p_vben;
    irs::gpio_pin_t* p_ee_cs;
    irs::gpio_pin_t* p_adc_cs;
    irs::gpio_pin_t* p_dac_cs;
    irs::gpio_pin_t* p_dac_ldac;
    irs::gpio_pin_t* p_dac_clr;
    irs::gpio_pin_t* p_dac_reset;
    irs::gpio_pin_t* p_dac_ti_cs;
    irs::gpio_pin_t* p_relay_bridge_pos_on;
    irs::gpio_pin_t* p_relay_bridge_pos_off;
    irs::gpio_pin_t* p_relay_bridge_neg_on;
    irs::gpio_pin_t* p_relay_bridge_neg_off;
    irs::gpio_pin_t* p_relay_gain_high;
    irs::gpio_pin_t* p_relay_gain_low;
    irs::gpio_pin_t* p_relay_voltage_high;
    irs::gpio_pin_t* p_relay_voltage_low;
    irs::gpio_pin_t* p_relay_prot;
    irs::gpio_pin_t* p_led_blink;
    irs::gpio_pin_t* p_led_hf;
    irs::gpio_pin_t* p_led_pon;
    irs::gpio_pin_t* p_buzzer;
  };
  cfg_t();
  irs::hardflow::simple_udp_flow_t* hardflow();
  irs::spi_t* spi();
  irs::spi_t* spi_2();
  pins_t* pins();
  inline adc_exti_t* adc_exti() { return &m_adc_exti; }
  void tick();
  
private:
  mxmac_t m_local_mac;
  irs::arm::st_ethernet_t::config_t m_config;
  irs::arm::st_ethernet_t m_arm_eth;
  mxip_t m_local_ip;
  irs_u16 m_local_port;
  mxip_t m_dest_ip;
  irs_u16 m_dest_port;
  irs::simple_tcpip_t m_tcpip;
  irs::hardflow::simple_udp_flow_t m_simple_hardflow;
  
  irs_u32 m_spi_bitrate;
  irs::arm::arm_spi_t m_spi;
  irs::arm::arm_spi_t m_spi_2;
  
  irs::arm::io_pin_t m_vben;
  irs::arm::io_pin_t m_ee_cs;
  irs::arm::io_pin_t m_adc_cs;
  irs::arm::io_pin_t m_dac_cs;
  irs::arm::io_pin_t m_dac_ldac;
  irs::arm::io_pin_t m_dac_clr;
  irs::arm::io_pin_t m_dac_reset;
  irs::arm::io_pin_t m_dac_ti_cs;
  irs::arm::io_pin_t m_relay_bridge_pos_on;
  irs::arm::io_pin_t m_relay_bridge_pos_off;
  irs::arm::io_pin_t m_relay_bridge_neg_on;
  irs::arm::io_pin_t m_relay_bridge_neg_off;
  irs::arm::io_pin_t m_relay_gain_high;
  irs::arm::io_pin_t m_relay_gain_low;
  irs::arm::io_pin_t m_relay_voltage_high;
  irs::arm::io_pin_t m_relay_voltage_low;
  irs::arm::io_pin_t m_relay_prot;
  irs::arm::io_pin_t m_led_blink;
  irs::arm::io_pin_t m_led_hf;
  irs::arm::io_pin_t m_led_pon;
  irs::pwm_pin_t m_buzzer;
  
  pins_t m_pins;
  adc_exti_t m_adc_exti;
};

} //  hrm

#endif // cfgh
