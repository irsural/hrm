#include <irspch.h>

#include "app.h"

#include <irslimits.h>

#include <irsfinal.h>

hrm::app_t::app_t(cfg_t* ap_cfg):
  mp_cfg(ap_cfg),
  m_eth_data(),
  m_mxnet_server(
    *mp_cfg->hardflow(), 
    m_eth_data.connect(&m_mxnet_server, 0) / sizeof(irs_u32)),
  mp_pins(mp_cfg->pins()),
  m_eth_timer(irs::make_cnt_ms(100)),
  m_blink_timer(irs::make_cnt_ms(100)),
  m_blink(false)
{
  mxip_t ip = mxip_t::zero_ip();
  ip.val[0] = IP_0;
  ip.val[1] = IP_1;
  ip.val[2] = IP_2;
  ip.val[3] = IP_3;

  char ip_str[IP_STR_LEN];
  mxip_to_cstr(ip_str, ip);
  mp_cfg->hardflow()->set_param("local_addr", ip_str);
}

void hrm::app_t::tick()
{ 
  mp_cfg->tick();
  m_mxnet_server.tick();
  
  if (m_eth_timer.check()) {
    m_eth_data.counter++;
  }
  if (m_blink_timer.check()) {
    if (m_blink) {
      mp_pins->p_led_blink->set();
    } else {
      mp_pins->p_led_blink->clear();
    }
    m_blink = !m_blink;
  }
}
