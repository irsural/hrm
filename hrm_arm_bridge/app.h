#ifndef apph
#define apph

#include <irsdefs.h>

#include <irsmbus.h>
#include <irsadc.h>
#include <irsdsp.h>
#include <mxnet.h>
#include <irsalg.h>
#include <irslimits.h>

#include <hrm_bridge_data.h>

#include "cfg.h"
#include "privatecfg.h"

#include <irsfinal.h>

namespace hrm {

class app_t
{
public:
  app_t(cfg_t* ap_cfg);
  void tick();

private:
  cfg_t* mp_cfg;
  eth_data_t m_eth_data;
  irs::mxnet_t m_mxnet_server;
  cfg_t::pins_t* mp_pins;
  
  irs::adc_ad7799_t m_adc;
  
  irs::loop_timer_t m_eth_timer;
  irs::loop_timer_t m_blink_timer;
  bool m_blink;
};

} //  hrm

#endif // apph
