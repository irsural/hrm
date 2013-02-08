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
#include "utils.h"

#include <irsfinal.h>

namespace hrm {

class app_t
{
public:
  app_t(cfg_t* ap_cfg);
  void tick();

private:
  enum dac_test_t {
    dt_neg = irs_false,
    dt_pos = irs_true
  };
  enum etalon_polarity_t {
    ep_neg,
    ep_pos
  };
  enum balancing_coil_t {
    bc_etalon,
    bc_checked
  };
  enum free_status_t {
    fs_prepare,
    fs_wait,
    fs_idle
  };
  enum balance_status_t {
    bs_prepare,
    bs_set_etpol,
    bs_set_etpol_wait,
    bs_set_range,
    bs_set_range_wait,
    bs_coils_on,
    bs_coils_on_wait,
    bs_zero_relay_on,
    bs_zero_relay_wait,
    bs_zero_adc_start,
    bs_zero_adc_wait,
    bs_dac_prepare,
    bs_adc_start,
    bs_adc_wait,
    bs_balance,
    bs_dac_set,
    bs_dac_wait,
    bs_report
  };
  enum manual_status_t {
    ms_prepare
  };
  enum adc_mode_t {
    am_single_conversion = 1
  };
  enum adc_filter_t {
    af_4Hz = 15
  };
  enum adc_channel_t {
    ac_1 = 0
  };
  enum adc_gain_t {
    ag_1 = 0
  };
  cfg_t* mp_cfg;
  eth_data_t m_eth_data;
  irs::mxnet_t m_mxnet_server;
  cfg_t::pins_t* mp_pins;
  
  //hrm::adc_dac_request_t m_adc_dac_request;
  irs::dac_ad5791_t m_raw_dac;
  //irs::dac_ad5791_data_t m_dac_data;
  dac_t m_dac;
  irs::adc_ad7799_t m_adc;
  
  irs::loop_timer_t m_eth_timer;
  irs::loop_timer_t m_blink_timer;
  irs::timer_t m_service_timer;
  bool m_blink;
  dac_test_t m_dac_test;
  irs::loop_timer_t m_dac_timer;
  mono_relay_t m_relay_1g;
  mono_relay_t m_relay_100m;
  mono_relay_t m_relay_10m;
  mono_relay_t m_relay_1m;
  mono_relay_t m_relay_100k;
  mono_relay_t m_relay_chon;
  mono_relay_t m_relay_eton;
  mono_relay_t m_relay_prot;
  bi_relay_t m_relay_zero;
  bi_relay_t m_relay_etpol;
  
  mode_t m_mode;
  
  free_status_t m_free_status;
  
  balance_status_t m_balance_status;
  size_t m_current_iteration;
  size_t m_iteration_count;
  irs_i32 m_dac_code;
  irs_i32 m_dac_step;
  double m_checked_dac_code;
  double m_etalon_dac_code;
  irs_i32 m_initial_dac_code;
  irs_i32 m_initial_dac_step;
  irs_i32 m_adc_value;
  range_t m_range;
  irs_bool m_etalon_polarity;
  balancing_coil_t m_balancing_coil;
  double m_checked;
  double m_etalon;
  double m_result;
  double m_result_error;
  
  manual_status_t m_manual_status;
};

} //  hrm

#endif // apph
