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
    ep_neg = 0,
    ep_pos = 1
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
    bs_set_zero_relay,
    bs_start_meas_zero,
    bs_wait_meas_zero,
    bs_set_neg_coils,
    bs_set_pos_coils,
    bs_coils_wait,
    bs_dac_prepare,
    bs_adc_start,
    bs_adc_wait,
    bs_balance,
    bs_dac_set,
    bs_dac_wait,
    bs_elab_prepare,
    bs_elab_relay_on,
    bs_elab_relay_wait,
    bs_elab_meas_zero,
    bs_elab_wait_zero,
    bs_elab_dac_prepare,
    bs_elab_dac_set,
    bs_elab_adc_start,
    bs_elab_adc_wait,
    bs_coils_off,
    bs_wait_relays,
    bs_report,
    bs_next_exp,
    bs_final_report
  };
  enum manual_status_t {
    ms_prepare,
    ms_check_user_changes
  };
  enum adc_mode_t {
    am_single_conversion = 1,
    am_offset = 6,
    am_fullscale = 5
  };
  enum adc_filter_t {
    af_4Hz = 15
  };
  enum adc_channel_t {
    ac_1 = 0
  };
  enum adc_gain_t {
    m_min_adc_gain = 0,
    m_max_adc_gain = 7
  };
  enum default_params_t {
    m_default_mode = 1,
    m_default_channel = 0,
    m_default_gain = 0,
    m_default_filter = 15
  };
  struct elab_point_t {
    double dac;
    double adc;
  };
  struct exp_t {
    double result;
    double error;
    double et_code;
    double ch_code;
  };
  
  cfg_t* mp_cfg;
  eth_data_t m_eth_data;
  irs::mxnet_t m_mxnet_server;
  cfg_t::pins_t* mp_pins;
  
  irs::dac_ad5791_t m_raw_dac;
  dac_t m_dac;
  irs::adc_ad7799_t m_raw_adc;
  adc_t m_adc;
  
  irs::loop_timer_t m_eth_timer;
  irs::loop_timer_t m_blink_timer;
  irs::timer_t m_service_timer;
  bool m_blink;
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
  size_t m_elab_iteration_count;
  dac_value_t m_dac_code;
  dac_value_t m_dac_step;
  irs_i32 m_int_dac_code;
  double m_initial_dac_code;
  double m_initial_dac_step;
  range_t m_range;
  etalon_polarity_t m_etalon_polarity;
  balancing_coil_t m_balancing_coil;
  double m_checked;
  double m_etalon;
  double m_result;
  double m_result_error;
  double m_checked_code;
  double m_etalon_code;
  counter_t m_relay_after_pause;
  counter_t m_dac_after_pause;
  vector<elab_point_t> m_elab_vector;
  irs_u8 m_exp_cnt;
  vector<exp_t> m_exp_vector;
  bool m_change_coils_polarity;
  etalon_polarity_t m_current_polarity;
  bool m_no_zero_balance;
  bool m_no_zero_elab;
  
  manual_status_t m_manual_status;
  manual_status_t m_manual_target_status;
  
  inline bool can_dec_gain()
  {
    //return (m_adc_value > m_adc_gain_dec_limit && m_adc_gain > m_min_adc_gain);
    return false;
  }
  inline bool can_inc_gain()
  {
    //return (m_adc_value < m_adc_gain_inc_limit && m_adc_gain < m_max_adc_gain);
    return false;
  }
  double calc_elab_code(vector<elab_point_t>* ap_elab_vector, 
    balancing_coil_t a_balancing_coil, etalon_polarity_t a_etpol = ep_neg);
};

} //  hrm

#endif // apph
