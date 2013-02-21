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
    fs_adc_set_default_filter,
    fs_adc_set_default_channel,
    fs_adc_set_default_gain,
    fs_adc_set_default_mode,
    fs_adc_wait,
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
    bs_zero_dac_on,
    bs_zero_set_adc_channel_2,
    bs_zero_start_adc_channel_2,
    bs_zero_wait_adc_channel_2,
    bs_zero_set_adc_channel_3,
    bs_zero_start_adc_channel_3,
    bs_zero_wait_adc_channel_3,
    bs_zero_relay_on,
    bs_zero_relay_wait,
    bs_zero_adc_set_channel,
    bs_zero_adc_set_gain,
    bs_zero_adc_set_mode_offset,
    bs_zero_adc_start_offset,
    bs_zero_adc_read_offset,
    bs_zero_adc_wait_offset,
    bs_zero_adc_set_mode_fullscale,
    bs_zero_adc_start_fullscale,
    bs_zero_adc_read_fullscale,
    bs_zero_adc_wait_fullscale,
    bs_zero_adc_set_mode_single,
    bs_zero_adc_wait,
    bs_dac_prepare,
    bs_adc_start,
    bs_adc_wait,
    bs_balance,
    bs_coil_change_prepare,
    bs_coil_change,
    bs_dac_set,
    bs_dac_wait,
    bs_elab_prepare,
    bs_elab_set_adc_channel_2,
    bs_elab_start_adc_channel_2,
    bs_elab_wait_adc_channel_2,
    bs_elab_set_adc_channel_3,
    bs_elab_start_adc_channel_3,
    bs_elab_wait_adc_channel_3,
    bs_elab_set_adc_channel_1,
    bs_elab_wait_adc_channel_1,
    bs_elab_relay_on,
    bs_elab_relay_wait,
    bs_elab_adc_set_mode_offset,
    bs_elab_adc_start_offset,
    bs_elab_adc_read_offset,
    bs_elab_adc_wait_offset,
    bs_elab_adc_set_mode_single,
    bs_elab_dac_prepare,
    bs_elab_dac_set,
    bs_elab_adc_start,
    bs_elab_adc_wait,
    bs_report
  };
  enum manual_status_t {
    ms_prepare,
    ms_check_user_changes,
    ms_adc_set_filter,
    ms_adc_set_mode,
    ms_adc_set_channel,
    ms_adc_set_gain,
    ms_adc_start,
    ms_adc_get_value
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
  struct elab_point_t {
    double dac;
    double adc;
  };
  
  cfg_t* mp_cfg;
  eth_data_t m_eth_data;
  irs::mxnet_t m_mxnet_server;
  cfg_t::pins_t* mp_pins;
  
  irs::dac_ad5791_t m_raw_dac;
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
  balance_status_t m_balance_target_status;
  size_t m_current_iteration;
  size_t m_iteration_count;
  dac_value_t m_dac_code;
  dac_value_t m_dac_step;
  irs_i32 m_int_dac_code;
  double m_checked_dac_code;
  double m_etalon_dac_code;
  double m_initial_dac_code;
  double m_initial_dac_step;
  adc_value_t m_adc_value;
  adc_value_t m_adc_value_ch2;
  adc_value_t m_adc_value_ch3;
  range_t m_range;
  irs_bool m_etalon_polarity;
  balancing_coil_t m_balancing_coil;
  double m_checked;
  double m_etalon;
  double m_result;
  double m_result_error;
  irs_u8 m_adc_gain;
  irs_u8 m_adc_channel;
  irs_u8 m_adc_mode;
  irs_u8 m_adc_filter;
  int m_adc_offset;
  int m_adc_fullscale;
  irs_i32 m_adc_gain_inc_limit;
  irs_i32 m_adc_gain_dec_limit;
  counter_t m_relay_after_pause;
  counter_t m_dac_after_pause;
  vector<elab_point_t> m_elab_vector;
  
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
};

} //  hrm

#endif // apph
