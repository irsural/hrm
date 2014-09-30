#ifndef apph
#define apph

#include <irsdefs.h>

#include <irsmbus.h>
#include <irsadc.h>
#include <irsdsp.h>
#include <mxnet.h>
#include <irsalg.h>
#include <irslimits.h>
#include <irsmem.h>
#include <irsmenu.h>

#include <hrm_bridge_data.h>

#include "cfg.h"
#include "privatecfg.h"
#include "utils.h"
#include "menu.h"

#include <irsfinal.h>

namespace hrm {

class init_eeprom_t //класс для инициализации eeprom
{
public:
  init_eeprom_t(irs::eeprom_at25128_data_t* ap_eeprom,
    eeprom_data_t* ap_eeprom_data = IRS_NULL);
  ~init_eeprom_t();
};

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
  enum balance_polarity_t {
    bp_neg = 0,
    bp_pos = 1
  };
  enum free_status_t {
    fs_prepare,
    fs_wait,
    fs_idle
  };
  enum balance_status_t {
    bs_prepare,
    bs_set_prot,
    bs_set_coils,
    bs_coils_wait,
    bs_coils_relay_pause,
    bs_set_pause,
    bs_pause,
    bs_adc_show,
    bs_meas_temperature,
    bs_wait_temperature,
    bs_dac_prepare,
    bs_adc_start,
    bs_adc_wait,
    bs_adc_average,
    bs_balance,
    bs_dac_set,
    bs_dac_wait,
    bs_elab_prepare,
    bs_elab_start,
    bs_elab_relay_on,
    bs_elab_relay_wait,
    bs_elab_dac_set,
    bs_elab_adc_start,
    bs_elab_adc_wait,
    bs_elab_result,
    bs_coils_off,
    bs_wait_relays,
    bs_report,
    bs_next_exp,
    bs_final_report
  };
  enum manual_status_t {
    ms_prepare,
    ms_check_user_changes,
    ms_adc_show,
    ms_adc_hide
  };
  enum scan_status_t {
    ss_prepare,
    ss_on,
    ss_dac_prepare,
    ss_first_dac_set,
    ss_wait,
    ss_dac_set,
    ss_start_adc,
    ss_adc_wait,
    ss_relay_off,
    ss_relay_wait
  };
  enum adc_mode_t {
    am_single_conversion = 1,
    am_offset = 6,
    am_fullscale = 5
  };
  enum adc_filter_t {
    af_4Hz = 15,
    af_80db = 9,
    m_fast_adc_filter = 1,
    m_slow_adc_filter = 15
  };
  enum adc_channel_t {
    ac_1 = 0,
    ac_voltage = 0,
    ac_temperature = 1
  };
  enum adc_gain_t {
    m_min_adc_gain = 0,
    m_max_adc_gain = 7
  };
  enum default_params_t {
    m_default_mode = 1,
    m_default_channel = 0,
    m_default_gain = 0,
    m_default_filter = 9//15
  };
  struct elab_point_t {
    double dac;
    double adc;
    double sko;
    double avg;
  };
  struct exp_t {
    double result;
    double error;
    double et_code;
    double ch_code;
  };
  struct elab_result_t {
    balance_polarity_t polarity;
    double start_code;
    double code;
  };

  cfg_t* mp_cfg;
  eth_data_t m_eth_data;
  irs::mxnet_t m_mxnet_server;

  irs::eeprom_at25128_data_t m_eeprom;
  eeprom_data_t m_eeprom_data;
  init_eeprom_t m_init_eeprom;

  mxdisplay_drv_gen_t m_lcd_drv;

  irs::mxkey_drv_mc_t m_keyboard_drv;

  irs::encoder_drv_mc_t m_encoder_drv;

  mxdisplay_drv_service_t m_lcd_drv_service;
  mxkey_event_t m_buzzer_kb_event;
  mxkey_event_t m_hot_kb_event;
  mxkey_event_t m_menu_kb_event;
  mxkey_event_gen_t m_keyboard_event_gen;

  irs::dac_ad5791_t m_raw_dac;
  dac_t m_dac;
  ad7799_cread_t m_adc;

  irs::th_lm95071_t m_ext_th;
  irs::th_lm95071_data_t m_ext_th_data;

  irs::loop_timer_t m_eth_timer;
  irs::loop_timer_t m_blink_timer;
  irs::timer_t m_service_timer;
  bool m_blink;
  bi_relay_t m_relay_bridge_pos;
  bi_relay_t m_relay_bridge_neg;
  mono_relay_t m_relay_prot;

  mode_t m_mode;

  free_status_t m_free_status;

  balance_status_t m_balance_status;
  size_t m_current_iteration;
  size_t m_iteration_count;
  size_t m_elab_iteration_count;
  irs_i32 m_elab_step;
  dac_value_t m_dac_code;
  dac_value_t m_dac_step;
  dac_value_t m_balanced_dac_code;
  dac_value_t m_initial_dac_code;
  dac_value_t m_initial_dac_step;
  balance_polarity_t m_balance_polarity;
  double m_checked;
  double m_etalon;
  double m_result;
  double m_result_error;
  double m_checked_code;
  double m_etalon_code;
  counter_t m_relay_after_pause;
  counter_t m_dac_after_pause;
  counter_t m_dac_elab_pause;
  irs_u32 m_prepare_pause;
  irs_u32 m_prepare_current_time;
  vector<elab_point_t> m_elab_vector;
  irs_u8 m_exp_cnt;
  vector<exp_t> m_exp_vector;
  bool m_no_prot;
  bool m_wild_relays;
  double m_balanced_sko;
  irs_u32 m_adc_experiment_gain;
  irs_u32 m_adc_experiment_filter;

  manual_status_t m_manual_status;

  scan_status_t m_scan_status;
  dac_value_t m_dac_center_scan;
  irs_u8 m_current_adc_point;
  irs::timer_t m_prepare_pause_timer;

  irs_u32 m_exp_time;
  irs_u32 m_prev_exp_time;
  irs_u32 m_sum_time;
  irs_u32 m_remaining_time;
  bool m_is_exp;
  irs::loop_timer_t m_exp_timer;
  bool m_optimize_balance;

  irs::fade_data_t m_adc_fade_data;
  adc_value_t m_adc_fade;
  adc_value_t m_voltage;
  adc_value_t m_temperature;

  bool m_meas_temperature;
  adc_value_t m_max_unsaturated_voltage;
  dac_value_t m_max_unsaturated_dac_code;
  bool m_auto_elab_step;
  adc_value_t m_dac_step_amplitude;
  size_t m_current_elab;
  size_t m_pos_current_elab;
  size_t m_min_elab_cnt;
  size_t m_max_elab_cnt;
  size_t m_ok_elab_cnt;
  vector<elab_result_t> m_elab_result_vector;
  vector<elab_point_t> m_prev_elab_vector;
  size_t m_prev_elab_cnt;
  balance_polarity_t m_elab_polarity;
  dac_value_t m_elab_step_multiplier;
  double m_elab_max_delta;

  irs::timer_t m_relay_pause_timer;

  buzzer_t m_buzzer;

  irs::handle_t<menu_t> mp_menu;

  irs::event_t m_escape_pressed_event;

  r_standard_type_t m_r_standard_type;


//  double calc_elab_code(vector<elab_point_t>* ap_elab_vector,
//    balancing_coil_t a_balancing_coil, etalon_polarity_t a_etpol = ep_neg);
  void init_keyboard_drv();
  void init_encoder_drv();
  void print_elab_result();
  void reset_network_config();
  double only_calc_elab_code(vector<elab_point_t>* ap_elab_vector,
    size_t a_num, size_t a_cnt);
  void print_elab_result(vector<elab_point_t>* ap_elab_vector,
    size_t a_num, size_t a_cnt);
  inline bool bridge_relays_ready()
  {
    return (m_relay_bridge_pos.status() == irs_st_ready)
      && (m_relay_bridge_neg.status() == irs_st_ready);
  }
  void print_voltage(adc_value_t a_value);
};

} //  hrm

#endif // apph
