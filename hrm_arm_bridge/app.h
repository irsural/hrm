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
#include <irsdsp.h>

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
  app_t(cfg_t* ap_cfg, version_t a_version, bool* ap_buf_ready = 0);
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
    bs_wait_adc_dac,
    bs_set_prot,
    bs_set_coils,
    bs_coils_wait,
    bs_coils_relay_pause,
    bs_wait_bridge_voltage_dac,
    bs_preset_dac_prepare,
    bs_preset_dac_adc_start,
    bs_preset_additional_pause_wait,
    bs_preset_dac_adc_wait,
    bs_preset_dac_meas_sensivity,
    bs_preset_dac_calc_sensivity,
    bs_prev_balance_prepare,
    bs_prev_balance_prepare_adc_wait,
    bs_prev_balance_dac_prepare,
    bs_prev_balance_adc_start,
    bs_prev_balance_adc_wait,
    bs_prev_balance,
    bs_prev_balance_dac_set,
    bs_prev_balance_dac_wait,
    bs_pid_prepare,
    bs_pid_process_dac,
    bs_pid_process_adc,
    bs_pid_elab_dac_1,
    bs_pid_elab_adc_1,
    bs_pid_elab_dac_2,
    bs_pid_elab_adc_2,
    bs_pid_result_processing,
    bs_set_pause,
    bs_pause,
    bs_adc_show,
    bs_adc_prepare,
    //
    bs_analog_adc_start_coils,
    bs_analog_adc_wait_coils,
    bs_analog_relay_switch_supply,
    bs_analog_relay_wait_supply,
    bs_analog_wait_repeat_dac,
    bs_analog_adc_start_supply,
    bs_analog_adc_wait_supply,
    bs_analog_half_report,
    bs_analog_dac_wait,
    //
    bs_dac_prepare,
    bs_termostat_off_adc_start,
    bs_adc_start,
    bs_adc_wait,
    //bs_adc_average,
    bs_balance,
    bs_dac_set,
    bs_termostat_off_dac_wait,
    bs_dac_wait,
    bs_none_elab_prepare,
    bs_fast_elab_prepare,
    bs_fast_elab_dac_set,
    bs_fast_elab_adc_start,
    bs_fast_elab_adc_read,
    bs_fast_elab_point_processing,
    bs_fast_elab_result,
    bs_elab_prepare,
    bs_elab_start,
    bs_elab_relay_on,
    bs_elab_relay_wait,
    bs_elab_dac_set,
    bs_termostat_off_elab_adc_start,
    bs_elab_adc_start,
    bs_elab_adc_wait,
    bs_elab_result,
    bs_coils_off,
    bs_wait_relays,
    bs_change_bridge_polarity,
    bs_report,
    bs_next_exp,
    bs_wait_dac_before_switching,
    bs_final_report
  };
  enum manual_status_t {
    ms_prepare,
    ms_adc_setup,
    ms_check_user_changes,
    ms_adc_show,
    ms_adc_hide,
    ms_pid_start,
    ms_pid_process,
    ms_pid_reset,
    ms_pid_reset_wait
    //ms_adc_continious_start,
    //ms_adc_continious
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
    size_t num_of_adc_points;
  };
  struct analog_point_t {
    double coils_voltage;
    double coils_sko;
    size_t coils_num_of_points;
    double supply_voltage;
    double supply_sko;
    size_t supply_num_of_points;
    double ratio;
    void clear()
    {
      coils_voltage = 0.0;
      coils_sko = 0.0;
      supply_voltage = 0.0;
      supply_sko = 0.0;
      ratio = 0.0;
    }
    double calc_ratio()
    {
      ratio = 0.0;
      if (supply_voltage != 0.0) {
        ratio = coils_voltage / supply_voltage;
      }
      return ratio;
    }
    void show_coils()
    {
      irs::mlog() << irsm("Напряжение на катушках:") << endl;
      irs::mlog() << setprecision(7) << fixed;
      irs::mlog() << coils_voltage << irsm(" В : ");
      irs::mlog() << setprecision(3);
      irs::mlog() << (coils_sko * 1e6) << irsm(" ppm : ");
      irs::mlog() << coils_num_of_points << endl;
    }
    void show_supply()
    {
      irs::mlog() << irsm("Напряжение на источнике:") << endl;
      irs::mlog() << setprecision(7) << fixed;
      irs::mlog() << supply_voltage << irsm(" В : ");
      irs::mlog() << setprecision(3);
      irs::mlog() << (supply_sko * 1e6) << irsm(" ppm : ");
      irs::mlog() << supply_num_of_points << endl;
    }
    void show_point()
    {
      irs::mlog() << irsm("Отношение:") << endl;
      irs::mlog() << setprecision(7) << fixed;
      irs::mlog() << ratio << endl;
    }
  };
  struct sensivity_data_t {
    double dac_1;
    double adc_1;
    double dac_2;
    double adc_2;
    double sensivity;
    bool empty;
    bool ready;
    void clear()
    {
      dac_1 = 0.0;
      adc_1 = 0.0;
      dac_2 = 0.0;
      adc_2 = 0.0;
      sensivity = 0.0;
      ready = false;
      empty = true;
    };
    void calc()
    {
      double div = adc_2 - adc_1;
      if (div != 0.0 && (empty == false)) {
        sensivity = abs((dac_2 - dac_1) / div);
        irs::mlog() << irsm("Измеренная чувствительность ");
        irs::mlog() << setprecision(8);
        irs::mlog() << sensivity;
        irs::mlog() << irsm(" шаг/В") << endl;
        ready = true;
      }
    };
    void add(double adc, double dac) {
      if (empty) {
        adc_1 = adc;
        dac_1 = dac;
        empty = false;
      } else {
        adc_2 = adc;
        dac_2 = dac;
        ready = true;
      }
    };
  };
  struct elab_data_t {
    double dac_1;
    double adc_1;
    double dac_2;
    double adc_2;
    double calc_zero()
    {
      double div = dac_2 - dac_1;
      double result = 0.0;
      if (div != 0.0) {
        double k = (adc_2 - adc_1) / div;
        double b = adc_2 - k * dac_2;
        if (k != 0.0) {
          result = -b / k;
        }
      }
      return result;
    }
  };
  struct exp_t {
    double result_old;
    double et_code;
    double ch_code;
    double et_balanced_code;
    double ch_balanced_code;
    double temperature_ext;
    double temperature_dac;
    double temperature_adc;
    irs_u32 exp_time;
    double target_sko;
    double target_balance_sko;
    double target_elab_sko;
    double target_sko_adc_neg;
    double target_sko_adc_pos;
    double target_sko_dac_neg;
    double target_sko_dac_pos;
    irs_u32 neg_n;
    irs_u32 pos_n;
    double coils_voltage_neg;
    double coils_sko_neg;
    irs_u32 coils_n_neg;
    double source_voltage_neg;
    double source_sko_neg;
    irs_u32 source_n_neg;
    double coils_voltage_pos;
    double coils_sko_pos;
    irs_u32 coils_n_pos;
    double source_voltage_pos;
    double source_sko_pos;
    irs_u32 source_n_pos;
    irs_u32 errors_cnt;
  };
  struct elab_result_t {
    balance_polarity_t polarity;
    double start_code;
    double start_elab_code;
    double code;
    double target_adc_sko;
    double dac_sko;
    irs_u32 num_op_iterations;
  };
  enum elab_mode_t {
    em_linear = 0,
    em_pid = 1,
    em_fast_2points = 2,
    em_none = 3,
    em_pid_linear = 4,
    em_analog = 5
  };
  enum {
    default_balance_points = 20,
    default_elab_points = 2
  };
  class remaining_time_calculator_t {
    enum {
      default_exp_time = 300,
      default_prepare_pause_percentage = 20,
      default_balance_percentage = 20,
      default_elab_percentage = 
        ((100 - (2 * default_balance_percentage) - 
          default_prepare_pause_percentage) / 2),
      min_balance_neg = default_prepare_pause_percentage,
      max_balance_neg = default_prepare_pause_percentage
        + default_balance_percentage,
      min_elab_neg = max_balance_neg,
      max_elab_neg = max_balance_neg + default_elab_percentage,
      min_balance_pos = max_elab_neg,
      max_balance_pos = min_balance_pos + default_balance_percentage,
      min_elab_pos = max_balance_pos,
      max_elab_pos = 100,
      max_balance_points_count = 20,
      elab_points_count = 2
    };
    irs_u32 m_meas_prepare_time;
    irs_u32 m_meas_balance_time;
    irs_u32 m_meas_elab_time;
    balance_action_t m_balance_action;
    irs_u32 m_remaining_time;
    irs_u32 m_current_time;
    irs_u32 m_current_percentage;
    irs_u32 m_prepare_pause;
    size_t m_balance_points_count;
    size_t m_balance_current_point;
  public:
    remaining_time_calculator_t();
    void reset();
    void start(irs_u32 a_prepare_pause);
    void secund_tick();
    irs_u32 get_remaining_time();
    irs_u32 get_percentage();
    void change_balance_action(balance_action_t a_balance_action);
    void set_balance_points_count(size_t a_balance_points_count);
    void set_current_balance_point(size_t a_point);
  };
  class adaptive_sko_calc_t {
    enum {
      default_len = 10
    };
    eth_data_t& mp_eth_data;
    eeprom_data_t& mp_ee_data;
    adc_value_t m_target_sko;
    bool m_started;
    bool m_used;
    adc_value_t m_target_balance_sko;
    adc_value_t m_target_elab_sko;
    adc_value_t m_adaptive_sko_balance_multiplier;
    adc_value_t m_adaptive_sko_elab_multiplier;
    irs::fast_sko_t<adc_value_t, adc_value_t> m_sko_calc;
  public:
    adaptive_sko_calc_t(eth_data_t& ap_eth_data, eeprom_data_t& ap_ee_data);
    void reset(size_t a_len);
    void stop();
    void add(adc_value_t a_sko);
    adc_value_t get_target_sko();
    adc_value_t get_target_balance_sko();
    adc_value_t get_target_elab_sko();
    void sync_parameters();
    inline bool used() { return m_used; }
  };

  cfg_t* mp_cfg;
  eth_data_t m_eth_data;
  bool* mp_buf_ready;
  
  version_t m_version;
  
  buzzer_t m_buzzer;
  
  mxdisplay_drv_gen_t m_lcd_drv;

  irs::mxkey_drv_mc_t m_keyboard_drv;

  //irs::encoder_drv_mc_t m_encoder_drv;

  mxdisplay_drv_service_t m_lcd_drv_service;
  mxkey_event_t m_buzzer_kb_event;
  mxkey_event_t m_hot_kb_event;
  mxkey_event_t m_menu_kb_event;
  mxkey_event_gen_t m_keyboard_event_gen;

  irs::handle_t<menu_t> mp_menu;

  irs::event_t m_escape_pressed_event;
  
  irs::mxnet_t m_mxnet_server;
  bool m_show_network_params;

  irs::eeprom_at25128_data_t m_eeprom;
  eeprom_data_t m_eeprom_data;
  init_eeprom_t m_init_eeprom;

  irs::dac_ad5791_t m_raw_dac;
  dac_t m_dac;
  ad7799_cread_t m_adc;
  ad4630_t m_adc_ad4630;
  irs_u8 m_n_avg;
  irs_u16 m_t_adc;
  double m_ef_smooth;
  bridge_voltage_dac_t m_bridge_voltage_dac;

  irs::loop_timer_t m_eth_timer;
  irs::loop_timer_t m_blink_timer;
  irs::timer_t m_service_timer;
  bool m_blink;
  bi_relay_t m_relay_bridge_pos;
  bi_relay_t m_relay_bridge_neg;
  //mono_relay_t m_relay_bridge_pos;
  //mono_relay_t m_relay_bridge_neg;
  mono_relay_t m_relay_prot;
  //  HV Relays
//  mono_relay_t m_relay_hv_polarity;
//  mono_relay_t m_relay_hv_amps_gain;
//  bi_relay_t m_relay_hv_pos;
//  bi_relay_t m_relay_hv_neg;
//  bi_relay_t m_relay_adc_src;
  //  Relays AD4630
  bi_relay_t m_relay_divp;
  bi_relay_t m_relay_divn;

  mode_t m_mode;

  free_status_t m_free_status;

  balance_status_t m_balance_status;
  size_t m_current_iteration;
  size_t m_iteration_count;
  elab_mode_t m_elab_mode;
  size_t m_elab_iteration_count;
  irs_i32 m_elab_step;
  dac_value_t m_dac_code;
  dac_value_t m_dac_step;
  dac_value_t m_balanced_dac_code;
  dac_value_t m_initial_dac_code;
  const dac_value_t m_default_initial_dac_step;
  dac_value_t m_initial_dac_step;
  dac_value_t m_start_elab_code;
  balance_polarity_t m_balance_polarity;
  double m_checked;
  double m_etalon;
  double m_result;
  double m_result_error;
  double m_checked_code;
  double m_checked_balanced_code;
  double m_etalon_code;
  double m_etalon_balanced_code;
  counter_t m_relay_after_pause;
  counter_t m_dac_after_pause;
  counter_t m_dac_elab_pause;
  irs_u32 m_prepare_pause;
  irs_u32 m_prepare_current_time;
  vector<elab_point_t> m_elab_vector;
  vector<elab_point_t> m_fast_elab_vector;
  vector<analog_point_t> m_analog_vector;
  analog_point_t m_analog_point;
  adc_value_t m_fast_elab_dac_step;
  adc_value_t m_fast_elab_dac_step_2;
  adc_value_t m_fast_elab_dac_step_3;
  irs_u8 m_exp_cnt;
  vector<exp_t> m_exp_vector;
  bool m_no_prot;
  bool m_wild_relays;
  double m_balanced_sko;
  //irs_u32 m_adc_experiment_gain;
  //irs_u32 m_adc_experiment_filter;
  const double m_default_sensivity;
  double m_imm_coef;
  double m_pid_sensivity;

  manual_status_t m_manual_status;

  scan_status_t m_scan_status;
  dac_value_t m_dac_center_scan;
  irs_u8 m_current_adc_point;
  irs::timer_t m_prepare_pause_timer;

  irs_u32 m_exp_time;
  irs_u32 m_prev_exp_time;
  irs_u32 m_sum_time;
  irs_u32 m_remaining_time;
  remaining_time_calculator_t m_remaining_time_calculator;
  bool m_is_exp;
  irs::loop_timer_t m_exp_timer;
  bool m_optimize_balance;

  irs::fade_data_t m_adc_fade_data;
  adc_value_t m_adc_fade;
  adc_value_t m_voltage;
  adc_value_t m_temperature;

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
  bool m_prev_balance_completed;
  bool m_prepare_pause_completed;
  
  //  Параметры АЦП в режиме ожидания при измерении напряжения
  adc_param_data_t m_adc_free_vx_param_data;
  bool m_new_adc_param_free_vx;
  //  Параметры АЦП в режиме ПИД-регулятора
  adc_param_data_t m_adc_pid_param_data;
  bool m_new_adc_param_pid;
  bool m_new_reg_param_pid;
  //  Параметры АЦП в ручном режиме
  adc_param_data_t m_adc_manual_param_data;
  bool m_new_adc_param_manual;
  //  Параметры АЦП в режиме измерения при уравновешивании
  adc_param_data_t m_adc_balance_param_data;
  adc_param_data_t m_adc_adaptive_balance_param_data;
  bool m_new_adc_param_balance;
  //  Параметры АЦП в режиме измерения при уточнении
  adc_param_data_t m_adc_elab_param_data;
  adc_param_data_t m_adc_adaptive_elab_param_data;
  bool m_new_adc_param_elab;
  //  Результаты измерения АЦП
  adc_result_data_t m_adc_result_data;

  irs::timer_t m_relay_pause_timer;

//  buzzer_t m_buzzer;
//
//  irs::handle_t<menu_t> mp_menu;
//
//  irs::event_t m_escape_pressed_event;
  
  //termostat_t m_termostat;
  irs::pid_data_t m_elab_pid;
  eth_pid_data_t m_eth_pid_data;
  bool m_elab_pid_on;
  irs_u16 m_elab_pid_min_time;
  irs_u16 m_elab_pid_max_time;
  double m_elab_pid_sko_meas_time;
  double m_elab_pid_kp;
  double m_elab_pid_ki;
  double m_elab_pid_kd;
  double m_elab_pid_td;
  irs::isodr_data_t m_elab_iso;
  double m_elab_iso_k;
  double m_elab_iso_t;
  const counter_t m_min_after_pause;
  double m_elab_pid_fade_t;
  size_t m_elab_pid_avg_cnt;
  irs::fade_data_t m_elab_pid_fade;
  irs::fast_sko_t<double, double> m_elab_pid_sko;
  //static_sko_t<irs_i32, adc_value_t, irs_i64, 1024> m_elab_pid_sko;
  double m_elab_pid_target_sko;
  double m_elab_pid_target_sko_norm;
  double m_elab_pid_ref;
  const irs_u32 m_elab_pid_max_iteration;
  sensivity_data_t m_elab_pid_sensivity_data;
  elab_data_t m_pid_elab_data;
  bool m_pid_meas_sensivity;
  bool m_pid_min_time_passed;
  counter_t m_pid_min_time;
  counter_t m_pid_limit_time;
  irs::timer_t m_pid_limit_timer;
  pid_ready_condition_t m_pid_ready_condition;
  double m_pid_adc_target_sko;
  double m_actual_sko_meas_time;
  size_t m_actual_cnv_cnt;
  adc_value_t m_adc_max_value_prot;
  adc_value_t m_adc_max_value_no_prot;
  double m_bac_old_coefficient;
  double m_bac_new_coefficient;
  irs_i32 m_bac_new_int_coefficient;
  irs_i32 m_bac_new_int_multiplier;
  //
  double m_dac_hv_correction;
  //
  /*const */double m_min_bridge_voltage;
  /*const */double m_max_bridge_voltage;
  /*const */double m_min_bridge_voltage_speed;
  /*const */double m_max_bridge_voltage_speed;
  /*const */double m_min_bridge_voltage_after_pause_ms;
  /*const */double m_max_bridge_voltage_after_pause_ms;
  double m_bridge_voltage;
  double m_bridge_voltage_reduced;
  double m_bridge_voltage_speed;
  double m_bridge_voltage_after_pause_ms;
  bool m_bridge_voltage_reduce_after_switching;
  //
  device_condition_controller_t m_device_condition_controller;
  //
  const double m_treg_operating_duty_time_interval_s;
  const double m_treg_operating_duty_deviation;
  const double m_treg_pwm_max_code_float;
  const double m_treg_polarity_map;
  const double m_treg_temperature_setpoint;
  temperature_sensor_conn_data_t m_treg_termosensor;
  peltier_t::parameters_t m_treg_peltier_parameters;
  peltier_t m_treg_peltier;
  sync_treg_parameters_t m_treg_sync_parameters;
  //
  const double m_treg_dac_operating_duty_time_interval_s;
  const double m_treg_dac_operating_duty_deviation;
  const double m_treg_dac_pwm_max_code_float;
  const double m_treg_dac_polarity_map;
  const double m_treg_dac_temperature_setpoint;
  temperature_sensor_conn_data_t m_treg_dac_termosensor;
  peltier_t::parameters_t m_treg_dac_peltier_parameters;
  peltier_t m_treg_dac_peltier;
  sync_treg_parameters_t m_treg_dac_sync_parameters;
  
  balance_action_t m_balance_action;
  adaptive_sko_calc_t m_adaptive_sko_calc;

  void init_keyboard_drv();
  void init_encoder_drv();
  void print_elab_result();
  void reset_network_config();
  void network_config_to_eth_data();
  double only_calc_elab_code(vector<elab_point_t>* ap_elab_vector,
    size_t a_num, size_t a_cnt);
  double calc_elab_code_2point(vector<elab_point_t>* ap_elab_vector,
    size_t a_shift);
  void print_elab_result(vector<elab_point_t>* ap_elab_vector,
    size_t a_num, size_t a_cnt);
  inline bool bridge_relays_ready()
  {
    return (m_relay_bridge_pos.status() == irs_st_ready)
      && (m_relay_bridge_neg.status() == irs_st_ready)
      && (m_relay_divp.status() == irs_st_ready)
      && (m_relay_divn.status() == irs_st_ready);
//      && (m_relay_hv_pos.status() == irs_st_ready)
//      && (m_relay_hv_neg.status() == irs_st_ready)
//      && (m_relay_adc_src.status() == irs_st_ready);
  }
  void print_voltage(adc_value_t a_value);
  void update_elab_pid_koefs(double a_td, double a_adc, 
    double a_dac, double a_sens);
  dac_value_t norm(dac_value_t a_in);
  dac_value_t denorm(dac_value_t a_in);
  
  //  Начальная загрузка параметров из eeprom в структуры АЦП и Ethernet
  void adc_params_load_from_eeprom();
  //  Передача текущих параметров АЦП в Ethernet
  void adc_params_translate_actual_to_eth();
  //  Проверка изменения параметров в Ethernet и сохраниение в структуры и EEPRO
  bool adc_params_recieve_and_save_free_vx(); // Канал напряжения в режиме free
  bool adc_params_recieve_and_save_pid();     // Напряжение в режиме ПИД-регулятора
  bool reg_params_recieve_and_save_pid();     // Параметры ПИД-регулятора
  bool adc_params_recieve_and_save_manual();  // Напряжение в ручном режиме 
  bool adc_params_recieve_and_save_balance(); // Напряжение при уравновешивании
  bool adc_params_recieve_and_save_elab();    // Напряжение при уточнении
  //  Преобразование показаний АЦП в температуру с датчика MCP9701
  adc_value_t adc_vx_to_th(adc_value_t a_voltage);
  //  Вывод текущих параметров эксперимента
  void show_experiment_parameters();
  void show_experiment_parameters_pid();
  void show_experiment_parameters_pid_linear();
  void show_last_result();
  elab_mode_t convert_u8_to_elab_mode(irs_u8 a_mode);
  void show_pid_params();
};

} //  hrm

#endif // apph
