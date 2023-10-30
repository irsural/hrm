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
  enum {
    start_div_relay = 0,
    stop_div_relay = 3
  };
  enum free_status_t {
    fs_prepare,
    fs_wait,
    fs_idle
  };
  enum balance_status_t {                    //  BRIDGE   DIV   PROT
    bs_prepare,                               //  OFF     0     1
    bs_prepare_set_relays,                    //  NEG     0     1
    bs_prepare_set_pause,
    bs_prepare_pause,    
    bs_meas_vcom_bridge_off,                  //  OFF     0     1
    bs_meas_vcom_prot_off,                    //  OFF     0     0
    bs_meas_vcom_prepare,
    bs_meas_vcom,
    bs_meas_vref_switch_on,
    bs_meas_vref_prepare,
    bs_meas_vref,
    bs_meas_vref_switch_off,
    bs_main_prot_on,                          //  OFF     0     1
    bs_main_change_polarity,                  //  ON      0     1
    bs_main_set_prot,                         //  ON      0     ETH
    bs_main_set_div_relays,                   //  ON      d     ETH
    bs_main_adc_prepare,
    bs_main_adc_read,
    bs_main_div_change,                       //  ON      d'    ETH
    bs_ending_prot_on,                        //  ON      d'    1
    bs_ending_bridge_off,                     //  OFF     0     1
    bs_ending_ready,
    bs_report,
    bs_next_exp,
    bs_final_report                           //  BRIDGE   DIVN  DIVP   PROT
  };
  enum manual_status_t {
    ms_prepare,
    ms_check_user_changes
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
    double vcom1;
    double vcom2;
    double vref1;
    double vref2;
    vector<double> v1;
    vector<double> v2;
    void clear()
    {
      vcom1 = 0.0;
      vcom2 = 0.0;
      vref1 = 0.0;
      vref2 = 0.0;
      v1.clear();
      v2.clear();
    }
  };
//  struct analog_point_t {
//    double coils_voltage;
//    double coils_sko;
//    size_t coils_num_of_points;
//    double supply_voltage;
//    double supply_sko;
//    size_t supply_num_of_points;
//    double ratio;
//    void clear()
//    {
//      coils_voltage = 0.0;
//      coils_sko = 0.0;
//      supply_voltage = 0.0;
//      supply_sko = 0.0;
//      ratio = 0.0;
//    }
//    double calc_ratio()
//    {
//      ratio = 0.0;
//      if (supply_voltage != 0.0) {
//        ratio = coils_voltage / supply_voltage;
//      }
//      return ratio;
//    }
//    void show_coils()
//    {
//      irs::mlog() << irsm("Напряжение на катушках:") << endl;
//      irs::mlog() << setprecision(7) << fixed;
//      irs::mlog() << coils_voltage << irsm(" В : ");
//      irs::mlog() << setprecision(3);
//      irs::mlog() << (coils_sko * 1e6) << irsm(" ppm : ");
//      irs::mlog() << coils_num_of_points << endl;
//    }
//    void show_supply()
//    {
//      irs::mlog() << irsm("Напряжение на источнике:") << endl;
//      irs::mlog() << setprecision(7) << fixed;
//      irs::mlog() << supply_voltage << irsm(" В : ");
//      irs::mlog() << setprecision(3);
//      irs::mlog() << (supply_sko * 1e6) << irsm(" ppm : ");
//      irs::mlog() << supply_num_of_points << endl;
//    }
//    void show_point()
//    {
//      irs::mlog() << irsm("Отношение:") << endl;
//      irs::mlog() << setprecision(7) << fixed;
//      irs::mlog() << ratio << endl;
//    }
//  };
  struct exp_t {
    double result;
    double Dn;
    double Dp;
    double K1;
    double vcom1;
    double vcom2;
    double vref1;
    double vref2;
    double temperature_ext;
    double temperature_dac;
    double temperature_adc;
    irs_u32 exp_time;
    irs_u32 errors_cnt;
  };
  struct exp_param_t {
    double ef_smooth;
    irs_u32 n_avg;
    irs_u32 t_adc;
    irs_u32 n_adc;
    double bridge_voltage;
    irs_u32 prepare_pause;
  };
  enum elab_mode_t {
    em_linear = 0,
    em_pid = 1,
    em_fast_2points = 2,
    em_none = 3,
    em_pid_linear = 4,
    em_analog = 5
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

  ad4630_t m_adc_ad4630;
  irs_u8 m_n_avg;
  irs_u16 m_t_adc;
  double m_ef_smooth;
  irs_u32 m_n_adc;
  irs_u8 m_current_div_relay;
  bridge_voltage_dac_t m_bridge_voltage_dac;

  irs::loop_timer_t m_eth_timer;
  irs::loop_timer_t m_blink_timer;
  irs::timer_t m_service_timer;
  bool m_blink;
  bi_relay_t m_relay_bridge_pos;
  bi_relay_t m_relay_bridge_neg;
  mono_relay_t m_relay_prot;
  //  Relays AD4630
  bi_relay_t m_relay_divp;
  bi_relay_t m_relay_divn;
  //  Relay DIVSW
  bi_relay_t m_relay_divsw;

  mode_t m_mode;

  free_status_t m_free_status;

  balance_status_t m_balance_status;
  size_t m_current_iteration;
  size_t m_iteration_count;
  elab_mode_t m_elab_mode;
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
  irs_u32 m_prepare_pause;
  irs_u32 m_prepare_current_time;
  analog_point_t m_analog_point;
  vector<analog_point_t> m_analog_vector;
  irs_u8 m_exp_cnt;
  vector<exp_t> m_exp_vector;
  exp_param_t m_exp_param;
  bool m_no_prot;
  bool m_wild_relays;
  double m_balanced_sko;

  manual_status_t m_manual_status;

  irs_u8 m_current_adc_point;
  irs::timer_t m_prepare_pause_timer;

  irs_u32 m_exp_time;
  irs_u32 m_prev_exp_time;
  irs_u32 m_sum_time;
  irs_u32 m_remaining_time;
  remaining_time_calculator_t m_remaining_time_calculator;
  bool m_is_exp;
  irs::loop_timer_t m_exp_timer;

  bool m_prepare_pause_completed;
  
  irs::timer_t m_relay_pause_timer;

//  buzzer_t m_buzzer;
//
//  irs::handle_t<menu_t> mp_menu;
//
//  irs::event_t m_escape_pressed_event;
  
  const counter_t m_min_after_pause;
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
    return  (m_relay_bridge_pos.status() == irs_st_ready)
          && (m_relay_bridge_neg.status() == irs_st_ready)
          && (m_relay_divp.status() == irs_st_ready)
          && (m_relay_divn.status() == irs_st_ready)
          && (m_relay_prot.status() == irs_st_ready)
          && (m_relay_divsw.status() == irs_st_ready);
  }
  inline int decode_relay_divn(irs_u8 a_current_div_relay)
  {
    return (1 & (a_current_div_relay >> 0));
  }
  inline int decode_relay_divp(irs_u8 a_current_div_relay)
  {
    return (1 & (a_current_div_relay >> 1));
  }
  //  Вывод текущих параметров эксперимента
  void show_experiment_parameters();
  void show_last_result();
  elab_mode_t convert_u8_to_elab_mode(irs_u8 a_mode);
};

} //  hrm

#endif // apph
