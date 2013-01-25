#ifndef APP_H
#define APP_H

#include <irsdefs.h>

#include <irsmbus.h>
#include <measmul.h>
#include <timer.h>
#include <irsstring.h>
#include <irscpp.h>

#include <hrm_defs.h>
#include "cfg.h"
#include "hrm_tbi_data.h"
#include "process.h"

#include <irsfinal.h>

namespace hrm {

class app_t {
public:
  struct balance_params_t {
    double pause;
    bool adaptive;
    irs_u8 meas_count;
    irs_u8 exp_count;
    double etalon_value;
    double checked_value;
    balance_mode_t balance_mode;
    bool subtract_bias;
    bool check_weight;
    double nplc;
    size_t lsb_count;
    size_t lsb_meas_count;
    size_t buffer_test_digits;
    double source_voltage;
    status_t target_action;
    size_t r2r_scan_start_digit;
    size_t r2r_scan_digits;
  };

  app_t(cfg_t& a_cfg);
  ~app_t();
  void tick();
  //  Main Memo
  bool get_new_log_line(irs::string_t* ap_line);
  bool get_new_report_line(irs::string_t* ap_line);
  //  Multimeter Box
  bool get_multimeter_value(irs::string_t* ap_value);
  bool get_multimeter_box_enabled();
  //  Comparator in Multimeter Box
  bool get_r2r_comparator_value(bool* ap_value);
  //  Balance Box
  bool get_balance_box_enabled();
  bool get_balance_box_data(balance_params_t* ap_params);
  bool get_calibre_vector(vector<double>* ap_calibre_vector,
    bool* ap_exist_bias);
  void set_calibre_vector(vector<double>* ap_calibre_vector);
  void action_button_pressed(balance_params_t* ap_params);
  //  Code Box
  void code_set_button_pressed(irs_u32 a_code);
  bool get_code_spin_box_value(irs_u32* ap_value);
  bool get_code_box_enabled();
  //  Led
  void led_check_box_pressed(bool a_checked);
  bool get_led_check_box_checked(bool* ap_checked);
  bool get_led_enabled();
  //  Abort Button
  void abort_button_pressed();
  //  Connected Label
  bool get_connected_label_text(irs::string_t* ap_text);
  bool get_connected_label_enabled();
  //  Core Temperature Label
  bool get_core_temperature_label_text(irs::string_t* ap_text);
  bool get_core_temperature_label_enabled();
  //  Time
  bool get_time_value(int *ap_time);
  //
private:
  enum eth_status_t {
    es_connected,
    es_disconnect_suspicion,
    es_disconnected
  };
  enum process_status_t {
    ps_prepare,
    ps_process,
    ps_report
  };

  const counter_t m_multimeter_wait_interval;
  const counter_t m_r2r_set_interval;

  cfg_t& m_cfg;
  irs::modbus_client_t m_modbus_client;
  hrm::eth_data_t m_eth_data;
  //  Main Memo
  bool m_new_log_string;
  irs::string_t m_log_string;
  bool m_new_report_string;
  irs::string_t m_report_string;
  //  Multimeter Box
  irs::agilent_34420a_t m_multimeter;
  //multimeter_service_t m_multimeter_service;
  double m_multimeter_value;
  double m_multimeter_value_LSD;
  bool m_new_multimeter_value;
  irs::string_t m_multimeter_value_string;
  bool m_multimeter_box_enabled;
  //  Comparator in Multimeter Box
  bool m_r2r_comparator_value;
  bool m_new_r2r_comparator_value;
  //  Balance Box
  bool m_need_start;
  bool m_balance_box_enabled;
  double m_balance_pause;
  bool m_balance_adaptive_pause;
  bool m_new_balance_box_data;
  bool m_balance_need_pause;
  double m_balance_adaptive_limit;
  balance_mode_t m_balance_mode;
  direction_t m_balance_direction;
  size_t m_calibre_meas_count;
  irs_u8 m_balance_exp_count;
  double m_etalon_value;
  double m_checked_value;
  bool m_new_calibre_data_available;
  bool m_calibre_data_available;
  bool m_calibre_subtract_bias;
  double m_calibre_bias;
  double m_calibre_bias_LSD;
  bool m_exist_calibre_bias;
  bool m_calibre_check_weight;
  bool m_calibre_MSD_ready;
  //  Code Box
  irs_u32 m_code;
  irs_u32 m_process_code;
  bool m_new_code;
  bool m_need_code_set;
  bool m_code_box_enabled;
  //  Led
  bool m_led;
  bool m_new_led;
  bool m_need_led_set;
  bool m_led_enabled;
  //  Abort Button
  bool m_need_abort;
  //  Connected Label
  bool m_new_connected_status;
  irs::string_t m_connected_label_string;
  bool m_connected_label_enabled;
  //  Core Temperature Label
  double m_core_temperature;
  bool m_new_core_temperature;
  irs::string_t m_core_temperature_label_string;
  bool m_core_temperature_label_enabled;
  //
  status_t m_status;
  eth_status_t m_eth_status;
  vector<double> m_calibre_vector;
  vector<code_t> m_code_vector;
  vector<double> m_voltage_vector;
  irs_u8 m_calibre_current_point;
  irs_u32 m_balance_step;
  irs::timer_t m_timer;
  irs::timer_t m_eth_timer;
  //  Processor
  vector<action_t*> m_process_vector;
  status_t m_target_status;
  process_status_t m_process_status;
  size_t m_process_point_count;
  size_t m_process_current_point;
  size_t m_process_iteration_count;
  size_t m_process_current_iteration;
  //
  counter_t m_pause_interval;
  action_pause_t m_pause_action;
  //
  irs_u32 m_r2r_code;
  counter_t m_r2r_error_interval;
  action_r2r_set_t m_r2r_action;
  //
  action_r2r_calibre_code_generate_t m_r2r_calibre_code_gen_MSD;
  action_r2r_calibre_code_generate_t m_r2r_calibre_code_gen_LSD;
  action_r2r_calibre_code_generate_t m_r2r_calibre_code_gen_zero;
  //
  size_t m_main_loop_return_point;
  action_loop_t m_main_loop_action;
  size_t m_meas_loop_return_point;
  size_t m_meas_current_iteration;
  action_loop_t m_calibre_meas_loop_action;
  size_t m_lsb_branch_bias_point;
  action_brlo_t<size_t> m_lsb_brlo_bias;
  size_t m_lsb_branch_ch1_point;
  action_brlo_t<size_t> m_lsb_brlo_ch1;
  size_t m_lsb_branch_ch2_point;
  action_brlo_t<size_t> m_lsb_brlo_ch2;
  //
  size_t m_lsb_meas_count;
  size_t m_lsb_count;
  size_t m_msb_count;
  size_t m_lsb_meas_bias_loop_return_point;
  size_t m_lsb_meas_bias_current_point;
  action_loop_t m_lsb_meas_bias_loop_action;
  size_t m_lsb_meas_ch1_loop_return_point;
  size_t m_lsb_meas_ch1_current_point;
  action_loop_t m_lsb_meas_ch1_loop_action;
  size_t m_lsb_meas_ch2_loop_return_point;
  size_t m_lsb_meas_ch2_current_point;
  action_loop_t m_lsb_meas_ch2_loop_action;
  //irs::string_t m_process_message;
  counter_t m_multimeter_error_interval;
  double m_calibre_nplc;
  vector<double> m_result_vector;
  action_set_mult_channel_t m_set_ch1_action;
  action_set_mult_channel_t m_set_ch2_action;
  action_mult_meas_t m_meas_voltage_action;
  action_mult_meas_t m_meas_value_action;
  action_mult_meas_t m_meas_idle_value_action;
  action_wait_mult_t m_wait_mult_action;
  action_push_back_t<double> m_push_back_action;
  //
  double m_idle_nplc;
  action_set_mult_nplc_t m_set_idle_nplc_action;
  action_set_mult_nplc_t m_set_calibre_nplc_action;
  size_t m_idle_loop_return_point;
  action_loop_t m_idle_loop_action;
  //
  size_t m_balance_meas_count;
  double m_balance_multimeter_value;
  code_t m_balance_code_bias;
  double m_adaptive_limit;
  size_t m_adaptive_branch_point;
  double m_zero_shift_value;
  double m_fake_zero_shift_value;
  size_t m_zero_loop_return_point;
  vector<double> m_zero_vector;
  vector<double> m_zero_process_vector;
  action_brlo_t<double> m_adaptive_brlo;
  action_loop_t m_balance_meas_loop_action;
  action_reset_average_t<double> m_balance_multimeter_average_action;
  action_loop_t m_balance_meas_zero_loop_action;
  action_reset_average_t<double> m_balance_zero_average_action;
  action_clear_vector_t<double> m_balance_clear_result_vector_action;
  action_clear_vector_t<double> m_balance_clear_zero_process_vector_action;
  action_r2r_balance_code_bias_generate_t m_balance_code_bias_generate_action;
  action_r2r_balance_dir_generate_t m_balance_dir_generate_action;
  action_r2r_balance_code_generate_t m_balance_code_generate_action;
  action_push_back_t<code_t> m_code_push_back_action;
  action_push_back_t<double> m_voltage_push_back_action;
  action_push_back_t<double> m_zero_push_back_action;
  action_push_back_t<double> m_zero_process_push_back_action;
  action_push_back_t<double> m_zero_fake_push_back_action;
  action_relay_set_t m_zero_relay_on_action;
  action_relay_set_t m_zero_relay_off_action;
  //
  double m_source_voltage;
  double m_bridge_emulate_value;
  code_t m_bridge_emulate_in_code;
  code_t m_bridge_emulate_out_code;
  action_convert_calibre_code_t m_convert_calibre_code_action;
  action_bridge_emulate_t m_bridge_emulate_action;
  action_r2r_balance_dir_generate_t m_emulate_dir_generate_action;
  action_r2r_balance_code_generate_t m_emulate_code_generate_action;
  action_push_back_t<code_t> m_emulate_code_push_back_action;
  action_push_back_t<double> m_emulate_voltage_push_back_action;
  //
  size_t m_buffer_test_meas_count;
  size_t m_buffer_test_digits;
  size_t m_buffer_test_current_digit;
  size_t m_buffer_test_return_point;
  size_t m_buffer_test_meas_ch1_return_point;
  size_t m_buffer_test_meas_ch2_return_point;
  size_t m_buffer_test_meas_current;
  action_loop_t m_buffer_test_loop_action;
  action_loop_t m_buffer_test_meas_ch1_loop_action;
  action_loop_t m_buffer_test_meas_ch2_loop_action;
  action_r2r_buffer_test_code_generate_t m_r2r_buffer_test_code_generate_action;
  //
  size_t m_r2r_scan_start_digit;
  size_t m_r2r_scan_digits;
  code_t m_r2r_scan_start_code;
  size_t m_r2r_scan_current_iteration;
  size_t m_r2r_scan_iteration_count;
  size_t m_r2r_scan_return_point;
  action_loop_t m_r2r_scan_loop_action;
  action_r2r_scan_code_generate_t m_r2r_scan_code_generate_action;
  action_push_back_t<double> m_emulate_push_back_action;
  action_r2r_scan_code_generate_t m_emulate_r2r_scan_code_generate_action;
  //
  size_t m_current_time;
  size_t m_process_time;
  bool m_new_time_value;

  void abort_procedure();
  bool calibre_data_is_valid(vector<double>* ap_calibre_vector);
};

} //  namespace hrm

#endif // APP_H
