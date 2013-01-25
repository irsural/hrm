#include "app.h"
#include "report.h"

hrm::app_t::app_t(cfg_t &a_cfg):
  m_multimeter_wait_interval(irs::make_cnt_s(40)),
  m_r2r_set_interval(irs::make_cnt_s(2)),
  m_cfg(a_cfg),
  m_modbus_client(m_cfg.get_eth_hardflow(),
    irs::modbus_client_t::mode_refresh_auto,
    m_read_bits_byte_count,
    m_rw_bits_byte_count,
    m_rw_regs_count,
    m_read_regs_count,
    irs::make_cnt_ms(200)),
  m_eth_data(&m_modbus_client),
  //  Main Memo
  m_new_log_string(true),
  m_log_string(),
  //  Multimeter Box
  m_multimeter(m_cfg.get_gpib_hardflow(), mul_mode_type_passive),
  m_multimeter_value(0.),
  m_multimeter_value_LSD(0.),
  m_new_multimeter_value(true),
  m_multimeter_value_string(irst("--------")),
  m_multimeter_box_enabled(true),
  //  Comparator
  m_r2r_comparator_value(false),
  m_new_r2r_comparator_value(true),
  //  Balance Box
  m_need_start(false),
  m_balance_box_enabled(false),
  m_balance_pause(10.),
  m_balance_adaptive_pause(true),
  m_new_balance_box_data(true),
  m_balance_need_pause(false),
  m_balance_adaptive_limit(0.01),
  m_balance_mode(bm_multimeter),
  m_balance_direction(di_up),
  m_calibre_meas_count(1),
  m_balance_exp_count(1),
  m_etalon_value(1.000003e6),
  m_checked_value(10.00003e6),
  m_new_calibre_data_available(false),
  m_calibre_data_available(false),
  m_calibre_subtract_bias(false),
  m_calibre_bias(0.),
  m_calibre_bias_LSD(0.),
  m_exist_calibre_bias(false),
  m_calibre_check_weight(false),
  m_calibre_MSD_ready(false),
  //  Code Box
  m_code(0),
  m_process_code(0),
  m_new_code(true),
  m_need_code_set(false),
  m_code_box_enabled(false),
  //  Led
  m_led(false),
  m_new_led(true),
  m_need_led_set(false),
  m_led_enabled(false),
  //  Abort Button
  m_need_abort(false),
  //  Connected Label
  m_new_connected_status(true),
  m_connected_label_string(irst("Нет контакт!")),
  m_connected_label_enabled(false),
  //  Core Temperature Label
  m_core_temperature(0.),
  m_new_core_temperature(true),
  m_core_temperature_label_string(irst("-----")),
  m_core_temperature_label_enabled(false),
  //
  m_status(st_idle),
  m_eth_status(es_disconnected),
  m_calibre_vector(0),
  m_code_vector(0),
  m_voltage_vector(0),
  m_calibre_current_point(0),
  m_balance_step(0),
  m_timer(),
  m_eth_timer(irs::make_cnt_s(5)),
  //  Processor
  m_process_vector(0),
  m_target_status(st_free),
  m_process_status(ps_prepare),
  m_process_point_count(r2r_resolution),
  m_process_current_point(0),
  m_process_iteration_count(r2r_resolution),
  m_process_current_iteration(0),
  //
  m_pause_interval(irs::make_cnt_s(1.5)),
  m_pause_action(
    &m_pause_interval,
    &m_log_string,
    &m_new_log_string),
  //
  m_r2r_code(99),
  m_r2r_error_interval(irs::make_cnt_s(2.)),
  m_r2r_action(
    &m_r2r_code,
    &m_eth_data.tbi_0,
    &m_eth_data.r2r_busy,
    &m_r2r_set_interval,
    &m_log_string,
    &m_new_log_string),
  m_r2r_calibre_code_gen_MSD(
    hrm::action_r2r_calibre_code_generate_t::ct_msd,
    &m_process_current_iteration,
    &m_process_iteration_count,
    &m_r2r_code,
    &m_log_string,
    &m_new_log_string),
  m_r2r_calibre_code_gen_LSD(
    hrm::action_r2r_calibre_code_generate_t::ct_lsd,
    &m_process_current_iteration,
    &m_process_iteration_count,
    &m_r2r_code,
    &m_log_string,
    &m_new_log_string),
  m_r2r_calibre_code_gen_zero(
    hrm::action_r2r_calibre_code_generate_t::ct_zero,
    &m_process_current_iteration,
    &m_process_iteration_count,
    &m_r2r_code,
    &m_log_string,
    &m_new_log_string),
  m_main_loop_return_point(0),
  m_main_loop_action(
    &m_process_current_point,
    &m_main_loop_return_point,
    &m_process_current_iteration,
    &m_process_iteration_count,
    &m_log_string,
    &m_new_log_string),
  m_meas_loop_return_point(0),
  m_meas_current_iteration(0),
  m_calibre_meas_loop_action(
    &m_process_current_point,
    &m_meas_loop_return_point,
    &m_meas_current_iteration,
    &m_calibre_meas_count,
    &m_log_string,
    &m_new_log_string),
  m_lsb_branch_bias_point(0),
  m_lsb_brlo_bias(
    &m_process_current_point,
    &m_lsb_branch_bias_point,
    &m_process_current_iteration,
    &m_msb_count,
    &m_log_string,
    &m_new_log_string),
  m_lsb_branch_ch1_point(0),
  m_lsb_brlo_ch1(
    &m_process_current_point,
    &m_lsb_branch_ch1_point,
    &m_process_current_iteration,
    &m_msb_count,
    &m_log_string,
    &m_new_log_string),
  m_lsb_branch_ch2_point(0),
  m_lsb_brlo_ch2(
    &m_process_current_point,
    &m_lsb_branch_ch2_point,
    &m_process_current_iteration,
    &m_msb_count,
    &m_log_string,
    &m_new_log_string),
  m_lsb_meas_count(0),
  m_lsb_count(0),
  m_msb_count(r2r_resolution),
  m_lsb_meas_bias_loop_return_point(0),
  m_lsb_meas_bias_current_point(0),
  m_lsb_meas_bias_loop_action(
    &m_process_current_point,
    &m_lsb_meas_bias_loop_return_point,
    &m_lsb_meas_bias_current_point,
    &m_lsb_meas_count,
    &m_log_string,
    &m_new_log_string),
  m_lsb_meas_ch1_loop_return_point(0),
  m_lsb_meas_ch1_current_point(0),
  m_lsb_meas_ch1_loop_action(
    &m_process_current_point,
    &m_lsb_meas_ch1_loop_return_point,
    &m_lsb_meas_ch1_current_point,
    &m_lsb_meas_count,
    &m_log_string,
    &m_new_log_string),
  m_lsb_meas_ch2_loop_return_point(0),
  m_lsb_meas_ch2_current_point(0),
  m_lsb_meas_ch2_loop_action(
    &m_process_current_point,
    &m_lsb_meas_ch2_loop_return_point,
    &m_lsb_meas_ch2_current_point,
    &m_lsb_meas_count,
    &m_log_string,
    &m_new_log_string),
  m_multimeter_error_interval(irs::make_cnt_s(15.)),
  m_calibre_nplc(50.),
  m_result_vector(),
  m_set_ch1_action(
    &m_multimeter,
    1,
    &m_multimeter_error_interval,
    &m_log_string,
    &m_new_log_string),
  m_set_ch2_action(
    &m_multimeter,
    2,
    &m_multimeter_error_interval,
    &m_log_string,
    &m_new_log_string),
  m_meas_voltage_action(
    &m_multimeter,
    action_mult_meas_t::mt_init,
    &m_calibre_nplc,
    &m_multimeter_value,
    &m_new_multimeter_value,
    &m_multimeter_error_interval,
    &m_log_string,
    &m_new_log_string),
  m_meas_value_action(
    &m_multimeter,
    action_mult_meas_t::mt_regular,
    &m_calibre_nplc,
    &m_multimeter_value,
    &m_new_multimeter_value,
    &m_multimeter_error_interval,
    &m_log_string,
    &m_new_log_string),
  m_meas_idle_value_action(
    &m_multimeter,
    action_mult_meas_t::mt_regular,
    &m_idle_nplc,
    &m_multimeter_value,
    &m_new_multimeter_value,
    &m_multimeter_error_interval,
    IRS_NULL,
    IRS_NULL),
  m_wait_mult_action(
    &m_multimeter,
    &m_multimeter_error_interval,
    &m_log_string,
    &m_new_log_string),
  m_push_back_action(
    &m_result_vector,
    &m_multimeter_value,
    &m_log_string,
    &m_new_log_string),
  m_idle_nplc(10.),
  m_set_idle_nplc_action(
    &m_multimeter,
    &m_idle_nplc,
    &m_r2r_error_interval,
    &m_log_string,
    &m_new_log_string),
  m_set_calibre_nplc_action(
    &m_multimeter,
    &m_calibre_nplc,
    &m_r2r_error_interval,
    &m_log_string,
    &m_new_log_string),
  m_idle_loop_action(
    &m_process_current_point,
    &m_idle_loop_return_point,
    IRS_NULL,
    IRS_NULL,
    IRS_NULL,
    IRS_NULL),
  m_balance_multimeter_value(0),
  m_balance_code_bias(0),
  m_adaptive_limit(0.05),
  m_adaptive_branch_point(0),
  m_zero_shift_value(0.),
  m_fake_zero_shift_value(0.),
  m_zero_vector(0),
  m_zero_process_vector(0),
  m_adaptive_brlo(
    &m_process_current_point,
    &m_adaptive_branch_point,
    &m_adaptive_limit,
    &m_multimeter_value,
    &m_log_string,
    &m_new_log_string),
  m_balance_meas_loop_action(
    &m_process_current_point,
    &m_meas_loop_return_point,
    &m_meas_current_iteration,
    &m_balance_meas_count,
    &m_log_string,
    &m_new_log_string),
  m_balance_multimeter_average_action(
    &m_result_vector,
    &m_balance_multimeter_value,
    &m_log_string,
    &m_new_log_string),
  m_balance_meas_zero_loop_action(
    &m_process_current_point,
    &m_zero_loop_return_point,
    &m_meas_current_iteration,
    &m_balance_meas_count,
    &m_log_string,
    &m_new_log_string),
  m_balance_zero_average_action(
    &m_zero_process_vector,
    &m_zero_shift_value,
    &m_log_string,
    &m_new_log_string),
  m_balance_clear_result_vector_action(
    &m_result_vector,
    &m_log_string,
    &m_new_log_string),
  m_balance_clear_zero_process_vector_action(
    &m_zero_process_vector,
    &m_log_string,
    &m_new_log_string),
  m_balance_code_bias_generate_action(
    &m_calibre_vector,
    &m_balance_code_bias,
    &m_log_string,
    &m_new_log_string),
  m_balance_dir_generate_action(
    &m_balance_direction,
    &m_balance_mode,
    &m_balance_multimeter_value,
    &m_r2r_comparator_value,
    &m_zero_shift_value,
    &m_log_string,
    &m_new_log_string),
  m_balance_code_generate_action(
    &m_balance_direction,
    &m_process_current_iteration,
    &m_process_iteration_count,
    &m_balance_code_bias,
    &m_r2r_code,
    &m_log_string,
    &m_new_log_string),
  m_code_push_back_action(
    &m_code_vector,
    &m_r2r_code,
    &m_log_string,
    &m_new_log_string),
  m_voltage_push_back_action(
    &m_voltage_vector,
    &m_balance_multimeter_value,
    &m_log_string,
    &m_new_log_string),
  m_zero_push_back_action(
    &m_zero_vector,
    &m_zero_shift_value,
    &m_log_string,
    &m_new_log_string),
  m_zero_process_push_back_action(
    &m_zero_process_vector,
    &m_multimeter_value,
    &m_log_string,
    &m_new_log_string),
  m_zero_fake_push_back_action(
    &m_zero_process_vector,
    &m_fake_zero_shift_value,
    &m_log_string,
    &m_new_log_string),
  m_zero_relay_on_action(
    action_relay_set_t::op_on,
    &m_eth_data.zero_relay,
    &m_eth_data.r2r_busy,
    &m_r2r_set_interval,
    &m_log_string,
    &m_new_log_string),
  m_zero_relay_off_action(
    action_relay_set_t::op_off,
    &m_eth_data.zero_relay,
    &m_eth_data.r2r_busy,
    &m_r2r_set_interval,
    &m_log_string,
    &m_new_log_string),
  m_source_voltage(0.),
  m_bridge_emulate_value(0.),
  m_bridge_emulate_in_code(0),
  m_bridge_emulate_out_code(0),
  m_convert_calibre_code_action(
    &m_calibre_vector,
    &m_bridge_emulate_in_code,
    &m_bridge_emulate_out_code,
    &m_log_string,
    &m_new_log_string),
  m_bridge_emulate_action(
    &m_source_voltage,
    &m_bridge_emulate_value,
    &m_etalon_value,
    &m_checked_value,
    &m_bridge_emulate_out_code,
    &m_log_string,
    &m_new_log_string),
  m_emulate_dir_generate_action(
    &m_balance_direction,
    &m_balance_mode,
    &m_bridge_emulate_value,
    &m_r2r_comparator_value,
    &m_fake_zero_shift_value,
    &m_log_string,
    &m_new_log_string),
  m_emulate_code_generate_action(
    &m_balance_direction,
    &m_process_current_iteration,
    &m_process_iteration_count,
    &m_balance_code_bias,
    &m_bridge_emulate_in_code,
    &m_log_string,
    &m_new_log_string),
  m_emulate_code_push_back_action(
    &m_code_vector,
    &m_bridge_emulate_in_code,
    &m_log_string,
    &m_new_log_string),
  m_emulate_voltage_push_back_action(
    &m_voltage_vector,
    &m_bridge_emulate_value,
    &m_log_string,
    &m_new_log_string),
  m_buffer_test_meas_count(0),
  m_buffer_test_digits(0),
  m_buffer_test_current_digit(0),
  m_buffer_test_return_point(0),
  m_buffer_test_meas_ch1_return_point(0),
  m_buffer_test_meas_ch2_return_point(0),
  m_buffer_test_meas_current(0),
  m_buffer_test_loop_action(
    &m_process_current_point,
    &m_buffer_test_return_point,
    &m_buffer_test_current_digit,
    &m_buffer_test_digits,
    &m_log_string,
    &m_new_log_string),
  m_buffer_test_meas_ch1_loop_action(
    &m_process_current_point,
    &m_buffer_test_meas_ch1_return_point,
    &m_buffer_test_meas_current,
    &m_buffer_test_meas_count,
    &m_log_string,
    &m_new_log_string),
  m_buffer_test_meas_ch2_loop_action(
    &m_process_current_point,
    &m_buffer_test_meas_ch2_return_point,
    &m_buffer_test_meas_current,
    &m_buffer_test_meas_count,
    &m_log_string,
    &m_new_log_string),
  m_r2r_buffer_test_code_generate_action(
    &m_buffer_test_current_digit,
    &m_r2r_code,
    &m_log_string,
    &m_new_log_string),
  m_r2r_scan_start_digit(0),
  m_r2r_scan_digits(0),
  m_r2r_scan_start_code(0),
  m_r2r_scan_current_iteration(0),
  m_r2r_scan_iteration_count(0),
  m_r2r_scan_return_point(0),
  m_r2r_scan_loop_action(
    &m_process_current_point,
    &m_r2r_scan_return_point,
    &m_r2r_scan_current_iteration,
    &m_r2r_scan_iteration_count,
    &m_log_string,
    &m_new_log_string),
  m_r2r_scan_code_generate_action(
    &m_r2r_code,
    &m_log_string,
    &m_new_log_string),
  m_emulate_push_back_action(
    &m_result_vector,
    &m_bridge_emulate_value,
    &m_log_string,
    &m_new_log_string),
  m_emulate_r2r_scan_code_generate_action(
    &m_bridge_emulate_in_code,
    &m_log_string,
    &m_new_log_string),
  m_current_time(0),
  m_process_time(0),
  m_new_time_value(false)
{
}

hrm::app_t::~app_t()
{
}

void hrm::app_t::tick()
{
  m_modbus_client.tick();
  m_multimeter.tick();

  switch (m_eth_status) {
    case es_disconnected: {
      if (m_modbus_client.connected()) {
        m_new_connected_status = true;
        m_connected_label_string = irst("Есть контакт!");
        m_connected_label_enabled = true;
        m_core_temperature_label_enabled = true;
        m_led_enabled = true;
        m_code_box_enabled = true;
        m_balance_box_enabled = true;
        m_eth_status = es_connected;
      }
      break;
    }
    case es_connected: {
      if (!m_modbus_client.connected()) {
        m_new_connected_status = true;
        m_connected_label_string = irst("Где контакт?");
        m_eth_timer.start();
        m_eth_status = es_disconnect_suspicion;
      } else {
        if (m_core_temperature != m_eth_data.internal_temp) {
          m_core_temperature = m_eth_data.internal_temp;
          irs::ostringstream_t stream;
          stream << fixed << setprecision(1) << m_core_temperature;
          m_core_temperature_label_string = stream.str();
          m_core_temperature_label_string += irst(" °C");
          m_new_core_temperature = true;
        }
        if (m_r2r_comparator_value != m_eth_data.r2r_comparator) {
          m_r2r_comparator_value = m_eth_data.r2r_comparator;
          m_new_r2r_comparator_value = true;
        }
        if (m_need_led_set) {
          m_need_led_set = false;
          m_eth_data.led_6 = m_led;
        } else {
          if (m_led != m_eth_data.led_6) {
            m_led = m_eth_data.led_6;
            m_new_led = true;
          }
        }
        if (m_need_code_set) {
          m_need_code_set = false;
          m_eth_data.tbi_0 = m_code;
        } else {
          if (m_code != m_eth_data.tbi_0) {
            m_code = m_eth_data.tbi_0;
            m_new_code = true;
          }
        }
      }
      break;
    }
    case es_disconnect_suspicion: {
      if (m_modbus_client.connected()) {
        m_new_connected_status = true;
        m_connected_label_string = irst("Есть контакт!");
      } else {
        if (m_eth_timer.check()) {
          m_eth_timer.stop();
          m_new_connected_status = true;
          m_connected_label_string = irst("Нет контакт!");
          m_connected_label_enabled = false;
          m_core_temperature_label_enabled = false;
          m_led_enabled = false;
//          m_code_box_enabled = false;
//          m_balance_box_enabled = false;
//          m_need_abort = true;
          m_log_string.clear();
          m_log_string += 13;
          m_log_string += irst("Пропало соединение с коммутатором");
          m_new_log_string = true;
          m_eth_status = es_disconnected;
        }
      }
      break;
    }
  }

  switch (m_process_status) {
    case ps_prepare: {
      m_process_vector.clear();
      m_result_vector.clear();
      m_code_vector.clear();
      m_voltage_vector.clear();
      m_zero_vector.clear();
      m_zero_process_vector.clear();
      m_new_log_string = true;
      m_process_current_point = 0;
      m_process_current_iteration = 0;
      m_msb_count = r2r_resolution - m_lsb_count;
      m_current_time = 0;
      m_process_status = ps_process;
      m_log_string.clear();
      m_log_string += 13;
      m_balance_code_bias = 0;
      switch (m_status) {
        case st_free: {
          break;
        }
        case st_balance: {
          m_log_string += irst("Уравновешивание");
          m_process_vector.push_back(&m_wait_mult_action);
          m_process_vector.push_back(&m_set_ch1_action);
          m_process_vector.push_back(&m_set_calibre_nplc_action);
          if (m_calibre_data_available) {
            m_process_vector.push_back(&m_balance_code_bias_generate_action);
          }
          m_main_loop_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_balance_dir_generate_action);
          m_process_vector.push_back(&m_balance_code_generate_action);
          m_process_vector.push_back(&m_r2r_action);
          m_process_vector.push_back(&m_code_push_back_action);
          m_process_vector.push_back(&m_meas_voltage_action);
          m_process_vector.push_back(&m_push_back_action);
          m_process_vector.push_back(&m_zero_fake_push_back_action);
          if (m_balance_adaptive_pause) {
            m_process_vector.push_back(&m_adaptive_brlo);
          }
          m_process_vector.push_back(&m_balance_clear_result_vector_action);

          m_process_vector.push_back(
            &m_balance_clear_zero_process_vector_action);
          m_process_vector.push_back(&m_zero_relay_on_action);
          m_process_vector.push_back(&m_pause_action);
          m_zero_loop_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_meas_value_action);
          m_process_vector.push_back(&m_zero_process_push_back_action);
          m_process_vector.push_back(&m_balance_meas_zero_loop_action);
          m_process_vector.push_back(&m_zero_relay_off_action);

          m_process_vector.push_back(&m_pause_action);
          m_meas_loop_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_meas_value_action);
          m_process_vector.push_back(&m_push_back_action);
          m_process_vector.push_back(&m_balance_meas_loop_action);
          m_adaptive_branch_point = m_process_vector.size();
          m_process_vector.push_back(&m_balance_zero_average_action);
          m_process_vector.push_back(&m_zero_push_back_action);
          m_process_vector.push_back(&m_balance_multimeter_average_action);
          m_process_vector.push_back(&m_voltage_push_back_action);
          m_process_vector.push_back(&m_main_loop_action);
          break;
        }
        case st_balance_emulate: {
          m_bridge_emulate_value = 0.;
          m_bridge_emulate_in_code = pow(2., r2r_resolution - 1);
          m_bridge_emulate_out_code = 0;

          if (m_calibre_data_available) {
            m_process_vector.push_back(&m_balance_code_bias_generate_action);
          }
          m_main_loop_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_emulate_dir_generate_action);
          m_process_vector.push_back(&m_emulate_code_generate_action);
          m_process_vector.push_back(&m_emulate_code_push_back_action);
          m_process_vector.push_back(&m_convert_calibre_code_action);
          m_process_vector.push_back(&m_bridge_emulate_action);
          m_process_vector.push_back(&m_emulate_voltage_push_back_action);
          m_process_vector.push_back(&m_zero_fake_push_back_action);
          m_process_vector.push_back(&m_balance_zero_average_action);
          m_process_vector.push_back(&m_zero_push_back_action);

          m_process_vector.push_back(&m_main_loop_action);

          m_log_string += irst("Эмуляция уравновешивания");
          break;
        }
        case st_calibre: {
          m_log_string += irst("Калибровка");
          m_process_vector.push_back(&m_wait_mult_action);
          m_process_vector.push_back(&m_set_calibre_nplc_action);
          m_main_loop_return_point = m_process_vector.size();
          m_meas_loop_return_point = m_process_vector.size();
          if (m_calibre_subtract_bias) {
            m_process_vector.push_back(&m_r2r_calibre_code_gen_zero);
            m_process_vector.push_back(&m_r2r_action);
            m_process_vector.push_back(&m_pause_action);
            m_process_vector.push_back(&m_set_ch2_action);
            m_process_vector.push_back(&m_meas_voltage_action);
            m_process_vector.push_back(&m_push_back_action);
            m_process_vector.push_back(&m_lsb_brlo_bias);
            m_lsb_meas_bias_loop_return_point = m_process_vector.size();
            m_process_vector.push_back(&m_meas_value_action);
            m_process_vector.push_back(&m_push_back_action);
            m_process_vector.push_back(&m_lsb_meas_bias_loop_action);
            m_lsb_branch_bias_point = m_process_vector.size();
            m_log_string += irst(" с вычитанием смещения");
          }
          m_process_vector.push_back(&m_r2r_calibre_code_gen_MSD);
          m_process_vector.push_back(&m_r2r_action);
          m_process_vector.push_back(&m_pause_action);
          m_process_vector.push_back(&m_set_ch1_action);
          m_process_vector.push_back(&m_meas_voltage_action);
          m_process_vector.push_back(&m_push_back_action);

          m_process_vector.push_back(&m_lsb_brlo_ch1);
          m_lsb_meas_ch1_loop_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_meas_value_action);
          m_process_vector.push_back(&m_push_back_action);
          m_process_vector.push_back(&m_lsb_meas_ch1_loop_action);
          m_lsb_branch_ch1_point = m_process_vector.size();

          m_process_vector.push_back(&m_set_ch2_action);
          m_process_vector.push_back(&m_meas_voltage_action);
          m_process_vector.push_back(&m_push_back_action);

          m_process_vector.push_back(&m_lsb_brlo_ch2);
          m_lsb_meas_ch2_loop_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_meas_value_action);
          m_process_vector.push_back(&m_push_back_action);
          m_process_vector.push_back(&m_lsb_meas_ch2_loop_action);
          m_lsb_branch_ch2_point = m_process_vector.size();

          if (m_calibre_check_weight) {
            m_log_string += irst(", проверкой веса");
            m_process_vector.push_back(&m_r2r_calibre_code_gen_LSD);
            m_process_vector.push_back(&m_r2r_action);
            m_process_vector.push_back(&m_pause_action);
            m_process_vector.push_back(&m_set_ch1_action);
            m_process_vector.push_back(&m_meas_voltage_action);
            m_process_vector.push_back(&m_push_back_action);
            m_process_vector.push_back(&m_set_ch2_action);
            m_process_vector.push_back(&m_meas_voltage_action);
            m_process_vector.push_back(&m_push_back_action);
          }
          m_process_vector.push_back(&m_calibre_meas_loop_action);
          m_process_vector.push_back(&m_main_loop_action);
          break;
        }
        case st_idle: {
          m_process_vector.push_back(&m_wait_mult_action);
          m_process_vector.push_back(&m_set_ch1_action);
          m_process_vector.push_back(&m_set_idle_nplc_action);
          m_process_vector.push_back(&m_meas_voltage_action);
          m_idle_loop_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_meas_idle_value_action);
          m_process_vector.push_back(&m_idle_loop_action);

          m_log_string += irst("Ждущий режим");
          break;
        }
        case st_buffer_test: {
          m_process_vector.push_back(&m_wait_mult_action);
          m_process_vector.push_back(&m_set_calibre_nplc_action);
          m_buffer_test_return_point = m_process_vector.size();

          m_process_vector.push_back(&m_r2r_buffer_test_code_generate_action);
          m_process_vector.push_back(&m_r2r_action);
          m_process_vector.push_back(&m_pause_action);
          m_process_vector.push_back(&m_set_ch1_action);
          m_process_vector.push_back(&m_meas_voltage_action);
          m_process_vector.push_back(&m_push_back_action);

          m_buffer_test_meas_ch1_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_meas_value_action);
          m_process_vector.push_back(&m_push_back_action);
          m_process_vector.push_back(&m_buffer_test_meas_ch1_loop_action);

          m_process_vector.push_back(&m_set_ch2_action);
          m_process_vector.push_back(&m_meas_voltage_action);
          m_process_vector.push_back(&m_push_back_action);

          m_buffer_test_meas_ch2_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_meas_value_action);
          m_process_vector.push_back(&m_push_back_action);
          m_process_vector.push_back(&m_buffer_test_meas_ch2_loop_action);

          m_process_vector.push_back(&m_buffer_test_loop_action);
          m_log_string += irst("Проверка буфера мультиметра");
          break;
        }
        case st_r2r_scan: {
          m_r2r_scan_start_code = static_cast<code_t>(
            pow(2, m_r2r_scan_start_digit) - pow(2, m_r2r_scan_digits));
          m_r2r_code = m_r2r_scan_start_code;
          m_r2r_scan_iteration_count =
            static_cast<code_t>(pow(2, m_r2r_scan_digits+1));

          m_process_vector.push_back(&m_wait_mult_action);
          m_process_vector.push_back(&m_set_calibre_nplc_action);
          m_process_vector.push_back(&m_set_ch1_action);
          m_r2r_scan_return_point = m_process_vector.size();

          m_process_vector.push_back(&m_r2r_scan_code_generate_action);
          m_process_vector.push_back(&m_r2r_action);
          m_process_vector.push_back(&m_pause_action);
          m_process_vector.push_back(&m_meas_voltage_action);
          m_process_vector.push_back(&m_push_back_action);

          m_process_vector.push_back(&m_r2r_scan_loop_action);

          m_log_string += irst("Снятие характеристики ЦАПа");
          break;
        }
        case st_r2r_scan_emulate: {
          m_r2r_scan_start_code = static_cast<code_t>(
            pow(2, m_r2r_scan_start_digit) - pow(2, m_r2r_scan_digits));
          m_r2r_scan_iteration_count =
            static_cast<code_t>(pow(2, m_r2r_scan_digits+1));
          m_bridge_emulate_in_code = m_r2r_scan_start_code;

          m_r2r_scan_return_point = m_process_vector.size();

          m_process_vector.push_back(&m_emulate_r2r_scan_code_generate_action);
          m_process_vector.push_back(&m_convert_calibre_code_action);
          m_process_vector.push_back(&m_bridge_emulate_action);
          m_process_vector.push_back(&m_emulate_push_back_action);

          m_process_vector.push_back(&m_r2r_scan_loop_action);

          m_log_string += irst("Эмуляция характеристики ЦАПа");
          break;
        }
        case st_one_meas: {
          m_process_vector.push_back(&m_wait_mult_action);
          m_process_vector.push_back(&m_set_calibre_nplc_action);
          m_process_vector.push_back(&m_set_ch1_action);
          m_process_vector.push_back(&m_meas_voltage_action);
          m_process_vector.push_back(&m_push_back_action);

          m_buffer_test_meas_ch1_return_point = m_process_vector.size();
          m_process_vector.push_back(&m_meas_value_action);
          m_process_vector.push_back(&m_push_back_action);
          m_process_vector.push_back(&m_buffer_test_meas_ch1_loop_action);
          m_process_vector.push_back(&m_balance_multimeter_average_action);

          m_log_string += irst("Одно измерение");
          break;
        }
      }

      if (m_status != st_idle) {
        for_each(m_process_vector.begin(), m_process_vector.end(),
          mem_fun(&action_t::reset));
        for (
          m_process_time = 0;
          m_process_current_point < m_process_vector.size();
          m_process_time++) {
          size_t point = m_process_current_point;
          if (m_process_vector[point]->linear()) {
            m_process_current_point++;
          } else {
            m_process_vector[m_process_current_point]->exec();
          }
        }
        m_process_current_point = 0;
        m_process_current_iteration = 0;
        m_current_time = 0;
        m_new_time_value = true;
      }
      for_each(m_process_vector.begin(), m_process_vector.end(),
        mem_fun(&action_t::reset));
      break;
    }
    case ps_process: {
      if (m_need_abort) {
        m_need_abort = false;
        m_code_box_enabled = true;
        m_balance_box_enabled = true;
        m_code = 0;
        m_eth_data.tbi_0 = 0;
        m_new_code = true;
        m_new_log_string = true;
        m_log_string.clear();
        m_log_string += 13;
        m_log_string += irst("Отмена");
        m_status = st_idle;
        m_process_status = ps_prepare;
      } else if (m_need_start) {
        m_need_start = false;
        m_process_status = ps_prepare;
        m_status = m_target_status;
      } else {
        if (m_process_current_point < m_process_vector.size()) {
          size_t point = m_process_current_point;
          irs_status_t status =
            m_process_vector[m_process_current_point]->exec();
          if (status == irs_st_ready) {
            if (m_process_vector[point]->linear()) {
              m_process_current_point++;
            }
            if (m_status != st_idle) {
              m_current_time++;
              m_new_time_value = true;
            }
          } else if (status == irs_st_error) {
            m_log_string += 13;
            m_log_string += irst("Ошибка action: элемент ");
            m_log_string += irs::irsstr_from_number_russian(irs::char_t(),
              point);
            m_new_log_string = true;
            m_process_status = ps_prepare;
            m_status = st_idle;
            m_code_box_enabled = true;
            m_balance_box_enabled = true;
          }
        } else {
          m_process_status = ps_report;
        }
      }
      break;
    }
    case ps_report: {
      m_log_string.clear();
      m_log_string += 13;
      m_log_string += irst("Отчёт");

      switch (m_status) {
        case st_calibre: {
          if (create_calibre_report(
            &m_result_vector,
            &m_calibre_vector,
            m_process_iteration_count,
            m_calibre_meas_count,
            m_lsb_count,
            m_lsb_meas_count+1,
            m_calibre_subtract_bias,
            m_calibre_check_weight,
            &m_report_string,
            &m_new_report_string)) {
            m_calibre_data_available =
              calibre_data_is_valid(&m_calibre_vector);
            m_new_calibre_data_available = m_calibre_data_available;
            m_report_string += 13;
            if (m_calibre_data_available) {
              m_report_string += irst("Калибровка хорошая, годная.");
              m_report_string += 13;
              m_report_string += irst("Калибровку можно сохранить.");
            } else {
              m_report_string += irst("Калибровка плохая.");
              m_report_string += 13;
              m_report_string += irst("Калибровку нельзя сохранить.");
            }
          } else {
            m_report_string = irst("Ошибка отчёта о калибровке");
            m_new_report_string = true;
          }
          break;
        }
        case st_buffer_test: {
          create_buffer_test_report(
            &m_result_vector,
            m_buffer_test_digits,
            m_buffer_test_meas_count + 1,
            &m_report_string,
            &m_new_report_string);
          break;
        }
        case st_r2r_scan: {
          create_r2r_scan_report(
            &m_result_vector,
            m_r2r_scan_start_code,
            &m_report_string,
            &m_new_report_string);
          break;
        }
        case st_r2r_scan_emulate: {
          create_r2r_scan_report(
            &m_result_vector,
            m_r2r_scan_start_code,
            &m_report_string,
            &m_new_report_string);
          break;
        }
        case st_balance: {
          create_balance_report(
            &m_code_vector,
            &m_voltage_vector,
            &m_zero_vector,
            &m_calibre_vector,
            m_etalon_value,
            m_checked_value,
            &m_report_string,
            &m_new_report_string);
          break;
        }
        case st_balance_emulate: {
          create_balance_report(
            &m_code_vector,
            &m_voltage_vector,
            &m_zero_vector,
            &m_calibre_vector,
            m_etalon_value,
            m_checked_value,
            &m_report_string,
            &m_new_report_string);
          break;
        }
        case st_one_meas: {
          create_one_meas_report(
            m_balance_multimeter_value,
            &m_report_string,
            &m_new_report_string);
        }
        default: {
          //
        }
      }
      m_process_status = ps_prepare;
      m_status = st_idle;

      m_new_log_string = true;
      m_code_box_enabled = true;
      m_balance_box_enabled = true;
      break;
    }
  }
}

//  Main Memo
bool hrm::app_t::get_new_log_line(irs::string_t* ap_line)
{
  bool new_text_line = m_new_log_string;
  if (m_new_log_string) {
    m_new_log_string = false;
    *ap_line = m_log_string;
  }
  return new_text_line;
}

bool hrm::app_t::get_new_report_line(irs::string_t* ap_line)
{
  bool new_text_line = m_new_report_string;
  if (m_new_report_string) {
    m_new_report_string = false;
    *ap_line = m_report_string;
  }
  return new_text_line;
}

//  Multimeter Box
bool hrm::app_t::get_multimeter_value(irs::string_t* ap_value)
{
  bool new_multimeter_value = m_new_multimeter_value;
  if (m_new_multimeter_value) {
    m_new_multimeter_value = false;
    m_multimeter_value_string.assign_russian(m_multimeter_value);
    *ap_value = m_multimeter_value_string;
  }
  return new_multimeter_value;
}

bool hrm::app_t::get_multimeter_box_enabled()
{
  return m_multimeter_box_enabled;
}

bool hrm::app_t::get_r2r_comparator_value(bool *ap_value)
{
  bool new_r2r_comparator_value = m_new_r2r_comparator_value;
  if (m_new_r2r_comparator_value) {
    m_new_r2r_comparator_value = false;
    *ap_value = m_r2r_comparator_value;
  }
  return new_r2r_comparator_value;
}

//  Balance Box

bool hrm::app_t::get_balance_box_enabled()
{
  return m_balance_box_enabled;
}

bool hrm::app_t::get_balance_box_data(balance_params_t* ap_params)
{
  if (m_new_balance_box_data) {
    ap_params->pause = m_balance_pause;
    ap_params->adaptive = m_balance_adaptive_pause;
    ap_params->balance_mode = m_balance_mode;
    ap_params->meas_count = m_calibre_meas_count;
    ap_params->exp_count = m_balance_exp_count;
    ap_params->etalon_value = m_etalon_value;
    ap_params->checked_value = m_checked_value;
    ap_params->subtract_bias = m_calibre_subtract_bias;
  }
  bool new_balance_box_data = m_new_balance_box_data;
  m_new_balance_box_data = false;
  return new_balance_box_data;
}

bool hrm::app_t::get_calibre_vector(vector<double>* ap_calibre_vector,
  bool* ap_exist_bias)
{
  if (m_new_calibre_data_available) {
    ap_calibre_vector->clear();
    ap_calibre_vector->resize(m_calibre_vector.size(), 1.);
    *ap_calibre_vector = m_calibre_vector;
    *ap_exist_bias = m_exist_calibre_bias;
    if (m_exist_calibre_bias) {
      ap_calibre_vector->push_back(m_calibre_bias);
    }
  }
  bool new_calibre_data_available = m_new_calibre_data_available;
  m_new_calibre_data_available = false;
  return new_calibre_data_available;
}

void hrm::app_t::set_calibre_vector(vector<double> *ap_calibre_vector)
{
  m_calibre_vector = *ap_calibre_vector;
  irs_u8 calibre_vector_size = m_calibre_vector.size();
  m_report_string.clear();
  m_report_string = irst("Загружена калибровка размером ");
  m_report_string +=
    irs::irsstr_from_number_russian(irs::char_t(), calibre_vector_size);
  m_report_string += 13;
  for (irs_u8 i = 0; i < calibre_vector_size; i++) {
    m_report_string +=
      irs::irsstr_from_number_russian(irs::char_t(), i);
    m_report_string += irst(" : ");
    m_report_string +=
      irs::irsstr_from_number_russian(irs::char_t(), m_calibre_vector[i]);
    m_report_string += 13;
  }
  m_calibre_data_available = calibre_data_is_valid(&m_calibre_vector);
  if (m_calibre_data_available) {
    m_report_string += irst("Калибровка хорошая, годная");
  } else {
    m_report_string += irst("Калибровка негодная и применена не будет");
  }
  m_new_report_string = true;
}

//  Calibre Box
void hrm::app_t::action_button_pressed(balance_params_t* ap_params)
{
  m_balance_pause = ap_params->pause;
  m_pause_interval = irs::make_cnt_s(ap_params->pause);
  m_balance_adaptive_pause = ap_params->adaptive;
  m_balance_mode = ap_params->balance_mode;
  m_calibre_meas_count = ap_params->meas_count;
  m_balance_meas_count = ap_params->meas_count;
  m_balance_exp_count = ap_params->exp_count;
  m_etalon_value = ap_params->etalon_value;
  m_checked_value = ap_params->checked_value;
  m_calibre_subtract_bias = ap_params->subtract_bias;
  m_calibre_check_weight = ap_params->check_weight;
  m_calibre_nplc = ap_params->nplc;
  m_lsb_count = ap_params->lsb_count;
  m_lsb_meas_count = ap_params->lsb_meas_count - 1;
  m_source_voltage = ap_params->source_voltage;
  m_buffer_test_meas_count = ap_params->meas_count - 1;
  m_buffer_test_digits = ap_params->buffer_test_digits;
  m_need_start = true;
  m_code_box_enabled = false;
  m_balance_box_enabled = false;
  m_target_status = ap_params->target_action;
  m_r2r_scan_start_digit = ap_params->r2r_scan_start_digit;
  m_r2r_scan_digits = ap_params->r2r_scan_digits;
}

//  Code Box
void hrm::app_t::code_set_button_pressed(irs_u32 a_code)
{
  m_need_code_set = true;
  m_code = a_code;
}

bool hrm::app_t::get_code_spin_box_value(irs_u32* ap_value)
{
  bool new_code = m_new_code;
  if (m_new_code) {
    m_new_code = false;
    *ap_value = m_code;
  }
  return new_code;
}

bool hrm::app_t::get_code_box_enabled()
{
  return m_code_box_enabled;
}

//  Led
void hrm::app_t::led_check_box_pressed(bool a_checked)
{
  m_led = a_checked;
  m_need_led_set = true;
}

bool hrm::app_t::get_led_check_box_checked(bool* ap_checked)
{
  *ap_checked = m_led;
  return m_new_led;
}

bool hrm::app_t::get_led_enabled()
{
  return m_led_enabled;
}

//  Abort Button
void hrm::app_t::abort_button_pressed()
{
  m_need_abort = true;
}

//  Connected Label
bool hrm::app_t::get_connected_label_text(irs::string_t* ap_text)
{
  bool new_connected_status = m_new_connected_status;
  if (m_new_connected_status) {
    m_new_connected_status = false;
    *ap_text = m_connected_label_string;
  }
  return new_connected_status;
}

bool hrm::app_t::get_connected_label_enabled()
{
  return m_connected_label_enabled;
}

//  Core Temperature Label
bool hrm::app_t::get_core_temperature_label_text(irs::string_t* ap_text)
{
  bool new_core_temperature = m_new_core_temperature;
  if (m_new_core_temperature) {
    m_new_core_temperature = false;
    *ap_text = m_core_temperature_label_string;
  }
  return new_core_temperature;
}

bool hrm::app_t::get_core_temperature_label_enabled()
{
  return m_core_temperature_label_enabled;
}

void hrm::app_t::abort_procedure()
{
  m_code_box_enabled = true;
  m_balance_box_enabled = true;
  m_code = 0;
  m_eth_data.tbi_0 = 0;
  m_new_code = true;
  m_new_log_string = true;
  m_log_string.clear();
  m_log_string += 13;
  m_log_string += irst("Отмена");
  m_status = st_free;
  m_target_status = st_free;
}

bool hrm::app_t::calibre_data_is_valid(vector<double>* ap_calibre_vector)
{
  bool calibre_valid = true;
  if (ap_calibre_vector->size() != r2r_resolution) {
    calibre_valid = false;
  }
  for (irs_u8 i = 0; i < ap_calibre_vector->size(); i++) {
    double point = (*ap_calibre_vector)[i];
    if (point < 0.9 || point > 1.1) {
      calibre_valid = false;
      break;
    }
  }
  return calibre_valid;
}

bool hrm::app_t::get_time_value(int *ap_time)
{
  bool new_time_value = m_new_time_value;
  if (m_new_time_value) {
    m_new_time_value = false;
    if (m_process_time != 0) {
      *ap_time = (m_current_time * 1000) / m_process_time;
    } else {
      *ap_time = 0;
    }
  }
  return new_time_value;
}

