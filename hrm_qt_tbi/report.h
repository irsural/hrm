#ifndef REPORT_H
#define REPORT_H

#include <irsdefs.h>
#include <hrm_defs.h>
#include <irsstrdefs.h>
#include <irsfinal.h>

namespace hrm {

bool create_calibre_report(
  vector<double>* ap_process_vector,
  vector<double>* ap_calibre_vector,
  size_t a_iteration_count,
  size_t a_meas_count,
  size_t a_lsb_count,
  size_t a_lsb_meas_count,
  bool a_substract_bias,
  bool a_check_weight,
  irs::string_t* ap_message_string,
  bool* ap_new_message);

bool create_buffer_test_report(
  vector<double>* ap_process_vector,
  size_t a_digits_count,
  size_t a_meas_count,
  irs::string_t* ap_message_string,
  bool* ap_new_message);

bool create_r2r_scan_report(
  vector<double>* ap_process_vector,
  code_t a_start_code,
  irs::string_t *ap_message_string,
  bool *ap_new_message);

bool create_balance_report(
  vector<code_t>* ap_code_vector,
  vector<double>* ap_voltage_vector,
  vector<double>* ap_zero_vector,
  vector<double>* ap_calibre_vector,
  double a_etalon,
  double a_checked,
  irs::string_t *ap_message_string,
  bool *ap_new_message);

bool create_one_meas_report(
  double a_meas_value,
  irs::string_t *ap_message_string,
  bool *ap_new_message);

} //  hrm

#endif // REPORT_H
