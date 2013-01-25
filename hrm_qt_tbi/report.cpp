#include "report.h"
#include <math.h>

bool hrm::create_calibre_report(
  vector<double> *ap_process_vector,
  vector<double> *ap_calibre_vector,
  size_t a_iteration_count,
  size_t a_meas_count,
  size_t a_lsb_count,
  size_t a_lsb_meas_count,
  bool a_substract_bias,
  bool a_check_weight, irs::string_t *ap_message_string, bool *ap_new_message)
{
  bool valid_input_data = (ap_process_vector || ap_calibre_vector ||
    ap_message_string || ap_new_message || a_meas_count || a_iteration_count);

  if (valid_input_data) {
    size_t ser_size = 6;  //  Размер набора данных одного измерения
    size_t bias_shift = 0;
    size_t ch1_shift = 1;
    size_t ch2_shift = 2;
    //size_t bias_lsd_shift = 3;
    //size_t ch1_lsd_shift = 4;
    //size_t ch2_lsd_shift = 5;
    size_t msb_count = a_iteration_count - a_lsb_count;
    if (a_substract_bias) {
      if (a_check_weight) {
        ser_size = 6;
      } else {
        ser_size = 3;
      }
    } else {
      ch1_shift = 0;
      ch2_shift = 1;
      if (a_check_weight) {
        ser_size = 4;
        //ch1_lsd_shift = 2;
        //ch2_lsd_shift = 3;
      } else {
        ser_size = 2;
      }
    }
    if (ap_process_vector->size() <
        ser_size * a_meas_count * (msb_count + a_lsb_count*a_lsb_meas_count)) {
      valid_input_data = false;
    } else {
      ap_message_string->clear();
      (*ap_message_string) += 13;
      (*ap_message_string) += irst("Калибровка");
      (*ap_message_string) += 13;
      if (a_substract_bias) {
        (*ap_message_string) += irst("С вычитанием смещения");
        (*ap_message_string) += 13;
      }
      if (a_check_weight) {
        (*ap_message_string) += irst("С проверкой веса");
        (*ap_message_string) += 13;
      }
      (*ap_message_string) += irst("Количество измерений: ");
      (*ap_message_string) += ntostr(a_meas_count);
      (*ap_message_string) += 13;

      ap_calibre_vector->clear();
      for (size_t i = 0; i < msb_count; i++) {
        double calibre_value = 0.;
        double ref_ratio = pow(2, i+1);
        for (size_t m = 0; m < a_meas_count; m++) {
          double bias = 0.;
          double ch1 = 0.;
          double ch2 = 0.;
          size_t index = (i * a_meas_count + m) * ser_size;
          if (a_substract_bias) {
            bias = (*ap_process_vector)[index + bias_shift];
          }
          ch1 = (*ap_process_vector)[index + ch1_shift];
          ch2 = (*ap_process_vector)[index + ch2_shift];
          if (ch2 - bias == 0.) {
            ch2 = 0.1e-6;
            bias = 0.;
          }
          calibre_value += ((ch1 / (ch2 - bias)) / ref_ratio);
        }
        calibre_value /= a_meas_count;
        ap_calibre_vector->push_back(calibre_value);

        (*ap_message_string) += ntostr(i);
        (*ap_message_string) += irst(" : ");
        (*ap_message_string) += ntostr(calibre_value);

//        if (a_check_weight) {
//          double calibre_value_lsd = 0.;
//          for (size_t m = 0; m < a_meas_count; m++) {
//            double bias_lsd = 0.;
//            double ch1_lsd = 0.;
//            double ch2_lsd = 0.;
//            size_t index = (i * a_meas_count + m) * ser_size;
//            if (a_substract_bias) {
//              bias_lsd = (*ap_process_vector)[index + bias_lsd_shift];
//            }
//            ch1_lsd = (*ap_process_vector)[index + ch1_lsd_shift];
//            ch2_lsd = (*ap_process_vector)[index * ser_size + ch2_lsd_shift];
//            if (ch2_lsd - bias_lsd == 0.) {
//              ch2_lsd = 0.1e-6;
//              bias_lsd = 0.;
//            }
//            calibre_value_lsd += ((ch1_lsd/(ch2_lsd - bias_lsd))/(ref_ratio-1));
//          }
//          calibre_value_lsd /= a_meas_count;
//          (*ap_message_string) += irst(" (");
//          (*ap_message_string) += ntostr(calibre_value_lsd);
//          (*ap_message_string) += irst(")");
//        }
        (*ap_message_string) += 13;
      }
      size_t lsb = 0;
      for (size_t i = msb_count; i < a_iteration_count; i++, lsb++) {
        double calibre_value = 0.;
        double ref_ratio = pow(2, i+1);
        for (size_t m = 0; m < a_meas_count; m++) {
          double bias = 0.;
          double ch1 = 0.;
          double ch2 = 0.;
          size_t index = msb_count * ser_size * a_meas_count +
            (lsb * a_meas_count + m) * ser_size * a_lsb_meas_count;
          if (a_substract_bias) {
            for (size_t b = 0; b < a_lsb_meas_count; b++) {
              bias += (*ap_process_vector)[index+bias_shift*a_lsb_meas_count+b];
            }
            bias /= a_lsb_meas_count;
          }
          for (size_t b = 0; b < a_lsb_meas_count; b++) {
            ch1 += (*ap_process_vector)[index + ch1_shift*a_lsb_meas_count + b];
            ch2 += (*ap_process_vector)[index + ch2_shift*a_lsb_meas_count + b];
          }
          ch1 /= a_lsb_meas_count;
          ch2 /= a_lsb_meas_count;
          if (ch2 - bias == 0.) {
            ch2 = 0.1e-6;
            bias = 0.;
          }
          calibre_value += ((ch1 / (ch2 - bias)) / ref_ratio);
        }
        calibre_value /= a_meas_count;
        ap_calibre_vector->push_back(calibre_value);

        (*ap_message_string) += ntostr(i);
        (*ap_message_string) += irst(" : ");
        (*ap_message_string) += ntostr(calibre_value);
        (*ap_message_string) += 13;
      }
      (*ap_new_message) = true;
    }
  }

  return valid_input_data;
}

bool hrm::create_buffer_test_report(
  vector<double>* ap_process_vector,
  size_t a_digits_count,
  size_t a_meas_count,
  irs::string_t* ap_message_string,
  bool* ap_new_message)
{
  bool valid_input_data = (ap_process_vector  || a_digits_count ||
    ap_message_string || ap_new_message || a_meas_count);

  if (valid_input_data) {
    ap_message_string->clear();
    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Проверка буфера");
    (*ap_message_string) += 13;

    size_t ser_size = a_meas_count * 2;
    for (size_t i = 0; i < a_digits_count; i++) {
      double ch1 = 0;
      for (size_t m = 0; m < a_meas_count; m++) {
        ch1 += (*ap_process_vector)[ser_size * i + m];
      }
      ch1 /= a_meas_count;
      double ch2 = 0;
      for (size_t m = 0; m < a_meas_count; m++) {
        ch2 += (*ap_process_vector)[ser_size * i + m + a_meas_count];
      }
      ch2 /= a_meas_count;

      (*ap_message_string) += ntostr(i);
      (*ap_message_string) += irst(" : ");
      (*ap_message_string) += ntostr(ch1);
      (*ap_message_string) += irst(" : ");
      (*ap_message_string) += ntostr(ch2);
      (*ap_message_string) += 13;
    }
    *ap_new_message = true;
  }
  return valid_input_data;
}

bool hrm::create_r2r_scan_report(
  vector<double>* ap_process_vector,
  code_t a_start_code,
  irs::string_t* ap_message_string,
  bool* ap_new_message)
{
  bool valid_input_data = (ap_message_string || ap_new_message
    || (ap_process_vector && ap_process_vector->size()));

  if (valid_input_data) {
    ap_message_string->clear();
    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Проверка ЦАПа");
    (*ap_message_string) += 13;

    for (size_t i = 0; i < ap_process_vector->size(); i++) {
      (*ap_message_string) += ntostr(a_start_code + i);
      (*ap_message_string) += irst(" : ");
      (*ap_message_string) += ntostr((*ap_process_vector)[i]);
      (*ap_message_string) += 13;
    }
    *ap_new_message = true;
  }
  return valid_input_data;
}

bool hrm::create_balance_report(
  vector<code_t>* ap_code_vector,
  vector<double>* ap_voltage_vector,
  vector<double>* ap_zero_vector,
  vector<double> *ap_calibre_vector,
  double a_etalon,
  double a_checked, irs::string_t *ap_message_string, bool *ap_new_message)
{
  bool valid_input_data = (ap_message_string || ap_new_message
    || (ap_code_vector && ap_code_vector->size())
    || (ap_voltage_vector &&
        ap_voltage_vector->size() == ap_code_vector->size())
    || (ap_zero_vector &&
        ap_zero_vector->size() == ap_code_vector->size())
    || (ap_calibre_vector && ap_calibre_vector->size() == r2r_resolution)
    || a_etalon || a_checked);

  if (valid_input_data) {
    double result_code = static_cast<double>(ap_code_vector->back());
    double real_code = convert_code(ap_calibre_vector, result_code);
    double relative_meas = r2r_max_code / real_code - 1.;
    double checked = a_etalon / relative_meas;
    double error = (checked / a_checked - 1.) * 100.;

    ap_message_string->clear();
    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Уравновешивание");


//    for (size_t i = 0; i < ap_code_vector->size(); i++) {
//      (*ap_message_string) += 13;
//      (*ap_message_string) += ntostr(i) + irst(" : ")
//        + ntostr((*ap_code_vector)[i]) + irst(" : ")
//        + ntostr((*ap_voltage_vector)[i] - (*ap_zero_vector)[i]) + irst(" В")
//        + irst(" : ") + ntostr((*ap_zero_vector)[i]) + irst(" В");
//    }
    for (size_t i = 0; i < ap_code_vector->size(); i++) {
      (*ap_message_string) += 13;
      (*ap_message_string) += ntostr(i);
      (*ap_message_string) += 9;
      (*ap_message_string) += ntostr((*ap_code_vector)[i]);
      (*ap_message_string) += 9;
      (*ap_message_string) += ntostr((*ap_voltage_vector)[i] - (*ap_zero_vector)[i]);
      (*ap_message_string) += 9;
      (*ap_message_string) += ntostr((*ap_zero_vector)[i]);
    }

    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Эталон: ") + ntostr(a_etalon) + irst(" Ом");
    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Поверяемая: ")+ntostr(a_checked)+irst(" Ом");
    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Код: ") + ntostr(result_code);
    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Приведённый код: ") + ntostr(real_code);
    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Измеренное значение поверяемой: ")
        + ntostr(checked) + irst(" Ом");
    (*ap_message_string) += 13;
    (*ap_message_string) += irst("Ошибка: ") + ntostr(error) + irst(" %");
    *ap_new_message = true;
  }
  return valid_input_data;
}

bool hrm::create_one_meas_report(
  double a_meas_value,
  irs::string_t *ap_message_string,
  bool *ap_new_message)
{
  bool valid_input_data = (ap_message_string || ap_new_message);
  if (valid_input_data) {
    ap_message_string->clear();
    *ap_message_string += 13;
    *ap_message_string += irst("Измеренное значение: ") + ntostr(a_meas_value);
    *ap_message_string += irst(" В");
    *ap_new_message = true;
  }
  return valid_input_data;
}
