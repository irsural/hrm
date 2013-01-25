#include "process.h"

hrm::action_pause_t::action_pause_t(
  const counter_t *ap_interval,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_interval(ap_interval),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message),
  m_timer(*mp_interval),
  m_status(irs_st_ready)
{
  m_timer.stop();
}

irs_status_t hrm::action_pause_t::exec()
{
  switch (m_status) {
    case irs_st_busy: {
      if (m_timer.check()) {
        m_status = irs_st_ready;
        m_timer.stop();
      }
      break;
    }
    case irs_st_ready: {
      m_status = irs_st_busy;
      m_timer.set(*mp_interval);
      m_timer.start();
      (*mp_message_string) = irst("Пауза (") +
        irs::irsstr_from_number_russian(irs::char_t(),
        CNT_TO_DBLTIME(*(mp_interval))) + irst(" c)");
      *mp_new_message = true;
      break;
    }
    default: {}
  }
  return m_status;
}

void hrm::action_pause_t::reset()
{
  m_timer.stop();
  m_status = irs_st_ready;
}

hrm::action_r2r_set_t::action_r2r_set_t(
  code_t* ap_target_code,
  irs::conn_data_t<irs_u32>* ap_device_code,
  irs::bit_data_t* ap_ready_bit,
  const counter_t *ap_interval,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_target_code(ap_target_code),
  mp_device_code(ap_device_code),
  mp_ready_bit(ap_ready_bit),
  mp_interval(ap_interval),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message),
  m_timer(*mp_interval),
  m_status(irs_st_ready)
{
  m_timer.stop();
}

irs_status_t hrm::action_r2r_set_t::exec()
{
  switch (m_status) {
    case irs_st_busy: {
      if (m_timer.check()) {
        m_timer.stop();
        if (*mp_ready_bit == false) {
          m_status = irs_st_ready;
        } else {
          m_status = irs_st_error;
        }
      }
      break;
    }
    case irs_st_ready: {
      m_status = irs_st_busy;
      m_timer.set(*mp_interval);
      m_timer.start();
      *mp_device_code = *mp_target_code;
      (*mp_message_string) = irst("Установка кода ");
      (*mp_message_string) += irs::irsstr_from_number_russian(irs::char_t(),
        *mp_target_code);
      *mp_new_message = true;
      break;
    }
    case irs_st_error: {
      (*mp_message_string) = irst("Ошибка: время ожидания готовности больше ") +
        irs::irsstr_from_number_russian(irs::char_t(),
        CNT_TO_DBLTIME(*mp_interval)) + irst(" c)");
      *mp_new_message = true;
      break;
    }
  }
  return m_status;
}

void hrm::action_r2r_set_t::reset()
{
  m_timer.stop();
  m_status = irs_st_ready;
}

hrm::action_relay_set_t::action_relay_set_t(
  operation_t a_operation,
  irs::bit_data_t* ap_on_bit,
  irs::bit_data_t* ap_ready_bit,
  const counter_t* ap_interval,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  m_operation(a_operation),
  mp_on_bit(ap_on_bit),
  mp_ready_bit(ap_ready_bit),
  mp_interval(ap_interval),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message),
  m_timer(*mp_interval),
  m_status(irs_st_ready)
{
  m_timer.stop();
}

irs_status_t hrm::action_relay_set_t::exec()
{
  switch (m_status) {
    case irs_st_busy: {
      if (m_timer.check()) {
        m_timer.stop();
        if (*mp_ready_bit == false) {
          m_status = irs_st_ready;
        } else {
          m_status = irs_st_error;
        }
      }
      break;
    }
    case irs_st_ready: {
      m_status = irs_st_busy;
      m_timer.set(*mp_interval);
      m_timer.start();
      if (m_operation == op_on) {
        *mp_on_bit = 1;
        (*mp_message_string) = irst("Включение реле");
      } else {
        *mp_on_bit = 0;
        (*mp_message_string) = irst("Выключение реле");
      }
      *mp_new_message = true;
      break;
    }
    case irs_st_error: {
      (*mp_message_string) =
          irst("Ошибка relay_set_t: время ожидания готовности больше ") +
        ntostr(CNT_TO_DBLTIME(*mp_interval)) + irst(" c");
      *mp_new_message = true;
      break;
    }
  }
  return m_status;
}

void hrm::action_relay_set_t::reset()
{
  m_timer.stop();
  m_status = irs_st_ready;
}

hrm::action_r2r_calibre_code_generate_t::
action_r2r_calibre_code_generate_t(
  code_type_t a_code_type,
  size_t* ap_iteration,
  size_t* ap_iteration_count,
  code_t* ap_result_code,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  m_code_type(a_code_type),
  mp_iteration(ap_iteration),
  mp_iteration_count(ap_iteration_count),
  mp_result_code(ap_result_code),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_r2r_calibre_code_generate_t::exec()
{
  irs_status_t status = irs_st_error;
  if ((*mp_iteration < *mp_iteration_count) &&
      (*mp_iteration_count < 8 * sizeof(*mp_result_code))) {
    status = irs_st_ready;
    switch (m_code_type) {
      case ct_msd: {
        *mp_result_code =  1 << (*mp_iteration_count - *mp_iteration - 1);
        break;
      }
      case ct_lsd: {
        *mp_result_code =  (1 << (*mp_iteration_count - *mp_iteration - 1)) - 1;
        break;
      }
      case ct_zero: {
        *mp_result_code = 0;
        break;
      }
    }
    (*mp_message_string) = irst("Сгенерирован код ");
    (*mp_message_string) += irs::irsstr_from_number_russian(irs::char_t(),
      *(mp_result_code));
  } else {
    (*mp_message_string) = irst("Генерация кода: ошибочные входные данные");
  }
  *mp_new_message = true;
  return status;
}

hrm::action_r2r_balance_code_generate_t::
action_r2r_balance_code_generate_t(
  direction_t* ap_direction,
  size_t* ap_iteration,
  size_t* ap_iteration_count,
  code_t* ap_code_bias,
  code_t* ap_result_code,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_direction(ap_direction),
  mp_iteration(ap_iteration),
  mp_iteration_count(ap_iteration_count),
  mp_code_bias(ap_code_bias),
  mp_result_code(ap_result_code),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_r2r_balance_code_generate_t::exec()
{
  irs_status_t status = irs_st_error;
  if ((*mp_iteration < *mp_iteration_count) &&
      (*mp_iteration_count < 8 * sizeof(*mp_result_code))) {
    status = irs_st_ready;
    if (*mp_iteration == 0) {
      *mp_result_code = (1 << (*mp_iteration_count - 1)) + *mp_code_bias;
    } else {
      code_t step = 1 << (*mp_iteration_count - *mp_iteration - 1);
      //if (step > abs(*mp_code_bias)) {
      if (true) {
        switch (*mp_direction) {
          case di_up: {
            *mp_result_code += step;
            break;
          }
          case di_down: {
            *mp_result_code -= step;
            break;
          }
        }
      } else {
        //
      }
    }
    (*mp_message_string) = irst("Сгенерирован код ");
    (*mp_message_string) += ntostr(*(mp_result_code));
  } else {
    (*mp_message_string) = irst("Генерация кода: ошибочные входные данные");
  }
  *mp_new_message = true;
  return status;
}

hrm::action_r2r_buffer_test_code_generate_t::
action_r2r_buffer_test_code_generate_t(
  size_t* ap_iteration,
  code_t* ap_result_code,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_iteration(ap_iteration),
  mp_result_code(ap_result_code),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_r2r_buffer_test_code_generate_t::exec()
{
  irs_status_t status = irs_st_error;
  bool text_exist = mp_new_message && mp_message_string;
  if (mp_iteration && mp_result_code) {
    status = irs_st_ready;
    *mp_result_code = (1 << (*mp_iteration));
    if (text_exist) {
      (*mp_message_string) = irst("Сгенерирован код ");
      (*mp_message_string) += ntostr(*(mp_result_code));
      (*mp_new_message) = true;
    }
  } else {
    if (text_exist) {
      (*mp_message_string) = irst("Генерация кода: ошибочные входные данные");
      *mp_new_message = true;
    }
  }
  return status;
}

hrm::action_r2r_scan_code_generate_t::action_r2r_scan_code_generate_t(
  code_t* ap_code,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_code(ap_code),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_r2r_scan_code_generate_t::exec()
{
  irs_status_t status = irs_st_error;
  bool text_exist = mp_new_message && mp_message_string;
  if (mp_code) {
    status = irs_st_ready;
    *mp_code = (*mp_code) + 1;
    if (text_exist) {
      (*mp_message_string) = irst("Сгенерирован код ");
      (*mp_message_string) += ntostr(*(mp_code));
      (*mp_new_message) = true;
    }
  } else {
    if (text_exist) {
      (*mp_message_string) = irst("Генерация кода: ошибочные входные данные");
      *mp_new_message = true;
    }
  }
  return status;
}

hrm::action_r2r_balance_dir_generate_t::
action_r2r_balance_dir_generate_t(
  direction_t* ap_direction,
  balance_mode_t* ap_balance_mode,
  double* ap_multimeter_value,
  bool* ap_comparator_value,
  double *ap_zero_shift_value,
  irs::string_t *ap_message_string, bool *ap_new_message):
  mp_direction(ap_direction),
  mp_balance_mode(ap_balance_mode),
  mp_multimeter_value(ap_multimeter_value),
  mp_comparator_value(ap_comparator_value),
  mp_zero_shift_value(ap_zero_shift_value),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_r2r_balance_dir_generate_t::exec()
{
  irs_status_t status = irs_st_error;
  bool text_exist = mp_message_string && mp_new_message;
  if (mp_direction && mp_balance_mode && mp_multimeter_value
      && mp_comparator_value) {
    status = irs_st_ready;
    switch (*mp_balance_mode) {
      case bm_multimeter: {
        if (*mp_multimeter_value - *mp_zero_shift_value > 0.) {
          *mp_direction = di_up;
          if (text_exist) {
            (*mp_message_string) = irst("Код будет увеличен");
          }
        } else {
          *mp_direction = di_down;
          if (text_exist) {
            (*mp_message_string) = irst("Код будет уменьшен");
          }
        }
        break;
      }
      case bm_comparator: {
        if (*mp_comparator_value) {
          *mp_direction = di_up;
          if (text_exist) {
            (*mp_message_string) = irst("Код будет увеличен");
          }
        } else {
          *mp_direction = di_down;
          if (text_exist) {
            (*mp_message_string) = irst("Код будет уменьшен");
          }
        }
        break;
      }
    }
  } else {
    if (text_exist) {
      (*mp_message_string) = irst("Генерация кода: ошибочные входные данные");
    }
  }
  if (text_exist) {
    *mp_new_message = true;
  }
  return status;
}

hrm::action_r2r_balance_code_bias_generate_t::
action_r2r_balance_code_bias_generate_t(
  vector<double>* ap_calibre_vector,
  code_t* ap_code_bias,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_calibre_vector(ap_calibre_vector),
  mp_code_bias(ap_code_bias),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_r2r_balance_code_bias_generate_t::exec()
{
  irs_status_t status = irs_st_error;
  bool text_exist = mp_message_string && mp_new_message;
  if (mp_calibre_vector && mp_calibre_vector->size() == r2r_resolution
    && mp_code_bias) {
    status = irs_st_ready;
    //  Последний элемент - младший разряд ЦАПа
    double lsd = mp_calibre_vector->back();
    double lsb_sum = lsd;
    size_t i = 0;
    for (i = 1; i < mp_calibre_vector->size() - 1; i++) {
      size_t inv_i = (mp_calibre_vector->size() - i - 1);
      double msd = (*mp_calibre_vector)[inv_i] * pow(2., i);
      if (fabs(lsb_sum - msd) > 2. * lsd) {
        *mp_code_bias = pow(2., i-1) + 1;
        break;
      }
      lsb_sum += msd;
    }

    if (text_exist) {
      (*mp_message_string) = irst("Хороших, годных разрядов: ");
      (*mp_message_string) += ntostr(mp_calibre_vector->size() - i);
      (*mp_message_string) += 13;
      (*mp_message_string) += irst("Смещение кода: ");
      (*mp_message_string) += ntostr(*mp_code_bias);
      *mp_new_message = true;
    }
  } else if (text_exist) {
    (*mp_message_string) =
      irst("Вычисление начального смещения: ошибочные входные данные");
    *mp_new_message = true;
  }
  return status;
}

hrm::action_loop_t::action_loop_t(
  size_t* ap_current_point,
  size_t* ap_return_loop_point,
  size_t* ap_current_iteration,
  size_t* ap_iteration_count,
  irs::string_t *ap_message_string,
  bool *ap_new_message):
  mp_current_point(ap_current_point),
  mp_return_loop_point(ap_return_loop_point),
  mp_current_iteration(ap_current_iteration),
  mp_iteration_count(ap_iteration_count),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
  if (mp_current_iteration) {
    *mp_current_iteration = 0;
  }
}

irs_status_t hrm::action_loop_t::exec()
{
  if (mp_current_iteration && mp_iteration_count) {
    if (*mp_current_iteration < *mp_iteration_count - 1) {
      *mp_current_point = *mp_return_loop_point;
      if (mp_message_string) {
        (*mp_message_string) = irst("Проход цикла ");
        (*mp_message_string) += irs::irsstr_from_number_russian(irs::char_t(),
          (*mp_current_iteration + 1));
        (*mp_message_string) += irst(" из ");
        (*mp_message_string) += irs::irsstr_from_number_russian(irs::char_t(),
          *mp_iteration_count);
        (*mp_message_string) += 13;
        if (mp_new_message) {
          *mp_new_message = true;
        }
      }
      (*mp_current_iteration)++;
    } else {
      if (mp_message_string) {
        (*mp_message_string) = irst("Цикл завершён");
        (*mp_message_string) += 13;
        if (mp_new_message) {
          *mp_new_message = true;
        }
      }
      (*mp_current_point)++;
      *mp_current_iteration = 0;
    }
  } else {
    //  Бесконечный цикл
    *mp_current_point = *mp_return_loop_point;
  }
  return irs_st_ready;
}

counter_t hrm::action_loop_t::time()
{
  return 0;
}

void hrm::action_loop_t::reset()
{
  if (mp_current_iteration) {
    *mp_current_iteration = 0;
  }
}

hrm::action_set_mult_channel_t::action_set_mult_channel_t(
  irs::agilent_34420a_t *ap_multimeter,
  size_t a_channel,
  counter_t *ap_error_interval,
  irs::string_t *ap_message_string,
  bool *ap_new_message):
  m_status(irs_st_ready),
  mp_multimeter(ap_multimeter),
  m_channel(a_channel),
  mp_error_interval(ap_error_interval),
  m_timer(*mp_error_interval),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_set_mult_channel_t::exec()
{
  switch (m_status) {
    case irs_st_ready: {
      if (mp_multimeter->status() == meas_status_success) {
        mp_multimeter->select_channel(m_channel);
        m_timer.set(*mp_error_interval);
        m_timer.start();
        (*mp_message_string) = irst("Установка канала мультиметра ") +
          irs::irsstr_from_number_russian(irs::char_t(), m_channel);
        m_status = irs_st_busy;
      } else {
        (*mp_message_string) = irst("Ошибка: мультиметр не готов");
        m_status = irs_st_error;
      }
      *mp_new_message = true;
      break;
    }
    case irs_st_busy: {
      if (mp_multimeter->status() == meas_status_success) {
        m_timer.stop();
        m_status = irs_st_ready;
      } else {
        if (m_timer.check()) {
          m_timer.stop();
          m_status = irs_st_error;
        }
      }
      break;
    }
    case irs_st_error: {
      (*mp_message_string) = irst("Ошибка: время ожидания готовности больше ") +
        irs::irsstr_from_number_russian(irs::char_t(),
        CNT_TO_DBLTIME(*mp_error_interval)) + irst(" c");
      *mp_new_message = true;
      m_status = irs_st_ready;
      break;
    }
  }
  return m_status;
}

void hrm::action_set_mult_channel_t::reset()
{
  m_timer.stop();
  m_status = irs_st_ready;
}

hrm::action_set_mult_nplc_t::action_set_mult_nplc_t(
  irs::agilent_34420a_t *ap_multimeter,
  double* ap_nplc,
  counter_t *ap_error_interval,
  irs::string_t *ap_message_string,
  bool *ap_new_message):
  m_status(irs_st_ready),
  mp_multimeter(ap_multimeter),
  mp_nplc(ap_nplc),
  mp_error_interval(ap_error_interval),
  m_timer(*mp_error_interval),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_set_mult_nplc_t::exec()
{
  switch (m_status) {
    case irs_st_ready: {
      if (mp_multimeter->status() == meas_status_success) {
        mp_multimeter->set_nplc(*mp_nplc);
        m_timer.set(*mp_error_interval);
        m_timer.start();
        (*mp_message_string) = irst("Установка NPLC мультиметра ") +
          irs::irsstr_from_number_russian(irs::char_t(), *mp_nplc);
        m_status = irs_st_busy;
      } else {
        (*mp_message_string) = irst("Ошибка: мультиметр не готов");
        m_status = irs_st_error;
      }
      *mp_new_message = true;
      break;
    }
    case irs_st_busy: {
      if (mp_multimeter->status() == meas_status_success) {
        m_timer.stop();
        m_status = irs_st_ready;
      } else {
        if (m_timer.check()) {
          m_timer.stop();
          m_status = irs_st_error;
        }
      }
      break;
    }
    case irs_st_error: {
      (*mp_message_string) = irst("Ошибка: время ожидания готовности больше ") +
        irs::irsstr_from_number_russian(irs::char_t(),
        CNT_TO_DBLTIME(*mp_error_interval)) + irst(" c)");
      *mp_new_message = true;
      m_status = irs_st_ready;
      break;
    }
  }
  return m_status;
}

void hrm::action_set_mult_nplc_t::reset()
{
  m_timer.stop();
  m_status = irs_st_ready;
}

hrm::action_mult_meas_t::action_mult_meas_t(
    irs::agilent_34420a_t* ap_multimeter,
    meas_type_t a_meas_type,
    double* ap_nplc,
    double* ap_value,
    bool *ap_new_value,
    counter_t *ap_error_interval,
    irs::string_t *ap_message_string, bool *ap_new_message):
  m_status(irs_st_ready),
  mp_multimeter(ap_multimeter),
  m_meas_type(a_meas_type),
  mp_nplc(ap_nplc),
  mp_value(ap_value),
  mp_new_value(ap_new_value),
  mp_error_interval(ap_error_interval),
  m_timer(*mp_error_interval),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_mult_meas_t::exec()
{
  switch (m_status) {
    case irs_st_ready: {
      if (mp_multimeter->status() == meas_status_success) {
        switch (m_meas_type) {
          case mt_init: {
            if (mp_message_string) {
              (*mp_message_string) = irst("Запуск измерения напряжения");
            }
            mp_multimeter->get_voltage(mp_value);
            break;
          }
          case mt_regular: {
            if (mp_message_string) {
              (*mp_message_string) = irst("Запуск текущего измерения");
            }
            mp_multimeter->get_value(mp_value);
            break;
          }
        }

        m_timer.set(*mp_error_interval +
          irs::make_cnt_ms(2000 + 20 * (*mp_nplc)));
        m_timer.start();
        m_status = irs_st_busy;
      } else {
        if (mp_message_string) {
          (*mp_message_string) = irst("Ошибка: мультиметр не готов");
        }
        m_status = irs_st_error;
      }
      if (mp_new_message) {
        *mp_new_message = true;
      }
      break;
    }
    case irs_st_busy: {
      if (mp_multimeter->status() == meas_status_success) {
        *mp_new_value = true;
        if (mp_message_string) {
          (*mp_message_string) = irst("Измеренное значение ") +
            irs::irsstr_from_number_russian(irs::char_t(), *mp_value) +
            irst(" В");
          if (mp_new_message) {
            *mp_new_message = true;
          }
        }
        m_timer.stop();
        m_status = irs_st_ready;
      } else {
        if (m_timer.check()) {
          m_timer.stop();
          m_status = irs_st_error;
        }
      }
      break;
    }
    case irs_st_error: {
      if (mp_message_string) {
        (*mp_message_string) =
          irst("Ошибка: время ожидания готовности больше ") +
          irs::irsstr_from_number_russian(irs::char_t(),
          CNT_TO_DBLTIME(*mp_error_interval) +
          irs::make_cnt_ms(2000 + 20 * (*mp_nplc))) + irst(" c)");
        if (mp_new_message) {
          *mp_new_message = true;
        }
      }
      m_status = irs_st_ready;
      break;
    }
  }
  return m_status;
}

void hrm::action_mult_meas_t::reset()
{
  m_timer.stop();
  m_status = irs_st_ready;
}

hrm::action_wait_mult_t::action_wait_mult_t(
  irs::agilent_34420a_t *ap_multimeter,
  counter_t *ap_error_interval,
  irs::string_t *ap_message_string,
  bool *ap_new_message):
  m_status(irs_st_ready),
  mp_multimeter(ap_multimeter),
  mp_error_interval(ap_error_interval),
  m_timer(*mp_error_interval),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_wait_mult_t::exec()
{
  switch (m_status) {
    case irs_st_ready: {
      if (mp_multimeter->status() == meas_status_success) {
        (*mp_message_string) = irst("Мультиметр готов");
        *mp_new_message = true;
      } else {
        m_timer.set(*mp_error_interval);
        m_timer.start();
        m_status = irs_st_busy;
      }
      break;
    }
    case irs_st_busy: {
      if (mp_multimeter->status() == meas_status_success) {
        (*mp_message_string) = irst("Мультиметр готов");
        *mp_new_message = true;
        m_timer.stop();
        m_status = irs_st_ready;
      } else {
        if (m_timer.check()) {
          m_timer.stop();
          m_status = irs_st_error;
        }
      }
      break;
    }
    case irs_st_error: {
      (*mp_message_string) = irst("Ошибка: время ожидания готовности больше ") +
        irs::irsstr_from_number_russian(irs::char_t(),
        CNT_TO_DBLTIME(*mp_error_interval)) + irst(" c)");
      *mp_new_message = true;
      m_status = irs_st_ready;
      break;
    }
  }
  return m_status;
}

void hrm::action_wait_mult_t::reset()
{
  m_timer.stop();
  m_status = irs_st_ready;
}

hrm::action_convert_calibre_code_t::action_convert_calibre_code_t(
  vector<double>* ap_calibre_vector,
  code_t* ap_in_code,
  code_t* ap_out_code,
  irs::string_t *ap_message_string,
  bool *ap_new_message):
  mp_calibre_vector(ap_calibre_vector),
  mp_in_code(ap_in_code),
  mp_out_code(ap_out_code),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_convert_calibre_code_t::exec()
{
  irs_status_t status = irs_st_error;
  bool text_exist = mp_message_string && mp_new_message;

  if (mp_calibre_vector && mp_calibre_vector->size() == r2r_resolution &&
      mp_out_code && mp_in_code) {
    status = irs_st_ready;
    //*mp_out_code = 0;
//    double code = 0;
//    for (size_t i = 0; i < mp_calibre_vector->size(); i++) {
//      size_t inv_i = mp_calibre_vector->size() - i - 1;
//      double weidht = pow(2., i) / (*mp_calibre_vector)[inv_i];
//      if (*mp_in_code & (1 << i)) {
//        //*mp_out_code += static_cast<code_t>(weidht);
//        code += weidht;
//      }
//    }
    *mp_out_code
      = static_cast<code_t>(convert_code(mp_calibre_vector, *mp_in_code)); //(code);
    if (text_exist) {
      (*mp_message_string) = irst("Код: ") +
        ntostr(*mp_in_code) + irst(" -> ") + ntostr(*mp_out_code);
      *mp_new_message = true;
    }
  } else {
    if (text_exist) {
      (*mp_message_string) =
        irst("action_convert_calibre_code_t: ошибочные входные данные");
      *mp_new_message = true;
    }
  }
  return status;
}

hrm::action_bridge_emulate_t::action_bridge_emulate_t(
  double* ap_bridge_in_voltage,
  double* ap_bridge_out_voltage,
  double* ap_etalon,
  double* ap_checked,
  code_t* ap_in_code,
  irs::string_t *ap_message_string,
  bool *ap_new_message):
  mp_bridge_in_voltage(ap_bridge_in_voltage),
  mp_bridge_out_voltage(ap_bridge_out_voltage),
  mp_etalon(ap_etalon),
  mp_checked(ap_checked),
  mp_in_code(ap_in_code),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

irs_status_t hrm::action_bridge_emulate_t::exec()
{
  irs_status_t status = irs_st_error;
  bool text_exist = mp_message_string && mp_new_message;

  if (mp_bridge_out_voltage && mp_bridge_in_voltage && *mp_bridge_in_voltage > 0
      && mp_etalon && *mp_etalon > 0. && mp_checked && *mp_checked > 0. &&
      mp_in_code) {
    status = irs_st_ready;
    double coils_mid = *mp_checked / (*mp_etalon + *mp_checked);
    double dac = static_cast<double>(*mp_in_code) / pow(2., 24);
    *mp_bridge_out_voltage = *mp_bridge_in_voltage * (coils_mid - dac);// + 60e-6;
    if (text_exist) {
      (*mp_message_string) =
        irst("Эмуляция моста (coils - dac), напряжение: ") +
        ntostr(*mp_bridge_out_voltage);
      (*mp_message_string) += 13;
      (*mp_message_string) += irst("Эталон: ") + ntostr(*mp_etalon) +
        irst(" Ом");
      (*mp_message_string) += 13;
      (*mp_message_string) += irst("Поверяемая: ") + ntostr(*mp_checked) +
        irst(" Ом");
      (*mp_message_string) += 13;
      (*mp_message_string) += irst("ЦАП: ") + ntostr(dac) +
        irst(" В");
      (*mp_message_string) += 13;
      (*mp_message_string) += irst("Катушки: ") + ntostr(coils_mid) +
        irst(" В");
      *mp_new_message = true;
    }
  } else {
    if (text_exist) {
      (*mp_message_string) =
        irst("action_bridge_emulate_t: ошибочные входные данные");
      *mp_new_message = true;
    }
  }
  return status;
}

