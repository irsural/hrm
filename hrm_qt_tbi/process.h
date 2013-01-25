#ifndef processH
#define processH

#include <irsdefs.h>
#include <hrm_defs.h>
#include <timer.h>
#include <mxdata.h>
#include <irsstring.h>
#include <measmul.h>
#include <irsfinal.h>

namespace hrm {

class action_t
{
public:
  virtual ~action_t() {}
  virtual irs_status_t exec() = 0;
  virtual bool linear() = 0;
  virtual counter_t time() = 0;
  virtual void reset() = 0;
};

class action_pause_t : public action_t
{
public:
  action_pause_t(
    const counter_t* ap_interval,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_pause_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return *mp_interval; }
  virtual void reset();
private:
  const counter_t* mp_interval;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
  irs::timer_t m_timer;
  irs_status_t m_status;
};

class action_r2r_set_t : public action_t
{
public:
  action_r2r_set_t(
    code_t* ap_target_code,
    irs::conn_data_t<irs_u32>* ap_device_code,
    irs::bit_data_t* ap_ready_bit,
    const counter_t* ap_interval,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_r2r_set_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return *mp_interval; }
  virtual void reset();
private:
  code_t* mp_target_code;
  irs::conn_data_t<irs_u32>* mp_device_code;
  irs::bit_data_t* mp_ready_bit;
  const counter_t* mp_interval;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
  irs::timer_t m_timer;
  irs_status_t m_status;
};

class action_relay_set_t : public action_t
{
public:
  enum operation_t {
    op_on,
    op_off
  };

  action_relay_set_t(
    operation_t a_operation,
    irs::bit_data_t* ap_on_bit,
    irs::bit_data_t* ap_ready_bit,
    const counter_t* ap_interval,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_relay_set_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return *mp_interval; }
  virtual void reset();
private:
  bool m_operation;
  irs::bit_data_t* mp_on_bit;
  irs::bit_data_t* mp_ready_bit;
  const counter_t* mp_interval;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
  irs::timer_t m_timer;
  irs_status_t m_status;
};

class action_r2r_calibre_code_generate_t : public action_t
{
public:
  enum code_type_t {
    ct_msd,
    ct_lsd,
    ct_zero
  };

  action_r2r_calibre_code_generate_t(
    code_type_t a_code_type,
    size_t* ap_iteration,
    size_t* ap_iteration_count,
    code_t* ap_result_code,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_r2r_calibre_code_generate_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  code_type_t m_code_type;
  size_t* mp_iteration;
  size_t* mp_iteration_count;
  code_t* mp_result_code;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_r2r_balance_code_generate_t : public action_t
{
public:
  action_r2r_balance_code_generate_t(
    direction_t* ap_direction,
    size_t* ap_iteration,
    size_t* ap_iteration_count,
    code_t* ap_code_bias,
    code_t* ap_result_code,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_r2r_balance_code_generate_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  direction_t* mp_direction;
  size_t* mp_iteration;
  size_t* mp_iteration_count;
  code_t* mp_code_bias;
  code_t* mp_result_code;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_r2r_buffer_test_code_generate_t : public action_t
{
public:
  action_r2r_buffer_test_code_generate_t(
    size_t* ap_iteration,
    code_t* ap_result_code,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_r2r_buffer_test_code_generate_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  size_t* mp_iteration;
  code_t* mp_result_code;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_r2r_scan_code_generate_t : public action_t
{
public:
  action_r2r_scan_code_generate_t(
    code_t* ap_code,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_r2r_scan_code_generate_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  code_t* mp_code;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_r2r_balance_dir_generate_t : public action_t
{
public:
  action_r2r_balance_dir_generate_t(
    direction_t* ap_direction,
    balance_mode_t* ap_balance_mode,
    double* ap_multimeter_value,
    bool* ap_comparator_value,
    double* ap_zero_shift_value,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_r2r_balance_dir_generate_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  direction_t* mp_direction;
  balance_mode_t* mp_balance_mode;
  double* mp_multimeter_value;
  bool* mp_comparator_value;
  double* mp_zero_shift_value;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_r2r_balance_code_bias_generate_t : public action_t
{
public:
  action_r2r_balance_code_bias_generate_t(
    vector<double>* ap_calibre_vector,
    code_t* ap_code_bias,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_r2r_balance_code_bias_generate_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  vector<double>* mp_calibre_vector;
  code_t* mp_code_bias;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

template <class T>
class action_reset_average_t : public action_t
{
public:
  action_reset_average_t(
    vector<T>* ap_vector,
    T *ap_result,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_reset_average_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  vector<T>* mp_vector;
  T* mp_result;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

template <class T>
class action_clear_vector_t : public action_t
{
public:
  action_clear_vector_t(
    vector<T>* ap_vector,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_clear_vector_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  vector<T>* mp_vector;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_loop_t : public action_t
{
public:
  action_loop_t(
    size_t* ap_current_point,
    size_t* ap_return_loop_point,
    size_t* ap_current_iteration,
    size_t* ap_iteration_count,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_loop_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return false; }
  virtual counter_t time();
  virtual void reset();
private:
  size_t* mp_current_point;
  size_t* mp_return_loop_point;
  size_t* mp_current_iteration;
  size_t* mp_iteration_count;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

template <class T>
class action_brlo_t : public action_t //  Branch if left < right
{
public:
  action_brlo_t(
    size_t* ap_current_point,
    size_t* ap_branch_point,
    T* ap_left_arg,
    T* ap_right_arg,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_brlo_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return false; }
  virtual counter_t time() { return 0; }
  virtual void reset() {}
private:
  size_t* mp_current_point;
  size_t* mp_branch_point;
  T* mp_left_arg;
  T* mp_right_arg;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_set_mult_channel_t : public action_t
{
public:
  action_set_mult_channel_t(
    irs::agilent_34420a_t* ap_multimeter,
    size_t a_channel,
    counter_t* ap_error_interval,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_set_mult_channel_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return irs::make_cnt_s(1); }
  virtual void reset();
private:
  irs_status_t m_status;
  irs::agilent_34420a_t* mp_multimeter;
  size_t m_channel;
  counter_t* mp_error_interval;
  irs::timer_t m_timer;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_set_mult_nplc_t : public action_t
{
public:
  action_set_mult_nplc_t(
    irs::agilent_34420a_t* ap_multimeter,
    double* ap_nplc,
    counter_t* ap_error_interval,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_set_mult_nplc_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return irs::make_cnt_s(1); }
  virtual void reset();
private:
  irs_status_t m_status;
  irs::agilent_34420a_t* mp_multimeter;
  double* mp_nplc;
  counter_t* mp_error_interval;
  irs::timer_t m_timer;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_mult_meas_t : public action_t
{
public:
  enum meas_type_t {
    mt_init,
    mt_regular
  };

  action_mult_meas_t(
    irs::agilent_34420a_t* ap_multimeter,
    meas_type_t a_meas_type,
    double* ap_nplc,
    double* ap_value,
    bool* ap_new_value,
    counter_t* ap_error_interval,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_mult_meas_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time()
  {
    return irs::make_cnt_ms(2000 + 20 * (*mp_nplc));
  }
  virtual void reset();
private:
  irs_status_t m_status;
  irs::agilent_34420a_t* mp_multimeter;
  meas_type_t m_meas_type;
  double* mp_nplc;
  double* mp_value;
  bool* mp_new_value;
  counter_t* mp_error_interval;
  irs::timer_t m_timer;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_wait_mult_t : public action_t
{
public:
  action_wait_mult_t(
    irs::agilent_34420a_t* ap_multimeter,
    counter_t* ap_error_interval,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_wait_mult_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return irs::make_cnt_ms(100); }
  virtual void reset();
private:
  irs_status_t m_status;
  irs::agilent_34420a_t* mp_multimeter;
  counter_t* mp_error_interval;
  irs::timer_t m_timer;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

template <class T>
class action_push_back_t : public action_t
{
public:
  action_push_back_t(
    vector<T>* ap_vector,
    T* ap_value,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_push_back_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  vector<T>* mp_vector;
  T* mp_value;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_convert_calibre_code_t : public action_t
{
public:
  action_convert_calibre_code_t(
    vector<double>* ap_calibre_vector,
    code_t* ap_in_code,
    code_t* ap_out_code,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_convert_calibre_code_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  vector<double>* mp_calibre_vector;
  code_t* mp_in_code;
  code_t* mp_out_code;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

class action_bridge_emulate_t : public action_t
{
public:
  action_bridge_emulate_t(
    double* ap_bridge_in_voltage,
    double* ap_bridge_out_voltage,
    double* ap_etalon,
    double* ap_checked,
    code_t* ap_in_code,
    irs::string_t* ap_message_string,
    bool* ap_new_message);
  virtual ~action_bridge_emulate_t() {}
  virtual irs_status_t exec();
  virtual inline bool linear() { return true; }
  virtual inline counter_t time() { return 0; }
  virtual inline void reset() {}
private:
  double* mp_bridge_in_voltage;
  double* mp_bridge_out_voltage;
  double* mp_etalon;
  double* mp_checked;
  code_t* mp_in_code;
  irs::string_t* mp_message_string;
  bool* mp_new_message;
};

} //  hrm

template <class T>
hrm::action_reset_average_t<T>::
action_reset_average_t(
  vector<T>* ap_vector,
  T* ap_result,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_vector(ap_vector),
  mp_result(ap_result),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

template <class T>
irs_status_t hrm::action_reset_average_t<T>::exec()
{
  irs_status_t status = irs_st_error;
  bool text_exist = mp_message_string && mp_new_message;
  if (mp_vector && mp_vector->size() && mp_result) {
    status = irs_st_ready;
    double count = mp_vector->size();
    *mp_result = accumulate(mp_vector->begin(), mp_vector->end(), 0.) / count;
    mp_vector->clear();
    if (text_exist) {
      (*mp_message_string) = irst("Усредненённое значение: ");
      (*mp_message_string) += ntostr(*mp_result);
      *mp_new_message = true;
    }
  } else if (text_exist) {
    (*mp_message_string) = irst("Усреднение: ошибочные входные данные");
    *mp_new_message = true;
  }
  return status;
}

template <class T>
hrm::action_clear_vector_t<T>::
action_clear_vector_t(
  vector<T>* ap_vector,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_vector(ap_vector),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

template <class T>
irs_status_t hrm::action_clear_vector_t<T>::exec()
{
  irs_status_t status = irs_st_error;
  bool text_exist = mp_message_string && mp_new_message;
  if (mp_vector) {
    status = irs_st_ready;
    mp_vector->clear();
    if (text_exist) {
      (*mp_message_string) = irst("Вектор очищен");
      *mp_new_message = true;
    }
  } else if (text_exist) {
    (*mp_message_string) = irst("Очистка вектора: ошибочные входные данные");
    *mp_new_message = true;
  }
  return status;
}

template <class T>
hrm::action_brlo_t<T>::action_brlo_t(
  size_t* ap_current_point,
  size_t* ap_branch_point,
  T *ap_left_arg,
  T *ap_right_arg,
  irs::string_t* ap_message_string,
  bool* ap_new_message):
  mp_current_point(ap_current_point),
  mp_branch_point(ap_branch_point),
  mp_left_arg(ap_left_arg),
  mp_right_arg(ap_right_arg),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

template <class T>
irs_status_t hrm::action_brlo_t<T>::exec()
{
  if (mp_left_arg && mp_right_arg) {
    if (*mp_left_arg < *mp_right_arg) {
      *mp_current_point = *mp_branch_point;
      if (mp_message_string) {
        (*mp_message_string) = irst("Ветвление (left < right): переход");
        (*mp_message_string) += 13;
        if (mp_new_message) {
          *mp_new_message = true;
        }
      }
    } else {
      (*mp_current_point)++;
      if (mp_message_string) {
        (*mp_message_string) = irst("Ветвление (left < right): проход");
        (*mp_message_string) += 13;
        if (mp_new_message) {
          *mp_new_message = true;
        }
      }
    }
  }
  return irs_st_ready;
}

template <class T>
hrm::action_push_back_t<T>::action_push_back_t(
  vector<T>* ap_vector,
  T* ap_value,
  irs::string_t *ap_message_string,
  bool *ap_new_message):
  mp_vector(ap_vector),
  mp_value(ap_value),
  mp_message_string(ap_message_string),
  mp_new_message(ap_new_message)
{
}

template <class T>
irs_status_t hrm::action_push_back_t<T>::exec()
{
  mp_vector->push_back(*mp_value);
  if (mp_message_string && mp_new_message) {
    (*mp_message_string) = irst("В вектор добавлен ");
    (*mp_message_string) += ntostr(mp_vector->size() - 1);
    (*mp_message_string) += irst("-й элемент");
    *mp_new_message = true;
  }
  return irs_st_ready;
}

#endif  //  processH
