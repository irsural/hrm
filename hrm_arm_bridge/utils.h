#ifndef utilsH
#define utilsH

#include <irsdefs.h>

#include <irsspi.h>
#include <irsgpio.h>
#include <timer.h>
#include <irserror.h>
#include <mxdata.h>
#include <irslimits.h>
#include <irsadc.h>
#include <irsdsp.h>
#include <irsalg.h>
#include <hrm_bridge_data.h>

#include "cfg.h"
#include "imp_filt.h"

#include <irsfinal.h>

namespace hrm {

enum const_t {
  m_adc_midscale = (2 << 22)
};

class adc_dac_request_t
{
public:
  adc_dac_request_t(
    irs::spi_t *ap_spi,
    irs::gpio_pin_t *ap_adc_cs_pin,
    irs::gpio_pin_t *ap_dac_cs_pin,
    irs::gpio_pin_t *ap_dac_rst_pin,
    irs::gpio_pin_t *ap_dac_ldac_pin);
  ~adc_dac_request_t();
  void tick();
private:
  enum status_t {
    st_adc_reset,
    st_adc_spi_prepare,
    st_adc_spi_wait,
    st_adc_read_prepare,
    st_adc_read,
    st_dac_reset,
    st_dac_spi_prepare,
    st_dac_spi_wait,
    st_dac_read_prepare,
    st_dac_readback_prepare,
    st_dac_readback,
    st_free
  };
  enum {
    m_spi_buf_size = 5,
    m_adc_reset_size = 4
  };
  status_t m_status;
  status_t m_target_status;
  irs::spi_t* mp_spi;
  irs_u8 mp_spi_buf[m_spi_buf_size];
  size_t m_spi_transaction_size;
  irs::timer_t m_timer;
  irs::gpio_pin_t* mp_adc_cs_pin;
  irs::gpio_pin_t* mp_dac_cs_pin;
  irs::gpio_pin_t* mp_dac_rst_pin;
  irs::gpio_pin_t* mp_dac_ldac_pin;
};

class relay_t
{
public:
  typedef int bit_t;

  virtual bit_t operator=(const bit_t a_elem) = 0;
  virtual operator bit_t() = 0;
  virtual irs_status_t status() = 0;
  virtual void set_after_pause(counter_t a_after_pause) = 0;
  virtual void tick() = 0;
};

class mono_relay_t: public relay_t
{
public:
  mono_relay_t(irs::gpio_pin_t* ap_pin, const irs::string_t& a_caption,
    relay_t::bit_t a_default_value = 0);
  ~mono_relay_t() {};
  virtual bit_t operator=(const bit_t a_elem);
  virtual operator bit_t();
  virtual irs_status_t status();
  virtual void set_after_pause(counter_t a_after_pause);
  virtual void tick();
private:
  bool m_pin;
  irs::gpio_pin_t* mp_pin;
  counter_t m_after_pause;
  irs_status_t m_status;
  irs::timer_t m_timer;
  const irs::string_t m_caption;
  void set(const bit_t a_value);
};

class bi_relay_t: public relay_t
{
public:
  bi_relay_t(
    irs::gpio_pin_t* ap_pin_0,
    irs::gpio_pin_t* ap_pin_1,
    const irs::string_t& a_caption,
    relay_t::bit_t a_default_value = 0,
    counter_t a_energization_time = irs::make_cnt_ms(50),
    bool a_wild = false);
  ~bi_relay_t() {};
  virtual bit_t operator=(const bit_t a_elem);
  virtual operator bit_t();
  virtual irs_status_t status();
  virtual void set_after_pause(counter_t a_after_pause);
  virtual void tick();
  inline void set_wild(bool a_wild) { m_wild = a_wild;}
  inline bool wild() { return m_wild; }
private:
  enum status_t {
    st_error,
    st_wild_energization_off,
    st_wild_energization_on,
    st_energization,
    st_after_pause,
    st_ready
  };
  enum {
    m_wild_cnt = 10
  };
  irs::gpio_pin_t* mp_pin_0;
  irs::gpio_pin_t* mp_pin_1;
  counter_t m_after_pause;
  counter_t m_energization_time;
  status_t m_status;
  irs::timer_t m_timer;
  bit_t m_current_value;
  const irs::string_t m_caption;
  bool m_wild;
  irs_u8 m_wild_current_iteration;
  void set(const bit_t a_value);
};

class range_t
{
public:
  range_t();
  ~range_t() {};
  size_t add_relay(relay_t* ap_relay);
  size_t bound_range(size_t a_range);
  void range_on(size_t a_range);
  void range_off();
  bool ready();
private:
  vector<relay_t*> m_relay_vector;
  size_t m_current_range;
  bool m_on;
};

class dac_t
{
public:
  dac_t(irs::mxdata_t* ap_raw_dac);
  ~dac_t() {};
  void on();
  void off();
  irs_bool is_on();
  void set_normalize_code(dac_value_t a_code);
  void set_code(dac_value_t a_code);
  void set_lin(irs_u8 a_lin);
  dac_value_t get_normalize_code();
  irs_i32 get_code();
  irs_u8 get_lin();
  void set_after_pause(counter_t a_after_pause);
  void set_voltage_pos(dac_value_t a_voltage_pos);
  void set_voltage_neg(dac_value_t a_voltage_neg);
  dac_value_t voltage_pos();
  dac_value_t voltage_neg();
  dac_value_t output_voltage();
  irs_status_t ready();
  inline void show() { m_show = true; }
  inline void hide() { m_show = false; }
  inline double max_code() { return (pow(2.0, 19) - 1.0); }
  inline double min_code() { return -pow(2.0, 19); }
  void tick();
private:
  enum status_t {
    st_wait,
    st_pause,
    st_ready
  };
  enum {
    m_ti_dac_mid = 0xFFFFF000 / 2
  };
  irs::dac_ad5791_data_t m_dac_data;
  status_t m_status;
  counter_t m_after_pause;
  irs::timer_t m_timer;
  dac_value_t m_voltage_pos;
  dac_value_t m_voltage_neg;
  bool m_show;
};

//  ����� �� ����������� ������� ����� �� irsadc dac_8531_t, 
//  �.�. � ���� ��� ������� ����������
//  ����������� ������� �������� ��������� ���� ��� �� ������ 
//  � ���� ����������� �������
class bridge_voltage_dac_t
{
public:
  bridge_voltage_dac_t(irs::spi_t *ap_spi, irs::gpio_pin_t* ap_cs_pin, 
    double a_min_voltage, double a_max_voltage, double a_trans_coef);
  ~bridge_voltage_dac_t() {};
  //  ��� ������� set_** ���������� ������������ bound-�� ��������
  double set_voltage(double a_voltage);
  double get_voltage();
  //  ����� ����� ������ ��������� ���������� � ��
  double set_after_pause_ms(double a_after_pause_ms);
  //  ������������ �������� ��������� ���������� (�� ���������� / �� �����)
  //  ����� � �������
  double set_speed(double a_speed);
  //  ������� �������������, ��������� ��������������� ������� set_**,
  //  ������ ���� ������� �������� ���������� �� ���������� ����������,
  //  ���������� ��������� ������� set_**, ���� ��� ���������
  double sync_voltage(double a_voltage);
  double sync_speed(double a_speed);
  double sync_after_pause_ms(double a_after_pause_ms);
  irs_status_t ready();
  inline void show() { m_show = true; }
  inline void hide() { m_show = false; }
  void tick();
private:
  enum status_t {
    st_idle,
    st_wait_spi,
    st_calc_code_and_write,
    st_wait_writing,
    st_wait_dt,
    st_wait_after_pause,
    st_finish
  };
  enum {
#ifdef BRIDGE_DAC_DPOT
    dac_resolution = 8,
    write_buf_size = 2,
    dac_max_value = 0xFF
#else //  BRIDGE_DAC_DPOT
    dac_resolution = 16,
    write_buf_size = 3,
    dac_max_value = 0xFFFF
#endif  //  BRIDGE_DAC_DPOT
  };
  struct spline_t {
    double a;
    double b;
    double d;
    double dt;
    double dv;
    size_t init(double a_v1, double a_v2, double a_S, double a_dt);
    double calc(size_t a_n);
  };
  const double m_min_voltage;
  const double m_max_voltage;
  const double m_trans_coef;
  irs::spi_t* mp_spi;
  irs::gpio_pin_t* mp_cs_pin;
  irs_u8 mp_spi_buf[write_buf_size];
  status_t m_status;
  counter_t m_after_pause;
  double m_after_pause_ms;
  const double m_dt;
  const counter_t m_step_interval;
  irs::timer_t m_timer;
  irs_u16 m_dac_code;
  double m_dac_voltage;
  double m_target_voltage;
  double m_speed;
  size_t m_current_point;
  size_t m_num_of_points;
  bool m_need_set_voltage;
  bool m_show;
  spline_t m_spline;
};

class adc_t
{
public:
  adc_t(
    irs::adc_request_t* ap_raw_adc,
    irs_u8 a_default_gain,
    irs_u8 a_default_channel,
    irs_u8 a_default_mode,
    irs_u8 a_default_filter);
  ~adc_t() {};
  inline irs_status_t status() { return m_return_status; }
  void set_gain(irs_u8 a_gain);
  void set_channel(irs_u8 a_channel);
  void set_mode(irs_u8 a_mode);
  void set_filter(irs_u8 a_filter);
  void set_additional_gain(adc_value_t a_gain);
  void set_ref(adc_value_t a_ref);
  inline irs_u8 gain() { return m_current_gain; }
  inline irs_u8 channel() { return m_current_channel; }
  inline irs_u8 mode() { return m_current_mode; }
  inline irs_u8 filter() { return m_current_filter; }
  inline adc_value_t additional_gain() { return m_additional_gain; }
  inline adc_value_t ref() { return m_ref; }
  inline adc_value_t zero() { return m_zero; }
  inline adc_value_t voltage() { return m_voltage; }
  inline adc_value_t temperature() { return m_temperature; }
  inline adc_value_t max_value() { return m_ref
     / (m_additional_gain * static_cast<adc_value_t>(1 << m_current_gain)); }
  inline void show() { m_show = true; }
  inline void hide() { m_show = false; }
  void meas_zero();
  void meas_voltage();
  void meas_voltage_and_temperature();
  void tick();
private:
  enum status_t {
    st_set_gain,
    st_set_channel,
    st_set_mode,
    st_set_filter,
    st_wait_param,
    st_set_mode_zero,
    st_start_meas_zero,
    st_get_zero,
    st_wait_zero,
    st_set_mode_single,
    st_wait_set_mode_single,
    st_select_temperature_channel,
    st_select_temperature_gain,
    st_start_temperature_conversion,
    st_get_temperature_value,
    st_restore_adc_channel,
    st_restore_adc_gain,
    st_start_convertion,
    st_get_value,
    st_free
  };
  enum {
    m_temperature_channel = 1,
    m_temperature_gain = 0
  };
  irs::adc_request_t* mp_raw_adc;
  status_t m_status;
  irs_status_t m_return_status;
  bool m_need_set_gain;
  bool m_need_set_channel;
  bool m_need_set_mode;
  bool m_need_set_filter;
  bool m_need_meas_zero;
  bool m_need_meas_voltage;
  bool m_need_meas_temperature;
  irs_u8 m_current_gain;
  irs_u8 m_current_channel;
  irs_u8 m_cash_channel;
  irs_u8 m_cash_gain;
  irs_u8 m_current_mode;
  irs_u8 m_current_filter;
  int m_adc_value;
  adc_value_t m_zero;
  adc_value_t m_voltage;
  adc_value_t m_temperature;
  bool m_show;
  adc_value_t m_additional_gain;
  adc_value_t m_ref;
};

//inline irs_i32 convert_adc(irs_i32 a_in_value)
//{
//  irs_i32 adc_value = a_in_value;
//  irs_i32 adc_mid = m_adc_midscale;
//  return adc_value - adc_mid;
//}

inline hrm::adc_value_t convert_adc32(irs_i32 a_in_value, irs_u8 a_adc_gain,
  adc_value_t a_additional_gain, adc_value_t a_ref)
{
  adc_value_t gain
    = static_cast<adc_value_t>(1 << a_adc_gain) * a_additional_gain;
  return (static_cast<adc_value_t>(a_in_value)
    / static_cast<adc_value_t>(1 << 31)) * (a_ref / gain);
}

inline hrm::adc_value_t convert_adc24(irs_i32 a_in_value, irs_u8 a_adc_gain,
  adc_value_t a_additional_gain, adc_value_t a_ref)
{
  adc_value_t gain
    = static_cast<adc_value_t>(1 << a_adc_gain) * a_additional_gain;
  return (static_cast<adc_value_t>(a_in_value - (1 << 23))
    / static_cast<adc_value_t>(1 << 23)) * (a_ref / gain);
}

class buzzer_t
{
public:
  buzzer_t(irs::gpio_pin_t* ap_buzzer_pin);
  ~buzzer_t();
  void bzzz();
  void bzz(size_t a_bzz_cnt = 0);
  void tick();
private:
  const counter_t m_bzz_interval;
  const counter_t m_bzz_pause_interval;
  const counter_t m_bzzz_interval;
  bool m_buzzed;
  irs::timer_t m_timer;
  irs::gpio_pin_t* mp_pin;
  size_t m_bzz_cnt;
  size_t m_current_bzz;
};

struct adc_seq_point_t {
  adc_value_t value;
  adc_value_t value_sqr;
  counter_t time;
};

enum impf_type_t {
  impf_none = 0,
  impf_mp = 1,
  impf_mk = 2
};

impf_type_t convert_to_impf_type(irs_u8 a_value);
irs_u8 convert_from_impf_type(impf_type_t a_value);

enum cont_mode_t {
  cont_mode_none = 0,
  cont_mode_cnt = 1,
  cont_mode_sko = 2,
  cont_mode_inf = 3
};

cont_mode_t convert_to_cont_mode(irs_u8 a_value);
irs_u8 convert_from_cont_mode(cont_mode_t a_value);

struct adc_result_data_t {
  adc_value_t avg;
  adc_value_t sko;
  irs_i32 raw;
  double measured_frequency;
  double point_time;
  size_t current_point;
  adc_value_t unnormalized_value;
  adc_value_t unfiltered_value;
  adc_value_t alternative_avg;
  adc_value_t alternative_sko;
  bool saturated;
  irs_u16 error_cnt;
};

struct adc_param_data_t {
  irs_u8 gain;
  irs_u8 filter;
  irs_u8 channel;
  size_t cnv_cnt;
  adc_value_t additional_gain;
  adc_value_t ref;
  size_t cont_cnv_cnt;
  size_t impf_iterations_cnt;
  impf_type_t impf_type;
  cont_mode_t cont_mode;
  adc_value_t cont_sko;
  void copy(adc_param_data_t* ap_data);
};

//------------------------------------------------------------------------------

template<class in_t, class out_t, class calc_t, size_t max_size>
class static_sko_t {
  in_t mp_points[max_size];
  in_t mp_sqs[max_size];
  size_t m_size;
  size_t m_index;
  calc_t m_average;
  out_t m_average_out;
  calc_t m_average_sum;
  calc_t m_sq_sum;
  bool m_is_full;
  static_sko_t();
public:
  static_sko_t(size_t a_size);
  ~static_sko_t() {};
  void clear();
  void add(in_t a_val);
  out_t sko()const;
  out_t average()const;
  void resize(size_t a_count);
  size_t size();
  bool is_full();
};

template<class in_t, class out_t, class calc_t, size_t max_size>
static_sko_t<in_t, out_t, calc_t, max_size>::static_sko_t(size_t a_size):
  m_size(a_size),
  m_index(0),
  m_average(0),
  m_average_out(0),
  m_average_sum(0),
  m_sq_sum(0),
  m_is_full(false)
{
  fill(mp_points, mp_points + max_size, 0);
  fill(mp_sqs, mp_points + max_size, 0);
}

template<class in_t, class out_t, class calc_t, size_t max_size>
void static_sko_t<in_t, out_t, calc_t, max_size>::clear()
{
  m_size = 0;
  m_index = 0;
  m_average = 0;
  m_average_out = 0;
  m_average_sum = 0;
  m_sq_sum = 0;
  m_is_full = false;
  fill(mp_points, mp_points + max_size, 0);
  fill(mp_sqs, mp_points + max_size, 0);
}

template<class in_t, class out_t, class calc_t, size_t max_size>
void static_sko_t<in_t, out_t, calc_t, max_size>::add(in_t a_val)
{
  calc_t in = static_cast<calc_t>(a_val);
  calc_t out = static_cast<calc_t>(mp_points[m_index]);
  calc_t div = m_size;
  size_t sqs_index = m_index;
  calc_t sq_out = static_cast<calc_t>(mp_sqs[sqs_index]);
  sq_out *= sq_out;
  mp_points[m_index] = a_val;
  
  m_index++;
  
  if (m_is_full) {
    m_average_sum -= out;
    m_sq_sum -= sq_out;
  } else {
    div = m_index;
  }
  
  if (m_index >= m_size) {
    m_is_full = true;
    m_index = 0;
  }
  
  m_average_sum += in;
  m_average = m_average_sum / div;
  m_average_out = static_cast<out_t>(m_average_sum) 
    / static_cast<out_t>(div);
  
  calc_t sq = static_cast<calc_t>(in - m_average);
  mp_sqs[sqs_index] = static_cast<in_t>(sq);
  sq *= sq;
  m_sq_sum += sq;
}

template<class in_t, class out_t, class calc_t, size_t max_size>
out_t static_sko_t<in_t, out_t, calc_t, max_size>::sko() const
{
  out_t sko = 0;
  out_t div = m_index;
  if (div == 0) {
    div = 1;
  }
  if (m_is_full) {
    div = m_size;
  }
  sko = sqrt(static_cast<out_t>(m_sq_sum) / div);
  return sko;
}

template<class in_t, class out_t, class calc_t, size_t max_size>
out_t static_sko_t<in_t, out_t, calc_t, max_size>::average() const
{
  return m_average_out;
}

template<class in_t, class out_t, class calc_t, size_t max_size>
void static_sko_t<in_t, out_t, calc_t, max_size>::resize(size_t a_size)
{
  clear();
  m_size = ((a_size >= 1) ? a_size : 1);
  m_size = ((a_size <= max_size) ? a_size : max_size);
}

template<class in_t, class out_t, class calc_t, size_t max_size>
size_t static_sko_t<in_t, out_t, calc_t, max_size>::size()
{
  return m_size;
}

template<class in_t, class out_t, class calc_t, size_t max_size>
bool static_sko_t<in_t, out_t, calc_t, max_size>::is_full()
{
  return m_is_full;
}

//------------------------------------------------------------------------------

enum adc_error_type_t {
  aet_none,
  aet_verify,
  aet_invalid,
  aet_overtime
};

class ad7799_cread_t
{
public:
  ad7799_cread_t(irs::spi_t *ap_spi, irs::gpio_pin_t* ap_cs_pin,
    adc_exti_t* ap_adc_exti);
  ~ad7799_cread_t() {};
  void start_conversion();
  void restart();
  inline void stop_continious() { m_need_stop = true; }
  inline irs_status_t status() { return m_return_status; }
  inline void continious_pause() { m_need_reconfigure = true; }
  //  ������ ����������
  void get_params(adc_param_data_t* ap_param_data, bool a_actual = true);
  //  ������ �����������
//  inline adc_value_t max_value() { return m_param_data.ref
//     / (2.0 * m_param_data.additional_gain 
//     * static_cast<adc_value_t>(1 << m_param_data.gain)); }
  inline adc_value_t max_value() { return m_max_value; }  //  ��-�� ���� ������
  inline bool new_result() { return new_data(&m_new_result); }
  void result(adc_result_data_t* ap_result_data);
  //  ����� � �������
  inline void show() { m_show = true; }
  inline void hide() { m_show = false; }
  inline void test_impf() { m_test_impf = true; }
  inline void set_show_points(bool a_show) { m_show_points = a_show; }
  void show_param_data(bool a_show, adc_param_data_t* ap_param_data);
  void show_current_param_data(bool a_show);
  //  ��������� ����������
  void set_params(adc_param_data_t* ap_param_data);
  //  ������� �������������� ���
  double get_reference_frequency(bool a_actual = true);
  //  ������ �������������, Td = N / Fp + Ts, ��� N - ������ ����, cnv_cnt, Fp -
  //  ������� �������������� ���, Ts - ����� ��������� ������� ���
  double get_td(bool a_actual = true);
  bool window_is_full();
  void set_max_value(adc_value_t a_max_value);
  bool in_infinity_mode(bool a_actual = true);
  void mute();
  void tick();
private:
  enum status_t {
    st_reset_prepare,
    st_reset,
    st_reset_wait,
    st_reconfigure,
    st_spi_prepare,
    st_write_cfg_reg,
    st_cs_pause_cfg,
    st_write_mode_reg,
    st_cs_pause_mode,
    st_verify,
    st_verify_wait,
    st_verify_read_start,
    st_verify_show,
    st_start,
    st_spi_first_wait,
    st_validate_point,
    st_spi_point_processing,
    st_spi_wait_stop,
    st_spi_error_processing,
    st_cread,
    st_free
  };
  enum {
    min_cnv_cnt = 3,
    max_cnv_cnt = 1024
  };
  enum  {
    instruction_cread = 0x5C,
    instruction_stop = 0x58,
    instruction_read_mode = 0x48,
    instruction_read_cfg = 0x50,
    instruction_write_mode = 0x08,
    instruction_write_cfg = 0x10,
    instruction_read_status = 0x40,
    instruction_reset = 0xFF
  };
  enum {
    start_buf_size = 1,
    stop_buf_size = 1,
    read_buf_size = 3,
    spi_buf_size = 4,
    read_mode_size = 3,
    read_cfg_size = 3,
    write_mode_size = 3,
    write_cfg_size = 3,
    read_status_size = 1,//2
    write_reset_size = 4
  };
  enum {
    gain_mask = 0x07,
    filter_mask = 0x0F,
    ub_mask = 0x10,
    channel_mask = 0x07,
    buf_bit_offset = 4
  };
  struct impf_test_data_t {
    adc_value_t max;
    size_t max_index;
    adc_value_t min;
    size_t min_index;
    adc_value_t P;
    adc_value_t N;
    adc_value_t Dtotal;
    adc_value_t avg;
    counter_t time;
  };
  
  irs::spi_t *mp_spi;
  irs::gpio_pin_t *mp_cs_pin;
  adc_exti_t* mp_adc_exti;
  irs::event_connect_t<ad7799_cread_t> m_int_event;
  status_t m_status;
  bool m_need_stop;
  irs_status_t m_return_status;
  size_t m_index;
  size_t m_cont_index;
  deque<adc_value_t> m_value_deque;
  irs::fast_sko_t<adc_value_t, adc_value_t> m_fast_sko;
  irs::fast_sko_t<adc_value_t, adc_value_t> m_impf_sko;
  //static_sko_t<irs_i32, adc_value_t, irs_i64, max_cnv_cnt> m_static_sko;
  irs_u8 mp_spi_buf[spi_buf_size];
  counter_t m_pause_interval;
  counter_t m_cs_interval;
  counter_t m_reset_interval;
  irs::timer_t m_timer;
  irs::timer_t m_overtime_timer;
  bool m_show;
  bool m_test_impf;
  bool m_need_prefilling;
  counter_t m_time_counter_prev;
  counter_t m_time_counter;
  counter_t m_point_time;
  bool m_need_reconfigure;
  bool m_need_error_processing;
  bool m_need_repeat_conversion;
  bool m_new_result;
  bool m_valid_frequency;
  bool m_show_points;
  bool m_adc_filter_ready;
  adc_result_data_t m_result_data;
  adc_param_data_t m_param_data;
  adc_param_data_t m_user_param_data;
  irs::filt_imp_noise_t m_imp_filt;
  adc_value_t m_max_value;
  irs_u16 m_error_cnt;
  adc_error_type_t m_error_type;
  irs_i32 m_adc_raw_value;
  bool m_muted;
  void event();
  inline adc_value_t convert_value(irs_i32 a_in_value)
  {
    adc_value_t gain
      = static_cast<adc_value_t>(1 << m_param_data.gain) 
      * m_param_data.additional_gain;
    return (static_cast<adc_value_t>(a_in_value - (1 << 23))
      / static_cast<adc_value_t>(1 << 23)) * (m_param_data.ref / gain);
  }
  inline adc_value_t normalize_value(adc_value_t a_in_value)
  {
//    adc_value_t gain
//      = static_cast<adc_value_t>(1 << m_param_data.gain) 
//      * m_param_data.additional_gain;
//    adc_value_t half_scale = pow(2.0, 23);
//    return ((a_in_value - half_scale)*m_param_data.ref) / (half_scale * gain);
    return ((a_in_value - 8388608.0)/2048000.0);
  }
  inline adc_value_t unnormalize_value(adc_value_t a_in_value)
  {
    adc_value_t gain
      = static_cast<adc_value_t>(1 << m_param_data.gain) 
      * m_param_data.additional_gain;
    adc_value_t half_scale = pow(2.0, 23);
    return half_scale + (half_scale * a_in_value * gain) / m_param_data.ref;
  }
  adc_value_t calc_impf(deque<adc_value_t>* ap_value_deque, 
    impf_test_data_t* ap_test_data = 0);
  void show_impf_test_data(
    deque<adc_value_t>* ap_value_deque, impf_test_data_t* ap_test_data, 
    size_t a_iteration_number, adc_value_t a_impf_result);
  bool new_data(bool* ap_new_data);
  //  ����� � �������
  void show_start_message(bool a_show, adc_param_data_t* ap_param_data);
  bool valid_verify_message(bool a_show, irs_u8 a_status_reg, 
    irs_u8 a_channel);
  void show_point_symbol(bool a_show);
  void show_points(bool a_show, adc_value_t a_value);
  //  �������
  irs_i32 reinterpret_adc_raw_value(irs_u8* ap_spi_buf);
  double calc_frequency(counter_t a_prev_cnt, counter_t a_cnt, 
    bool a_valid_time);
  counter_t get_settle_time(irs_u8 a_filter);
  bool valid_value(irs_i32 a_value);
};

class ad4630_t
{
public:
  ad4630_t(irs::spi_t *ap_spi, 
    irs::gpio_pin_t* ap_rst_pin,
    irs::gpio_pin_t* ap_cs_pin,
    irs::gpio_pin_t* ap_busy_pin,
    irs::gpio_pin_t* ap_pwr_pin,
    irs::gpio_pin_t* ap_point_control_pin,
    pulse_gen_t* ap_pulse_gen,
    adc_ad4630_ready_t* ap_adc_ready,
    spi_dma_reader_t* ap_spi_dma_reader);
  ~ad4630_t() {};
  void pulse_event();
  void cnv_complete_event();
  void read_complete_event();
  void start_single_conversion() { m_need_start_single = true; }
  void start_continious(irs_u32 a_num_of_points = 0);
  inline irs_u32 current_point() { return m_current_point; }
  inline void stop_continious() { m_need_stop = true; }
  inline irs_status_t status() { return m_return_status; }
  inline bool ready() { return m_return_status == irs_st_ready; }
  bool new_data();
  double voltage1();
  double voltage2();
  double voltage_ef1();
  double voltage_ef2();
  double set_ef_smooth(double a_ef_smooth);
  void ef_preset();
  irs_u8 set_avg(irs_u8 a_avg);
  irs_u16 set_t(irs_u16 a_t);
  irs_u32 correct_n_adc(irs_u32 a_n_adc);
  void set_cnv_freq(double a_cnv_freq);
  void set_continious_freq(double a_continious_freq);
  irs_u32 error_cnt() { return m_error_cnt; }
  inline double vref() { return m_vref; }
  void tick();
private:
  enum {
    spi_write_buf_size = 16,
    spi_read_buf_size = 16,
    read_reg_size = 3,
    write_cfg_mode_size = 1,
    read_cfg_mode_size = 3,
    write_reg_size = 3,
    interleaved_read_size = 6,
    interleaved_avg_read_size = 8,
    max_avg = 16,
    min_t = 10,
    max_t = 0xFFFF,
    min_n_adc = 2,
    max_n_adc = 1000000
  };
  enum {
    sync_bit1_pos = 0,
    or_bit1_pos = 2,
    sync_bit2_pos = 1,
    or_bit2_pos = 3,
  };
  enum {
    command_configuration_mode = 0xA0,
    command_read = 0x80,
    command_write = 0x00,
    command_exit_cfg_mode = 0x01,
    command_interleaved_lane_mode = 0xC0,
    command_output_data_mode_avg = 0x03,
    command_avg_sync = 0x80
  };
  enum {
    address_spi_revision_register = 0x0B,
    address_spi_exit_cfg_mode_register = 0x14,
    address_modes_register = 0x20,
    address_avg_mode_register = 0x15
  };
  enum status_t {
    st_reset_prepare,
    st_reset,
    st_check_adc_cfg_mode_enter,
    st_check_adc_cfg_mode,
    st_check_adc_cfg_mode_wait,
    st_check_adc_read_enter,
    st_check_adc_read,
    st_write_modes_register_enter,
    st_write_modes_register,
    st_exit_cfg_mode_enter,
    st_exit_cfg_mode,
    st_write_avg_register_enter,
    st_write_avg_register,
    st_verify_avg_enter,
    st_verify_avg,
    st_meas_prepare,
    st_meas_cnv_set,
    st_meas_cnv_clear,
    st_meas_busy_check,
    st_meas_read_prepare,
    st_point_processing,
    st_free
  };
  irs::spi_t *mp_spi;
  irs::gpio_pin_t *mp_rst_pin;
  irs::gpio_pin_t *mp_cs_pin;
  irs::gpio_pin_t *mp_busy_pin;
  irs::gpio_pin_t *mp_pwr_pin;
  irs::gpio_pin_t *mp_point_control_pin;
  pulse_gen_t* mp_pulse_gen;
  adc_ad4630_ready_t* mp_adc_ready;
  spi_dma_reader_t* mp_spi_dma_reader;
  irs::event_connect_t<ad4630_t> m_pulse_event;
  irs::event_connect_t<ad4630_t> m_cnv_complete_event;
  irs::event_connect_t<ad4630_t> m_read_complete_event;
  irs_u8 mp_spi_write_buf[spi_write_buf_size];
  irs_u8 mp_spi_read_buf[spi_read_buf_size];
  status_t m_status;
  counter_t m_reset_interval;
  counter_t m_cs_wait_interval;
  counter_t m_overtime_interval;
  irs::timer_t m_timer;
  bool m_need_start_single;
  bool m_need_start_continious;
  bool m_need_stop;
  bool m_new_data;
  bool m_need_reconfigure;
  irs_i32 m_raw_data1;
  irs_i32 m_raw_data2;
  const double m_vref;
  const double m_trans_k;
  double m_voltage1;
  double m_voltage2;
  irs_status_t m_return_status;
  irs_u8 m_avg;
  irs_u8 m_avg_buffer;
  irs_u16 m_t;
  double m_voltage_ef1;
  double m_voltage_ef2;
  double m_ef_smooth;
  irs_u32 m_error_cnt;
  irs_u32 m_current_point;
  irs_u32 m_num_of_points;
  bool m_need_ef_preset;
};

class termostat_t
{
public:
  explicit termostat_t(irs::gpio_pin_t* ap_off_pin);
  ~termostat_t() {};
  void set_off(bool a_off);
  bool is_off();
  irs_status_t status();
  void set_after_pause(counter_t a_after_pause);
  void tick();
private:
  irs::gpio_pin_t* mp_off_pin;
  bool m_is_off;
  irs::timer_t m_timer;
  irs_status_t m_status;
};

class show_network_params_t
{
public:
  show_network_params_t(network_config_t* ap_config);
};

class adc_conditioner_t
{
public:
  explicit adc_conditioner_t(irs::conn_data_t<th_value_t>* ap_out_data,
    th_value_t a_sub, th_value_t a_div, th_value_t a_vref, th_value_t a_tau);
  ~adc_conditioner_t() {};
  th_value_t convert(irs_u16 a_value);
  th_value_t convert(th_value_t a_value);
private:
  irs::conn_data_t<th_value_t>* mp_out_data;
  th_value_t m_sub;
  th_value_t m_div;
  irs::fade_data_t m_fade_data;
  bool m_need_fade_preset;
  bool m_need_conditioning;
  const th_value_t m_fade_tau;
  const th_value_t m_vref;
};

class device_condition_controller_t
{
public:
  explicit device_condition_controller_t(
    irs::gpio_pin_t* ap_fan_ac_on,
    irs::gpio_pin_t* ap_fan_dc_ls,
    irs::gpio_pin_t* ap_fan_dc_hs,
    irs::gpio_pin_t* ap_fan_dc_sen,
    irs::conn_data_t<th_value_t>* ap_th_dac_data,
    irs::conn_data_t<th_value_t>* ap_th_box_ldo_data,
    irs::conn_data_t<th_value_t>* ap_th_box_adc_data,
    irs::conn_data_t<th_value_t>* ap_th_mcu_data,
    irs::conn_data_t<th_value_t>* ap_th_ext_1_data,
    irs::conn_data_t<th_value_t>* ap_th_ext_2_data,
    irs::conn_data_t<th_value_t>* ap_volt_box_neg_data,
    irs::conn_data_t<th_value_t>* ap_volt_box_pos_data,
    irs::conn_data_t<irs_u8>* ap_fan_mode_data,
    irs::conn_data_t<irs_u8>* ap_fan_mode_ee_data,
    irs::conn_data_t<irs_u8>* ap_fan_status_data,
    irs::conn_data_t<irs_u8>* ap_fan_ac_speed_data,
    irs::conn_data_t<irs_u8>* ap_fan_ac_speed_ee_data,
    irs::conn_data_t<irs_u8>* ap_fan_dc_speed_data,
    irs::conn_data_t<irs_u8>* ap_fan_dc_speed_ee_data,
    irs::conn_data_t<float>* ap_fan_dc_speed_sence_data);
  ~device_condition_controller_t() {};
  void tick();
  inline void set_idle(bool a_idle) { m_idle = a_idle; m_need_changes = true; }
  inline void show(bool a_show) { m_show = a_show; }
private:
  enum {
    adc1_address = IRS_ADC1_BASE,
    adc2_address = IRS_ADC3_BASE,
    th_dac_ch = irs::arm::st_adc_t::ADC12_PA4_CH4,
    th_box_ldo_ch = irs::arm::st_adc_t::ADC123_PA3_CH3,
    th_box_adc_ch = irs::arm::st_adc_t::ADC3_PF9_CH7,
    volt_box_neg_ch = irs::arm::st_adc_t::ADC123_PC0_CH10,
    volt_box_pos_ch = irs::arm::st_adc_t::ADC3_PF10_CH8,
    th_mcu_ch = 0,//irs::arm::st_adc_t::ADC1_TEMPERATURE,
    th_ext_1_ch = irs::arm::st_adc_t::ADC123_PA0_CH0,
    th_ext_2_ch = irs::arm::st_adc_t::ADC12_PA6_CH6,
    adc1_mask = th_dac_ch | th_box_ldo_ch | th_mcu_ch | th_ext_1_ch|th_ext_2_ch,
    adc2_mask = th_box_adc_ch | volt_box_pos_ch,
    m_fan_ac_max_speed = 1,
    m_fan_dc_max_speed = 2
  };
  const th_value_t m_fade_tau;
  const th_value_t m_vref;
  irs::gpio_pin_t* mp_fan_ac_on;
  irs::gpio_pin_t* mp_fan_dc_ls;
  irs::gpio_pin_t* mp_fan_dc_hs;
  irs::gpio_pin_t* mp_fan_dc_sen;
  irs::conn_data_t<irs_u8>* mp_fan_mode_data;
  irs::conn_data_t<irs_u8>* mp_fan_mode_ee_data;
  irs::conn_data_t<irs_u8>* mp_fan_status_data;
  irs::conn_data_t<irs_u8>* mp_fan_ac_speed_data;
  irs::conn_data_t<irs_u8>* mp_fan_ac_speed_ee_data;
  irs::conn_data_t<irs_u8>* mp_fan_dc_speed_data;
  irs::conn_data_t<irs_u8>* mp_fan_dc_speed_ee_data;
  irs::conn_data_t<float>* mp_fan_dc_speed_sence_data;
  irs_u8 m_th_dac_channel_number;
  irs_u8 m_th_box_ldo_channel_number;
  irs_u8 m_th_box_adc_channel_number;
  irs_u8 m_volt_box_neg_channel_number;
  irs_u8 m_volt_box_pos_channel_number;
  irs_u8 m_th_mcu_channel_number;
  irs_u8 m_th_ext_1_channel_number;
  irs_u8 m_th_ext_2_channel_number;
  irs::arm::st_adc_t m_adc1;
  irs::arm::st_adc_t m_adc2;
  irs::loop_timer_t m_polling_timer;
  adc_conditioner_t m_th_dac_conditioner;
  adc_conditioner_t m_th_box_ldo_conditioner;
  adc_conditioner_t m_th_box_adc_conditioner;
  adc_conditioner_t m_volt_box_neg_conditioner;
  adc_conditioner_t m_volt_box_pos_conditioner;
  adc_conditioner_t m_th_mcu_conditioner;
  adc_conditioner_t m_th_ext_1_conditioner;
  adc_conditioner_t m_th_ext_2_conditioner;
  fan_mode_t m_fan_mode;
  fan_status_t m_fan_status;
  irs_u8 m_fan_ac_speed;
  irs_u8 m_fan_dc_speed;
  bool m_idle;
  bool m_need_changes;
  bool m_show;
  void set_fan_speed_ac(irs_u8 a_speed);
  void set_fan_speed_dc(irs_u8 a_speed);
  fan_mode_t convert_u8_to_fan_mode(irs_u8 a_mode);
  irs_u8 convert_fan_mode_to_u8(fan_mode_t a_fan_mode);
  irs_u8 convert_fan_status_to_u8(fan_status_t a_fan_status);
};

//------------------------------------------------------------------------------

//! \brief ����� ��� �������������� ������ �� ������� �����
class operating_duty_detector_t
{
public:
  typedef std::size_t size_type;
  operating_duty_detector_t(double a_allowable_diviation,
    double a_time_interval);
  //! \brief ��������� ������� ���������� ��������
  void add_current_value(double a_value);
  //! \brief ������������� ���������� ���������� �� ������� � �������������
  //!   �������� [0, 1]
  void set_allowable_diviation(double a_diviation);
  //! \brief ������ �������
  void set_reference_value(double a_reference_value);
  void set_range_min(double a_range_min);
  //! \brief ������ ��������� ��������, � ������� �������� ������� ����������
  //!   �������� �� ������ �������� �� ���������� ����������
  void set_time_interval(double a_time_interval);
  //! \brief ����� ������� � �������� ������� ������ �� ������� �����
  void reset();
  //! \brief ���������� \c true, ���� ������� ����� ����������
  bool ready() const;
private:
  operating_duty_detector_t();
  void update_allowable_deviation();
  double m_range_allowable_diviation;
  double m_allowable_deviation;
  double m_reference_value;
  double m_range_min;
  double m_time_interval;
  irs::measure_time_t m_time;
};

//------------------------------------------------------------------------------

class temperature_sensor_t
{
public:
  virtual ~temperature_sensor_t() {}
  virtual double get_temperature() = 0;
  virtual bool temperature_is_normal() = 0;
  virtual bool overheat() = 0;
  virtual void tick() = 0;
};

class temperature_sensor_conn_data_t: public temperature_sensor_t
{
public:
  explicit temperature_sensor_conn_data_t(irs::conn_data_t<double>* ap_th_data,
    double a_min_temperature, double a_max_temperature);
  ~temperature_sensor_conn_data_t() {};
  virtual double get_temperature() { return *mp_th_data; }
  virtual bool temperature_is_normal();
  virtual bool overheat() { return (*mp_th_data > m_max_temperature); }
  virtual void tick() {};
private:
  irs::conn_data_t<double>* mp_th_data;
  const double m_min_temperature;
  const double m_max_temperature;
};

//! \brief ���������� ��������� ������� � �������������� ���-����������, �
//!   �������� ������ �� ������������.
//! \details ��������� ���������������� � �������� �����������
class peltier_t
{
public:
  enum polarity_map_t { default_polarity_map, inverse_polarity_map };
  struct parameters_t
  {
    temperature_sensor_t* temperature_sensor;
    irs::pwm_gen_t* pwm;
    gpio_channel_t pwm_channel;
    irs::gpio_pin_t* polarity_pin;
    double operating_duty_time_interval_s;
    double operating_duty_deviation;
    // �� 0 �� 1
    double pwm_max_code_float;
    polarity_map_t polarity_map;
    irs::conn_data_t<double>* temperature_setpoint;
    irs::conn_data_t<double>* temperature;
    irs::conn_data_t<double>* pid_k;
    irs::conn_data_t<double>* pid_ki;
    irs::conn_data_t<double>* pid_kd;
    irs::conn_data_t<double>* iso_k;
    irs::conn_data_t<double>* iso_t;
    irs::conn_data_t<double>* pwm_rate_slope;
    irs::conn_data_t<double>* pid_out;
    irs::conn_data_t<double>* amplitude_code_float;
    irs::bit_data_as_bool_t* enabled;
    irs::bit_data_as_bool_t* pid_reg_enabled;
    irs::bit_data_as_bool_t* polarity_pin_bit_data;
    parameters_t(
      temperature_sensor_t* ap_temperature_sensor,
      irs::pwm_gen_t* ap_pwm,
      gpio_channel_t a_pwm_channel,
      irs::gpio_pin_t* ap_polarity_pin,
      double a_operating_duty_time_interval_s,
      double a_operating_duty_deviation,
      double a_pwm_max_code_float,
      polarity_map_t a_polarity_map,
      irs::conn_data_t<double>* ap_temperature_setpoint,
      irs::conn_data_t<double>* ap_temperature,
      irs::conn_data_t<double>* ap_pid_k,
      irs::conn_data_t<double>* ap_pid_ki,
      irs::conn_data_t<double>* ap_pid_kd,
      irs::conn_data_t<double>* ap_iso_k,
      irs::conn_data_t<double>* ap_iso_t,
      irs::conn_data_t<double>* ap_pwm_rate_slope,
      irs::conn_data_t<double>* ap_pid_out,
      irs::conn_data_t<double>* ap_amplitude_code_float,
      irs::bit_data_as_bool_t* ap_enabled,
      irs::bit_data_as_bool_t* ap_pid_reg_enabled,
      irs::bit_data_as_bool_t* ap_polarity_pin_bit_data):
      temperature_sensor(ap_temperature_sensor),
      pwm(ap_pwm),
      pwm_channel(a_pwm_channel),
      polarity_pin(ap_polarity_pin),
      operating_duty_time_interval_s(a_operating_duty_time_interval_s),
      operating_duty_deviation(a_operating_duty_deviation),
      pwm_max_code_float(a_pwm_max_code_float),
      polarity_map(a_polarity_map),
      temperature_setpoint(ap_temperature_setpoint),
      temperature(ap_temperature),
      pid_k(ap_pid_k),
      pid_ki(ap_pid_ki),
      pid_kd(ap_pid_kd),
      iso_k(ap_iso_k),
      iso_t(ap_iso_t),
      pwm_rate_slope(ap_pwm_rate_slope),
      pid_out(ap_pid_out),
      amplitude_code_float(ap_amplitude_code_float),
      enabled(ap_enabled),
      pid_reg_enabled(ap_pid_reg_enabled),
      polarity_pin_bit_data(ap_polarity_pin_bit_data)
    {
    }
    parameters_t():
      temperature_sensor(NULL),
      pwm((NULL)),
      pwm_channel(PNONE),
      polarity_pin((NULL)),
      operating_duty_time_interval_s(1),
      operating_duty_deviation(0.05),
      pwm_max_code_float(0.35),
      polarity_map(default_polarity_map),
      temperature_setpoint(NULL),
      temperature(NULL),
      pid_k(NULL),
      pid_ki(NULL),
      pid_kd(NULL),
      iso_k(NULL),
      iso_t(NULL),
      pwm_rate_slope(NULL),
      pid_out(NULL),
      amplitude_code_float(NULL),
      enabled(NULL),
      pid_reg_enabled(NULL),
      polarity_pin_bit_data(NULL)
    {
    }
  };
  peltier_t(const parameters_t& a_parameters);
  double get_temperature();
  void set_temperature_setpoint(double a_setpoint);
  bool ready() const;
  bool regulator_enabled() const;
  void regulator_enabled(bool a_enabled);
  void pwm_regulator();
  void tick();
private:
  enum polarity_t {
    #if (CLB_HARDWARE_REV >= CLB_HW_REV_2)
    polarity_cool_def = 1, //!< \brief �������� ���� ��� ����������
    polarity_heat_def = 0  //!< \brief �������� ���� ��� ����������
    #elif (CLB_HARDWARE_REV == CLB_HW_REV_1)
    polarity_cool_def = 0, //!< \brief �������� ���� ��� ����������
    polarity_heat_def = 1  //!< \brief �������� ���� ��� ����������
    #endif // (CLB_HARDWARE_REV == CLB_HW_REV_1)
  };
  void sync_remote_data();
  //! \brief ���������� \� true, ���� ���������� �� ����������
  bool is_same_polarity(double a_duty);
  void set_duty_soft(double a_duty);
  void set_duty_hard(double a_duty);
  void set_polarity(polarity_t a_polarity);
  polarity_t get_bit_data_polarity() const;
  parameters_t m_parameters;
  temperature_sensor_t* mp_temp_sensor;
  irs::arm::st_pwm_gen_t* mp_pwm;
  irs::gpio_pin_t* mp_polarity_pin;
  typedef irs::arm::st_pwm_gen_t pwm_type;

  bool m_enabled;
  polarity_t m_polarity;
  double m_duty_setpoint;
  double m_duty_actual;
  double m_duty_max;
  double m_setpoint;
  bool m_pid_reg_enabled;
  const double m_reg_pid_interval;
  irs::loop_timer_t m_reg_pid_timer;
  irs::pid_data_t m_reg_pid_data;
  double m_pid_reg_k_factor;
  irs::isodr_data_t m_iso_data;
  irs::rate_limiter_t<double> m_rate_data;
  hrm::operating_duty_detector_t m_operating_duty_detector;
  bool m_ready;
  irs::loop_timer_t m_sync_data_timer;
  irs::timer_t m_change_polarity_timer;
  bool m_polarity_change;
  polarity_t polarity_cool;
  polarity_t polarity_heat;
};

class sync_treg_parameters_t
{
public:
  explicit sync_treg_parameters_t(
    irs::conn_data_t<th_value_t>* ap_eth_treg_ref,
    irs::conn_data_t<th_value_t>* ap_eth_treg_k,
    irs::conn_data_t<th_value_t>* ap_eth_treg_ki,
    irs::conn_data_t<th_value_t>* ap_eth_treg_kd,
    irs::conn_data_t<th_value_t>* ap_eth_treg_iso_k,
    irs::conn_data_t<th_value_t>* ap_eth_treg_iso_t,
    irs::conn_data_t<th_value_t>* ap_eth_treg_pwm_rate_slope,
    irs::bit_data_as_bool_t* ap_eth_treg_enabled,
    irs::bit_data_as_bool_t* ap_eth_treg_pid_reg_enabled,
    irs::bit_data_as_bool_t* ap_eth_treg_polarity_pin_bit_data,
    irs::conn_data_t<th_value_t>* ap_ee_treg_ref,
    irs::conn_data_t<th_value_t>* ap_ee_treg_k,
    irs::conn_data_t<th_value_t>* ap_ee_treg_ki,
    irs::conn_data_t<th_value_t>* ap_ee_treg_kd,
    irs::conn_data_t<th_value_t>* ap_ee_treg_iso_k,
    irs::conn_data_t<th_value_t>* ap_ee_treg_iso_t,
    irs::conn_data_t<th_value_t>* ap_ee_treg_pwm_rate_slope,
    irs::bit_data_as_bool_t* ap_ee_treg_enabled,
    irs::bit_data_as_bool_t* ap_ee_treg_pid_reg_enabled,
    irs::bit_data_as_bool_t* ap_ee_treg_polarity_pin_bit_data);
  ~sync_treg_parameters_t() {};
  void sync();
private:
  irs::conn_data_t<th_value_t>* mp_eth_treg_ref;
  irs::conn_data_t<th_value_t>* mp_eth_treg_k;
  irs::conn_data_t<th_value_t>* mp_eth_treg_ki;
  irs::conn_data_t<th_value_t>* mp_eth_treg_kd;
  irs::conn_data_t<th_value_t>* mp_eth_treg_iso_k;
  irs::conn_data_t<th_value_t>* mp_eth_treg_iso_t;
  irs::conn_data_t<th_value_t>* mp_eth_treg_pwm_rate_slope;
  irs::bit_data_as_bool_t* mp_eth_treg_enabled;
  irs::bit_data_as_bool_t* mp_eth_treg_pid_reg_enabled;
  irs::bit_data_as_bool_t* mp_eth_treg_polarity_pin_bit_data;
  //
  irs::conn_data_t<th_value_t>* mp_ee_treg_ref;
  irs::conn_data_t<th_value_t>* mp_ee_treg_k;
  irs::conn_data_t<th_value_t>* mp_ee_treg_ki;
  irs::conn_data_t<th_value_t>* mp_ee_treg_kd;
  irs::conn_data_t<th_value_t>* mp_ee_treg_iso_k;
  irs::conn_data_t<th_value_t>* mp_ee_treg_iso_t;
  irs::conn_data_t<th_value_t>* mp_ee_treg_pwm_rate_slope;
  irs::bit_data_as_bool_t* mp_ee_treg_enabled;
  irs::bit_data_as_bool_t* mp_ee_treg_pid_reg_enabled;
  irs::bit_data_as_bool_t* mp_ee_treg_polarity_pin_bit_data;
};

struct show_dac_adc_data_t {
  irs_u32 iteration;
  irs_i32 dac_code;
  irs_u32 dac_step;
  double adc;
  double sko;
  irs_u32 cnt;
};

void show_dac_adc(show_dac_adc_data_t& a_data);

struct show_pid_data_t {
  irs_u32 iteration;
  irs_i32 dac_code;
  double dac;
  double dac_avg;
  double adc;
  double adc_sko;
  irs_u32 adc_cnt;
  double dac_sko;
  bool dac_sko_ready;
};

struct eth_pid_data_t {
  bool new_data;
  double fade_out;
  double int_value;
  double dac_avg;
  double dac_sko;
  double iso_out;
  double pid_time;
  eth_pid_data_t()
  {
    new_data = false;
    fade_out = 0.0;
    int_value = 0.0;
    dac_avg = 0.0;
    dac_sko = 0.0;
    iso_out = 0.0;
    pid_time = 0.0;
  };
};

void show_pid(show_pid_data_t& a_data);

dac_value_t calc_initial_dac_code(dac_value_t a_etalon, dac_value_t a_checked);

enum pid_ready_condition_t {
  prc_dac_sko = 1,
  prc_adc_sko = 2,
  prc_adc_value = 4,
  prc_dac_adc_sko = 5,
  prc_dac_adc_value = 8,
  prc_dac_adc_sko_value = 9
};

irs_u8 pid_ready_condition_to_u8(pid_ready_condition_t a_condition);
irs_u32 pid_ready_condition_to_u32(pid_ready_condition_t a_condition);
pid_ready_condition_t u8_to_pid_ready_condition(irs_u8 a_condition);
pid_ready_condition_t apply_pid_ready_condition(irs_u8 a_condition);

struct pid_ready_data_t {
  pid_ready_condition_t pid_ready_condition;
  double target_accuracy;
  double dac_sko;
  double dac_target_sko;
  double adc_value;
  double adc_target_value_accuracy;
  double adc_sko;
  double adc_target_sko;
  bool is_overtime;
  bool dac_sko_ready;
  bool prepare_pause_completed;
};

bool pid_ready(pid_ready_data_t* ap_pid_ready_data);

}

#endif  //  utilsH
