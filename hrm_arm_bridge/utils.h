#ifndef utilsH
#define utilsH

#include <irsdefs.h>

#include <irsspi.h>
#include <irsgpio.h>
#include <timer.h>
#include <irserror.h>
#include <mxdata.h>
#include <irsadc.h>
#include <irsdsp.h>
#include <irsalg.h>
#include <hrm_bridge_data.h>

#include "cfg.h"

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
  void bzz();
  void tick();
private:
  const counter_t m_bzz_interval;
  const counter_t m_bzzz_interval;
  bool m_buzzed;
  irs::timer_t m_timer;
  irs::gpio_pin_t* mp_pin;
};

struct adc_seq_point_t {
  irs_i32 value;
  counter_t time;
};

class ad7799_cread_t
{
public:
  ad7799_cread_t(irs::spi_t *ap_spi, irs::gpio_pin_t* ap_cs_pin,
    adc_exti_t* ap_adc_exti);
  ~ad7799_cread_t() {};
  void start_conversion();
  inline irs_status_t status() { return m_return_status; }
  inline irs_u8 gain() { return m_gain; }
  inline irs_u8 filter() { return m_filter; }
  inline irs_u8 channel() { return m_channel; }
  inline size_t cnv_cnt() { return m_cnv_cnt; }
  inline size_t skip_cnt() { return m_skip_cnt; }
  inline adc_value_t additional_gain() { return m_additional_gain; }
  inline adc_value_t ref() { return m_ref; }
  inline adc_value_t avg() { return m_avg; }
  inline adc_value_t sko() { return m_sko; }
  inline adc_value_t max_value() { return m_ref
     / (2.0 * m_additional_gain * static_cast<adc_value_t>(1 << m_gain)); }
  inline void show() { m_show = true; }
  inline void show_simply() { m_show_simply = true; }
  inline void hide() { m_show = false; m_show_simply = false; }
  inline void set_gain(irs_u8 a_gain) { m_gain = a_gain & gain_mask; }
  inline void set_channel(irs_u8 a_channel)
    { m_channel = a_channel & channel_mask; }
  inline void set_filter(irs_u8 a_filter)
    { m_filter = a_filter & filter_mask; }
  inline void set_additional_gain(adc_value_t a_gain)
    { m_additional_gain = a_gain; }
  inline void set_ref(adc_value_t a_ref) { m_ref = a_ref; }
  inline void set_pause(counter_t a_pause_interval)
    { m_pause_interval = a_pause_interval; }
  inline void set_cnv_cnt(size_t a_cnv_cnt) { m_cnv_cnt = a_cnv_cnt; }
  inline void set_skip_cnt(size_t a_skip_cnt) { m_skip_cnt = a_skip_cnt; }
  double get_adc_frequency();
  void tick();
private:
  enum status_t {
    st_spi_prepare,
    st_write_cfg_reg,
    st_cs_pause_cfg,
    st_write_mode_reg,
    st_cs_pause_mode,
    st_start,
    st_spi_first_wait,
    st_spi_wait,
    st_spi_wait_stop,
    st_cread,
    st_free
  };
  enum  {
    instruction_cread = 0x5C,
    instruction_stop = 0x58,
    instruction_read_mode = 0x48,
    instruction_read_cfg = 0x50,
    instruction_write_mode = 0x08,
    instruction_write_cfg = 0x10
  };
  enum {
    start_buf_size = 1,
    stop_buf_size = 1,
    read_buf_size = 3,
    spi_buf_size = 4,
    read_mode_size = 3,
    read_cfg_size = 3,
    write_mode_size = 3,
    write_cfg_size = 3
  };
  enum {
    gain_mask = 0x07,
    filter_mask = 0x0F,
    ub_mask = 0x10,
    channel_mask = 0x07,
    buf_bit_offset = 4
  };
  irs::spi_t *mp_spi;
  irs::gpio_pin_t *mp_cs_pin;
  adc_exti_t* mp_adc_exti;
  irs::event_connect_t<ad7799_cread_t> m_int_event;
  status_t m_status;
  irs_status_t m_return_status;
  size_t m_cnv_cnt;
  size_t m_skip_cnt;
  size_t m_index;
  vector<adc_seq_point_t> m_value_vector;
  irs::fast_sko_t<adc_value_t, adc_value_t> m_value_sko;
  irs::fast_sko_t<double, double> m_time_sko;
  irs_u8 mp_spi_buf[spi_buf_size];
  irs_u8 m_gain;
  irs_u8 m_channel;
  irs_u8 m_filter;
  counter_t m_pause_interval;
  counter_t m_cs_interval;
  irs::timer_t m_timer;
  adc_seq_point_t m_cur_point;
  bool m_show;
  bool m_show_simply;
  adc_value_t m_additional_gain;
  adc_value_t m_ref;
  adc_value_t m_avg;
  adc_value_t m_sko;
  counter_t m_time_counter_prev;
  counter_t m_time_counter;
  void event();
  inline adc_value_t convert_value(irs_i32 a_in_value)
  {
    adc_value_t gain
      = static_cast<adc_value_t>(1 << m_gain) * m_additional_gain;
    return -(static_cast<adc_value_t>(a_in_value - (1 << 23))
      / static_cast<adc_value_t>(1 << 23)) * (m_ref / gain);
  }
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

}

#endif  //  utilsH
