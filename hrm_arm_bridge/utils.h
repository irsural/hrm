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
#include <hrm_bridge_data.h>

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
  dac_t(irs::mxdata_t* ap_raw_dac, irs::dac_1220_t* ap_ti_dac);
  ~dac_t() {};
  void on();
  void off();
  irs_bool is_on();
  void set_code(dac_value_t a_code);
  void set_int_code(irs_i32 a_int_code);
  void set_lin(irs_u8 a_lin);
  dac_value_t get_code();
  irs_i32 get_int_code();
  irs_u8 get_lin();
  void set_after_pause(counter_t a_after_pause);
  irs_status_t ready();
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
  irs::dac_1220_t* mp_ti_dac;
  status_t m_status;
  counter_t m_after_pause;
  irs::timer_t m_timer;
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
  inline irs_u8 gain() { return m_current_gain; }
  inline irs_u8 channel() { return m_current_channel; }
  inline irs_u8 mode() { return m_current_mode; }
  inline irs_u8 filter() { return m_current_filter; }
  inline adc_value_t zero() { return m_zero; }
  inline adc_value_t voltage() { return m_voltage; }
  inline adc_value_t temperature() { return m_temperature; }
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
  const adc_value_t m_additional_gain;
  const adc_value_t m_ref;
};

//inline irs_i32 convert_adc(irs_i32 a_in_value)
//{
//  irs_i32 adc_value = a_in_value;
//  irs_i32 adc_mid = m_adc_midscale;
//  return adc_value - adc_mid;
//}

inline hrm::adc_value_t convert_adc(irs_i32 a_in_value, irs_u8 a_adc_gain,
  adc_value_t a_additional_gain, adc_value_t a_ref)
{
  adc_value_t gain 
    = static_cast<adc_value_t>(1 << a_adc_gain) * a_additional_gain;
  return (static_cast<adc_value_t>(a_in_value) 
    / static_cast<adc_value_t>(1 << 31)) * (a_ref / gain);
}

}

#endif  //  utilsH
