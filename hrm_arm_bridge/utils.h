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
    counter_t a_energization_time = irs::make_cnt_ms(50));
  ~bi_relay_t() {};
  virtual bit_t operator=(const bit_t a_elem);
  virtual operator bit_t();
  virtual irs_status_t status();
  virtual void set_after_pause(counter_t a_after_pause);
  virtual void tick();
private:
  enum status_t {
    st_error,
    st_energization,
    st_after_pause,
    st_ready
  };
  irs::gpio_pin_t* mp_pin_0;
  irs::gpio_pin_t* mp_pin_1;
  counter_t m_after_pause;
  counter_t m_energization_time;
  status_t m_status;
  irs::timer_t m_timer;
  bit_t m_current_value;
  const irs::string_t m_caption;
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
  void set_code(irs_i32 a_code);
  void set_after_pause(counter_t a_after_pause);
  irs_status_t ready();
  void tick();
private:
  enum status_t {
    st_wait,
    st_pause,
    st_ready
  };
  irs::dac_ad5791_data_t m_dac_data;
  status_t m_status;
  counter_t m_after_pause;
  irs::timer_t m_timer;
};

inline irs_i32 convert_adc(irs_u32 a_in_value)
{
  irs_i32 adc_value = a_in_value;
  irs_i32 adc_mid = m_adc_midscale;
  return adc_value - adc_mid;
}

}

#endif  //  utilsH
