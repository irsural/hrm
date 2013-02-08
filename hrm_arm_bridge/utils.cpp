#include "utils.h"

hrm::adc_dac_request_t::adc_dac_request_t(
  irs::spi_t *ap_spi,
  irs::gpio_pin_t *ap_adc_cs_pin,
  irs::gpio_pin_t *ap_dac_cs_pin,
  irs::gpio_pin_t *ap_dac_rst_pin,
  irs::gpio_pin_t *ap_dac_ldac_pin):
  m_status(st_adc_reset),
  m_target_status(st_adc_read),
  mp_spi(ap_spi),
  m_spi_transaction_size(m_adc_reset_size),
  m_timer(irs::make_cnt_ms(1)),
  mp_adc_cs_pin(ap_adc_cs_pin),
  mp_dac_cs_pin(ap_dac_cs_pin),
  mp_dac_rst_pin(ap_dac_rst_pin),
  mp_dac_ldac_pin(ap_dac_ldac_pin)
{
  mp_adc_cs_pin->set();
  mp_dac_cs_pin->set();
  mp_dac_rst_pin->set();
  mp_dac_ldac_pin->set();
  irs::mlog() << irsm("ADC DAC request") << endl;
}

hrm::adc_dac_request_t::~adc_dac_request_t()
{
  mp_adc_cs_pin->set();
  mp_dac_cs_pin->set();
  mp_dac_rst_pin->clear();
  mp_dac_ldac_pin->clear();
}

void hrm::adc_dac_request_t::tick()
{
  mp_spi->tick();
  switch (m_status) {
    case st_adc_reset: {
      memset(mp_spi_buf, 0xFF, m_adc_reset_size);
      m_spi_transaction_size = m_adc_reset_size;
      m_status = st_adc_spi_prepare;
      m_target_status = st_adc_read_prepare;
      break;
    }
    case st_adc_spi_prepare: {
      if ((mp_spi->get_status() == irs::spi_t::FREE) && !mp_spi->get_lock()) {
        mp_adc_cs_pin->clear();
        mp_spi->set_order(irs::spi_t::MSB);
        mp_spi->set_polarity(irs::spi_t::POSITIVE_POLARITY);
        mp_spi->set_phase(irs::spi_t::TRAIL_EDGE);
        mp_spi->lock();
        mp_spi->read_write(mp_spi_buf, mp_spi_buf, m_spi_transaction_size);
        m_status = st_adc_spi_wait;
      }
      break;
    }
    case st_adc_spi_wait: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_adc_cs_pin->set();
        mp_spi->unlock();
        m_status = m_target_status;
      }
      break;
    }
    case st_adc_read_prepare: {
      mp_spi_buf[0] = 0x60; //  read id
      m_spi_transaction_size = 2;
      m_status = st_adc_spi_prepare;
      m_target_status = st_adc_read;
      break;
    }
    case st_adc_read: {
      irs_u32 id = mp_spi_buf[1];
      irs::mlog() << irsm("AD7799 id = 0x") << hex << id << endl;
      m_status = st_dac_reset;
      break;
    }
    case st_dac_reset: {
      mp_dac_rst_pin->set();
      m_timer.start();
      m_status = st_dac_read_prepare;
      break;
    }
    case st_dac_read_prepare: {
      if (m_timer.check()) {
        mp_dac_rst_pin->clear();
        mp_spi_buf[0] = 0x20; //  w cr
        mp_spi_buf[1] = 0x00;
        mp_spi_buf[2] = 0x0A;
        m_spi_transaction_size = 3;
        m_status = st_dac_spi_prepare;
        m_target_status = st_dac_readback_prepare;
      }
      break;
    }
    case st_dac_readback_prepare: {
      mp_spi_buf[0] = 0xA0; //  w cr
      mp_spi_buf[1] = 0x00;
      mp_spi_buf[2] = 0x00;
      m_spi_transaction_size = 3;
      m_status = st_dac_spi_prepare;
      m_target_status = st_dac_readback;
      break;
    }
    case st_dac_readback: {
      irs_u32 b0 = mp_spi_buf[0];
      irs_u32 b1 = mp_spi_buf[1];
      irs_u32 b2 = mp_spi_buf[2];
      irs::mlog() << irsm("AD5791 cr = 0x") << hex << b0 
        << irsm(" 0x") << b1 << irsm(" 0x") << b2 << endl;
      m_status = st_free;
      break;
    }
    case st_dac_spi_prepare: {
      if ((mp_spi->get_status() == irs::spi_t::FREE) && !mp_spi->get_lock()) {
        mp_dac_cs_pin->clear();
        mp_spi->set_order(irs::spi_t::MSB);
        mp_spi->set_polarity(irs::spi_t::NEGATIVE_POLARITY);
        mp_spi->set_phase(irs::spi_t::TRAIL_EDGE);
        mp_spi->lock();
        mp_spi->read_write(mp_spi_buf, mp_spi_buf, m_spi_transaction_size);
        m_status = st_dac_spi_wait;
      }
      break;
    }
    case st_dac_spi_wait: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_dac_cs_pin->set();
        mp_spi->unlock();
        m_status = m_target_status;
      }
      break;
    }
    case st_free: {
      break;
    }
  }
}

//------------------------------------------------------------------------------

hrm::mono_relay_t::mono_relay_t(
  irs::gpio_pin_t* ap_pin, 
  const irs::string_t& a_caption, 
  relay_t::bit_t a_default_value):
  mp_pin(ap_pin),
  m_after_pause(0),
  m_status(irs_st_ready),
  m_timer(m_after_pause),
  m_caption(a_caption)
{
  if (!mp_pin) {
    m_status = irs_st_error;
  }
  set(a_default_value);
}

hrm::relay_t::bit_t hrm::mono_relay_t::operator=(const hrm::relay_t::bit_t 
  a_value)
{
  set(a_value);
  return a_value;
}

hrm::mono_relay_t::operator bit_t()
{
  return mp_pin->pin();
}

irs_status_t hrm::mono_relay_t::status()
{
  return m_status;
}

void hrm::mono_relay_t::set_after_pause(counter_t a_after_pause)
{
  m_after_pause = a_after_pause;
}

void hrm::mono_relay_t::set(const hrm::relay_t::bit_t a_value)
{
  if (m_status != irs_st_error) {
    if (a_value) {
      mp_pin->set();
    } else {
      mp_pin->clear();
    }
    m_status = irs_st_busy;
    m_timer.set(m_after_pause);
    m_timer.start();
  }
}

void hrm::mono_relay_t::tick()
{
  switch (m_status) {
    case irs_st_busy: {
      if (m_timer.check()) {
        irs::mlog() << irsm("Ğåëå ") << m_caption << irsm(" = ") 
          << mp_pin->pin() << endl;
        m_status = irs_st_ready;
      }
      break;
    }
    default: {
      break;
    }
  }
}

//------------------------------------------------------------------------------

hrm::bi_relay_t::bi_relay_t(
  irs::gpio_pin_t* ap_pin_0, 
  irs::gpio_pin_t* ap_pin_1, 
  const irs::string_t& a_caption, 
  relay_t::bit_t a_default_value,
  counter_t a_energization_time):
  mp_pin_0(ap_pin_0),
  mp_pin_1(ap_pin_1),
  m_after_pause(0),
  m_energization_time(a_energization_time),
  m_status(st_ready),
  m_timer(m_after_pause),
  m_current_value(a_default_value),
  m_caption(a_caption)
{
  if (!mp_pin_0 || !mp_pin_1) {
    m_status = st_error;
  }
  set(m_current_value);
}

hrm::relay_t::bit_t hrm::bi_relay_t::operator=(const hrm::relay_t::bit_t 
  a_value)
{
  set(a_value);
  return a_value;
}

hrm::bi_relay_t::operator bit_t()
{
  return m_current_value;
}

irs_status_t hrm::bi_relay_t::status()
{
  irs_status_t return_status = irs_st_busy;
  if (m_status == st_error) {
    return_status = irs_st_error;
  } else if (m_status == st_ready) {
    return_status = irs_st_ready;
  }
  return return_status;
}

void hrm::bi_relay_t::set_after_pause(counter_t a_after_pause)
{
  m_after_pause = a_after_pause;
}

void hrm::bi_relay_t::set(const hrm::relay_t::bit_t a_value)
{
  if (m_status != st_error) {
    if (a_value) {
      mp_pin_1->set();
    } else {
      mp_pin_0->set();
    }
    m_current_value = a_value;
    m_status = st_energization;
    m_timer.set(m_energization_time);
    m_timer.start();
  }
}

void hrm::bi_relay_t::tick()
{
  switch (m_status) {
    case st_energization: {
      if (m_timer.check()) {
        mp_pin_0->clear();
        mp_pin_1->clear();
        m_timer.set(m_after_pause);
        m_timer.start();
        m_status = st_after_pause;
      }
      break;
    }
    case st_after_pause: {
      if (m_timer.check()) {
        irs::mlog() << irsm("Ğåëå ") << m_caption << irsm(" = ") 
          << m_current_value << endl;
        m_status = st_ready;
      }
      break;
    }
    default: {
      break;
    }
  }
}

//------------------------------------------------------------------------------

hrm::range_t::range_t():
  m_relay_vector(),
  m_current_range(0),
  m_on(false)
{
}

size_t hrm::range_t::add_relay(relay_t* ap_relay)
{
  m_relay_vector.push_back(ap_relay);
  return m_relay_vector.size() - 1;
}

size_t hrm::range_t::bound_range(size_t a_range)
{
  size_t max = 0;
  size_t min = 0;
  if (m_relay_vector.size() > 0) {
    max = m_relay_vector.size() - 1;
  }
  return irs::bound(a_range, min, max);
}

void hrm::range_t::range_on(size_t a_range)
{
  if (m_on) {
    *m_relay_vector[m_current_range] = 0;
  }
  m_on = true;
  size_t max = 0;
  if (m_relay_vector.size() > 0) {
    max = m_relay_vector.size() - 1;
  }
  size_t min = 0;
  m_current_range = irs::bound(a_range, min, max);
  *m_relay_vector[m_current_range] = 1;
  irs::mlog() << irsm("Range ") << m_current_range << irsm(" ON") << endl;
}

void hrm::range_t::range_off()
{
  m_on = false;
  *m_relay_vector[m_current_range] = 0;
  irs::mlog() << irsm("Range ") << m_current_range << irsm(" OFF") << endl;
}
  
bool hrm::range_t::ready()
{
  bool ready = true;
  for (size_t i = 0; i < m_relay_vector.size(); i++) {
    if (m_relay_vector[m_current_range]->status() != irs_st_ready) {
      ready = false;
      break;
    }
  }
  return ready;
}

//------------------------------------------------------------------------------

hrm::dac_t::dac_t(irs::mxdata_t* ap_raw_dac):
  m_dac_data(ap_raw_dac),
  m_status(st_wait),
  m_after_pause(0),
  m_timer(m_after_pause)
{
  m_dac_data.rbuf_bit = 1;
  m_dac_data.opgnd_bit = 1;
  m_dac_data.dactri_bit = 1;
  m_dac_data.bin2sc_bit = 0;
  m_dac_data.sdodis_bit = 1;
  m_dac_data.lin_comp = 0xC;
}

void hrm::dac_t::on()
{
  m_dac_data.opgnd_bit = 0;
  m_dac_data.dactri_bit = 0;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  irs::mlog() << irsm("ÖÀÏ: âêëş÷¸í") << endl;
}

void hrm::dac_t::off()
{
  m_dac_data.opgnd_bit = 1;
  m_dac_data.dactri_bit = 1;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  irs::mlog() << irsm("ÖÀÏ: âûêëş÷åí") << endl;
}

void hrm::dac_t::set_code(irs_i32 a_code)
{
  m_dac_data.signed_voltage_code = a_code;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  irs::mlog() << irsm("ÖÀÏ: êîä = ") << a_code << endl;
}

void hrm::dac_t::set_after_pause(counter_t a_after_pause)
{
  m_after_pause = a_after_pause;
}

irs_status_t hrm::dac_t::ready()
{
  irs_status_t return_status = irs_st_busy;
  if (m_status == st_ready) {
    return_status = irs_st_ready;
  }
  return return_status;
}

void hrm::dac_t::tick()
{
  switch (m_status) {
    case st_wait: {
      if (m_dac_data.ready_bit == 1) {
        m_timer.start();
        m_status = st_pause;
      }
      break;
    }
    case st_pause: {
      if (m_timer.check()) {
        m_status = st_ready;
      }
      break;
    }
    case st_ready: {
      break;
    }
  }
}
