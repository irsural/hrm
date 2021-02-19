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
  m_pin(a_default_value),
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
  //return m_pin;
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
      m_pin = true;
    } else {
      mp_pin->clear();
      m_pin= false;
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
        irs::mlog() << irsm("Relay ") << m_caption << irsm(" = ")
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
  counter_t a_energization_time,
  bool a_wild):
  mp_pin_0(ap_pin_0),
  mp_pin_1(ap_pin_1),
  m_after_pause(0),
  m_energization_time(a_energization_time),
  m_status(st_ready),
  m_timer(m_after_pause),
  m_current_value(a_default_value),
  m_caption(a_caption),
  m_wild(a_wild),
  m_wild_current_iteration(0)
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
    m_current_value = a_value;
    if (m_current_value) {
      mp_pin_1->set();
    } else {
      mp_pin_0->set();
    }
    if (m_wild && m_current_value) {
      m_wild_current_iteration = 0;
      m_status = st_wild_energization_off;
    } else {
      m_status = st_energization;
    }
    m_timer.set(m_energization_time);
    m_timer.start();
  }
}

void hrm::bi_relay_t::tick()
{
  switch (m_status) {
    case st_wild_energization_off:
      if (m_timer.check()) {
        if (m_current_value) {
          mp_pin_1->clear();
          mp_pin_0->set();
        } else {
          mp_pin_1->set();
          mp_pin_0->clear();
        }
        if (m_wild_current_iteration < m_wild_cnt) {
          m_status = st_wild_energization_on;
        } else {
          mp_pin_0->clear();
          mp_pin_1->clear();
          m_timer.set(m_after_pause);
          m_timer.start();
          m_status = st_after_pause;
        }
        m_timer.start();
      }
      break;
    case st_wild_energization_on:
      if (m_timer.check()) {
        m_wild_current_iteration++;
        if (m_current_value) {
          mp_pin_0->clear();
          mp_pin_1->set();
        } else {
          mp_pin_1->clear();
          mp_pin_0->set();
        }
        m_timer.start();
        m_status = st_wild_energization_off;
      }
      break;
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
        irs::mlog() << irsm("Relay ") << m_caption << irsm(" = ")
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
  m_timer(m_after_pause),
  m_voltage_pos(12.288),
  m_voltage_neg(-12.288),
  m_show(false)
{
  m_dac_data.rbuf_bit = 0;//1
  m_dac_data.opgnd_bit = 0;
  m_dac_data.dactri_bit = 0;
  m_dac_data.bin2sc_bit = 0;
  m_dac_data.sdodis_bit = 1;
  m_dac_data.lin_comp = 0xC;
  m_dac_data.signed_voltage_code = 0;
  irs::mlog() << irsm("DAC: on") << endl;
  irs::mlog() << irsm("DAC: code = 0") << endl;
}

void hrm::dac_t::on()
{
  m_dac_data.opgnd_bit = 0;
  m_dac_data.dactri_bit = 0;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  if (m_show) {
    irs::mlog() << irsm("DAC: on") << endl;
  }
}

void hrm::dac_t::off()
{
  m_dac_data.opgnd_bit = 1;
  m_dac_data.dactri_bit = 1;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  if (m_show) {
    irs::mlog() << irsm("DAC: off") << endl;
  }
}

irs_bool hrm::dac_t::is_on()
{
  return (m_dac_data.opgnd_bit == 0 && m_dac_data.dactri_bit == 0);
}

void hrm::dac_t::set_normalize_code(hrm::dac_value_t a_code)
{
  dac_value_t code = irs::bound(a_code, -1., 1.);
  m_dac_data.signed_voltage_code =
    static_cast<irs_i32>(floor(code * (pow(2., 31) - 1.0) + 0.5));

  m_status = st_wait;
  m_timer.set(m_after_pause);
  if (m_show) {
    irs::mlog() << irsm("DAC: code = ") << code << irsm(" / ")
      << (m_dac_data.signed_voltage_code >> 12) << endl;
  }
}

void hrm::dac_t::set_code(dac_value_t a_code)
{
  irs_i32 code = static_cast<irs_i32>(a_code);
  m_dac_data.signed_voltage_code = code << 12;
  m_status = st_wait;
  if (m_show) {
    irs::mlog() << irsm("DAC: code = ")
      << static_cast<dac_value_t>(code) / pow(2., 19) << irsm(" / ")
      << code << endl;
  }
}

void hrm::dac_t::set_lin(irs_u8 a_lin)
{
  m_dac_data.lin_comp = a_lin;
  m_status = st_wait;
  if (m_show) {
    irs::mlog() << irsm("DAC: lin = 0x") << hex << static_cast<int>(a_lin);
    irs::mlog() << dec << endl;
  }
}

hrm::dac_value_t hrm::dac_t::get_normalize_code()
{
  return static_cast<dac_value_t>(m_dac_data.signed_voltage_code) / pow(2., 31);
}

irs_i32 hrm::dac_t::get_code()
{
  return m_dac_data.signed_voltage_code >> 12;
}

irs_u8 hrm::dac_t::get_lin()
{
  return m_dac_data.lin_comp;
}

void hrm::dac_t::set_after_pause(counter_t a_after_pause)
{
  double pause = CNT_TO_DBLTIME(a_after_pause);
  irs::mlog() << irsm("DAC: pause = ") << pause << irsm(" s.") << endl;
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

void hrm::dac_t::set_voltage_pos(dac_value_t a_voltage_pos)
{
  m_voltage_pos = a_voltage_pos;
}

void hrm::dac_t::set_voltage_neg(dac_value_t a_voltage_neg)
{
  m_voltage_neg = a_voltage_neg;
}

hrm::dac_value_t hrm::dac_t::voltage_pos()
{
  return m_voltage_pos;
}

hrm::dac_value_t hrm::dac_t::voltage_neg()
{
  return m_voltage_neg;
}

hrm::dac_value_t hrm::dac_t::output_voltage()
{
  dac_value_t code = (static_cast<dac_value_t>(m_dac_data.signed_voltage_code)
                      - 0.5) / pow(2., 31);
  return m_voltage_pos * (2. * code - 1.);
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

//------------------------------------------------------------------------------

hrm::adc_t::adc_t(
  irs::adc_request_t* ap_raw_adc,
  irs_u8 a_default_gain,
  irs_u8 a_default_channel,
  irs_u8 a_default_mode,
  irs_u8 a_default_filter
):
  mp_raw_adc(ap_raw_adc),
  m_status(st_set_gain),
  m_return_status(irs_st_busy),
  m_need_set_gain(true),
  m_need_set_channel(true),
  m_need_set_mode(true),
  m_need_set_filter(true),
  m_need_meas_zero(false),
  m_need_meas_voltage(false),
  m_need_meas_temperature(false),
  m_current_gain(a_default_gain),
  m_current_channel(a_default_channel),
  m_cash_channel(a_default_channel),
  m_cash_gain(a_default_gain),
  m_current_mode(a_default_mode),
  m_current_filter(a_default_filter),
  m_adc_value(0),
  m_zero(0.),
  m_voltage(0.),
  m_show(false),
  m_additional_gain(3.0),
  m_ref(4.096)
{
}

void hrm::adc_t::set_gain(irs_u8 a_gain)
{
  m_current_gain = a_gain;
  m_need_set_gain = true;
  m_return_status = irs_st_busy;
}

void hrm::adc_t::set_channel(irs_u8 a_channel)
{
  m_current_channel = a_channel;
  m_need_set_channel = true;
  m_return_status = irs_st_busy;
}

void hrm::adc_t::set_mode(irs_u8 a_mode)
{
  m_current_mode = a_mode;
  m_need_set_mode = true;
  m_return_status = irs_st_busy;
}

void hrm::adc_t::set_filter(irs_u8 a_filter)
{
  m_current_filter = a_filter;
  m_need_set_filter = true;
  m_return_status = irs_st_busy;
}

void hrm::adc_t::set_additional_gain(adc_value_t a_gain)
{
  m_additional_gain = a_gain;
  if (m_show) {
    irs::mlog() << irsm("ADC Gain Add = ") << m_additional_gain << endl;
  }
}

void hrm::adc_t::set_ref(adc_value_t a_ref)
{
  m_ref = a_ref;
  if (m_show) {
    irs::mlog() << irsm("ADC REF = ") << m_ref << endl;
  }
}

void hrm::adc_t::meas_zero()
{
  m_need_meas_zero = true;
  m_return_status = irs_st_busy;
}

void hrm::adc_t::meas_voltage()
{
  m_need_meas_voltage = true;
  m_return_status = irs_st_busy;
}

void hrm::adc_t::meas_voltage_and_temperature()
{
  m_need_meas_voltage = true;
  m_need_meas_temperature = true;
  m_return_status = irs_st_busy;
}

void hrm::adc_t::tick()
{
  mp_raw_adc->tick();
  switch (m_status) {
  case st_set_gain:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_set_gain = false;
      mp_raw_adc->set_param(irs::adc_gain, m_current_gain);
      m_status = st_wait_param;
      if (m_show) {
        irs::mlog() << irsm("ADC Gain = ") << (1 << m_current_gain) << endl;
      }
    }
    break;
  case st_set_channel:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_set_channel = false;
      mp_raw_adc->set_param(irs::adc_channel, m_current_channel);
      m_status = st_wait_param;
      if (m_show) {
        irs::mlog() << irsm("ADC Channel = ") << (m_current_channel + 1) << endl;
      }
    }
    break;
  case st_set_mode:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_set_mode = false;
      mp_raw_adc->set_param(irs::adc_mode, m_current_mode);
      m_status = st_wait_param;
      if (m_show) {
        irs::mlog() << irsm("ADC Mode = ")
          << static_cast<int>(m_current_mode) << endl;
      }
    }
    break;
  case st_set_filter:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_set_filter = false;
      mp_raw_adc->set_param(irs::adc_freq, m_current_filter);
      m_status = st_wait_param;
      if (m_show) {
        irs::mlog() << irsm("ADC Filter = ")
          << static_cast<int>(m_current_filter) << endl;
      }
    }
    break;
  case st_wait_param:
    if (mp_raw_adc->status() == meas_status_success) {
      m_status = st_free;
    }
    break;
  case st_set_mode_zero:
    if (mp_raw_adc->status() == meas_status_success) {
      m_current_mode = irs::adc_mode_system_zero_scale;
      mp_raw_adc->set_param(irs::adc_mode, m_current_mode);
      m_status = st_start_meas_zero;
    }
    break;
  case st_start_meas_zero:
    if (mp_raw_adc->status() == meas_status_success) {
      mp_raw_adc->start();
      m_status = st_get_zero;
    }
    break;
  case st_get_zero:
    if (mp_raw_adc->status() == meas_status_success) {
      mp_raw_adc->get_param(irs::adc_offset, &m_adc_value);
      m_status = st_wait_zero;
    }
    break;
  case st_wait_zero:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_meas_zero = false;
      m_status = st_free;
      m_zero = convert_adc32(
        m_adc_value, m_current_gain, m_additional_gain, m_ref);
      if (m_show) {
        irs::mlog() << irsm("ADC Zero = ") << (m_zero * 1.e6)
          /*<< irsm(" ˜˜˜")*/ << endl;
      }
    }
    break;
  case st_set_mode_single:
    if (mp_raw_adc->status() == meas_status_success) {
      if (m_current_mode != irs::adc_mode_single_conversion) {
        m_current_mode = irs::adc_mode_single_conversion;
        mp_raw_adc->set_param(irs::adc_mode, m_current_mode);
        m_status = st_wait_set_mode_single;
      } else {
        m_status = st_select_temperature_channel;
      }
    }
    break;
  case st_wait_set_mode_single:
    if (mp_raw_adc->status() == meas_status_success) {
      m_status = st_select_temperature_channel;
    }
    break;
  case st_select_temperature_channel:
    if (m_need_meas_temperature) {
      m_need_meas_temperature = false;
      m_cash_channel = m_current_channel;
      m_current_channel = m_temperature_channel;
      mp_raw_adc->set_param(irs::adc_channel, m_current_channel);
      m_status = st_select_temperature_gain;
    } else {
      m_status = st_start_convertion;
    }
    break;
  case st_select_temperature_gain:
    if (mp_raw_adc->status() == meas_status_success) {
      m_cash_gain = m_current_gain;
      m_current_gain = m_temperature_gain;
      mp_raw_adc->set_param(irs::adc_gain, m_current_gain);
      m_status = st_start_temperature_conversion;
    }
    break;
  case st_start_temperature_conversion:
    if (mp_raw_adc->status() == meas_status_success) {
      mp_raw_adc->start();
      m_status = st_get_temperature_value;
    }
    break;
  case st_get_temperature_value:
    if (mp_raw_adc->status() == meas_status_success) {
      adc_value_t adc_value = convert_adc32(
        mp_raw_adc->get_value(), m_current_gain, 1.0, m_ref);
      m_temperature = (-adc_value - 0.4) / 0.0195;
      if (m_show) {
        irs::mlog() << irsm("Temperature = ") << m_temperature << irsm(" °C");
        irs::mlog() << endl;
      }
      m_status = st_restore_adc_channel;
    }
    break;
  case st_restore_adc_channel:
    if (mp_raw_adc->status() == meas_status_success) {
      m_current_channel = m_cash_channel;
      mp_raw_adc->set_param(irs::adc_channel, m_current_channel);
      m_status = st_restore_adc_gain;
    }
    break;
  case st_restore_adc_gain:
    if (mp_raw_adc->status() == meas_status_success) {
      m_current_gain = m_cash_gain;
      mp_raw_adc->set_param(irs::adc_gain, m_current_gain);
      m_status = st_start_convertion;
    }
    break;
  case st_start_convertion:
    if (mp_raw_adc->status() == meas_status_success) {
      mp_raw_adc->start();
      m_status = st_get_value;
    }
    break;
  case st_get_value:
    if (mp_raw_adc->status() == meas_status_success) {
      m_adc_value = mp_raw_adc->get_value();
      m_need_meas_voltage = false;
      m_status = st_free;
      m_voltage = convert_adc32(
        m_adc_value, m_current_gain, m_additional_gain, m_ref);
      if (m_show) {
        irs::mlog() << irsm("ADC Voltage = ");
        if (abs(m_voltage) < 1.1e-3) {
          irs::mlog() << (m_voltage * 1.e6) << irsm(" uV");
        } else if (abs(m_voltage < 1.1)) {
          irs::mlog() << (m_voltage * 1.e3) << irsm(" mV");
        } else {
          irs::mlog() << m_voltage << irsm(" V");
        }
        irs::mlog() << endl;
      }
    }
    break;
  case st_free:
    if (m_return_status == irs_st_busy) {
      if (m_need_set_gain) {
        m_status = st_set_gain;
      } else if (m_need_set_channel) {
        m_status = st_set_channel;
      } else if (m_need_set_mode) {
        m_status = st_set_mode;
      } else if (m_need_set_filter) {
        m_status = st_set_filter;
      } else if (m_need_meas_zero) {
        m_status = st_set_mode_zero;
      } else if (m_need_meas_voltage) {
        m_status = st_set_mode_single;
      } else {
        m_return_status = irs_st_ready;
      }
    }
    break;
  }
}

//------------------------------------------------------------------------------

hrm::buzzer_t::buzzer_t(irs::gpio_pin_t* ap_buzzer_pin):
  m_bzz_interval(irs::make_cnt_ms(30)),
  m_bzz_pause_interval(irs::make_cnt_ms(150)),
  m_bzzz_interval(irs::make_cnt_ms(500)),
  m_buzzed(false),
  m_timer(),
  mp_pin(ap_buzzer_pin),
  m_bzz_cnt(0)
{
  mp_pin->clear();
  m_timer.stop();
}

hrm::buzzer_t::~buzzer_t()
{
  mp_pin->clear();
}

void hrm::buzzer_t::bzzz()
{
  m_timer.set(m_bzzz_interval);
  m_timer.start();
  m_buzzed = true;
  mp_pin->set();
}

void hrm::buzzer_t::bzz(size_t a_bzz_cnt)
{
  m_timer.set(m_bzz_interval);
  m_timer.start();
  m_buzzed = true;
  m_bzz_cnt = a_bzz_cnt;
  mp_pin->set();
}

void hrm::buzzer_t::tick()
{
  if (m_buzzed) {
    if (m_timer.check()) {
      if (mp_pin->pin()) {
        mp_pin->clear();
        if (m_bzz_cnt > 1) {
          m_bzz_cnt--;
          m_timer.set(m_bzz_pause_interval);
          m_timer.start();
        } else {
          m_timer.stop();
          m_buzzed = false;
        }
      } else {
        mp_pin->set();
        m_timer.set(m_bzz_interval);
        m_timer.start();
      }
    }
  }
}

//------------------------------------------------------------------------------

void hrm::adc_param_data_t::copy(adc_param_data_t* ap_data)
{
  gain = ap_data->gain;
  filter = ap_data->filter;
  channel = ap_data->channel;
  cnv_cnt = ap_data->cnv_cnt;
  additional_gain = ap_data->additional_gain;
  ref = ap_data->ref;
  cont_cnv_cnt = ap_data->cont_cnv_cnt;
  impf_iterations_cnt = ap_data->impf_iterations_cnt;
  impf_type = ap_data->impf_type;
  cont_mode = ap_data->cont_mode;
  cont_sko = ap_data->cont_sko;
}

hrm::ad7799_cread_t::ad7799_cread_t(irs::spi_t *ap_spi,
  irs::gpio_pin_t* ap_cs_pin, adc_exti_t* ap_adc_exti):
  mp_spi(ap_spi),
  mp_cs_pin(ap_cs_pin),
  mp_adc_exti(ap_adc_exti),
  m_int_event(this, &ad7799_cread_t::event),
  m_status(st_reset_prepare),
  m_need_stop(false),
  m_return_status(irs_st_ready),
  m_index(0),
  m_cont_index(0),
  m_value_deque(),
  m_fast_sko(min_cnv_cnt, min_cnv_cnt),
  m_impf_sko(min_cnv_cnt, min_cnv_cnt),
  m_pause_interval(0),
  m_cs_interval(irs::make_cnt_us(1)),
  m_reset_interval(irs::make_cnt_us(500)),
  m_timer(m_cs_interval),
  m_show(false),
  m_need_prefilling(false),
  m_time_counter_prev(0),
  m_time_counter(0),
  m_point_time(0),
  m_need_reconfigure(false),
  m_new_result(false),
  m_valid_frequency(false),
  m_show_points(false),
  m_adc_filter_ready(false),
  m_imp_filt(),
  m_max_value(1.0)
{
  mp_cs_pin->set();
  memset(mp_spi_buf, 0, spi_buf_size);
  mp_adc_exti->add_event(&m_int_event);
  mp_adc_exti->stop();
  m_result_data.avg = 0.0;
  m_result_data.sko = 0.0;
  m_result_data.saturated = false;
}

void hrm::ad7799_cread_t::start_conversion()
{
  if (m_status == st_free) {
    m_status = st_reconfigure;
    m_return_status = irs_st_busy;
  }
}

void hrm::ad7799_cread_t::event()
{
  m_time_counter_prev = m_time_counter;
  m_time_counter = counter_get();
  if (m_need_stop) {
    mp_spi_buf[0] = instruction_stop;
    mp_spi->write(mp_spi_buf, stop_buf_size);
    m_status = st_spi_wait_stop;
  } else {
    mp_spi->read(mp_spi_buf, read_buf_size);
    m_status = st_spi_point_processing;
  }
  mp_adc_exti->stop();
}

double hrm::ad7799_cread_t::calc_frequency(counter_t a_prev_cnt, 
  counter_t a_cnt, bool a_valid_time)
{
  double freq = 0.0;
  freq = CNT_TO_DBLTIME(a_cnt - a_prev_cnt);
  if (freq > 0.0 && a_valid_time) {
    freq = 1.0 / freq;
  } else {
    freq = 0.0;
  }
  return freq;
}

double hrm::ad7799_cread_t::get_reference_frequency()
{
  double freq = 0.0;
  switch (m_param_data.filter) {
    case  1: freq = 470.0; break;
    case  2: freq = 242.0; break;
    case  3: freq = 123.0; break;
    case  4: freq =  62.0; break;
    case  5: freq =  50.0; break;
    case  6: freq =  39.0; break;
    case  7: freq =  33.2; break;
    case  8: freq =  19.6; break;
    case  9: freq =  16.7; break;
    case 10: freq =  16.7; break;
    case 11: freq =  12.5; break;
    case 12: freq =  10.0; break;
    case 13: freq =  8.33; break;
    case 14: freq =  6.25; break;
    case 15: freq =  4.17; break;
    default: freq = 0.0;
  }
  return freq;
}

counter_t hrm::ad7799_cread_t::get_settle_time(irs_u8 a_filter)
{
  irs_u32 t_set = 0;
  switch (a_filter) {
    case  1: t_set =   4; break;
    case  2: t_set =   8; break;
    case  3: t_set =  16; break;
    case  4: t_set =  32; break;
    case  5: t_set =  40; break;
    case  6: t_set =  48; break;
    case  7: t_set =  60; break;
    case  8: t_set = 101; break;
    case  9: t_set = 120; break;
    case 10: t_set = 120; break;
    case 11: t_set = 160; break;
    case 12: t_set = 200; break;
    case 13: t_set = 240; break;
    case 14: t_set = 320; break;
    case 15: t_set = 480; break;
    default: t_set =   0;
  }
  return irs::make_cnt_ms(t_set);
}

void hrm::ad7799_cread_t::tick()
{
  mp_spi->tick();
  switch(m_status) {
    case st_reset_prepare: {
      if ((mp_spi->get_status() == irs::spi_t::FREE) && !mp_spi->get_lock()) {
        mp_spi->set_order(irs::spi_t::MSB);
        mp_spi->set_polarity(irs::spi_t::POSITIVE_POLARITY);
        mp_spi->set_phase(irs::spi_t::TRAIL_EDGE);
        mp_spi->lock();
        mp_spi_buf[0] = instruction_reset;
        mp_spi_buf[1] = instruction_reset;
        mp_spi_buf[2] = instruction_reset;
        mp_spi_buf[3] = instruction_reset;
        mp_cs_pin->clear();
        mp_spi->write(mp_spi_buf, write_reset_size);
        m_status = st_cs_pause_cfg;
      }
      break;
    }
    case st_reset: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        mp_spi->unlock();
        m_timer.set(m_reset_interval);
        m_timer.start();
        m_status = st_reset_wait;
      }
      break;
    }
    case st_reset_wait: {
      if (m_timer.check()) {
        m_status = st_free;
      }
      break;
    }
    case st_free: {
      break;
    }
    case st_reconfigure: {
      m_index = 0;
      m_cont_index = 0;
      m_need_stop = false;
      //
      m_param_data.gain = m_user_param_data.gain;
      m_param_data.filter = m_user_param_data.filter;
      m_param_data.channel = m_user_param_data.channel;
      m_param_data.cnv_cnt = m_user_param_data.cnv_cnt;
      m_param_data.additional_gain = m_user_param_data.additional_gain;
      m_param_data.ref = m_user_param_data.ref;
      m_param_data.cont_cnv_cnt = m_user_param_data.cont_cnv_cnt;
      m_param_data.impf_iterations_cnt = m_user_param_data.impf_iterations_cnt;
      m_param_data.impf_type = m_user_param_data.impf_type;
      m_param_data.cont_mode = m_user_param_data.cont_mode;
      m_param_data.cont_sko = m_user_param_data.cont_sko;
      show_param_data(m_show, &m_param_data);
      //
      m_result_data.saturated = false;
      //
      m_value_deque.clear();
      m_imp_filt.clear();
      m_imp_filt.max_size(m_param_data.cnv_cnt);
      m_fast_sko.clear();
      m_fast_sko.resize(m_param_data.cnv_cnt);
      m_fast_sko.resize_average(m_param_data.cnv_cnt);
      m_need_prefilling = true;
      m_need_reconfigure = false;
      m_adc_filter_ready = false;
      m_status = st_spi_prepare;
      break;
    }
    case st_spi_prepare: {
      if ((mp_spi->get_status() == irs::spi_t::FREE) && !mp_spi->get_lock()) {
        mp_spi->set_order(irs::spi_t::MSB);
        mp_spi->set_polarity(irs::spi_t::POSITIVE_POLARITY);
        mp_spi->set_phase(irs::spi_t::TRAIL_EDGE);
        mp_spi->lock();
        m_index = 0;
        m_status = st_write_cfg_reg;
      }
    } break;
    case st_write_cfg_reg: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_spi_buf[0] = instruction_write_cfg;
        mp_spi_buf[1] = m_param_data.gain;
        mp_spi_buf[2] = m_param_data.channel | (1 << buf_bit_offset);
        mp_cs_pin->clear();
        mp_spi->write(mp_spi_buf, write_cfg_size);
        m_status = st_cs_pause_cfg;
      }
    } break;
    case st_cs_pause_cfg: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        m_timer.set(m_cs_interval);
        m_timer.start();
        m_status = st_write_mode_reg;
      }
    } break;
    case st_write_mode_reg: {
      if (m_timer.check()) {
        mp_spi_buf[0] = instruction_write_mode;
        mp_spi_buf[1] = 0;
        mp_spi_buf[2] = m_param_data.filter;
        mp_cs_pin->clear();
        mp_spi->write(mp_spi_buf, write_mode_size);
        m_status = st_cs_pause_mode;
      }
    } break;
    case st_cs_pause_mode: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        //m_pause_interval = get_settle_time(m_param_data.filter);
        m_timer.set(m_cs_interval/* + m_pause_interval*/);
        m_timer.start();
        m_status = st_verify;
      }
    } break;
    case st_verify: {
      if (m_timer.check()) {
        mp_spi_buf[0] = instruction_read_status;
        mp_cs_pin->clear();
        mp_spi->write(mp_spi_buf, read_status_size);
        m_status = st_verify_wait;
      }
    } break;
    case st_verify_wait: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        m_timer.set(m_cs_interval + m_pause_interval);
        m_timer.start();
        m_status = st_verify_read_start;
      }
    } break;
    case st_verify_read_start: {
      if (m_timer.check()) {
        mp_cs_pin->clear();
        mp_spi->read(mp_spi_buf, read_status_size);
        m_status = st_verify_show;
      }
    } break;
    case st_verify_show: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        show_verify_message(m_show, mp_spi_buf[0], m_param_data.channel);
        m_timer.set(m_cs_interval + m_pause_interval);
        m_timer.start();
        m_status = st_start;
      }
    } break;
    case st_start: {
      if (m_timer.check()) {
        //show_start_message(m_show, &m_param_data);
        mp_spi_buf[0] = instruction_cread;
        mp_cs_pin->clear();
        mp_spi->write(mp_spi_buf, start_buf_size);
        m_status = st_spi_first_wait;
      }
    } break;
    case st_spi_first_wait: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        m_timer.set(2 * get_settle_time(m_param_data.filter));
        m_timer.start();
        mp_adc_exti->start();
        m_status = st_cread;
      }
    } break;
    case st_cread: {
      break;
    }
    case st_spi_point_processing: {
      if (!m_adc_filter_ready) {
        if (m_timer.check()) {
          m_adc_filter_ready = true;
        }
      }
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        m_point_time = counter_get();
        irs_i32 adc_raw_value = reinterpret_adc_raw_value(mp_spi_buf);
        adc_value_t adc_non_normalized_value = 
          static_cast<adc_value_t>(adc_raw_value);
        adc_value_t adc_value = convert_value(adc_raw_value);
        m_result_data.raw = adc_raw_value;
        
        if (m_adc_filter_ready) {
          if (m_value_deque.size() >= m_param_data.cnv_cnt) {
            m_value_deque.pop_front();
          }
          m_value_deque.push_back(adc_non_normalized_value);
          
          if (m_index >= (m_param_data.cnv_cnt - 1)) {
            m_need_prefilling = false;
          }
          
          if (m_index > 0) {
            m_valid_frequency = true;
          }
          
          
          switch (m_param_data.impf_type) {
            case impf_none: {
              m_fast_sko.add(adc_non_normalized_value);
              break;
            }
            case impf_mp: {
              if (!m_need_prefilling) {
                size_t iterations_cnt = m_param_data.impf_iterations_cnt;
                size_t max_iterations_cnt = m_param_data.cnv_cnt / 2;
                
                if (iterations_cnt == 0 || iterations_cnt > max_iterations_cnt) {
                  iterations_cnt = max_iterations_cnt;
                }
                adc_value_t impf = 0.0;
                for (size_t i = 0; i < iterations_cnt; i++) {
                  impf = calc_impf(&m_value_deque/*, &impf_test_data*/);
                  /*if (m_test_impf) {
                    show_impf_test_data(&m_value_deque, &impf_test_data, i, 
                      m_result_data.impf);
                  }*/
                }
                m_fast_sko.add(impf);
                m_test_impf = false;
              } else {
                m_fast_sko.add(adc_non_normalized_value);
              }
              break;
            }
            case impf_mk: {
              m_imp_filt.add(adc_non_normalized_value);
              if (!m_need_prefilling) {
                m_fast_sko.add(m_imp_filt.get());
              } else {
                m_fast_sko.add(adc_non_normalized_value);
              }
              break;
            }
          }
          
          m_result_data.sko = m_fast_sko / pow(2.0, 24);//max_non_normolized_value();
          m_result_data.avg = normalize_value(m_fast_sko.average());
          m_result_data.unnormalized_value = m_fast_sko.average() - pow(2.0, 23);
          m_result_data.current_point = m_cont_index + 1;
          if (abs(m_result_data.avg) > m_max_value) {
            m_result_data.saturated = true;
          }
         
          show_points(m_show_points, adc_value);
          show_point_symbol(m_show);
          
          m_index++;
          m_cont_index++;
          
          if (m_index >= m_param_data.cnv_cnt) {
            //m_index = 0;
            switch (m_param_data.cont_mode) {
              case cont_mode_none: {
                m_need_stop = true;
                if (m_show) {
                  irs::mlog() << irsm("Stop NONE cont index = ");
                  irs::mlog() << m_cont_index << endl;
                }
                break;
              }
              case cont_mode_cnt: {
                if (m_cont_index >= m_param_data.cont_cnv_cnt) {
                  if (m_show) {
                    irs::mlog() << irsm("Stop CNT cont index = ");
                    irs::mlog() << m_cont_index << endl;
                  }
                  m_need_stop = true;
                }
                break;
              }
              case cont_mode_sko: {
                if (m_result_data.sko <= m_param_data.cont_sko) {
                  if (m_show) {
                    irs::mlog() << irsm("Stop SKO cont index = ");
                    irs::mlog() << m_cont_index;
                    irs::mlog() << irsm(" sko = ") << m_result_data.sko << endl;
                  }
                  m_need_stop = true;
                }
                if (m_cont_index >= m_param_data.cont_cnv_cnt) {
                  if (m_show) {
                    irs::mlog() << irsm("Stop SKO cont index = ");
                    irs::mlog() << m_cont_index << endl;
                  }
                  m_need_stop = true;
                }
                break;
              }
            }
            if (m_need_reconfigure) {
              m_need_stop = true;
              m_valid_frequency = false;
              if (m_show) {
                irs::mlog() << endl;
              }
            }
          } else {
            if (abs(m_result_data.avg) > m_max_value) {
              if (m_show) {
                irs::mlog() << irsm("Stop Overvoltage") << endl;
              }
              m_need_stop = true;
            }
          }
          
          m_new_result = true;
        }
        if (m_need_stop) {
          //irs::mlog() << irsm("-------------------") << endl;
          //irs::mlog() << m_result_data.avg << endl;
        }
        m_status = st_cread;
        
        if (m_valid_frequency) {
          m_result_data.measured_frequency = calc_frequency(m_time_counter_prev, 
            m_time_counter, m_valid_frequency);
        }
        m_point_time = counter_get() - m_point_time;
        m_result_data.point_time = CNT_TO_DBLTIME(m_point_time);
        mp_adc_exti->start();
      }
    } break;
    case st_spi_wait_stop: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        mp_spi->unlock();
        if (m_need_reconfigure) {
          m_status = st_reconfigure;
        } else {
          m_return_status = irs_st_ready;
          if (m_show) {
            irs::mlog() << endl;
          }
          m_status = st_free;
        }
      }
    } break;
  }
}

void hrm::ad7799_cread_t::set_params(adc_param_data_t* ap_param_data)
{
  m_user_param_data.gain = ap_param_data->gain & gain_mask;
  m_user_param_data.filter = ap_param_data->filter & filter_mask;
  m_user_param_data.channel = ap_param_data->channel & channel_mask;
  size_t min = min_cnv_cnt;
  size_t max = max_cnv_cnt;
  m_user_param_data.cnv_cnt = 
    irs::bound(ap_param_data->cnv_cnt, min, max);
  m_user_param_data.additional_gain = ap_param_data->additional_gain;
  m_user_param_data.ref = ap_param_data->ref;
  m_user_param_data.cont_cnv_cnt = ap_param_data->cont_cnv_cnt;
  m_user_param_data.impf_iterations_cnt = ap_param_data->impf_iterations_cnt;
  m_user_param_data.impf_type = ap_param_data->impf_type;
  m_user_param_data.cont_mode = ap_param_data->cont_mode;
  m_user_param_data.cont_sko = ap_param_data->cont_sko;
  m_need_reconfigure = true;
}

void hrm::ad7799_cread_t::get_params(adc_param_data_t* ap_param_data)
{
  ap_param_data->gain = m_param_data.gain;
  ap_param_data->filter = m_param_data.filter;
  ap_param_data->channel = m_param_data.channel;
  ap_param_data->cnv_cnt = m_param_data.cnv_cnt;
  ap_param_data->additional_gain = m_param_data.additional_gain;
  ap_param_data->ref = m_param_data.ref;
  ap_param_data->cont_cnv_cnt = m_param_data.cont_cnv_cnt;
  ap_param_data->impf_iterations_cnt = m_param_data.impf_iterations_cnt;
  ap_param_data->impf_type = m_param_data.impf_type;
  ap_param_data->cont_mode = m_param_data.cont_mode;
  ap_param_data->cont_sko = m_param_data.cont_sko;
}

hrm::adc_value_t hrm::ad7799_cread_t::calc_impf(
  deque<adc_value_t>* ap_impf_deque, impf_test_data_t* ap_test_data)
{
  counter_t time = counter_get();
  size_t cnt = ap_impf_deque->size();
  adc_value_t max = (*ap_impf_deque)[0];
  adc_value_t min = (*ap_impf_deque)[0];
  size_t max_index = 0;
  size_t min_index = 0;
  adc_value_t summa = 0.0;
  adc_value_t avg = 0.0;
  irs_i32 P = 0;
  irs_i32 N = 0;
  adc_value_t Dtotal = 0.0;
  deque<adc_value_t> impf_deque;
  
  for (size_t i = 0; i < cnt; i++) {
    adc_value_t value = (*ap_impf_deque)[i];
    impf_deque.push_back(value);
    summa += value;
    if (value > max) {
      max = value;
      max_index = i;
    } else if (value < min) {
      min = value;
      min_index = i;
    }
  }
  avg = (summa - min - max) / (cnt - 2);
  impf_deque[min_index] = avg;
  impf_deque[max_index] = avg;
  for (size_t i = 0; i < cnt; i++) {
    adc_value_t value = impf_deque[i];
    if (value > avg) {
      P++;
      Dtotal += abs(avg - value);
    } else if (value < avg) {
      N++;
    }
  }
  adc_value_t delta = static_cast<adc_value_t>(P-N);
  adc_value_t impf = avg + (delta * Dtotal) / (cnt * cnt);
  
  if (ap_test_data) {
    ap_test_data->max = max;
    ap_test_data->max_index = max_index;
    ap_test_data->min = min;
    ap_test_data->min_index = min_index;
    ap_test_data->P = P;
    ap_test_data->N = N;
    ap_test_data->Dtotal = Dtotal;
    ap_test_data->avg = avg;
    ap_test_data->time = counter_get() - time;
  }
  return impf;
}

void hrm::ad7799_cread_t::show_impf_test_data(
  deque<adc_value_t>* ap_value_deque, impf_test_data_t* ap_test_data, 
  size_t a_iteration_number, adc_value_t a_impf_result)
{
  double time = CNT_TO_DBLTIME(ap_test_data->time);
  size_t cnt = ap_value_deque->size();
  irs::mlog() << irsm("---------------------------------") << endl;
  irs::mlog() << irsm("ADC result ");
  irs::mlog() << a_iteration_number << endl;
  for (size_t i = 0; i < cnt; i++) {
    irs::mlog() << (*ap_value_deque)[i];
    if (i == ap_test_data->max_index) {
      irs::mlog() << irsm(" < MAX");
    } else if (i == ap_test_data->min_index) {
      irs::mlog() << irsm(" < MIN");
    }
    irs::mlog() << endl;
  }
  irs::mlog() << irsm("Max = ") << ap_test_data->max << irsm(" (");
  irs::mlog() << ap_test_data->max_index << irsm(")") << endl;
  irs::mlog() << irsm("Min = ") << ap_test_data->min << irsm(" (");
  irs::mlog() << ap_test_data->min_index << irsm(")") << endl;
  irs::mlog() << irsm("P = ") << ap_test_data->P << endl;
  irs::mlog() << irsm("N = ") << ap_test_data->N << endl;
  irs::mlog() << irsm("Dtotal = ") << ap_test_data->Dtotal << endl;
  irs::mlog() << irsm("ImpfAvg = ") << ap_test_data->avg << endl;
  irs::mlog() << irsm("Time = ") << time << endl;
  irs::mlog() << irsm("Impf = ") << a_impf_result << endl;
}

void hrm::ad7799_cread_t::show_param_data(bool a_show, 
  adc_param_data_t* ap_param_data)
{
  if (a_show) {
    irs::mlog() << irsm("Gain          = ");
    irs::mlog() << static_cast<irs_u32>(ap_param_data->gain);
    irs::mlog() << endl;
    irs::mlog() << irsm("Filter        = ");
    irs::mlog() << static_cast<irs_u32>(ap_param_data->filter);
    irs::mlog() << endl;
    irs::mlog() << irsm("Channel       = ");
    irs::mlog() << static_cast<irs_u32>(ap_param_data->channel);
    irs::mlog() << endl;
    irs::mlog() << irsm("Cnv cnt       = ");
    irs::mlog() << ap_param_data->cnv_cnt;
    irs::mlog() << endl;
    irs::mlog() << irsm("Ad. gain      = ");
    irs::mlog() << ap_param_data->additional_gain;
    irs::mlog() << endl;
    irs::mlog() << irsm("Ref           = ");
    irs::mlog() << ap_param_data->ref;
    irs::mlog() << endl;
    irs::mlog() << irsm("Cont cnv cnt  = ");
    irs::mlog() << ap_param_data->cont_cnv_cnt;
    irs::mlog() << endl;
    irs::mlog() << irsm("Impf iter cnt = ");
    irs::mlog() << ap_param_data->impf_iterations_cnt;
    irs::mlog() << endl;
    
    irs::mlog() << irsm("Impf type     = ");
    switch (ap_param_data->impf_type) {
      case impf_mp: irs::mlog() << irsm("MP"); break;
      case impf_mk: irs::mlog() << irsm("MK"); break;
      default: irs::mlog() << irsm("None");
    }
    irs::mlog() << endl;
    
    irs::mlog() << irsm("Cont mode     = ");
    switch (ap_param_data->cont_mode) {
      case cont_mode_cnt: irs::mlog() << irsm("CNT"); break;
      case cont_mode_sko: irs::mlog() << irsm("SKO"); break;
      default: irs::mlog() << irsm("None");
    }
    irs::mlog() << endl;
    
    irs::mlog() << irsm("Cont sko      = ");
    irs::mlog() << ap_param_data->cont_sko;
    irs::mlog() << endl;
  }
}

bool hrm::ad7799_cread_t::new_data(bool* ap_new_data)
{
  bool new_data = *ap_new_data;
  *ap_new_data = false;
  return new_data;
}

void hrm::ad7799_cread_t::show_start_message(bool a_show, 
  adc_param_data_t* ap_param_data)
{
  if (a_show) {
    irs::mlog() << irsm("---------------------------------") << endl;
    irs::mlog() << irsm("ADC: ") << ap_param_data->cnv_cnt << irsm(" points");
    irs::mlog() << endl;
    irs::mlog() << irsm("Gain: ") << static_cast<int>(ap_param_data->gain);
    irs::mlog() << endl;
    irs::mlog() << irsm("Filter: ") << static_cast<int>(ap_param_data->filter);
    irs::mlog() << endl;
    irs::mlog() << irsm("Channel: ") << static_cast<int>(ap_param_data->channel);
    irs::mlog() << endl;
    irs::mlog() << irsm("Add.Gain: ");
    irs::mlog() << static_cast<int>(ap_param_data->additional_gain);
    irs::mlog() << endl;
    irs::mlog() << irsm("Ref: ") << static_cast<int>(ap_param_data->ref);
    irs::mlog() << endl;
    irs::mlog() << irsm("Continious: ");
    switch (ap_param_data->cont_mode) {
      case cont_mode_none: {
        irs::mlog() << irsm("SINGLE");
        break;
      }
      case cont_mode_cnt: {
        irs::mlog() << irsm("CNT = ") << ap_param_data->cont_cnv_cnt;
        break;
      }
      case cont_mode_sko: {
        irs::mlog() << irsm("SKO = ") << (ap_param_data->cont_sko * 1e6);
        irs::mlog() << irsm(" ppm");
        break;
      }
    }
    irs::mlog() << endl;
    irs::mlog() << irsm("Impf Iter. cnt: ");
    irs::mlog() << static_cast<int>(ap_param_data->impf_iterations_cnt);
    irs::mlog() << endl;
    irs::mlog() << irsm("Impf type: ");
    switch (ap_param_data->impf_type) {
      case impf_none: irs::mlog() << irsm("None"); break;
      case impf_mp: irs::mlog() << irsm("Polyakov"); break;
      case impf_mk: irs::mlog() << irsm("Krasheninnikov"); break;
    }
    irs::mlog() << endl;
  }
}

void hrm::ad7799_cread_t::show_verify_message(bool a_show, irs_u8 a_status_reg,
  irs_u8 a_channel)
{
  irs_u8 channel = (a_status_reg) & 0x07;
  irs_u8 err = (a_status_reg >> 6) & 0x01;
  if (channel != a_channel) {
    irs::mlog() << irsm("---------------------------------") << endl;
    irs::mlog() << irsm("ADC Invalid channel:") << endl;
    irs::mlog() << irsm("Write = ") << static_cast<irs_u32>(a_channel) << endl;
    irs::mlog() << irsm("Read =  ") << static_cast<irs_u32>(channel) << endl;
    a_show = true;
  }
  if (err) {
    irs::mlog() << irsm("---------------------------------") << endl;
    irs::mlog() << irsm("ADC Error bit = 1") << endl;
    a_show = true;
  }
  if (a_show) {
    irs_u8 adc_type = (a_status_reg >> 3) & 0x01;
    irs_u8 no_ref = (a_status_reg >> 5) & 0x01;
    irs_u8 rdy = (a_status_reg >> 7) & 0x01;
    irs::mlog() << irsm("---------------------------------") << endl;
    irs::mlog() << irsm("STATUS register:") << endl;
    irs::mlog() << irsm("RAW =     0x");
    irs::mlog() << hex << static_cast<int>(a_status_reg);
    irs::mlog() << endl;
    irs::mlog() << irsm("Channel:  ") << dec << static_cast<int>(channel);
    irs::mlog() << endl;
    irs::mlog() << irsm("ADC type: ") << static_cast<int>(adc_type);
    irs::mlog() << endl;
    irs::mlog() << irsm("NOREF:    ") << static_cast<int>(no_ref);
    irs::mlog() << endl;
    irs::mlog() << irsm("ERR:      ") << static_cast<int>(err);
    irs::mlog() << endl;
    irs::mlog() << irsm("RDY:      ") << static_cast<int>(rdy);
    irs::mlog() << endl;
    irs::mlog() << irsm("---------------------------------") << endl;
  }
}

void hrm::ad7799_cread_t::show_point_symbol(bool a_show)
{
  if (a_show) {
    irs::mlog() << irsm(".") << flush;
  }
}

void hrm::ad7799_cread_t::show_points(bool a_show, adc_value_t a_value)
{
  if (a_show) {
    irs::mlog() << a_value << endl;
  }
}

irs_i32 hrm::ad7799_cread_t::reinterpret_adc_raw_value(irs_u8* ap_spi_buf)
{
  irs_i32 adc_raw_value = 0;
  irs_u8* p_adc_raw_value = reinterpret_cast<irs_u8*>(&adc_raw_value);
  p_adc_raw_value[0] = ap_spi_buf[2];
  p_adc_raw_value[1] = ap_spi_buf[1];
  p_adc_raw_value[2] = ap_spi_buf[0];
  return adc_raw_value;
}

void hrm::ad7799_cread_t::result(adc_result_data_t* ap_result_data)
{
  ap_result_data->avg = m_result_data.avg;
  ap_result_data->sko = m_result_data.sko;
  //ap_result_data->impf = m_result_data.impf;
  //ap_result_data->impf_sko = m_result_data.impf_sko;
  ap_result_data->raw = m_result_data.raw;
  ap_result_data->measured_frequency = m_result_data.measured_frequency;
  ap_result_data->point_time = m_result_data.point_time;
  ap_result_data->current_point = m_result_data.current_point;
  ap_result_data->unnormalized_value = m_result_data.unnormalized_value;
  ap_result_data->saturated = m_result_data.saturated;
}

void hrm::ad7799_cread_t::set_max_value(adc_value_t a_max_value) 
{ 
  m_max_value = a_max_value; 
  irs::mlog() << irsm("ADC saturation ") << m_max_value << irsm(" V") << endl;
}

hrm::impf_type_t hrm::convert_to_impf_type(irs_u8 a_value)
{
  impf_type_t impf_type = impf_none;
  switch (a_value) {
    case 1: impf_type = impf_mp; break;
    case 2: impf_type = impf_mk; break;
    default: impf_type = impf_none;
  }
  return impf_type;
}

irs_u8 hrm::convert_from_impf_type(hrm::impf_type_t a_value)
{
  irs_u8 impf_type;
  switch (a_value) {
    case impf_mp: impf_type = 1; break;
    case impf_mk: impf_type = 2; break;
    default: impf_type = 0;
  }
  return impf_type;
}

hrm::cont_mode_t hrm::convert_to_cont_mode(irs_u8 a_value)
{
  cont_mode_t cont_mode;
  switch (a_value) {
    case 1: cont_mode = cont_mode_cnt; break;
    case 2: cont_mode = cont_mode_sko; break;
    default: cont_mode = cont_mode_none;
  }
  return cont_mode;
}

irs_u8 hrm::convert_from_cont_mode(hrm::cont_mode_t a_value)
{
  irs_u8 cont_mode;
  switch (a_value) {
    case cont_mode_cnt: cont_mode = 1; break;
    case cont_mode_sko: cont_mode = 2; break;
    default: cont_mode = 0;
  }
  return cont_mode;
}

//------------------------------------------------------------------------------

hrm::termostat_t::termostat_t(irs::gpio_pin_t* ap_off_pin):
  mp_off_pin(ap_off_pin),
  m_is_off(false),
  m_timer(0),
  m_status(irs_st_ready)
{
}

void hrm::termostat_t::set_off(bool a_off)
{
  if (a_off) {
    mp_off_pin->set();
    //irs::mlog() << irsm("˜˜˜˜˜˜˜˜˜ OFF") << endl;
  } else {
    mp_off_pin->clear();
    //irs::mlog() << irsm("˜˜˜˜˜˜˜˜˜ ON") << endl;
  }
  m_status = irs_st_busy;
  m_timer.start();
}

bool hrm::termostat_t::is_off()
{
  return mp_off_pin->pin();
}

void hrm::termostat_t::set_after_pause(counter_t a_after_pause)
{
  m_timer.set(a_after_pause);
  irs::mlog() << irsm("Termostat pause ") << CNT_TO_DBLTIME(a_after_pause);
  irs::mlog() << irsm(" s.") << endl;
}

irs_status_t hrm::termostat_t::status()
{
  return m_status;
}

void hrm::termostat_t::tick()
{
  if (m_timer.check()) {
    m_status = irs_st_ready;
    //irs::mlog() << irsm("˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜") << endl;
  }
}

//------------------------------------------------------------------------------

hrm::show_network_params_t::show_network_params_t(network_config_t* ap_config)
{
  mxip_t ip;
  mxip_t mask; 
  mxip_t gateway;
  bool dhcp_enabled;
  mxmac_t mac;
  
  ap_config->get(&ip, &mask, &gateway, &dhcp_enabled);
  ap_config->get_mac(&mac);
  
  irs::mlog() << endl;
  irs::mlog() << irsm("Network parameters") << endl;
  irs::mlog() << irsm("IP:   ");
  irs::mlog() << static_cast<int>(ip.val[0]) << irsm(".");
  irs::mlog() << static_cast<int>(ip.val[1]) << irsm(".");
  irs::mlog() << static_cast<int>(ip.val[2]) << irsm(".");
  irs::mlog() << static_cast<int>(ip.val[3]) << endl;
  irs::mlog() << irsm("MASK: ");
  irs::mlog() << static_cast<int>(mask.val[0]) << irsm(".");
  irs::mlog() << static_cast<int>(mask.val[1]) << irsm(".");
  irs::mlog() << static_cast<int>(mask.val[2]) << irsm(".");
  irs::mlog() << static_cast<int>(mask.val[3]) << endl;
  irs::mlog() << irsm("GATE: ");
  irs::mlog() << static_cast<int>(gateway.val[0]) << irsm(".");
  irs::mlog() << static_cast<int>(gateway.val[1]) << irsm(".");
  irs::mlog() << static_cast<int>(gateway.val[2]) << irsm(".");
  irs::mlog() << static_cast<int>(gateway.val[3]) << endl;
  irs::mlog() << irsm("DHCP: ");
  irs::mlog() << static_cast<int>(dhcp_enabled) << endl;
  irs::mlog() << irsm("MAC:  ");
  irs::mlog().width(2);
  irs::mlog() << hex << right << uppercase << setfill('0');
  irs::mlog() << static_cast<int>(mac.val[0]) << irsm(":");
  irs::mlog() << static_cast<int>(mac.val[1]) << irsm(":");
  irs::mlog() << static_cast<int>(mac.val[2]) << irsm(":");
  irs::mlog() << static_cast<int>(mac.val[3]) << irsm(":");
  irs::mlog() << static_cast<int>(mac.val[4]) << irsm(":");
  irs::mlog() << static_cast<int>(mac.val[5]) << endl;
  irs::mlog() << endl;
  irs::mlog() << dec << setfill(' ');
}

//------------------------------------------------------------------------------

hrm::device_condition_controller_t::device_condition_controller_t(
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
  irs::conn_data_t<float>* ap_fan_dc_speed_sence_data):
  m_fade_tau(30.0),
  m_vref(3.3),
  mp_fan_ac_on(ap_fan_ac_on),
  mp_fan_dc_ls(ap_fan_dc_ls),
  mp_fan_dc_hs(ap_fan_dc_hs),
  mp_fan_dc_sen(ap_fan_dc_sen),
  mp_fan_mode_data(ap_fan_mode_data),
  mp_fan_mode_ee_data(ap_fan_mode_ee_data),
  mp_fan_status_data(ap_fan_status_data),
  mp_fan_ac_speed_data(ap_fan_ac_speed_data),
  mp_fan_ac_speed_ee_data(ap_fan_ac_speed_ee_data),
  mp_fan_dc_speed_data(ap_fan_dc_speed_data),
  mp_fan_dc_speed_ee_data(ap_fan_dc_speed_ee_data),
  mp_fan_dc_speed_sence_data(ap_fan_dc_speed_sence_data),
  m_th_dac_channel_number(2),
  m_th_box_ldo_channel_number(1),
  m_th_box_adc_channel_number(0),
  m_volt_box_neg_channel_number(2),
  m_volt_box_pos_channel_number(1),
  m_th_mcu_channel_number(5),
  m_th_ext_1_channel_number(0),
  m_th_ext_2_channel_number(4),
  m_adc1(adc1_address, adc1_mask),
  m_adc2(adc2_address, adc2_mask),
  m_polling_timer(irs::make_cnt_ms(100)),
  m_th_dac_conditioner(ap_th_dac_data, 0.4, 0.0195, m_vref, m_fade_tau),
  m_th_box_ldo_conditioner(ap_th_box_ldo_data, 0.4, 0.0195, m_vref, m_fade_tau),
  m_th_box_adc_conditioner(ap_th_box_adc_data, 0.4, 0.0195, m_vref, m_fade_tau),
  m_volt_box_neg_conditioner(ap_volt_box_neg_data, 3.3,4.92,m_vref, m_fade_tau),
  m_volt_box_pos_conditioner(ap_volt_box_pos_data,0.0,1.255,m_vref, m_fade_tau),
//  m_volt_box_neg_conditioner(ap_volt_box_neg_data, 0.0,1.0, 1.0, m_fade_tau),
//  m_volt_box_pos_conditioner(ap_volt_box_pos_data,0.0,1.0, 1.0, m_fade_tau),
  m_th_mcu_conditioner(ap_th_mcu_data, 1.0, 1.0, m_vref, m_fade_tau),
  m_th_ext_1_conditioner(ap_th_ext_1_data, 0.4, 0.0195, m_vref, m_fade_tau),
  m_th_ext_2_conditioner(ap_th_ext_2_data, 0.4, 0.0195, m_vref, m_fade_tau),
  m_fan_mode(fan_already_off),
  m_fan_status(fan_off_now),
  m_fan_ac_speed(0),
  m_fan_dc_speed(0),
  m_idle(true),
  m_need_changes(true),
  m_show(false)
{
  mp_fan_ac_on->clear();
  mp_fan_dc_ls->clear();
  mp_fan_dc_hs->clear();
  m_fan_mode = convert_u8_to_fan_mode(*mp_fan_mode_ee_data);
  *mp_fan_mode_data = convert_fan_mode_to_u8(m_fan_mode);
  *mp_fan_status_data = convert_fan_status_to_u8(m_fan_status);
  m_fan_ac_speed = *mp_fan_ac_speed_ee_data;
  *mp_fan_ac_speed_data = m_fan_ac_speed;
  m_fan_dc_speed = *mp_fan_dc_speed_ee_data;
  *mp_fan_dc_speed_data = m_fan_dc_speed;
}

void hrm::device_condition_controller_t::set_fan_speed_ac(irs_u8 a_speed)
{
  if (a_speed > 0) {
    mp_fan_ac_on->set();
  } else {
    mp_fan_ac_on->clear();
  }
}

void hrm::device_condition_controller_t::set_fan_speed_dc(irs_u8 a_speed)
{
  if ((a_speed > 0) && (a_speed <= 1)) {
    mp_fan_dc_ls->set();
    mp_fan_dc_hs->clear();
  } else if (a_speed > 1) {
    mp_fan_dc_ls->clear();
    mp_fan_dc_hs->set();
  } else {
    mp_fan_dc_ls->clear();
    mp_fan_dc_hs->clear();
  }
}


hrm::fan_mode_t hrm::device_condition_controller_t::convert_u8_to_fan_mode(
  irs_u8 a_mode)
{
  fan_mode_t result = fan_already_off;
  switch (a_mode) {
    case 1: result = fan_already_on; break;
    case 2: result = fan_idle_on; break;
    default: result = fan_already_off;
  };
  return result;
}

irs_u8 hrm::device_condition_controller_t::convert_fan_mode_to_u8(
  fan_mode_t a_fan_mode)
{
  irs_u8 result = 0;
  switch (a_fan_mode) {
    case fan_already_off: result = 0; break;
    case fan_already_on: result = 1; break;
    case fan_idle_on: result = 2; break;
    default: result = 0;
  }
  return result;
}


irs_u8 hrm::device_condition_controller_t::convert_fan_status_to_u8(
  fan_status_t a_fan_status)
{
  irs_u8 result = 0;
  if (a_fan_status == fan_on_now) {
    result = 1;
  }
  return result;
}

void hrm::device_condition_controller_t::tick()
{
  m_adc1.tick();
  m_adc2.tick();
  if (m_polling_timer.check()) {
    //  Temperatures / Voltages
    m_th_dac_conditioner.convert(m_adc1.get_u16_data(m_th_dac_channel_number));
    m_th_box_ldo_conditioner.convert(
      m_adc1.get_u16_data(m_th_box_ldo_channel_number));
    m_th_box_adc_conditioner.convert(
      m_adc2.get_u16_data(m_th_box_adc_channel_number));
    m_volt_box_neg_conditioner.convert(
      m_adc1.get_u16_data(m_volt_box_neg_channel_number));
    m_volt_box_pos_conditioner.convert(
      m_adc2.get_u16_data(m_th_box_adc_channel_number));
    m_th_mcu_conditioner.convert(
      m_adc1.get_temperature_degree_celsius(m_vref));
    m_th_ext_1_conditioner.convert(
      m_adc1.get_u16_data(m_th_ext_1_channel_number));
    m_th_ext_2_conditioner.convert(
      m_adc1.get_u16_data(m_th_ext_2_channel_number));
    //  Fans
    if (m_fan_mode != *mp_fan_mode_data) {
      bool valid_mode = false;
      switch (*mp_fan_mode_data) {
        case fan_already_off: valid_mode = true; break;
        case fan_already_on: valid_mode = true; break;
        case fan_idle_on: valid_mode = true; break;
      }
      if (valid_mode) {
        m_fan_mode = convert_u8_to_fan_mode(*mp_fan_mode_data);
        *mp_fan_mode_ee_data = convert_fan_mode_to_u8(m_fan_mode);
        m_need_changes = true;
      } else {
        *mp_fan_mode_data = m_fan_mode;
      }
    }
    if (m_fan_ac_speed != *mp_fan_ac_speed_data) {
      if (*mp_fan_ac_speed_data <= m_fan_ac_max_speed) {
        m_fan_ac_speed = *mp_fan_ac_speed_data;
        *mp_fan_ac_speed_ee_data = m_fan_ac_speed;
        m_need_changes = true;
      }
    }
    if (m_fan_dc_speed != *mp_fan_dc_speed_data) {
      if (*mp_fan_dc_speed_data <= m_fan_dc_max_speed) {
        m_fan_dc_speed = *mp_fan_dc_speed_data;
        *mp_fan_dc_speed_ee_data = m_fan_dc_speed;
        m_need_changes = true;
      }
    }
    
    if (m_need_changes) {
      m_need_changes = false;
      switch (m_fan_mode) {
        case fan_already_off: {
          set_fan_speed_ac(0);
          set_fan_speed_dc(0);
          m_fan_status = fan_off_now;
          if (m_show) {
            irs::mlog() << irsm("FAN OFF") << endl;
          }
          break;
        }
        case fan_already_on: {
          set_fan_speed_ac(m_fan_ac_speed);
          set_fan_speed_dc(m_fan_dc_speed);
          m_fan_status = fan_on_now;
          if (m_show) {
            irs::mlog() << irsm("FAN AC ");
            irs::mlog() << static_cast<int>(m_fan_ac_speed) << endl;
            irs::mlog() << irsm("FAN DC ");
            irs::mlog() << static_cast<int>(m_fan_dc_speed) << endl;
          }
          break;
        }
        case fan_idle_on: {
          if (m_idle) {
            set_fan_speed_ac(m_fan_ac_speed);
            set_fan_speed_dc(m_fan_dc_speed);
            m_fan_status = fan_on_now;
            if (m_show) {
              irs::mlog() << irsm("FAN IDLE AC ");
              irs::mlog() << static_cast<int>(m_fan_ac_speed) << endl;
              irs::mlog() << irsm("FAN IDLE DC ");
              irs::mlog() << static_cast<int>(m_fan_dc_speed) << endl;
            }
          } else {
            set_fan_speed_ac(0);
            set_fan_speed_dc(0);
            m_fan_status = fan_off_now;
            if (m_show) {
              irs::mlog() << irsm("FAN IDLE OFF") << endl;
            }
          }
          break;
        }
      }
    }
    *mp_fan_status_data = convert_fan_status_to_u8(m_fan_status);
  }
}

//------------------------------------------------------------------------------

hrm::adc_conditioner_t::adc_conditioner_t(
  irs::conn_data_t<th_value_t>* ap_out_data,
  hrm::th_value_t a_sub, hrm::th_value_t a_div, 
  th_value_t a_vref, th_value_t a_tau):
  mp_out_data(ap_out_data),
  m_sub(a_sub),
  m_div(a_div),
  m_fade_data(),
  m_need_fade_preset(true),
  m_fade_tau(a_tau),
  m_vref(a_vref)
{
  m_fade_data.t = m_fade_tau;
}

hrm::th_value_t hrm::adc_conditioner_t::convert(irs_u16 a_value)
{
  th_value_t result = (static_cast<th_value_t>(a_value)*m_vref) / pow(2.0, 16);
  result = (result - m_sub) / m_div;
  if (m_need_fade_preset) {
    m_fade_data.x1 = result;
    m_fade_data.y1 = result;
    //fade(&m_fade_data, result);
    m_need_fade_preset = false;
  } else {
    result = fade(&m_fade_data, result);
  }
  *mp_out_data = result;
  return result;
}

hrm::th_value_t hrm::adc_conditioner_t::convert(hrm::th_value_t a_value)
{
  if (m_need_fade_preset) {
    m_fade_data.x1 = a_value;
    m_fade_data.y1 = a_value;
    //fade(&m_fade_data, a_value);
    m_need_fade_preset = false;
  } else {
    a_value = fade(&m_fade_data, a_value);
  }
  *mp_out_data = a_value;
  return a_value;
}

//-----------------------------------------------------------------------------
// class operating_duty_detector_t
hrm::operating_duty_detector_t::operating_duty_detector_t(
    double a_allowable_diviation, double a_time_interval):
  m_range_allowable_diviation(a_allowable_diviation),
  m_allowable_deviation(a_allowable_diviation),
  m_reference_value(0),
  m_range_min(0),
  m_time_interval(a_time_interval),
  m_time()
{
}

void hrm::operating_duty_detector_t::add_current_value(double a_value)
{
  float deviation = fabs(1.f - (a_value/m_reference_value));
  if (deviation > m_allowable_deviation) {
    m_time.start();
  }

  /*static irs::loop_timer_t t(irs::make_cnt_s(1));
  if ((m_reference_value == 1) && t.check()) {
    CLB_DBG_MSG("dev = " << deviation);
    CLB_DBG_MSG("adev = " << m_allowable_deviation);
  }*/
}

void hrm::operating_duty_detector_t::set_allowable_diviation(
  double a_diviation)
{
  m_range_allowable_diviation = a_diviation;
  update_allowable_deviation();
}

void hrm::operating_duty_detector_t::set_reference_value(
  double a_reference_value)
{
  m_reference_value = a_reference_value;
  update_allowable_deviation();
}

void hrm::operating_duty_detector_t::set_range_min(double a_range_min)
{
  m_range_min = a_range_min;
  update_allowable_deviation();
}

void hrm::operating_duty_detector_t::update_allowable_deviation()
{
  m_allowable_deviation = m_range_allowable_diviation;
  // ˜˜˜˜ ˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜, ˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜
  // ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜
  if ((m_reference_value < m_range_min) && (m_range_min > 0)) {
    const double k = m_range_min/m_reference_value;

    double k1 = 1;
    /*if (k > 50) {
      k1 = 5; // ˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜. ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜
    }*/

    m_allowable_deviation *= (k1*k);
  }
  //irs::mlog() << irsm("ad = ") << m_allowable_deviation << endl;
  //CLB_DBG_MSG("ad = " << m_allowable_deviation);
}

void hrm::operating_duty_detector_t::set_time_interval(
  double a_time_interval)
{
  m_time_interval = a_time_interval;
}

void hrm::operating_duty_detector_t::reset()
{
  m_time.start();
}

bool hrm::operating_duty_detector_t::ready() const
{
  return (m_time.get() >= m_time_interval);
}

//-----------------------------------------------------------------------------

hrm::temperature_sensor_conn_data_t::temperature_sensor_conn_data_t(
  irs::conn_data_t<double>* ap_th_data,
  double a_min_temperature, double a_max_temperature):
  mp_th_data(ap_th_data),
  m_min_temperature(a_min_temperature),
  m_max_temperature(a_max_temperature)
{
}

bool hrm::temperature_sensor_conn_data_t::temperature_is_normal()
{
  bool result = false;
  if (*mp_th_data > m_min_temperature && *mp_th_data < m_max_temperature) {
    result = true;
  }
  return result;
}

//-----------------------------------------------------------------------------
// class peltier_t
hrm::peltier_t::peltier_t(const parameters_t& a_parameters):
  m_parameters(a_parameters),
  mp_temp_sensor(a_parameters.temperature_sensor),
  mp_pwm(dynamic_cast<irs::arm::st_pwm_gen_t*>(a_parameters.pwm)),
  mp_polarity_pin(a_parameters.polarity_pin),
  m_enabled(true),
  m_polarity(polarity_cool),
  m_duty_setpoint(0),
  m_duty_actual(0),
  m_duty_max(min(fabs(a_parameters.pwm_max_code_float), 1.)),
  m_setpoint(20),
  m_pid_reg_enabled(true),
  m_reg_pid_interval(0.1),
  m_reg_pid_timer(irs::make_cnt_s(m_reg_pid_interval)),
  m_reg_pid_data(),
  m_pid_reg_k_factor(0),
  m_iso_data(irs::zero_struct_t<irs::isodr_data_t>::get()),
  m_rate_data(),
  m_operating_duty_detector(a_parameters.operating_duty_deviation,
    a_parameters.operating_duty_time_interval_s),
  m_ready(false),
  m_sync_data_timer(irs::make_cnt_ms(100)),
  m_change_polarity_timer(irs::make_cnt_s(10)),
  m_polarity_change(false),
  polarity_cool(
    a_parameters.polarity_map == default_polarity_map ?
    polarity_cool_def : polarity_heat_def
  ),
  polarity_heat(
    a_parameters.polarity_map == default_polarity_map ?
    polarity_heat_def : polarity_cool_def
  )
{
  //IRS_LIB_ASSERT(mp_pwm);

  mp_polarity_pin->clear();

  //  ˜˜˜˜˜˜˜˜˜
  m_reg_pid_data.k = 1;
  m_reg_pid_data.ki = 0;
  m_reg_pid_data.kd = 0;
  m_reg_pid_data.min = -1*m_duty_max;
  m_reg_pid_data.max = 1*m_duty_max;
  m_reg_pid_data.prev_e = 0.;
  m_reg_pid_data.pp_e = 0.;
  m_reg_pid_data.prev_out = 0.;
  m_reg_pid_data.block_int = 0;
  m_reg_pid_data.block_int_ext = 0;
  m_reg_pid_data.int_val = 0.;
  m_reg_pid_data.k_d_pid = 0.1;

  m_rate_data.cur = 0;
  m_rate_data.dt = m_reg_pid_interval;
  m_rate_data.slope = 0.03;
  m_rate_data.recalc = 0;
  m_rate_data.dl = 0;
  m_rate_data.slope_prev = 0;
  m_rate_data.dt_prev = 0;

  m_iso_data.fd.t = 0;
  m_iso_data.k = 1;

  *m_parameters.amplitude_code_float = m_duty_actual;
  *m_parameters.enabled = m_enabled;
  *m_parameters.pid_reg_enabled = m_pid_reg_enabled;
  *m_parameters.polarity_pin_bit_data = (m_polarity == polarity_heat);

  m_operating_duty_detector.set_reference_value(m_setpoint);

  mp_pwm->start();
}

double hrm::peltier_t::get_temperature()
{
  return mp_temp_sensor->get_temperature();
}

void hrm::peltier_t::set_temperature_setpoint(double a_setpoint)
{
  if (m_setpoint != a_setpoint) {
    m_ready = false;
  }
  m_setpoint = a_setpoint;
  *m_parameters.temperature_setpoint = m_setpoint;
  m_operating_duty_detector.set_reference_value(m_setpoint);
}

bool hrm::peltier_t::ready() const
{
  return m_ready;
}

bool hrm::peltier_t::regulator_enabled() const
{
  return m_pid_reg_enabled;
}

void hrm::peltier_t::regulator_enabled(bool a_enabled)
{
  m_pid_reg_enabled = a_enabled;
}

void hrm::peltier_t::pwm_regulator()
{
  #define CLB_POLARITY_DELAY

  #ifdef CLB_POLARITY_DELAY
  // ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜ ˜˜˜˜˜˜˜˜ ˜˜ 0 ˜˜ m_duty_max
  double setpoint = 0;
  if (is_same_polarity(m_duty_setpoint)) {
    setpoint = irs::range(fabs(m_duty_setpoint), 0., m_duty_max);
  } else {
    setpoint = 0;
  }

  // ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜˜
  const double cur_duty = irs::rate_limiter(&m_rate_data, setpoint);
  set_duty_hard(cur_duty);

  // ˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜, ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜
  if (is_same_polarity(m_duty_setpoint)) {
    m_polarity_change = false;
  } else {

    // ˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜ ˜˜˜˜˜˜˜, ˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜ 0 ˜ ˜˜˜˜ ˜˜˜˜˜
    // ˜˜ ˜˜˜˜˜˜˜

    if (m_polarity_change) {
      if (m_change_polarity_timer.check()) {
        m_polarity_change = false;
        if (m_duty_setpoint >= 0) {
          set_polarity(polarity_cool);
        } else {
          set_polarity(polarity_heat);
        }
        set_duty_hard(0);
      }
    } else {
      if (cur_duty == 0) {
        m_polarity_change = true;
        m_change_polarity_timer.start();
      }
    }
  }
  #else //CLB_POLARITY_DELAY
  double setpoint = irs::range(fabs(m_duty_setpoint), 0., m_duty_max);
  const double cur_duty = irs::rate_limiter(&m_rate_data, setpoint);
  set_duty_hard(cur_duty);
  if (m_duty_setpoint >= 0) {
    set_polarity(polarity_cool);
  } else {
    set_polarity(polarity_heat);
  }
  #endif //CLB_POLARITY_DELAY

}

void hrm::peltier_t::set_polarity(polarity_t a_polarity)
{
  if (m_polarity != a_polarity) {
    set_duty_hard(0);
  }
  m_polarity = a_polarity;
  m_parameters.polarity_pin->set_pin(m_polarity);
  *m_parameters.polarity_pin_bit_data = m_polarity;
}

hrm::peltier_t::polarity_t
hrm::peltier_t::get_bit_data_polarity() const
{
  if (*m_parameters.polarity_pin_bit_data == static_cast<bool>(polarity_cool))
  {
    return polarity_cool;
  } else {
    return polarity_heat;
  }
}

void hrm::peltier_t::tick()
{
  sync_remote_data();
  if (m_reg_pid_timer.check()) {

    const double cur_ref_value = m_setpoint;

    double cur_value = get_temperature();

    m_operating_duty_detector.add_current_value(cur_value);
    m_ready = m_operating_duty_detector.ready();

    const double e = irs::isodr(&m_iso_data, cur_value) - cur_ref_value;

    if (m_enabled && m_pid_reg_enabled) {
      const double setpoint = irs::pid_reg(&m_reg_pid_data, e);
      set_duty_soft(setpoint);
    } else {
      pid_reg_sync(&m_reg_pid_data, e, m_duty_setpoint);
    }
    pwm_regulator();
  }
}

bool hrm::peltier_t::is_same_polarity(double a_duty)
{
  return (a_duty >= 0) == (m_polarity == polarity_cool);
}

void hrm::peltier_t::set_duty_soft(double a_duty)
{
  m_duty_setpoint = irs::range<double>(a_duty, -1*m_duty_max, m_duty_max);
  *m_parameters.pid_out = -m_duty_setpoint;
}

void hrm::peltier_t::set_duty_hard(double a_duty)
{
  #if (CLB_HARDWARE_REV >= CLB_HW_REV_2)
  m_duty_actual = a_duty;
  #elif (CLB_HARDWARE_REV == CLB_HW_REV_1)
  m_duty_actual = (m_polarity == polarity_cool) ? a_duty: (1 - a_duty);
  #endif // (CLB_HARDWARE_REV == CLB_HW_REV_1)
  const float duty = irs::range<float>(m_duty_actual, 0, 1);
  mp_pwm->set_duty(m_parameters.pwm_channel, duty);
  *m_parameters.amplitude_code_float =
    (m_polarity == polarity_cool) ? -a_duty: a_duty;
}

void hrm::peltier_t::sync_remote_data()
{
  if (m_sync_data_timer.check()) {

    if (m_setpoint != *m_parameters.temperature_setpoint) {
      m_setpoint = *m_parameters.temperature_setpoint;
      m_operating_duty_detector.set_reference_value(m_setpoint);
      m_ready = false;
    }

    //*m_parameters.temperature = get_temperature();

    m_reg_pid_data.k = *m_parameters.pid_k;
    m_reg_pid_data.ki = *m_parameters.pid_ki*m_reg_pid_interval;
    m_reg_pid_data.kd = *m_parameters.pid_kd/m_reg_pid_interval;
    pid_reg_sync(&m_reg_pid_data);

    m_iso_data.k = *m_parameters.iso_k;
    m_iso_data.fd.t = *m_parameters.iso_t;

    m_rate_data.slope = *m_parameters.pwm_rate_slope;

    if (*m_parameters.pid_out != m_duty_setpoint) {
      set_duty_soft(*m_parameters.pid_out);
    }

    /*if (*m_parameters.amplitude_code_float != m_duty_actual) {
      m_duty_actual = *m_parameters.amplitude_code_float;
      mp_pwm->set_duty(m_parameters.pwm_channel, m_duty_actual);
    }*/
    if (*m_parameters.polarity_pin_bit_data != static_cast<bool>(m_polarity)) {
      set_polarity(get_bit_data_polarity());
    }

    if (*m_parameters.pid_reg_enabled != m_pid_reg_enabled) {
      m_pid_reg_enabled = *m_parameters.pid_reg_enabled;
    }

    if (*m_parameters.enabled != m_enabled) {
      m_enabled = *m_parameters.enabled;
      if (!m_enabled) {
        set_duty_soft(0);
      }
    }
  }
}

//------------------------------------------------------------------------------

hrm::sync_treg_parameters_t::sync_treg_parameters_t(
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
  irs::bit_data_as_bool_t* ap_ee_treg_polarity_pin_bit_data):
  mp_eth_treg_ref(ap_eth_treg_ref),
  mp_eth_treg_k(ap_eth_treg_k),
  mp_eth_treg_ki(ap_eth_treg_ki),
  mp_eth_treg_kd(ap_eth_treg_kd),
  mp_eth_treg_iso_k(ap_eth_treg_iso_k),
  mp_eth_treg_iso_t(ap_eth_treg_iso_t),
  mp_eth_treg_pwm_rate_slope(ap_eth_treg_pwm_rate_slope),
  mp_eth_treg_enabled(ap_eth_treg_enabled),
  mp_eth_treg_pid_reg_enabled(ap_eth_treg_pid_reg_enabled),
  mp_eth_treg_polarity_pin_bit_data(ap_eth_treg_polarity_pin_bit_data),
  mp_ee_treg_ref(ap_ee_treg_ref),
  mp_ee_treg_k(ap_ee_treg_k),
  mp_ee_treg_ki(ap_ee_treg_ki),
  mp_ee_treg_kd(ap_ee_treg_kd),
  mp_ee_treg_iso_k(ap_ee_treg_iso_k),
  mp_ee_treg_iso_t(ap_ee_treg_iso_t),
  mp_ee_treg_pwm_rate_slope(ap_ee_treg_pwm_rate_slope),
  mp_ee_treg_enabled(ap_ee_treg_enabled),
  mp_ee_treg_pid_reg_enabled(ap_ee_treg_pid_reg_enabled),
  mp_ee_treg_polarity_pin_bit_data(ap_ee_treg_polarity_pin_bit_data)
{
  *mp_eth_treg_ref = *mp_ee_treg_ref;
  *mp_eth_treg_k = *mp_ee_treg_k;
  *mp_eth_treg_ki = *mp_ee_treg_ki;
  *mp_eth_treg_kd = *mp_ee_treg_kd;
  *mp_eth_treg_iso_k = *mp_ee_treg_iso_k;
  *mp_eth_treg_iso_t = *mp_ee_treg_iso_t;
  *mp_eth_treg_pwm_rate_slope = *mp_ee_treg_pwm_rate_slope;
  *mp_eth_treg_enabled = *mp_ee_treg_enabled;
  *mp_eth_treg_pid_reg_enabled = *mp_ee_treg_pid_reg_enabled;
  *mp_eth_treg_polarity_pin_bit_data = *mp_ee_treg_polarity_pin_bit_data;
}

void hrm::sync_treg_parameters_t::sync()
{
  if (*mp_eth_treg_ref != *mp_ee_treg_ref) {
    *mp_ee_treg_ref = *mp_eth_treg_ref;
  }
  if (*mp_eth_treg_k != *mp_ee_treg_k) {
    *mp_ee_treg_k = *mp_eth_treg_k;
  }
  if (*mp_eth_treg_ki != *mp_ee_treg_ki) {
    *mp_ee_treg_ki = *mp_eth_treg_ki;
  }
  if (*mp_eth_treg_kd != *mp_ee_treg_kd) {
    *mp_ee_treg_kd = *mp_eth_treg_kd;
  }
  if (*mp_eth_treg_iso_k != *mp_ee_treg_iso_k) {
    *mp_ee_treg_iso_k = *mp_eth_treg_iso_k;
  }
  if (*mp_eth_treg_iso_t != *mp_ee_treg_iso_t) {
    *mp_ee_treg_iso_t = *mp_eth_treg_iso_t;
  }
  if (*mp_eth_treg_pwm_rate_slope != *mp_ee_treg_pwm_rate_slope) {
    *mp_ee_treg_pwm_rate_slope = *mp_eth_treg_pwm_rate_slope;
  }
  if (*mp_eth_treg_enabled != *mp_ee_treg_enabled) {
    
    *mp_ee_treg_enabled = *mp_eth_treg_enabled;
  }
  if (*mp_eth_treg_pid_reg_enabled != *mp_ee_treg_pid_reg_enabled) {
    *mp_ee_treg_pid_reg_enabled = *mp_eth_treg_pid_reg_enabled;
  }
  if (*mp_eth_treg_polarity_pin_bit_data != *mp_ee_treg_polarity_pin_bit_data) {
    *mp_ee_treg_polarity_pin_bit_data = *mp_eth_treg_polarity_pin_bit_data;
  }
}
