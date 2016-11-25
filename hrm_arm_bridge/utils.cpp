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
        irs::mlog() << irsm("Реле ") << m_caption << irsm(" = ")
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
        irs::mlog() << irsm("Реле ") << m_caption << irsm(" = ")
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
  irs::mlog() << irsm("ЦАП: включён") << endl;
  irs::mlog() << irsm("ЦАП: код = 0") << endl;
}

void hrm::dac_t::on()
{
  m_dac_data.opgnd_bit = 0;
  m_dac_data.dactri_bit = 0;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  if (m_show) {
    irs::mlog() << irsm("ЦАП: включён") << endl;
  }
}

void hrm::dac_t::off()
{
  m_dac_data.opgnd_bit = 1;
  m_dac_data.dactri_bit = 1;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  if (m_show) {
    irs::mlog() << irsm("ЦАП: выключен") << endl;
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
    irs::mlog() << irsm("ЦАП: код = ") << code << irsm(" / ")
      << (m_dac_data.signed_voltage_code >> 12) << endl;
  }
}

void hrm::dac_t::set_code(dac_value_t a_code)
{
  irs_i32 code = static_cast<irs_i32>(a_code);
  m_dac_data.unsigned_voltage_code = code << 12;
  m_status = st_wait;
  if (m_show) {
    irs::mlog() << irsm("ЦАП: код = ")
      << static_cast<dac_value_t>(code) / pow(2., 19) << irsm(" / ")
      << code << endl;
  }
}

void hrm::dac_t::set_lin(irs_u8 a_lin)
{
  m_dac_data.lin_comp = a_lin;
  m_status = st_wait;
  if (m_show) {
    irs::mlog() << irsm("ЦАП: lin = 0x") << hex << static_cast<int>(a_lin);
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
  irs::mlog() << irsm("ЦАП: пауза = ") << pause << irsm(" c.") << endl;
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
    irs::mlog() << irsm("АЦП Kдоп = ") << m_additional_gain << endl;
  }
}

void hrm::adc_t::set_ref(adc_value_t a_ref)
{
  m_ref = a_ref;
  if (m_show) {
    irs::mlog() << irsm("АЦП REF = ") << m_ref << endl;
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
        irs::mlog() << irsm("Усиление АЦП = ") << (1 << m_current_gain) << endl;
      }
    }
    break;
  case st_set_channel:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_set_channel = false;
      mp_raw_adc->set_param(irs::adc_channel, m_current_channel);
      m_status = st_wait_param;
      if (m_show) {
        irs::mlog() << irsm("Канал АЦП = ") << (m_current_channel + 1) << endl;
      }
    }
    break;
  case st_set_mode:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_set_mode = false;
      mp_raw_adc->set_param(irs::adc_mode, m_current_mode);
      m_status = st_wait_param;
      if (m_show) {
        irs::mlog() << irsm("Режим АЦП = ")
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
        irs::mlog() << irsm("Фильтр АЦП = ")
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
        irs::mlog() << irsm("Ноль АЦП = ") << (m_zero * 1.e6)
          << irsm(" мкВ") << endl;
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
        irs::mlog() << irsm("Температура = ") << m_temperature << irsm(" °C");
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
        irs::mlog() << irsm("Напряжение АЦП = ");
        if (abs(m_voltage) < 1.1e-3) {
          irs::mlog() << (m_voltage * 1.e6) << irsm(" мкВ");
        } else if (abs(m_voltage < 1.1)) {
          irs::mlog() << (m_voltage * 1.e3) << irsm(" мВ");
        } else {
          irs::mlog() << m_voltage << irsm(" В");
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
  m_bzz_interval(irs::make_cnt_ms(10)),
  m_bzzz_interval(irs::make_cnt_ms(500)),
  m_buzzed(false),
  m_timer(),
  mp_pin(ap_buzzer_pin)
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

void hrm::buzzer_t::bzz()
{
  m_timer.set(m_bzz_interval);
  m_timer.start();
  m_buzzed = true;
  mp_pin->set();
}

void hrm::buzzer_t::tick()
{
  if (m_timer.check())
  {
    m_timer.stop();
    m_buzzed = false;
    mp_pin->clear();
  }
}

//------------------------------------------------------------------------------

hrm::ad7799_cread_t::ad7799_cread_t(irs::spi_t *ap_spi,
  irs::gpio_pin_t* ap_cs_pin, adc_exti_t* ap_adc_exti):
  mp_spi(ap_spi),
  mp_cs_pin(ap_cs_pin),
  mp_adc_exti(ap_adc_exti),
  m_int_event(this, &ad7799_cread_t::event),
  m_status(st_free),
  m_return_status(irs_st_ready),
  m_cnv_cnt(min_cnv_cnt),
  m_user_cnv_cnt(min_cnv_cnt),
  m_index(0),
  m_value_deque(),
  m_fast_sko(m_cnv_cnt, m_cnv_cnt),
  m_impf_sko(m_cnv_cnt, m_cnv_cnt),
  m_gain(0),
  m_channel(0),
  m_filter(0),
  m_user_gain(0),
  m_user_channel(0),
  m_user_filter(0),
  m_pause_interval(0),
  m_cs_interval(irs::make_cnt_us(1)),
  m_timer(m_cs_interval),
  m_show(false),
  m_additional_gain(0.0),
  m_ref(0.0),
  m_avg(0.0),
  m_sko(0.0),
  m_impf(0.0),
  m_use_impf(false),
  m_impf_iterations_cnt(0),
  m_continious_mode(false),
  m_new_avg(false),
  m_new_sko(false),
  m_new_impf(false),
  m_new_impf_sko(false),
  m_need_prefilling(false),
  m_time_counter_prev(0),
  m_time_counter(0),
  m_point_time(0),
  m_new_point_time(false),
  m_need_reconfigure(false),
  m_new_result(false)
{
  mp_cs_pin->set();
  //m_cur_point.value = 0;
  //m_cur_point.time = 0;
  memset(mp_spi_buf, 0, spi_buf_size);
  mp_adc_exti->add_event(&m_int_event);
  mp_adc_exti->stop();
  m_result_data.avg = 0.0;
  m_result_data.sko = 0.0;
  m_result_data.impf = 0.0;
  m_result_data.impf_sko = 0.0;
  m_result_data.min = 0.0;
  m_result_data.max = 0.0;
}

void hrm::ad7799_cread_t::start_conversion()
{
  if (m_status == st_free) {
    m_time_counter_prev = m_time_counter;
    m_time_counter = counter_get();
    m_status = st_reconfigure;
    m_return_status = irs_st_busy;
  }
}

void hrm::ad7799_cread_t::event()
{
  //m_cur_point.time = counter_get();
  mp_spi->read(mp_spi_buf, read_buf_size);
  m_status = st_spi_point_processing;
  mp_adc_exti->stop();
}

double hrm::ad7799_cread_t::get_adc_frequency()
{
  double freq = 0.0;
  freq = CNT_TO_DBLTIME(m_time_counter - m_time_counter_prev);
  if (freq > 0.0) {
    freq = 1.0 / freq;
  } else {
    freq = 0.0;
  }
  return freq;
}

void hrm::ad7799_cread_t::tick()
{
  mp_spi->tick();
  switch(m_status) {
    case st_free: {
      break;
    }
    case st_reconfigure: {
      m_index = 0;
      m_cnv_cnt = m_user_cnv_cnt;
      m_gain = m_user_gain;
      m_filter = m_user_filter;
      m_channel = m_user_channel;
      while (m_value_deque.size() > m_cnv_cnt) {
        m_value_deque.pop_front();
      }
      m_fast_sko.resize(m_cnv_cnt);
      m_fast_sko.resize_average(m_cnv_cnt);
      m_impf_sko.resize(m_cnv_cnt);
      m_impf_sko.resize_average(m_cnv_cnt);
      m_need_prefilling = true;
      m_need_reconfigure = false;
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
        mp_spi_buf[1] = m_gain;
        mp_spi_buf[2] = m_channel | (1 << buf_bit_offset);
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
        mp_spi_buf[2] = m_filter;
        mp_cs_pin->clear();
        mp_spi->write(mp_spi_buf, write_mode_size);
        m_status = st_cs_pause_mode;
      }
    } break;
    case st_cs_pause_mode: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        m_timer.set(m_cs_interval + m_pause_interval);
        m_timer.start();
        m_status = st_start;
      }
    } break;
    case st_start: {
      if (m_timer.check()) {
        show_start_message(m_show, m_cnv_cnt, m_gain, m_filter, m_channel);
        mp_spi_buf[0] = instruction_cread;
        mp_cs_pin->clear();
        mp_spi->write(mp_spi_buf, start_buf_size);
        m_status = st_spi_first_wait;
      }
    } break;
    case st_spi_first_wait: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_adc_exti->start();
        m_status = st_cread;
      }
    } break;
    case st_cread: {
      break;
    }
    case st_spi_point_processing: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        m_point_time = counter_get();
        irs_i32 adc_raw_value = reinterpret_adc_raw_value(mp_spi_buf);
        adc_value_t adc_value = convert_value(adc_raw_value);
        if (m_value_deque.size() >= m_cnv_cnt) {
          m_value_deque.pop_front();
        }
        m_value_deque.push_back(adc_value);
        m_fast_sko.add(adc_value);
        
        if (m_index >= (m_cnv_cnt - 1)) {
          m_need_prefilling = false;
        }
        
        impf_test_data_t impf_test_data;
        if (!m_need_prefilling && m_use_impf) {
          size_t iterations_cnt = m_impf_iterations_cnt;
          size_t max_iterations_cnt = m_cnv_cnt / 2;
          
          if (iterations_cnt == 0 || iterations_cnt > max_iterations_cnt) {
            iterations_cnt = max_iterations_cnt;
          }
          for (size_t i = 0; i < iterations_cnt; i++) {
            m_impf = calc_impf(&m_value_deque, &impf_test_data);
            if (m_test_impf) {
              show_impf_test_data(&m_value_deque, &impf_test_data, i, m_impf);
            }
          }
          m_impf_sko.add(m_impf);
          m_new_impf = true;
          m_test_impf = false;
        }
        m_sko = m_fast_sko.relative();
        m_avg = m_fast_sko.average();
        m_new_avg = true;
        m_new_sko = true;
       
        m_result_data.avg = m_avg;
        m_result_data.sko = m_fast_sko.relative();
        m_result_data.impf = m_impf;
        m_result_data.impf_sko = m_impf_sko.relative();
        m_result_data.min = impf_test_data.min;
        m_result_data.max = impf_test_data.max;
        m_result_data.raw = adc_raw_value;
        m_new_result = true;
        
        show_point_symbol(m_show);
        
        m_index++;
        if (m_index < m_cnv_cnt) {
          mp_adc_exti->start();
          m_status = st_cread;
        } else {
          m_index = 0;
          if (m_need_reconfigure || !m_continious_mode) {
            mp_spi_buf[0] = instruction_stop;
            mp_spi->write(mp_spi_buf, stop_buf_size);
            m_status = st_spi_wait_stop;
          } else {
            mp_adc_exti->start();
            m_status = st_cread;
          }
        }
        m_point_time = counter_get() - m_point_time;
        m_new_point_time = true;
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

void hrm::ad7799_cread_t::set_gain(irs_u8 a_gain) 
{ 
  m_user_gain = a_gain & gain_mask;
  m_need_reconfigure = true;
}

void hrm::ad7799_cread_t::set_channel(irs_u8 a_channel)
{ 
  m_user_channel = a_channel & channel_mask;
  m_need_reconfigure = true;
}

void hrm::ad7799_cread_t::set_filter(irs_u8 a_filter)
{ 
  m_user_filter = a_filter & filter_mask;
  m_need_reconfigure = true;
}

void hrm::ad7799_cread_t::set_cnv_cnt(size_t a_cnv_cnt)
{ 
  if (a_cnv_cnt < min_cnv_cnt) {
    a_cnv_cnt = min_cnv_cnt;
  } else if (a_cnv_cnt > max_cnv_cnt) {
    a_cnv_cnt = max_cnv_cnt;
  }
  m_user_cnv_cnt = a_cnv_cnt; 
  m_need_reconfigure = true;
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
  //  Вычисление среднего и замена абсолютных максимума 
  //  и минимума на среднее
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
  //  Подсчёт количества и суммы отклонений от среднего
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
  irs::mlog() << irsm("Итерация ");
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

bool hrm::ad7799_cread_t::new_data(bool* ap_new_data)
{
  bool new_data = *ap_new_data;
  *ap_new_data = false;
  return new_data;
}

void hrm::ad7799_cread_t::show_start_message(bool a_show, size_t a_cnv_cnt, 
  irs_u8 a_gain, irs_u8 a_filter, irs_u8 a_channel)
{
  if (a_show) {
    irs::mlog() << irsm("---------------------------------") << endl;
    irs::mlog() << irsm("АЦП: ") << a_cnv_cnt << irsm(" точек");
    irs::mlog() << endl;
    irs::mlog() << irsm("Gain: ") << static_cast<int>(a_gain);
    irs::mlog() << endl;
    irs::mlog() << irsm("Filter: ") << static_cast<int>(a_filter);
    irs::mlog() << endl;
    irs::mlog() << irsm("Channel: ") << static_cast<int>(a_channel);
    irs::mlog() << endl;
  }
}

void hrm::ad7799_cread_t::show_point_symbol(bool a_show)
{
  if (a_show) {
    irs::mlog() << irsm(".") << flush;
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
  ap_result_data->impf = m_result_data.impf;
  ap_result_data->impf_sko = m_result_data.impf_sko;
  ap_result_data->min = m_result_data.min;
  ap_result_data->max = m_result_data.max;
  ap_result_data->raw = m_result_data.raw;
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
    //irs::mlog() << irsm("Термостат OFF") << endl;
  } else {
    mp_off_pin->clear();
    //irs::mlog() << irsm("Термостат ON") << endl;
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
  irs::mlog() << irsm("Термостат пауза ") << CNT_TO_DBLTIME(a_after_pause);
  irs::mlog() << irsm(" с.") << endl;
}

irs_status_t hrm::termostat_t::status()
{
  return m_status;
}

void hrm::termostat_t::tick()
{
  if (m_timer.check()) {
    m_status = irs_st_ready;
    //irs::mlog() << irsm("Термостат готов") << endl;
  }
}
