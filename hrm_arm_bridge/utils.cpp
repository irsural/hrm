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
        irs::mlog() << irsm("���� ") << m_caption << irsm(" = ")
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
        irs::mlog() << irsm("���� ") << m_caption << irsm(" = ")
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
  irs::mlog() << irsm("���: �������") << endl;
  irs::mlog() << irsm("���: ��� = 0") << endl;
}

void hrm::dac_t::on()
{
  m_dac_data.opgnd_bit = 0;
  m_dac_data.dactri_bit = 0;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  if (m_show) {
    irs::mlog() << irsm("���: �������") << endl;
  }
}

void hrm::dac_t::off()
{
  m_dac_data.opgnd_bit = 1;
  m_dac_data.dactri_bit = 1;
  m_status = st_wait;
  m_timer.set(m_after_pause);
  if (m_show) {
    irs::mlog() << irsm("���: ��������") << endl;
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
    irs::mlog() << irsm("���: ��� = ") << code << irsm(" / ")
      << (m_dac_data.signed_voltage_code >> 12) << endl;
  }
}

void hrm::dac_t::set_code(dac_value_t a_code)
{
  irs_i32 code = static_cast<irs_i32>(a_code);
  m_dac_data.unsigned_voltage_code = code << 12;
  m_status = st_wait;
  if (m_show) {
    irs::mlog() << irsm("���: ��� = ")
      << static_cast<dac_value_t>(code) / pow(2., 19) << irsm(" / ")
      << code << endl;
  }
}

void hrm::dac_t::set_lin(irs_u8 a_lin)
{
  m_dac_data.lin_comp = a_lin;
  m_status = st_wait;
  if (m_show) {
    irs::mlog() << irsm("���: lin = 0x") << hex << static_cast<int>(a_lin);
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
  irs::mlog() << irsm("���: ����� = ") << pause << irsm(" c.") << endl;
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
    irs::mlog() << irsm("��� K��� = ") << m_additional_gain << endl;
  }
}

void hrm::adc_t::set_ref(adc_value_t a_ref)
{
  m_ref = a_ref;
  if (m_show) {
    irs::mlog() << irsm("��� REF = ") << m_ref << endl;
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
        irs::mlog() << irsm("�������� ��� = ") << (1 << m_current_gain) << endl;
      }
    }
    break;
  case st_set_channel:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_set_channel = false;
      mp_raw_adc->set_param(irs::adc_channel, m_current_channel);
      m_status = st_wait_param;
      if (m_show) {
        irs::mlog() << irsm("����� ��� = ") << (m_current_channel + 1) << endl;
      }
    }
    break;
  case st_set_mode:
    if (mp_raw_adc->status() == meas_status_success) {
      m_need_set_mode = false;
      mp_raw_adc->set_param(irs::adc_mode, m_current_mode);
      m_status = st_wait_param;
      if (m_show) {
        irs::mlog() << irsm("����� ��� = ")
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
        irs::mlog() << irsm("������ ��� = ")
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
        irs::mlog() << irsm("���� ��� = ") << (m_zero * 1.e6)
          << irsm(" ���") << endl;
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
        irs::mlog() << irsm("����������� = ") << m_temperature << irsm(" �C");
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
        irs::mlog() << irsm("���������� ��� = ");
        if (abs(m_voltage) < 1.1e-3) {
          irs::mlog() << (m_voltage * 1.e6) << irsm(" ���");
        } else if (abs(m_voltage < 1.1)) {
          irs::mlog() << (m_voltage * 1.e3) << irsm(" ��");
        } else {
          irs::mlog() << m_voltage << irsm(" �");
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
  m_cnv_cnt(0),
  m_skip_cnt(0),
  m_index(0),
  m_value_vector(),
  m_value_sko(0.0, 0.0),
  m_time_sko(0.0, 0.0),
  m_gain(0),
  m_channel(0),
  m_filter(0),
  m_pause_interval(0),
  m_cs_interval(irs::make_cnt_us(1)),
  m_timer(m_cs_interval),
  m_show(false),
  m_show_simply(false),
  m_additional_gain(0.0),
  m_ref(0.0),
  m_avg(0.0),
  m_sko(0.0)
{
  mp_cs_pin->set();
  m_cur_point.value = 0;
  m_cur_point.time = 0;
  memset(mp_spi_buf, 0, spi_buf_size);
  mp_adc_exti->add_event(&m_int_event);
  mp_adc_exti->stop();
}

void hrm::ad7799_cread_t::start_conversion()
{
  if (m_status == st_free) {
    m_value_vector.clear();
    m_status = st_spi_prepare;
    m_return_status = irs_st_busy;
  }
}

void hrm::ad7799_cread_t::event()
{
  m_cur_point.time = counter_get();
  mp_adc_exti->stop();
  if (m_index < m_cnv_cnt) {
    mp_spi->read(mp_spi_buf, read_buf_size);
    m_index++;
    m_status = st_spi_wait;
  } else {
    mp_spi_buf[0] = instruction_stop;
    mp_spi->write(mp_spi_buf, stop_buf_size);
    //mp_cs_pin->set();
    m_status = st_spi_wait_stop;
  }
}

void hrm::ad7799_cread_t::tick()
{
  mp_spi->tick();
  switch(m_status) {
    case st_free: {
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
        if (m_show) {
          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("���: ") << m_cnv_cnt << irsm(" �����");
          irs::mlog() << endl;
          irs::mlog() << irsm("Gain: ") << static_cast<int>(m_gain);
          irs::mlog() << endl;
          irs::mlog() << irsm("Filter: ") << static_cast<int>(m_filter);
          irs::mlog() << endl;
          irs::mlog() << irsm("Channel: ") << static_cast<int>(m_channel);
          irs::mlog() << endl;
        }
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
    case st_spi_wait: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_adc_exti->start();
        irs_i32 adc_raw_value = 0;
        irs_u8* p_adc_raw_value = reinterpret_cast<irs_u8*>(&adc_raw_value);
        p_adc_raw_value[0] = mp_spi_buf[2];
        p_adc_raw_value[1] = mp_spi_buf[1];
        p_adc_raw_value[2] = mp_spi_buf[0];
        m_cur_point.value = adc_raw_value;
        m_value_vector.push_back(m_cur_point);
        if (m_show) {
          irs::mlog() << irsm(".") << flush;
        }
        m_status = st_cread;
      }
    } break;
    case st_spi_wait_stop: {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        mp_spi->unlock();
        m_return_status = irs_st_ready;
        if (m_show) {
          irs::mlog() << endl;
        }
        m_value_sko.clear();
        m_value_sko.resize(m_cnv_cnt);
        m_value_sko.resize_average(m_cnv_cnt);
        if (m_cnv_cnt > 2) {
          m_time_sko.clear();
          m_time_sko.resize(m_cnv_cnt);
          m_time_sko.resize_average(m_cnv_cnt);
        }

        for (size_t i = 0; i < m_cnv_cnt; i++) {
          adc_value_t value = convert_value(m_value_vector[i].value);
          m_value_sko.add(value);
          if (m_show || m_show_simply) {
            irs::mlog() << value << endl << flush;
          }
          if (i > 0 && m_cnv_cnt > 2) {
            double delta =
              static_cast<double>(m_value_vector[i].time
                                  - m_value_vector[i-1].time);
            double time = delta
              / static_cast<double>(irs::cpu_traits_t::frequency());
            m_time_sko.add(time);
          }
        }

        m_avg = m_value_sko.average();
        m_sko = m_value_sko;

        if (m_show) {
          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("���     ") << m_value_sko << irsm(" �");
          irs::mlog() << endl;
          irs::mlog() << irsm("������� ") << m_value_sko.average() << irsm(" �");
          irs::mlog() << endl;
          adc_value_t rel = 1e6 * abs(m_value_sko / m_value_sko.average());
          irs::mlog() << irsm("���/��. ") << rel << irsm(" ppm") << endl;
          if (m_cnv_cnt > 2) {
            double jitter = m_time_sko * 1e6 / m_time_sko.average();
            double freq = 1.0 / m_time_sko.average();
            irs::mlog() << irsm("�����.  ") << jitter << irsm(" ppm");
            irs::mlog() << endl;
            irs::mlog() << irsm("F��     ") << freq << irsm(" ��");
            irs::mlog() << endl;
          }
          irs::mlog() << irsm("---------------------------------") << endl;
        }

        m_status = st_free;
      }
    } break;
  }
}
