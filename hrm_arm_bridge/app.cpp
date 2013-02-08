#include <irspch.h>

#include "app.h"

#include <irslimits.h>

#include <irsfinal.h>

hrm::app_t::app_t(cfg_t* ap_cfg):
  mp_cfg(ap_cfg),
  m_eth_data(),
  m_mxnet_server(
    *mp_cfg->hardflow(), 
    m_eth_data.connect(&m_mxnet_server, 0) / sizeof(irs_u32)),
  mp_pins(mp_cfg->pins()),
//  m_adc_dac_request(
//    mp_cfg->spi(), 
//    mp_pins->p_adc_cs,
//    mp_pins->p_dac_cs,
//    mp_pins->p_dac_reset,
//    mp_pins->p_dac_ldac),
  m_raw_dac(
    mp_cfg->spi(),
    mp_pins->p_dac_cs,
    mp_pins->p_dac_ldac,
    mp_pins->p_dac_clr,
    mp_pins->p_dac_reset,
    0),
  m_dac(&m_raw_dac),
  m_adc(
    mp_cfg->spi(),
    mp_pins->p_adc_cs),
  m_eth_timer(irs::make_cnt_ms(100)),
  m_blink_timer(irs::make_cnt_ms(500)),
  m_service_timer(irs::make_cnt_ms(1000)),
  m_blink(false),
  m_dac_test(dt_pos),
  m_dac_timer(irs::make_cnt_ms(1)),
  m_relay_1g(mp_pins->p_relay_1g, irst("1G"), 0),
  m_relay_100m(mp_pins->p_relay_100m, irst("100M"), 0),
  m_relay_10m(mp_pins->p_relay_10m, irst("10M"), 0),
  m_relay_1m(mp_pins->p_relay_1m, irst("1M"), 0),
  m_relay_100k(mp_pins->p_relay_100k, irst("100K"), 0),
  m_relay_chon(mp_pins->p_relay_chon, irst("CHON"), 0),
  m_relay_eton(mp_pins->p_relay_eton, irst("ETON"), 0),
  m_relay_prot(mp_pins->p_relay_prot, irst("PROT"), 1),
  m_relay_zero(
    mp_pins->p_relay_zero_off,
    mp_pins->p_relay_zero_on,
    irst("ZERO"),
    1,
    irs::make_cnt_ms(100)),
  m_relay_etpol(
    mp_pins->p_relay_etpol_off,
    mp_pins->p_relay_etpol_on,
    irst("ETPOL"),
    1,
    irs::make_cnt_ms(100)),
  m_mode(md_free),
  m_free_status(fs_prepare),
  m_balance_status(bs_prepare),
  m_current_iteration(0),
  m_iteration_count(20),
  m_dac_code(0),
  m_dac_step(0),
  m_checked_dac_code(0),
  m_etalon_dac_code(0),
  m_initial_dac_code(0),
  m_initial_dac_step(1 << 31),
  m_adc_value(0),
  m_range(),
  m_etalon_polarity(ep_neg),
  m_balancing_coil(bc_checked),
  m_checked(0.),
  m_etalon(0.),
  m_result(0.),
  m_result_error(0.),
  m_manual_status(ms_prepare)
{
  mxip_t ip = mxip_t::zero_ip();
  ip.val[0] = IP_0;
  ip.val[1] = IP_1;
  ip.val[2] = IP_2;
  ip.val[3] = IP_3;

  char ip_str[IP_STR_LEN];
  mxip_to_cstr(ip_str, ip);
  mp_cfg->hardflow()->set_param("local_addr", ip_str);
  
  m_range.add_relay(&m_relay_100k);
  m_range.add_relay(&m_relay_1m);
  m_range.add_relay(&m_relay_10m);
  m_range.add_relay(&m_relay_100m);
  m_range.add_relay(&m_relay_1g);
}

void hrm::app_t::tick()
{ 
  mp_cfg->tick();
  m_mxnet_server.tick();
  //m_adc_dac_request.tick();
  m_raw_dac.tick();
  m_dac.tick();
  m_adc.tick();
  
  m_relay_1g.tick();
  m_relay_100m.tick();
  m_relay_10m.tick();
  m_relay_1m.tick();
  m_relay_100k.tick();
  m_relay_chon.tick();
  m_relay_eton.tick();
  m_relay_prot.tick();
  m_relay_zero.tick();
  m_relay_etpol.tick();
  
  if (m_eth_timer.check()) {
    m_eth_data.counter++;
  }
  if (m_blink_timer.check()) {
    if (m_blink) {
      mp_pins->p_led_blink->set();
    } else {
      mp_pins->p_led_blink->clear();
    }
    m_blink = !m_blink;
  }
//  if (m_dac_timer.check()) {
//    const irs_i32 d = 1000000;
//    const irs_i32 L = 2147483647;
//    irs_i32 code = m_dac_data.signed_voltage_code;
//    switch (m_dac_test) {
//      case dt_pos: {
//        if (code <= L - d) {
//          code += d;
//        } else {
//          m_dac_test = dt_neg;
//        }
//        break;
//      }
//      case dt_neg: {
//        if (code >= -L + d) {
//          code -= d;
//        } else {
//          m_dac_test = dt_pos;
//        }
//        break;
//      }
//    }
//    m_dac_data.signed_voltage_code = code;
//  }
  switch (m_mode) {
    case md_free: {
      switch (m_free_status) {
        case fs_prepare: {
          //  reset to default
          m_range.range_off();
          m_relay_chon = 0;
          m_relay_eton = 0;
          m_relay_prot = 1;
          m_relay_zero = 0;
          //m_adc.set_gain(ag_1);
          //m_adc.set_channel(ac_1);
          //m_adc.set_freq(af_4Hz);
          m_dac.off();
          m_dac.set_code(0);
          m_service_timer.start();
          //
          m_free_status = fs_wait;
          break;
        }
        case fs_wait: {
          if (m_service_timer.check()) {
            irs::mlog() << endl;
            irs::mlog() << irsm("---------------------------------") << endl;
            irs::mlog() << irsm("--------- Режим ожидания --------") << endl;
            irs::mlog() << irsm("---------------------------------") << endl;
            m_free_status = fs_idle;
          }
          break;
        }
        case fs_idle: {
          if (m_eth_data.apply == 1) {
            m_eth_data.apply = 0;
            switch (m_eth_data.mode) {
              case md_manual: {
                m_mode = md_manual;
                m_free_status = fs_prepare;
                m_eth_data.current_mode = md_manual;
                break;
              }
              case md_balance: {
                m_mode = md_balance;
                m_free_status = fs_prepare;
                m_eth_data.current_mode = md_balance;
                break;
              }
              default: {
                m_eth_data.mode = md_free;
                m_eth_data.current_mode = md_free;
              }
            }
          } else {
            if (m_adc.status() == meas_status_success) {
              m_eth_data.adc = convert_adc(m_adc.get_value());
              m_adc.start();
            }
          }
          break;
        }
      }
      break;
    }
    case md_manual: {
      switch (m_manual_status) {
        case ms_prepare: {
          irs::mlog() << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("--- Режим ручного управления ----") << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
          //
          m_manual_status = ms_prepare;
          m_mode = md_free;
          break;
        }
      }
      break;
    }
    case md_balance: {
      switch (m_balance_status) {
        case bs_prepare: {
          irs::mlog() << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("----- Режим уравновешивания -----") << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
          
          //  on start:
          //  range off
          //  dac off
          //  dac = 0
          //  adc mode = single conversion
          //  adc filter = 4Hz
          //  adc channel = ch 1
          //  adc gain = 1
          //  prot on
          //  etpol undefined
          //  eton = 0
          //  chon = 0
          //  zero off
          m_etalon = m_eth_data.etalon;
          m_checked = m_eth_data.checked;
          m_dac.set_after_pause(irs::make_cnt_ms(2000));
          m_relay_prot = 1;
          m_balancing_coil = bc_checked;
          m_balance_status = bs_set_etpol;
          break;
        }
        case bs_set_etpol: {
          if (m_relay_prot.status() == irs_st_ready) {
            m_etalon_polarity = m_eth_data.etpol;
            m_relay_etpol = m_etalon_polarity;
            m_balance_status = bs_set_etpol_wait;
          }
          break;
        }
        case bs_set_etpol_wait: {
          if (m_relay_etpol.status() == irs_st_ready) {
            if (m_etalon_polarity == ep_neg) {
              m_balance_status = bs_set_range;
            } else {
              m_balance_status = bs_coils_on;
            }
          }
          break;
        }
        case bs_set_range: {
          m_eth_data.current_range = m_range.bound_range(m_eth_data.range);
          m_range.range_on(m_eth_data.current_range);
          m_balance_status = bs_set_range_wait;
          break;
        }
        case bs_set_range_wait: {
          if (m_range.ready()) {
            m_balance_status = bs_coils_on;
          }
          break;
        }
        case bs_coils_on: {
          m_relay_chon = 1;
          if (m_etalon_polarity == ep_pos) {
            m_relay_eton = 1;
          }
          m_balance_status = bs_coils_on_wait;
          break;
        }
        case bs_coils_on_wait: {
          if (m_relay_chon.status() == irs_st_ready 
            && m_relay_eton.status() == irs_st_ready) {
            m_balance_status = bs_dac_prepare;
          }
          break;
        }
        case bs_zero_relay_on: {
          m_zero_relay = 1;
          m_balance_status = bs_zero_relay_wait;
          break;
        }
        case bs_zero_relay_wait: {
          if (m_zero_relay.status() == irs_st_ready) {
            m_balance_status = bs_zero_adc_start;
          }
          break;
        }
        case bs_zero_adc_start: {
          //
          break;
        }
        case bs_zero_adc_wait: {
          //
          break;
        }
        case bs_dac_prepare: {
          if (m_dac.ready()) {
            m_dac_code = m_initial_dac_code;
            m_dac_step = m_initial_dac_step;
            m_current_iteration = 0;
            m_dac.set_code(m_initial_dac_code);
            m_dac.on();
            m_balance_status = bs_adc_start;
          }
          break;
        }
        case bs_adc_start: {
          if (m_adc.status() == meas_status_success) {
            m_adc.start();
            m_balance_status = bs_adc_wait;
          }
          break;
        }
        case bs_adc_wait: {
          if (m_adc.status() == meas_status_success) {
            m_adc_value = convert_adc(m_adc.get_value());
            irs::mlog() << irsm("АЦП = ") << m_adc_value << endl;
            m_balance_status = bs_balance;
          }
          break;
        }
        case bs_balance: {
          if (m_current_iteration < m_iteration_count) {
            m_current_iteration++;
            m_dac_step /= 2;
            if (m_adc_value > 0) {
              m_dac_code += m_dac_step;
            } else {
              m_dac_code -= m_dac_step;
            }
            m_balance_status = bs_dac_set;
            irs::mlog() << irsm("Итерация № ") << m_current_iteration << endl;
            irs::mlog() << irsm("Шаг ЦАПа = ") << m_dac_step << endl;
          } else {
            if (m_etalon_polarity == ep_pos) {
              m_balance_status = bs_report;
              m_checked_dac_code = static_cast<double>(m_dac_code);
            } else {
              if (m_balancing_coil == bc_checked) {
                m_checked_dac_code = static_cast<double>(m_dac_code);
                m_balancing_coil = bc_etalon;
                m_balance_status = bs_dac_prepare;
              } else {
                m_etalon_dac_code = static_cast<double>(m_dac_code);
                m_balance_status = bs_report;
              }
            }
          }
          break;
        }
        case bs_dac_set: {
          if (m_dac.ready()) {
            m_dac.set_code(m_dac_code);
            m_balance_status = bs_dac_wait;
          }
          break;
        }
        case bs_dac_wait: {
          if (m_dac.ready()) {
            m_balance_status = bs_adc_start;
          }
          break;
        }
        case bs_report: {
          irs::mlog() << irsm("---------------------------------") << endl;
          if (m_etalon_polarity == ep_neg) {
            if (m_etalon_dac_code == 0.0 || m_checked == 0.0) {
              //  error
              irs::mlog() << irsm("Ошибка входных данных") << endl;
            } else {
              m_result = m_etalon * m_checked_dac_code / m_etalon_dac_code;
              m_result_error = ((m_result - m_checked) / m_checked) * 100.;
              m_eth_data.result = m_result;
              m_eth_data.result_error = m_result_error;
              irs::mlog() << irsm("Результат ") << m_result 
                << irsm(" Ом") << endl;
              irs::mlog() << irsm("Отклонение ") << m_result_error 
                << irsm(" %") << endl;
            }
          } else {
            double d = m_checked_dac_code / pow(2, 32);
            if (d <= -0.5 || d >= 0.5 || m_checked == 0.0) {
              //  error
              irs::mlog() << irsm("Ошибка входных данных") << endl;
            } else {
              m_result = m_etalon / (1.0 / (d + 0.5) - 1.0);
              m_result_error = ((m_result - m_checked) / m_checked) * 100.;
              irs::mlog() << irsm("Результат ") << m_result 
                << irsm(" Ом") << endl;
              irs::mlog() << irsm("Отклонение ") << m_result_error 
                << irsm(" %") << endl;
            }
          }
          irs::mlog() << irsm("---------------------------------") << endl;
          m_balance_status = bs_prepare;
          m_mode = md_free;
          break;
        }
      }
      break;
    }
  }
}
