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
  m_raw_dac(
    mp_cfg->spi(),
    mp_pins->p_dac_cs,
    mp_pins->p_dac_ldac,
    mp_pins->p_dac_clr,
    mp_pins->p_dac_reset,
    0),
  m_dac(&m_raw_dac),
  m_raw_adc(
    mp_cfg->spi(),
    mp_pins->p_adc_cs),
  m_adc(&m_raw_adc, m_default_gain, m_default_channel, 
    m_default_mode, m_default_filter),
  m_eth_timer(irs::make_cnt_ms(100)),
  m_blink_timer(irs::make_cnt_ms(500)),
  m_service_timer(irs::make_cnt_ms(1000)),
  m_blink(false),
  m_relay_1g(mp_pins->p_relay_1g, irst("1G"), 0),
  m_relay_100m(mp_pins->p_relay_100m, irst("100M"), 0),
  m_relay_10m(mp_pins->p_relay_10m, irst("10M"), 0),
  m_relay_1m(mp_pins->p_relay_1m, irst("1M"), 0),
  m_relay_100k(mp_pins->p_relay_100k, irst("100K"), 0),
  m_relay_chon(mp_pins->p_relay_chon, irst("CHON"), 0),
  m_relay_eton(mp_pins->p_relay_eton, irst("ETON"), 0),
  m_relay_prot(mp_pins->p_relay_prot, irst("PROT"), 0),
  m_relay_zero(
    mp_pins->p_relay_zero_off,
    mp_pins->p_relay_zero_on,
    irst("ZERO"),
    0,
    irs::make_cnt_ms(100)),
  m_relay_etpol(
    mp_pins->p_relay_etpol_off,
    mp_pins->p_relay_etpol_on,
    irst("ETPOL"),
    0,
    irs::make_cnt_ms(100)),
  m_mode(md_free),
  m_free_status(fs_prepare),
  m_balance_status(bs_prepare),
  m_current_iteration(0),
  m_iteration_count(19),
  m_elab_iteration_count(10),
  m_dac_code(0.),
  m_dac_step(0.),
  m_int_dac_code(0),
  m_initial_dac_code(0.),
  m_initial_dac_step(0.5),
  m_range(),
  m_etalon_polarity(ep_neg),
  m_balancing_coil(bc_checked),
  m_checked(1.),
  m_etalon(1.),
  //m_checked(100008.313),
  //m_etalon(99997.5335),
  //m_checked(10000.38),
  //m_etalon(10001.16),
  m_result(0.),
  m_result_error(0.),
  m_checked_code(0.),
  m_etalon_code(0.),
  m_relay_after_pause(irs::make_cnt_ms(500)),
  m_dac_after_pause(irs::make_cnt_ms(100)),
  m_elab_vector(),
  m_exp_cnt(30),
  m_exp_vector(),
  m_change_coils_polarity(false),
  m_current_polarity(ep_neg),
  m_no_zero_balance(false),
  m_no_zero_elab(false),
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
  
  m_relay_1g.set_after_pause(m_relay_after_pause);
  m_relay_100m.set_after_pause(m_relay_after_pause);
  m_relay_10m.set_after_pause(m_relay_after_pause);
  m_relay_1m.set_after_pause(m_relay_after_pause);
  m_relay_100k.set_after_pause(m_relay_after_pause);
  m_relay_chon.set_after_pause(m_relay_after_pause);
  m_relay_eton.set_after_pause(m_relay_after_pause);
  m_relay_prot.set_after_pause(m_relay_after_pause);
  m_relay_zero.set_after_pause(m_relay_after_pause);
  m_relay_etpol.set_after_pause(m_relay_after_pause);
  
  m_eth_data.etalon = m_etalon;
  m_eth_data.checked = m_checked;
  m_eth_data.exp_cnt = m_exp_cnt;
  m_eth_data.elab_cnt = m_elab_iteration_count;
  
  irs::mlog() << setprecision(8);
}

void hrm::app_t::tick()
{ 
  mp_cfg->tick();
  m_mxnet_server.tick();
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
    //  Обновление сетевых переменных от программы - пользователю
    if (m_mode != md_manual) {
      //  ЦАП
      if (m_eth_data.dac_code != m_dac.get_code()) {
        m_eth_data.dac_code = m_dac.get_code();
      }
      if (m_eth_data.int_dac_code != m_dac.get_int_code()) {
        m_eth_data.int_dac_code = m_dac.get_int_code();
      }
      if (m_eth_data.dac_on != m_dac.is_on()) {
        m_eth_data.dac_on = m_dac.is_on();
      }
      if (m_eth_data.dac_lin != m_dac.get_lin()) {
        m_eth_data.dac_lin = m_dac.get_lin();
      }
      //  АЦП
      if (m_eth_data.adc_filter != m_adc.filter()) {
        m_eth_data.adc_filter = m_adc.filter();
      }
      if (m_eth_data.adc_mode != m_adc.mode()) {
        m_eth_data.adc_mode = m_adc.mode();
      }
      if (m_eth_data.adc_channel != m_adc.channel()) {
        m_eth_data.adc_channel = m_adc.channel();
      }
      if (m_eth_data.adc_gain != m_adc.gain()) {
        m_eth_data.adc_gain = m_adc.gain();
      }
      //  Реле
      if (m_eth_data.relay_1g != m_relay_1g) {
        m_eth_data.relay_1g = m_relay_1g;
      }
      if (m_eth_data.relay_100m != m_relay_100m) {
        m_eth_data.relay_100m = m_relay_100m;
      }
      if (m_eth_data.relay_10m != m_relay_10m) {
        m_eth_data.relay_10m = m_relay_10m;
      }
      if (m_eth_data.relay_1m != m_relay_1m) {
        m_eth_data.relay_1m = m_relay_1m;
      }
      if (m_eth_data.relay_100k != m_relay_100k) {
        m_eth_data.relay_100k = m_relay_100k;
      }
      if (m_eth_data.relay_chon != m_relay_chon) {
        m_eth_data.relay_chon = m_relay_chon;
      }
      if (m_eth_data.relay_eton != m_relay_eton) {
        m_eth_data.relay_eton = m_relay_eton;
      }
      if (m_eth_data.relay_prot != m_relay_prot) {
        m_eth_data.relay_prot = m_relay_prot;
      }
      if (m_eth_data.relay_zero != m_relay_zero) {
        m_eth_data.relay_zero = m_relay_zero;
      }
      if (m_eth_data.relay_etpol != m_relay_etpol) {
        m_eth_data.relay_etpol = m_relay_etpol;
      }
    }
    if (m_eth_data.adc_value != m_adc.voltage()) {
      m_eth_data.adc_value = m_adc.voltage();
    }
    if (m_eth_data.adc_zero != m_adc.zero()) {
      m_eth_data.adc_zero = m_adc.zero();
    }
    if (m_eth_data.current_mode != m_mode) {
      m_eth_data.current_mode = m_mode;
    }
  }
  if (m_blink_timer.check()) {
    if (m_blink) {
      mp_pins->p_led_blink->set();
    } else {
      mp_pins->p_led_blink->clear();
    }
    m_blink = !m_blink;
  }
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
          m_relay_etpol = 0;
          //
          m_dac.on();
          m_dac.set_code(0);
          //
          //m_adc.set_gain(m_min_adc_gain);
          m_adc.set_gain(m_max_adc_gain);
          m_adc.set_channel(ac_1);
          m_adc.set_filter(af_4Hz);
          m_adc.hide();
          //
          m_eth_data.apply = 0;
          m_service_timer.start();
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
                break;
              }
              case md_balance: {
                m_mode = md_balance;
                m_free_status = fs_prepare;
                break;
              }
              default: {
                m_eth_data.mode = md_free;
              }
            }
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_adc.meas_voltage();
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
          m_manual_status = ms_check_user_changes;
          break;
        }
        case ms_check_user_changes: {
          if (m_eth_data.reset == 1) {
            m_eth_data.reset = 0;
            m_manual_status = ms_prepare;
            m_mode = md_free;
            m_eth_data.mode = md_free;
          } else {
            //  Реле
            if (m_eth_data.relay_1g != m_relay_1g) {
              m_relay_1g = m_eth_data.relay_1g;
            }
            if (m_eth_data.relay_100m != m_relay_100m) {
              m_relay_100m = m_eth_data.relay_100m;
            }
            if (m_eth_data.relay_10m != m_relay_10m) {
              m_relay_10m = m_eth_data.relay_10m;
            }
            if (m_eth_data.relay_1m != m_relay_1m) {
              m_relay_1m = m_eth_data.relay_1m;
            }
            if (m_eth_data.relay_100k != m_relay_100k) {
              m_relay_100k = m_eth_data.relay_100k;
            }
            if (m_eth_data.relay_chon != m_relay_chon) {
              m_relay_chon = m_eth_data.relay_chon;
            }
            if (m_eth_data.relay_eton != m_relay_eton) {
              m_relay_eton = m_eth_data.relay_eton;
            }
            if (m_eth_data.relay_prot != m_relay_prot) {
              m_relay_prot = m_eth_data.relay_prot;
            }
            if (m_eth_data.relay_zero != m_relay_zero) {
              m_relay_zero = m_eth_data.relay_zero;
            }
            if (m_eth_data.relay_etpol != m_relay_etpol) {
              m_relay_etpol = m_eth_data.relay_etpol;
            }
            //  ЦАП
            if (m_eth_data.dac_on != m_dac.is_on()) {
              if (m_eth_data.dac_on) {
                m_dac.on();
              } else {
                m_dac.off();
              }
            }
            if (m_eth_data.dac_code != m_dac.get_code()) {
              m_dac.set_code(m_eth_data.dac_code);
              m_eth_data.int_dac_code = m_dac.get_int_code();
              m_int_dac_code = m_dac.get_int_code();
              if (m_eth_data.dac_code != m_dac.get_code()) {
                m_eth_data.dac_code = m_dac.get_code();
              }
            }
            if (m_eth_data.int_dac_code != m_dac.get_int_code()) {
              m_int_dac_code = m_eth_data.int_dac_code;
              m_dac.set_int_code(m_eth_data.int_dac_code);
              m_eth_data.dac_code = m_dac.get_code();
              if (m_eth_data.int_dac_code != m_dac.get_int_code()) {
                m_eth_data.int_dac_code = m_dac.get_int_code();
              }
            }
            if (m_eth_data.dac_lin != m_dac.get_lin()) {
              m_dac.set_lin(m_eth_data.dac_lin);
            }
            //  АЦП
            if (m_eth_data.adc_filter != m_adc.filter()) {
              m_adc.set_filter(m_eth_data.adc_filter);
            }
            if (m_eth_data.adc_mode != m_adc.mode()) {
              m_adc.set_mode(m_eth_data.adc_mode);
            }
            if (m_eth_data.adc_channel != m_adc.channel()) {
              m_adc.set_channel(m_eth_data.adc_channel);
            }
            if (m_eth_data.adc_gain != m_adc.gain()) {
              m_adc.set_gain(m_eth_data.adc_gain);
            }
            if (m_adc.status() == irs_st_ready) {
              m_adc.meas_voltage();
            }
            break;
          }
        }
      }
      break;
    }
    case md_balance: {
      switch (m_balance_status) {
        case bs_prepare:
          irs::mlog() << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("----- Режим уравновешивания -----") << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
          
          //  on start:
          //  range off
          //  dac on
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
          //m_etalon_polarity = m_eth_data.etpol;
          if (m_eth_data.etpol == 0) {
            m_etalon_polarity = ep_neg;
          } else {
            m_etalon_polarity = ep_pos;
          }
          m_current_polarity = m_etalon_polarity;
          m_dac.set_after_pause(m_dac_after_pause);
          m_balancing_coil = bc_checked;
          m_elab_iteration_count = m_eth_data.elab_cnt;
          m_change_coils_polarity = m_eth_data.change_coils_polarity;
          if (m_elab_iteration_count < 3) {
            m_elab_iteration_count = 3;
            m_eth_data.elab_cnt = m_elab_iteration_count;
          }
          m_no_zero_balance = m_eth_data.no_zero_balance;
          m_no_zero_elab = m_eth_data.no_zero_elab;
          m_exp_vector.clear();
          m_elab_vector.clear();
          m_exp_cnt = m_eth_data.exp_cnt;
          if (m_exp_cnt > 1) {
            m_eth_data.current_exp_cnt = m_exp_cnt;
            irs::mlog() << irsm("---------------------------------") << endl;
            irs::mlog() << irsm("Эксперимент № 1 из ");
            irs::mlog() << static_cast<int>(m_exp_cnt) << endl;
            irs::mlog() << irsm("---------------------------------") << endl;
          } else {
            m_eth_data.current_exp_cnt = 1;
          }
          
          m_balance_status = bs_set_etpol;
          break;
        case bs_set_etpol:
          if (m_relay_etpol.status() == irs_st_ready) {
            m_relay_etpol = m_etalon_polarity;
            m_balance_status = bs_set_zero_relay;
          }
          break;
        case bs_set_zero_relay:
          if (m_relay_zero.status() == irs_st_ready) {
            if (!m_no_zero_balance) {
              m_relay_zero = 1;
            }
            m_balance_status = bs_start_meas_zero;
          }
          break;
        case bs_start_meas_zero:
          if (m_relay_zero.status() == irs_st_ready
            && m_adc.status() == irs_st_ready
          ) {
            m_adc.show();
            if (!m_no_zero_balance) {
              m_adc.meas_zero();
            }
            m_balance_status = bs_wait_meas_zero;
          }
          break;
        case bs_wait_meas_zero:
          if (m_adc.status() == irs_st_ready) {
            switch (m_etalon_polarity) {
              case ep_neg: m_balance_status = bs_set_neg_coils; break;
              case ep_pos: m_balance_status = bs_set_pos_coils; break;
            }
          }
          break;
        case bs_set_neg_coils:
          switch (m_balancing_coil) {
          case bc_checked:
            irs::mlog() << irsm("---------- Поверяемая -----------") << endl;
            m_relay_chon = 1;
            break;
          case bc_etalon:
            irs::mlog() << irsm("------------ Эталон -------------") << endl;
            m_relay_eton = 1;
            //m_relay_chon = 1;
            break;
          }
          m_eth_data.current_range = m_range.bound_range(m_eth_data.range);
          m_range.range_on(m_eth_data.current_range);
          m_balance_status = bs_coils_wait;
          break;
        case bs_set_pos_coils:
          if (m_change_coils_polarity) {
            if (m_current_polarity == m_etalon_polarity) {
              switch (m_etalon_polarity) {
              case ep_neg:
                irs::mlog() << irsm("------------- (-) ---------------") <<endl;
                m_relay_etpol = ep_neg;
                break;
              case ep_pos:
                irs::mlog() << irsm("------------- (+) ---------------") <<endl;
                m_relay_etpol = ep_pos;
                break;
              }
            } else {
              switch (m_etalon_polarity) {
              case ep_neg:
                irs::mlog() << irsm("------------- (+) ---------------") <<endl;
                m_relay_etpol = ep_pos;
                break;
              case ep_pos:
                irs::mlog() << irsm("------------- (-) ---------------") <<endl;
                m_relay_etpol = ep_neg;
                break;
              }
            }
          } else {
            m_relay_chon = 1;
            m_relay_eton = 1;
          }
          m_balance_status = bs_coils_wait;
          break;
        case bs_coils_wait:
          if (m_relay_chon.status() == irs_st_ready 
            && m_relay_eton.status() == irs_st_ready
            && m_range.ready()
            && m_relay_etpol.status() == irs_st_ready
          ) {
            if (!m_no_zero_balance) {
              m_relay_zero = 0;
            }
            m_balance_status = bs_dac_prepare;  
          }
          break;
        case bs_dac_prepare:
          if (m_dac.ready() && m_relay_zero.status() == irs_st_ready) {
            m_dac_code = m_initial_dac_code;
            m_dac.set_code(m_initial_dac_code);
            m_dac_step = m_initial_dac_step;
            m_current_iteration = 0;
            m_iteration_count = 19;
            m_dac.on();
            m_balance_status = bs_adc_start;
          }
          break;
        case bs_adc_start:
          if (m_adc.status() == irs_st_ready) {
            m_adc.meas_voltage();
            m_balance_status = bs_adc_wait;
          }
          break;
        case bs_adc_wait:
          if (m_adc.status() == irs_st_ready) {
            m_balance_status = bs_balance;
          }
          break;
        case bs_balance:
          if (m_current_iteration < m_iteration_count) {
            m_current_iteration++;
            irs::mlog() << irsm("Итерация № ") << m_current_iteration << endl;
            irs::mlog() << irsm("Шаг ЦАПа = ") << m_dac_step << endl;
            if (m_adc.voltage() < 0) {
              m_dac_code += m_dac_step;
            } else {
              m_dac_code -= m_dac_step;
            }
            m_dac_step /= 2.;
            m_balance_status = bs_dac_set;
          } else {
            m_balance_status = bs_elab_prepare;
          }
          break;
        case bs_dac_set:
          if (m_dac.ready()) {
            m_dac.set_code(m_dac_code);
            m_balance_status = bs_dac_wait;
          }
          break;
        case bs_dac_wait:
          if (m_dac.ready()) {
            m_balance_status = bs_adc_start;
          }
          break;
        case bs_elab_prepare:
          irs::mlog() << irsm("----------- Уточнение -----------") << endl;
          m_current_iteration = 0;
          m_int_dac_code = m_dac.get_int_code();
          m_int_dac_code -= (m_elab_iteration_count / 2);
          irs::mlog() << irsm("Число точек = ") << m_elab_iteration_count << endl;
          irs::mlog() << irsm("Начальный код = ") << m_int_dac_code << endl;
          m_balance_status = bs_elab_relay_on;
          break;
        case bs_elab_relay_on:
          if (m_relay_zero.status() == irs_st_ready) {
            if (!m_no_zero_elab) {
              m_relay_zero = 1;
            }
            m_balance_status = bs_elab_relay_wait;
          }
          break;
        case bs_elab_relay_wait:
          if (m_relay_zero.status() == irs_st_ready) {
            m_balance_status = bs_elab_meas_zero;
          }
          break;
        case bs_elab_meas_zero:
          if (m_adc.status() == irs_st_ready) {
            //m_adc.set_gain(7);
            if (!m_no_zero_elab) {
              m_adc.meas_zero();
            }
            m_balance_status = bs_elab_wait_zero;
          }
          break;
        case bs_elab_wait_zero:
          if (m_adc.status() == irs_st_ready) {
            if (!m_no_zero_elab) {
              m_relay_zero = 0;
            }
            m_relay_prot = 0;
            m_balance_status = bs_elab_dac_prepare;
          }
          break;
        case bs_elab_dac_prepare:
          if (m_relay_zero.status() == irs_st_ready
              && m_relay_prot.status() == irs_st_ready
          ) {
            m_balance_status = bs_elab_dac_set;
          }
          break;
        case bs_elab_dac_set:
          if (m_dac.ready()) {
            m_dac.set_int_code(m_int_dac_code);
            m_balance_status = bs_elab_adc_start;
          }
          break;
        case bs_elab_adc_start:
          if (m_dac.ready()) {
            m_adc.meas_voltage();
            m_balance_status = bs_elab_adc_wait;
          }
          break;
        case bs_elab_adc_wait:
          if (m_adc.status() == irs_st_ready) {
            elab_point_t elab_point;
            elab_point.dac = m_dac.get_code();
            elab_point.adc = m_adc.voltage();
            m_elab_vector.push_back(elab_point);
            if ((m_current_iteration + 1) < m_elab_iteration_count) {
              m_current_iteration++;
              m_int_dac_code++;
              m_balance_status = bs_elab_dac_set;
            } else {
              m_relay_prot = 1;
              //m_adc.set_gain(0);
              m_balance_status = bs_coils_off;
            }
          }
          break;
        case bs_coils_off:
          if (m_relay_prot.status() == irs_st_ready) {
            m_relay_chon = 0;
            m_relay_eton = 0;
            m_range.range_off();
            m_balance_status = bs_wait_relays;
          }
          break;
        case bs_wait_relays:
          if (m_relay_chon.status() == irs_st_ready
            && m_relay_eton.status() == irs_st_ready
            && m_range.ready()
          ) {
            switch (m_etalon_polarity) {
            case ep_neg:
              switch (m_balancing_coil) {
              case bc_checked:
                m_balancing_coil = bc_etalon;
                m_balance_status = bs_set_zero_relay;
                break;
              case bc_etalon:
                m_balance_status = bs_report;
                break;
              }
              break;
            case ep_pos:
              if (m_change_coils_polarity) {
                if (m_current_polarity == m_etalon_polarity) {
                  switch (m_current_polarity) {
                  case ep_neg:
                    m_current_polarity = ep_pos;
                    break;
                  case ep_pos:
                    m_current_polarity = ep_neg;
                    break;
                  }
                  m_balance_status = bs_set_zero_relay;
                } else {
                  m_balance_status = bs_report;
                }
              } else {
                m_balance_status = bs_report;
              }
              break;
            }
          }
          break;
        case bs_report:
          irs::mlog() << irsm("---------------------------------") << endl;
          if (m_etalon_polarity == ep_neg) {
            //
            m_checked_code = calc_elab_code(&m_elab_vector, bc_checked);
            m_etalon_code = calc_elab_code(&m_elab_vector, bc_etalon);
            if (m_etalon_code == 0.0 || m_checked == 0.0) {
              //  error
              m_eth_data.result = 0.;
              m_eth_data.result_error = 0.;
              irs::mlog() << irsm("Ошибка входных данных") << endl;
            } else {
              double dcp = 1. + m_checked_code;
              double dcm = 1. - m_checked_code;
              double dep = 1. + m_etalon_code;
              double dem = 1. - m_etalon_code;
              m_result = m_etalon * dcp * dem / (dcm * dep);
              m_result_error = ((m_result - m_checked) / m_checked) * 100.;
              m_eth_data.result = m_result;
              m_eth_data.result_error = m_result_error;
              irs::mlog() << irsm("Результат ") << m_result 
                << irsm(" Ом") << endl;
              irs::mlog() << irsm("Отклонение ") << m_result_error 
                << irsm(" %") << endl;
            }
          } else {
            if (m_change_coils_polarity) {
              m_checked_code = calc_elab_code(&m_elab_vector, bc_checked);
              m_etalon_code = calc_elab_code(&m_elab_vector, bc_etalon);
              //m_result = 1. - (m_checked_code - m_etalon_code) / 2.;
              //m_result = m_result * m_etalon;
              m_result = (2. - m_checked_code + m_etalon_code);
              m_result /= (2. + m_checked_code - m_etalon_code);
              m_result *= m_etalon;
              m_result_error = ((m_result - m_checked) / m_checked) * 100.;
              irs::mlog() << irsm("Результат ") << m_result 
                << irsm(" Ом") << endl;
              irs::mlog() << irsm("Отклонение ") << m_result_error 
                << irsm(" %") << endl;
            } else {
              m_checked_code = calc_elab_code(&m_elab_vector, 
                bc_checked, ep_pos);
              if (m_checked_code <= -0.5 || m_checked_code >= 0.5 
                || m_checked == 0.0
              ) {
                //  error
                m_eth_data.result = 0.;
                m_eth_data.result_error = 0.;
                irs::mlog() << irsm("Ошибка входных данных") << endl;
              } else {
                m_result = m_etalon / (1.0 / (m_checked_code + 0.5) - 1.0);
                m_result_error = ((m_result - m_checked) / m_checked) * 100.;
                irs::mlog() << irsm("Результат ") << m_result 
                  << irsm(" Ом") << endl;
                irs::mlog() << irsm("Отклонение ") << m_result_error 
                  << irsm(" %") << endl;
              }
            }
          }
          m_eth_data.result = m_result;
          m_eth_data.result_error = m_result_error;
          m_balance_status = bs_next_exp;
          break;
        case bs_next_exp:
          exp_t exp;
          exp.result = m_result;
          exp.error = m_result_error;
          exp.ch_code = m_checked_code;
          exp.et_code = m_etalon_code;
          m_exp_vector.push_back(exp);
          m_eth_data.current_exp_cnt--;
          if (m_exp_vector.size() >= m_exp_cnt) {
            m_balance_status = bs_final_report;
          } else {
            irs::mlog() << irsm("---------------------------------") << endl;
            irs::mlog() << irsm("Эксперимент № ");
            irs::mlog() << m_exp_vector.size() + 1;
            irs::mlog() << irsm(" из ") << static_cast<int>(m_exp_cnt) << endl;
            irs::mlog() << irsm("---------------------------------") << endl;
            m_elab_vector.clear();
            m_balancing_coil = bc_checked;
            m_current_polarity = m_etalon_polarity;
            m_balance_status = bs_set_zero_relay;
          }
          break;
        case bs_final_report:
          irs::mlog() << irsm("----------- Результат -----------") << endl;
          for (size_t i = 0; i < m_exp_cnt; i++) {
            irs::mlog() << setw(3) << i + 1;
            irs::mlog() << irsm(" ") << setw(12) << m_exp_vector[i].result;
            irs::mlog() << irsm(" ") << setw(12) << m_exp_vector[i].error;
            irs::mlog() << irsm(" %");
            irs::mlog() << endl;
          }
          if (!m_change_coils_polarity) {
            irs::mlog() << irsm("-------------- Данные (П/Э-1) -------------");
          } else {
            irs::mlog() << irsm("------------- Данные ([П+Э]/2) ------------");
          }
          irs::mlog() << endl;
          for (size_t i = 0; i < m_exp_cnt; i++) {
            double ppg = 0.;
            if (m_change_coils_polarity) {
              ppg = 1e8 * 
                (m_exp_vector[i].ch_code + m_exp_vector[i].et_code) * 0.5;
            } else {
              ppg = 1e8 *  
                (m_exp_vector[i].ch_code / m_exp_vector[i].et_code - 1.);
            }
            irs::mlog() << setw(3) << i + 1;
            irs::mlog() << irsm(" ");
            irs::mlog() << setw(12) << m_exp_vector[i].ch_code * pow(2., 19);
            irs::mlog() << irsm(" "); 
            irs::mlog() << setw(12) << m_exp_vector[i].et_code * pow(2., 19);
            irs::mlog() << irsm(" "); 
            irs::mlog() << setw(12) << ppg << irsm(" ppg");
            irs::mlog() << endl;
          }
          irs::mlog() << irsm("-------------------------------------------");
          irs::mlog() << endl;
          m_balance_status = bs_prepare;
          m_mode = md_free;
          break;
      }
      if (m_eth_data.reset == 1) {
        irs::mlog() << irsm("---------------------------------") << endl;
        irs::mlog() << irsm("------------- Сброс -------------") << endl;
        m_eth_data.reset = 0;
        m_balance_status = bs_prepare;
        m_mode = md_free;
        m_eth_data.mode = md_free;
      }
      break;
    }
  }
}

double hrm::app_t::calc_elab_code(vector<elab_point_t>* ap_elab_vector, 
  balancing_coil_t a_balancing_coil, etalon_polarity_t a_etpol)
{
  size_t shift = 0;
  size_t cnt = 0;
  if (a_etpol == ep_neg) {
    cnt = ap_elab_vector->size() / 2;
    if (a_balancing_coil == bc_etalon) {
      irs::mlog() << irsm("Эталон") << endl;
      shift = ap_elab_vector->size() / 2;
    } else {
      irs::mlog() << irsm("Поверяемая") << endl;
    }
  } else {
    cnt = ap_elab_vector->size();
  }
  
  for (size_t i = 0; i < cnt; i++) {
    irs_i32 int_dac_code = 
      static_cast<irs_i32>((*ap_elab_vector)[i + shift].dac * pow(2., 19));
    irs::mlog() << (i + 1) << irsm(": ")
      << int_dac_code << irsm(" : ")
      << ((*ap_elab_vector)[i + shift].adc * 1.0e6) << irsm(" мкВ") << endl;
  }
  size_t left = 0;
  for (size_t i = 0; i+1 < cnt; i++) {
    if ((*ap_elab_vector)[i+shift].adc 
        * (*ap_elab_vector)[i+shift+1].adc < 0.
    ) {
      left = i;
      break;
    }
  }
  
  double x1 = (*ap_elab_vector)[left + shift].adc;
  double y1 = (*ap_elab_vector)[left + shift].dac;
  double x2 = (*ap_elab_vector)[left + shift + 1].adc;
  double y2 = (*ap_elab_vector)[left + shift + 1].dac;
  double k = (y2 - y1) / (x2 - x1);
  double b = y2 - k * x2;
  double int_result = b * pow(2.,19);
  
  //  mnk
  double sum_x = 0.;
  double sum_x2 = 0.;
  double sum_y = 0.;
  double sum_xy = 0.;
  double n = static_cast<double>(cnt);
  for (size_t i = 0; i < cnt; i++) {
    sum_x += (*ap_elab_vector)[i+shift].adc;
    sum_x2 += (*ap_elab_vector)[i+shift].adc * (*ap_elab_vector)[i+shift].adc;
    sum_y += (*ap_elab_vector)[i+shift].dac;
    sum_xy += (*ap_elab_vector)[i+shift].adc * (*ap_elab_vector)[i+shift].dac;
  }
  double k_mnk = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
  double b_mnk = (sum_y - k_mnk * sum_x) / n;
  double int_result_mnk = b_mnk * pow(2.,19);
  
  irs::mlog() << irsm("Уточнённый код = ") << int_result << endl;
  irs::mlog() << irsm("Уточнённый код МНК = ") << int_result_mnk << endl;
  irs::mlog() << irsm("---------------------------------") << endl;
  return b_mnk;
}
