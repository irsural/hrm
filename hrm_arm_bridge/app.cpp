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
  m_ti_dac(
    mp_cfg->spi(),
    mp_pins->p_dac_ti_cs,
    0.5f,
    irs::make_cnt_ms(50)),
  m_dac(&m_raw_dac, &m_ti_dac),
  m_raw_adc(
    mp_cfg->spi_2(),
    mp_pins->p_adc_cs),
  m_adc(&m_raw_adc, m_default_gain, m_default_channel, 
    m_default_mode, m_default_filter),
  m_eth_timer(irs::make_cnt_ms(100)),
  m_blink_timer(irs::make_cnt_ms(500)),
  m_service_timer(irs::make_cnt_ms(1000)),
  m_blink(false),
  m_relay_bridge_pos(
    mp_pins->p_relay_bridge_pos_off,
    mp_pins->p_relay_bridge_pos_on,
    irst("BPOS"),
    0,
    irs::make_cnt_ms(100)),
  m_relay_bridge_neg(
    mp_pins->p_relay_bridge_neg_off,
    mp_pins->p_relay_bridge_neg_on,
    irst("BNEG"),
    0,
    irs::make_cnt_ms(100)),
  m_relay_gain(
    mp_pins->p_relay_gain_low,
    mp_pins->p_relay_gain_high,
    irst("GAIN"),
    1,
    irs::make_cnt_ms(100)),
  m_relay_voltage(
    mp_pins->p_relay_voltage_low,
    mp_pins->p_relay_voltage_high,
    irst("VOLT"),
    1,
    irs::make_cnt_ms(100)),
  m_relay_prot(mp_pins->p_relay_prot, irst("PROT"), 1),
  m_mode(md_free),
  m_free_status(fs_prepare),
  m_balance_status(bs_prepare),
  m_current_iteration(0),
  m_iteration_count(19),
  m_elab_iteration_count(3),
  m_dac_code(0.),
  m_dac_step(0.),
  m_int_dac_code(0),
  m_initial_dac_code(0.),
  m_initial_dac_step(0.5),
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
  m_relay_after_pause(irs::make_cnt_ms(100)),
  m_dac_after_pause(irs::make_cnt_ms(100)),
  m_prepare_pause(5),
  m_elab_vector(),
  m_exp_cnt(1),
  m_exp_vector(),
  m_change_coils_polarity(false),
  m_current_polarity(ep_neg),
  m_inc_elab_voltage(false),
  m_no_prot(false),
  m_adc_average_cnt(0),
  m_adc_average_point(0),
  m_adc_average_value(0.),
  m_manual_status(ms_prepare),
  m_scan_status(ss_prepare),
  m_dac_center_scan(0.),
  m_int_dac_center_scan(0),
  m_current_adc_point(0),
  m_prepare_pause_timer(0),
  m_exp_time(0),
  m_prev_exp_time(0),
  m_is_exp(false),
  m_exp_timer(irs::make_cnt_ms(1000)),
  m_optimize_balance(false),
  m_relay_pause_timer(m_relay_after_pause)
{
  mxip_t ip = mxip_t::zero_ip();
  ip.val[0] = IP_0;
  ip.val[1] = IP_1;
  ip.val[2] = IP_2;
  ip.val[3] = IP_3;

  char ip_str[IP_STR_LEN];
  mxip_to_cstr(ip_str, ip);
  mp_cfg->hardflow()->set_param("local_addr", ip_str);
    
  m_relay_bridge_pos.set_after_pause(m_relay_after_pause);
  m_relay_bridge_neg.set_after_pause(m_relay_after_pause);
  m_relay_gain.set_after_pause(m_relay_after_pause);
  m_relay_voltage.set_after_pause(m_relay_after_pause);
  m_relay_prot.set_after_pause(m_relay_after_pause);
  
  m_eth_data.etalon = m_etalon;
  m_eth_data.checked = m_checked;
  m_eth_data.exp_cnt = m_exp_cnt;
  m_eth_data.elab_cnt = m_elab_iteration_count;
  m_eth_data.dac_pause_ms = 
    static_cast<irs_u32>(CNT_TO_DBLTIME(m_dac_after_pause) * 1000.0f);
  m_eth_data.relay_pause_ms = 
    static_cast<irs_u32>(CNT_TO_DBLTIME(m_relay_after_pause) * 1000.0f);
  m_eth_data.prepare_pause = m_prepare_pause;
  
  mp_pins->p_vben->set();
  
  irs::mlog() << setprecision(8);
}

void hrm::app_t::tick()
{ 
  mp_cfg->tick();
  m_mxnet_server.tick();
  m_raw_dac.tick();
  m_dac.tick();
  m_adc.tick();
  
  m_relay_bridge_pos.tick();
  m_relay_bridge_neg.tick();
  m_relay_gain.tick();
  m_relay_voltage.tick();
  m_relay_prot.tick();
  
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
      if (m_eth_data.relay_bridge_pos != m_relay_bridge_pos) {
        m_eth_data.relay_bridge_pos = m_relay_bridge_pos;
      }
      if (m_eth_data.relay_bridge_neg != m_relay_bridge_neg) {
        m_eth_data.relay_bridge_neg = m_relay_bridge_neg;
      }
      if (m_eth_data.relay_gain != m_relay_gain) {
        m_eth_data.relay_gain = m_relay_gain;
      }
      if (m_eth_data.relay_voltage != m_relay_voltage) {
        m_eth_data.relay_voltage = m_relay_voltage;
      }
      if (m_eth_data.relay_prot != m_relay_prot) {
        m_eth_data.relay_prot = m_relay_prot;
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
    if (m_eth_data.adc_temperature != m_adc.temperature()) {
      m_eth_data.adc_temperature = m_adc.temperature();
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
  if (m_exp_timer.check()) {
    if (m_is_exp) {
      m_exp_time++;
      m_eth_data.exp_time = m_exp_time;
    }
    if (m_mode == md_balance) {
      m_sum_time++;
      m_eth_data.sum_time = m_sum_time;
    }
  }
  switch (m_mode) {
    case md_free: {
      switch (m_free_status) {
        case fs_prepare: {
          //  reset to default
          m_relay_bridge_pos = 0;
          m_relay_bridge_neg = 0;
          m_relay_gain = 1;
          m_relay_voltage = 1;
          m_relay_prot = 1;
          //
          m_dac.on();
          m_dac.set_code(0);
          //
          m_adc.set_gain(m_min_adc_gain);
          //m_adc.set_gain(m_max_adc_gain);
          m_adc.set_channel(ac_1);
          m_adc.set_filter(af_4Hz);
          m_adc.hide();
          //
          m_relay_bridge_pos.set_wild(false);
          m_relay_bridge_neg.set_wild(false);
          //
          m_eth_data.apply = 0;
          m_eth_data.prepare_pause = m_prepare_pause;
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
              case md_scan: {
                m_mode = md_scan;
                m_free_status = fs_prepare;
                break;
              }
              default: {
                m_eth_data.mode = md_free;
              }
            }
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_adc.meas_voltage_and_temperature();
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
            if (m_eth_data.relay_bridge_pos != m_relay_bridge_pos) {
              m_relay_bridge_pos = m_eth_data.relay_bridge_pos;
            }
            if (m_eth_data.relay_bridge_neg != m_relay_bridge_neg) {
              m_relay_bridge_neg = m_eth_data.relay_bridge_neg;
            }
            if (m_eth_data.relay_gain != m_relay_gain) {
              m_relay_gain = m_eth_data.relay_gain;
            }
            if (m_eth_data.relay_voltage != m_relay_voltage) {
              m_relay_voltage = m_eth_data.relay_voltage;
            }
            if (m_eth_data.relay_prot != m_relay_prot) {
              m_relay_prot = m_eth_data.relay_prot;
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
    case md_scan: {
      switch (m_scan_status) {
        case ss_prepare: {
          irs::mlog() << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("------ Режим сканирования -------") << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
                    
          m_elab_iteration_count = m_eth_data.elab_cnt;
          if (m_elab_iteration_count < 3) {
            m_elab_iteration_count = 3;
            m_eth_data.elab_cnt = m_elab_iteration_count;
          }
          m_exp_cnt = m_eth_data.exp_cnt;
          m_dac_center_scan = m_eth_data.dac_center_scan;
          m_int_dac_center_scan = m_eth_data.int_dac_center_scan;
          
          m_prepare_pause = m_eth_data.prepare_pause;
          m_prepare_pause_timer.set(
            irs::make_cnt_s(static_cast<int>(m_prepare_pause)));
          
          m_scan_status = ss_on;
          break;
        }
        case ss_on: {
          if (bridge_relays_ready()) {
            m_relay_bridge_pos = 1;
            m_scan_status = ss_dac_prepare;
          }
          break;
        }
        case ss_dac_prepare: {
          if (bridge_relays_ready()) {
            m_current_adc_point = 0;
            m_current_iteration = 0;
            m_int_dac_code = m_int_dac_center_scan;
            m_int_dac_code -= (m_elab_iteration_count / 2);
            irs::mlog() << irsm("Число точек = ") << m_elab_iteration_count;
            irs::mlog() << endl;
            irs::mlog() << irsm("Начальный код = ") << m_int_dac_code << endl;
            m_scan_status = ss_first_dac_set;
          }
          break;
        }
        case ss_first_dac_set: {
          if (m_dac.ready()) {
            m_dac.set_int_code(m_int_dac_code);
            m_prepare_pause_timer.start();
            irs::mlog() << endl << irsm("Пауза ") << m_eth_data.prepare_pause;
            irs::mlog() << irsm(" c") << endl;
            m_scan_status = ss_wait;
          }
          break;
        }
        case ss_wait: {
          if (m_prepare_pause_timer.check()) {
            m_prepare_pause_timer.stop();
            m_adc.set_gain(m_max_adc_gain);
            m_adc.show();
            m_scan_status = ss_start_adc;
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_adc.meas_voltage_and_temperature();
            }
          }
          break;
        }
        case ss_dac_set: {
          if (m_dac.ready()) {
            m_dac.set_int_code(m_int_dac_code);
            m_scan_status = ss_start_adc;
          }
          break;
        }
        case ss_start_adc: {
          if (m_dac.ready() && m_adc.status() == irs_st_ready) {
            m_adc.meas_voltage();
            m_scan_status = ss_adc_wait;
          }
          break;
        }
        case ss_adc_wait: {
          if (m_adc.status() == irs_st_ready) {
            if ((m_current_adc_point + 1) < m_exp_cnt) {
              m_current_adc_point++;
              m_scan_status = ss_start_adc;
            } else {
              m_current_adc_point = 0;
              if ((m_current_iteration + 1) < m_elab_iteration_count) {
                m_current_iteration++;
                m_int_dac_code++;
                m_scan_status = ss_dac_set;
              } else {
                m_scan_status = ss_relay_off;
              }
            }
          }
          break;
        }
        case ss_relay_off: {
          if (bridge_relays_ready()) {
            m_relay_bridge_pos = 0;
            m_scan_status = ss_relay_wait;
          }
          break;
        }
        case ss_relay_wait: {
          if (bridge_relays_ready()) {
            m_mode = md_free;
            m_scan_status = ss_prepare;
          }
          break;
        }
      }
      if (m_eth_data.reset == 1) {
        irs::mlog() << irsm("---------------------------------") << endl;
        irs::mlog() << irsm("------------- Сброс -------------") << endl;
        m_eth_data.reset = 0;
        m_scan_status = ss_prepare;
        m_mode = md_free;
        m_eth_data.mode = md_free;
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
          //  bridge pos = 0
          //  bridge neg = 0
          //  gain = low
          //  voltage = low
          
//          m_adc.set_gain(m_max_adc_gain);
//          m_adc.show();
          
          m_etalon = m_eth_data.etalon;
          m_checked = m_eth_data.checked;
          if (m_eth_data.etpol == 0) {
            m_etalon_polarity = ep_neg;
          } else {
            m_etalon_polarity = ep_pos;
          }
          m_current_polarity = m_etalon_polarity;
          m_dac_after_pause = irs::make_cnt_ms(m_eth_data.dac_pause_ms);
          m_dac.set_after_pause(m_dac_after_pause);
          m_relay_after_pause = irs::make_cnt_ms(m_eth_data.relay_pause_ms);
          //m_relay_bridge_pos.set_after_pause(m_relay_after_pause);
          //m_relay_bridge_neg.set_after_pause(m_relay_after_pause);
          //m_relay_prot.set_after_pause(m_relay_after_pause);
          
          if (m_eth_data.wild_relays) {
            m_relay_bridge_pos.set_wild(true);
            m_relay_bridge_neg.set_wild(true);
          }
          
          m_relay_pause_timer.set(m_relay_after_pause);
          m_balancing_coil = bc_checked;
          m_elab_iteration_count = m_eth_data.elab_cnt;
          m_change_coils_polarity = m_eth_data.change_coils_polarity;
          m_inc_elab_voltage = m_eth_data.inc_elab_voltage;
          if (m_elab_iteration_count < 3) {
            m_elab_iteration_count = 3;
            m_eth_data.elab_cnt = m_elab_iteration_count;
          }
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
          
          m_no_prot = m_eth_data.no_prot;
          m_adc_average_cnt = m_eth_data.adc_average_cnt;
          
          m_prepare_pause = m_eth_data.prepare_pause;
          m_prepare_pause_timer.set(irs::make_cnt_ms(1000));
          
          m_exp_time = 0;
          m_sum_time = 0;
          m_optimize_balance = m_eth_data.optimize_balance;
          
          m_balance_status = bs_set_prot;
          break;
        case bs_set_prot:
          if (bridge_relays_ready()) {
            if (m_no_prot) {
              m_relay_prot = 0;
            }
            m_balance_status = bs_set_coils;
          }
          break;
        case bs_set_coils:
          if (bridge_relays_ready()) {
            m_is_exp = true;
            m_exp_timer.start();
            if (m_current_polarity == m_etalon_polarity) {
              switch (m_etalon_polarity) {
              case ep_neg:
                irs::mlog() << irsm("------------- (-) ---------------") <<endl;
                m_relay_bridge_neg = 1;
                break;
              case ep_pos:
                irs::mlog() << irsm("------------- (+) ---------------") <<endl;
                m_relay_bridge_pos = 1;
                break;
              }
            } else {
              switch (m_etalon_polarity) {
              case ep_neg:
                irs::mlog() << irsm("------------- (+) ---------------") <<endl;
                m_relay_bridge_pos = 1;
                break;
              case ep_pos:
                irs::mlog() << irsm("------------- (-) ---------------") <<endl;
                m_relay_bridge_neg = 1;
                break;
              }
            }
          }
          m_balance_status = bs_coils_wait;
          break;
        case bs_coils_wait:
          if (bridge_relays_ready()) {
            m_relay_pause_timer.start();
            float pause 
              = 0.001 * static_cast<float>(m_eth_data.relay_pause_ms);
            irs::mlog() << irsm("Пауза реле ") << pause << irsm(" c") << endl;
            m_balance_status = bs_coils_relay_pause;  
          }
          break;
        case bs_coils_relay_pause:
          if (m_relay_pause_timer.check()) {
            m_balance_status = bs_set_pause;  
          }
          break;
        case bs_set_pause:
          if (m_exp_vector.size() < 1 
            && m_current_polarity == m_etalon_polarity) {
            m_is_exp = false;
            m_prepare_pause_timer.start();
            m_prepare_current_time = m_prepare_pause;
            irs::mlog() << endl << irsm("Пауза ") << m_eth_data.prepare_pause;
            irs::mlog() << irsm(" c") << endl;
            m_balance_status = bs_pause;
          } else {
            m_adc.set_gain(m_max_adc_gain);
            m_adc.show();
            m_balance_status = bs_meas_temperature;
          }
          break;
        case bs_pause:
          if (m_prepare_pause_timer.check()) {
            if (m_prepare_current_time > 0) {
              m_prepare_pause_timer.start();
              m_prepare_current_time--;
              m_eth_data.prepare_pause = m_prepare_current_time;
            } else {
              m_eth_data.prepare_pause = m_prepare_pause;
              m_balance_status = bs_adc_show;  
            }
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_adc.meas_voltage_and_temperature();
            }
          }
          break;
        case bs_adc_show:
          if (m_adc.status() == irs_st_ready) {
            m_is_exp = true;
            m_exp_timer.start();
            m_adc.set_gain(m_max_adc_gain);
            m_adc.show();
            m_balance_status = bs_meas_temperature;
          }
          break;
        case bs_meas_temperature:
          if (m_adc.status() == irs_st_ready) {
            m_adc.meas_voltage_and_temperature();
            m_balance_status = bs_dac_prepare;
          }
          break;
        case bs_dac_prepare:
          if (m_dac.ready()) {
            m_dac_code = m_initial_dac_code;
            m_dac.set_code(m_initial_dac_code);
            m_dac_step = m_initial_dac_step;
            m_current_iteration = 0;
            m_iteration_count = 19;
            m_dac.on();
            m_adc_average_value = 0.0;
            m_adc_average_point = 0;
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
            m_balance_status = bs_adc_average;
          }
          break;
        case bs_adc_average:
          m_adc_average_value += m_adc.voltage();
          m_adc_average_point++;
          if ((m_adc_average_point < m_adc_average_cnt) 
              && (abs(m_adc.voltage()) < 0.001)) {
            m_balance_status = bs_adc_start;
          } else {
            m_adc_average_value 
              /= static_cast<adc_value_t>(m_adc_average_point);
            //m_adc_average_value = 0.0;
            //m_adc_average_point = 0;
            irs::mlog() << irsm("АЦП (") << m_adc_average_point;
            irs::mlog() << irsm(" раз) = ");
            if (abs(m_adc_average_value) < 1.1e-3) {
              irs::mlog() << (m_adc_average_value * 1.e6) << irsm(" мкВ");
            } else if (abs(m_adc_average_value < 1.1)) {
              irs::mlog() << (m_adc_average_value * 1.e3) << irsm(" мВ");
            } else {
              irs::mlog() << m_adc_average_value << irsm(" В");
            }
            irs::mlog() << endl;
            m_balance_status = bs_balance;
          }
          break;
        case bs_balance:
          if (m_current_iteration == 0 
              && m_optimize_balance && abs(m_adc_average_value) < 0.01) {
            // Корректировка
            double ratio = (pow(2.0, 20) / 12.0) * abs(m_adc_average_value);
            double log_ratio = floor(log2(ratio) - 1.0);
            irs_u32 int_log_ratio = static_cast<size_t>(log_ratio);
            m_current_iteration = m_iteration_count - int_log_ratio - 1;
            irs_i32 int_dac_code = static_cast<irs_u32>(pow(2.0,int_log_ratio));
            irs_i32 int_dac_step = int_dac_code;
            if (m_adc_average_value > 0.0) {
              int_dac_code = -int_dac_code;
            }
            m_dac_code = int_dac_code / pow(2.0, 19);
            m_dac_step = int_dac_step / pow(2.0, 19);
            irs::mlog() << irsm("Код ЦАПа = ") << int_dac_code << endl;
            irs::mlog() << irsm("Шаг ЦАПа = ") << int_dac_step << endl;
          }
          if (m_current_iteration < m_iteration_count) {
            m_current_iteration++;
            irs::mlog() << irsm("Итерация № ") << m_current_iteration << endl;
            irs::mlog() << irsm("Шаг ЦАПа = ") << m_dac_step << endl;
            if (m_adc_average_value < 0) {
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
            m_adc_average_value = 0.;
            m_adc_average_point = 0;
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
          if (elab_relays_ready()) {
            if (m_inc_elab_voltage) {
              m_relay_voltage = 1;
              m_relay_gain = 1;
            }
            if (!m_no_prot) {
              m_relay_prot = 0;
            }
            m_balance_status = bs_elab_relay_wait;
          }
          break;
        case bs_elab_relay_wait:
          if (elab_relays_ready()) {
            m_balance_status = bs_elab_dac_set;
          }
          break;
        case bs_elab_dac_set:
          if (m_dac.ready()) {
            m_dac.set_int_code(m_int_dac_code);
            
            m_adc_average_value = 0.;
            m_adc_average_point = 0;
            
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
            m_balance_status = bs_elab_adc_average;
          }
          break;
        case bs_elab_adc_average:
          m_adc_average_value += m_adc.voltage();
          m_adc_average_point++;
          if (m_adc_average_point < m_adc_average_cnt) {
            m_balance_status = bs_elab_adc_start;
          } else {
            m_adc_average_value 
              /= static_cast<adc_value_t>(m_adc_average_point);
            irs::mlog() << irsm("АЦП (") << m_adc_average_point;
            irs::mlog() << irsm(" раз) = ");
            if (abs(m_adc_average_value) < 1.1e-3) {
              irs::mlog() << (m_adc_average_value * 1.e6) << irsm(" мкВ");
            } else if (abs(m_adc_average_value < 1.1)) {
              irs::mlog() << (m_adc_average_value * 1.e3) << irsm(" мВ");
            } else {
              irs::mlog() << m_adc_average_value << irsm(" В");
            }
            irs::mlog() << endl;
            elab_point_t elab_point;
            elab_point.dac = m_dac.get_code();
            elab_point.adc = m_adc_average_value;
            m_elab_vector.push_back(elab_point);
            if ((m_current_iteration + 1) < m_elab_iteration_count) {
              m_current_iteration++;
              m_int_dac_code++;
              m_balance_status = bs_elab_dac_set;
            } else {
              if (!m_no_prot) {
                m_relay_prot = 1;
              }
              //m_adc.set_gain(0);
              m_balance_status = bs_coils_off;
            }
          }
          break;
        case bs_coils_off:
          if (m_relay_prot.status() == irs_st_ready) {
            if (m_inc_elab_voltage) {
              m_relay_gain = 0;
              m_relay_voltage = 0;
            }
            m_relay_bridge_pos = 0;
            m_relay_bridge_neg = 0;
            m_balance_status = bs_wait_relays;
          }
          break;
        case bs_wait_relays:
          if (elab_relays_ready() && bridge_relays_ready()) {
            if (m_current_polarity == m_etalon_polarity) {
              switch (m_current_polarity) {
              case ep_neg:
                m_current_polarity = ep_pos;
                break;
              case ep_pos:
                m_current_polarity = ep_neg;
                break;
              }
              m_balance_status = bs_set_coils;
            } else {
              m_balance_status = bs_report;
            }
          }
          break;
        case bs_report:
          irs::mlog() << irsm("---------------------------------") << endl;
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
          m_prev_exp_time = m_exp_time;
          m_eth_data.prev_exp_time = m_prev_exp_time;
          m_exp_time = 0;
          m_eth_data.exp_time = m_exp_time;
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
            m_balance_status = bs_set_coils;
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
          m_is_exp = false;
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
