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
    0,
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
  m_balance_target_status(bs_prepare),
  m_current_iteration(0),
  m_iteration_count(19),
  m_dac_code(0.),
  m_dac_step(0.),
  m_int_dac_code(0),
  m_checked_dac_code(0),
  m_etalon_dac_code(0),
  m_initial_dac_code(0.),
  m_initial_dac_step(0.5),
  m_adc_value(0.),
  m_adc_value_ch2(0.),
  m_adc_value_ch3(0.),
  m_range(),
  m_etalon_polarity(ep_neg),
  m_balancing_coil(bc_checked),
  m_checked(10000.38),
  m_etalon(10001.16),
  m_result(0.),
  m_result_error(0.),
  m_adc_gain(m_min_adc_gain),
  m_adc_channel(ac_1),
  m_adc_mode(irs::adc_mode_single_conversion),
  m_adc_filter(af_4Hz),
  m_adc_offset(0),
  m_adc_fullscale(0),
  m_adc_gain_inc_limit(m_adc_midscale / 2),
  m_adc_gain_dec_limit(m_adc_midscale / 4),
  m_relay_after_pause(irs::make_cnt_ms(500)),
  m_dac_after_pause(irs::make_cnt_ms(1)),
  m_elab_vector(),
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
          m_relay_etpol = 1;
          //
          m_adc_mode = irs::adc_mode_single_conversion;
          m_adc_filter = af_4Hz;
          m_adc_gain = m_min_adc_gain;
          m_adc_channel = ac_1;
          m_dac.off();
          m_dac.set_code(0);
          //
          m_eth_data.apply = 0;
          //
          m_eth_data.relay_1g = m_relay_1g;
          m_eth_data.relay_100m = m_relay_100m;
          m_eth_data.relay_10m = m_relay_10m;
          m_eth_data.relay_1m = m_relay_1m;
          m_eth_data.relay_100k = m_relay_100k;
          m_eth_data.relay_chon = m_relay_chon;
          m_eth_data.relay_eton = m_relay_eton;
          m_eth_data.relay_prot = m_relay_prot;
          m_eth_data.relay_zero = m_relay_zero;
          m_eth_data.relay_etpol = m_relay_etpol;
          //
          m_eth_data.dac_on = m_dac.is_on();
          m_eth_data.dac_code = m_dac_code;
          m_eth_data.int_dac_code = m_int_dac_code;
          m_eth_data.adc_filter = m_adc_filter;
          m_eth_data.adc_mode = m_adc_mode;
          m_eth_data.adc_channel = m_adc_channel;
          m_eth_data.adc_gain = m_adc_gain;
          m_eth_data.adc_value = m_adc_value;
          //
          m_eth_data.current_mode = m_mode;
          //
          m_service_timer.start();
          //
          m_free_status = fs_adc_set_default_filter;
          break;
        }
        case fs_wait: {
          if (m_service_timer.check()) {
            m_free_status = fs_adc_set_default_filter;
          }
          break;
        }
        case fs_adc_set_default_filter: {
          if (m_adc.status() == meas_status_success) {
            m_adc.set_param(irs::adc_freq, m_adc_filter);
            m_free_status = fs_adc_set_default_channel;
          }
          break;
        }
        case fs_adc_set_default_channel: {
          if (m_adc.status() == meas_status_success) {
            m_adc.set_param(irs::adc_channel, m_adc_channel);
            m_free_status = fs_adc_set_default_gain;
          }
          break;
        }
        case fs_adc_set_default_gain: {
          if (m_adc.status() == meas_status_success) {
            m_adc.set_param(irs::adc_gain, m_adc_gain);
            m_free_status = fs_adc_set_default_mode;
          }
          break;
        }
        case fs_adc_set_default_mode: {
          if (m_adc.status() == meas_status_success) {
            m_adc.set_param(irs::adc_mode, m_adc_mode);
            m_free_status = fs_adc_wait;
          }
          break;
        }
        case fs_adc_wait: {
          if (m_adc.status() == meas_status_success) {
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
              m_eth_data.adc_value = convert_adc(m_adc.get_value());
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
          m_eth_data.relay_1g = m_relay_1g;
          m_eth_data.relay_100m = m_relay_100m;
          m_eth_data.relay_10m = m_relay_10m;
          m_eth_data.relay_1m = m_relay_1m;
          m_eth_data.relay_100k = m_relay_100k;
          m_eth_data.relay_chon = m_relay_chon;
          m_eth_data.relay_eton = m_relay_eton;
          m_eth_data.relay_prot = m_relay_prot;
          m_eth_data.relay_zero = m_relay_zero;
          m_eth_data.relay_etpol = m_relay_etpol;
          //
          m_eth_data.dac_on = m_dac.is_on();
          m_eth_data.dac_code = m_dac_code;
          m_eth_data.int_dac_code = m_int_dac_code;
          m_eth_data.adc_filter = m_adc_filter;
          m_eth_data.adc_mode = m_adc_mode;
          m_eth_data.adc_channel = m_adc_channel;
          m_eth_data.adc_gain = m_adc_gain;
          m_eth_data.adc_value = m_adc_value;
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
              //  Нефиг тут
              //m_relay_prot = 1;
              //m_eth_data.relay_prot = 1;
            }
            if (m_eth_data.relay_zero != m_relay_zero) {
              m_relay_zero = m_eth_data.relay_zero;
            }
            if (m_eth_data.relay_etpol != m_relay_etpol) {
              m_relay_etpol = m_eth_data.relay_etpol;
            }
            //
            if (m_eth_data.dac_on != m_dac.is_on()) {
              if (m_eth_data.dac_on) {
                m_dac.on();
              } else {
                m_dac.off();
              }
            }
            if (m_eth_data.dac_code != m_dac_code) {
              m_dac_code = m_eth_data.dac_code;
              m_int_dac_code = 
                static_cast<irs_i32>(m_dac_code * pow(2., 31)) >> 12;
              m_eth_data.int_dac_code = m_int_dac_code;
              m_dac.set_code(m_dac_code);
            }
            if (m_eth_data.int_dac_code != m_int_dac_code) {
              m_int_dac_code = m_eth_data.int_dac_code;
              m_dac_code = 
                static_cast<dac_value_t>(m_int_dac_code << 12) / pow(2., 31);
              m_eth_data.dac_code = m_dac_code;
              m_dac.set_code(m_dac_code);
            }
            //
            m_manual_target_status = ms_adc_start;
            //
            if (m_eth_data.adc_filter != m_adc_filter) {
              m_adc_filter = m_eth_data.adc_filter;
              m_manual_target_status = ms_adc_set_filter;
            }
            if (m_eth_data.adc_mode != m_adc_mode) {
              m_adc_mode = m_eth_data.adc_mode;
              m_manual_target_status = ms_adc_set_mode;
            }
            if (m_eth_data.adc_channel != m_adc_channel) {
              m_adc_channel = m_eth_data.adc_channel;
              m_manual_target_status = ms_adc_set_channel;
            }
            if (m_eth_data.adc_gain != m_adc_gain) {
              m_adc_gain = m_eth_data.adc_gain;
              m_manual_target_status = ms_adc_set_gain;
            }
            m_manual_status = m_manual_target_status;
            break;
          }
          case ms_adc_set_filter: {
            if (m_adc.status() == meas_status_success) {
              m_adc.set_param(irs::adc_freq, m_adc_filter);
              m_manual_status = ms_adc_start;
            }
            break;
          }
          case ms_adc_set_mode: {
            if (m_adc.status() == meas_status_success) {
              m_adc.set_param(irs::adc_mode, m_adc_mode);
              m_manual_status = ms_adc_start;
            }
            break;
          }
          case ms_adc_set_channel: {
            if (m_adc.status() == meas_status_success) {
              m_adc.set_param(irs::adc_channel, m_adc_channel);
              m_manual_status = ms_adc_start;
            }
            break;
          }
          case ms_adc_set_gain: {
            if (m_adc.status() == meas_status_success) {
              m_adc.set_param(irs::adc_gain, m_adc_gain);
              m_manual_status = ms_adc_start;
            }
            break;
          }
          case ms_adc_start: {
            if (m_adc.status() == meas_status_success) {
              m_adc.start();
              m_manual_status = ms_adc_get_value;
            }
            break;
          }
          case ms_adc_get_value: {
            if (m_adc.status() == meas_status_success) {
              m_adc_value = convert_adc(m_adc.get_value());
              m_eth_data.adc_value = m_adc_value;
              m_manual_status = ms_check_user_changes;
            }
            break;
          }
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
          m_adc_gain = m_min_adc_gain;
          m_adc_offset = 0;
          m_etalon = m_eth_data.etalon;
          m_checked = m_eth_data.checked;
          m_dac.set_after_pause(m_dac_after_pause);
          m_relay_prot = 1;
          m_balancing_coil = bc_checked;
          m_balance_status = bs_set_etpol;
          m_elab_vector.clear();
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
              irs::mlog() << irsm("---------- Поверяемая -----------") << endl;
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
            m_balance_status = bs_zero_dac_on;
            //m_balance_status = bs_zero_adc_set_channel;
            m_balance_target_status = bs_dac_prepare;
          }
          break;
        }
        case bs_zero_dac_on: {
          if (m_dac.ready()) {
            m_dac_code = 0;
            m_int_dac_code = 0;
            m_dac.on();
            m_dac.set_code(0);
            m_balance_status = bs_zero_set_adc_channel_2;
          }
          break;
        }
        case bs_zero_set_adc_channel_2: {
          if (m_adc.status() == meas_status_success) {
            m_adc_channel = 1;
            m_adc.set_param(irs::adc_channel, m_adc_channel);
            m_balance_status = bs_zero_start_adc_channel_2;
          }
          break;
        }
        case bs_zero_start_adc_channel_2: {
          if (m_adc.status() == meas_status_success) {
            m_adc.start();
            m_balance_status = bs_zero_wait_adc_channel_2;
          }
          break;
        }
        case bs_zero_wait_adc_channel_2: {
          if (m_adc.status() == meas_status_success) {
            m_adc_value_ch2 = convert_adc(m_adc.get_value());
            irs::mlog() << irsm("АЦП канал ") 
              << static_cast<irs_u32>(m_adc_channel) << irsm(" = ")
              << (m_adc_value_ch2 * 3.) << irsm(" В") << endl;
            m_balance_status = bs_zero_set_adc_channel_3;
          }
          break;
        }
        case bs_zero_set_adc_channel_3: {
          if (m_adc.status() == meas_status_success) {
            m_adc_channel = 2;
            m_adc.set_param(irs::adc_channel, m_adc_channel);
            m_balance_status = bs_zero_start_adc_channel_3;
          }
          break;
        }
        case bs_zero_start_adc_channel_3: {
          if (m_adc.status() == meas_status_success) {
            m_adc.start();
            m_balance_status = bs_zero_wait_adc_channel_3;
          }
          break;
        }
        case bs_zero_wait_adc_channel_3: {
          if (m_adc.status() == meas_status_success) {
            m_adc_value_ch3 = convert_adc(m_adc.get_value());
            irs::mlog() << irsm("АЦП канал ") 
              << static_cast<irs_u32>(m_adc_channel) << irsm(" = ")
              << (m_adc_value_ch3 * 3.) << irsm(" В") << endl;
            //
            m_adc_channel = 0;
            //
            m_balance_status = bs_zero_relay_on;
            m_balance_target_status = bs_dac_prepare;
          }
          break;
        }
        case bs_zero_relay_on: {
          if (m_relay_zero.status() == irs_st_ready) {
            m_relay_zero = 1;
            m_balance_status = bs_zero_relay_wait;
          }
          break;
        }
        case bs_zero_relay_wait: {
          if (m_relay_zero.status() == irs_st_ready) {
            m_balance_status = bs_zero_adc_set_channel;
          }
          break;
        }
        case bs_zero_adc_set_channel: {
          if (m_adc.status() == meas_status_success) {
            m_adc.set_param(irs::adc_channel, m_adc_channel);
            irs::mlog() << irsm("Канал АЦП = ") << (m_adc_channel + 1) << endl;
            m_balance_status = bs_zero_adc_set_gain;
          }
          break;
        }
        case bs_zero_adc_set_gain: {
          if (m_adc.status() == meas_status_success) {
            m_adc.set_param(irs::adc_gain, m_adc_gain);
            irs::mlog() << irsm("Усиление АЦП = ") << (1 << m_adc_gain) << endl;
            m_balance_status = bs_zero_adc_set_mode_offset;
          }
          break;
        }
        case bs_zero_adc_set_mode_offset: {
          if (m_adc.status() == meas_status_success) {
            m_adc_mode = irs::adc_mode_system_zero_scale;
            m_adc.set_param(irs::adc_mode, m_adc_mode);
            irs::mlog() << irsm("Режим АЦП = System Zero") << endl;
            m_balance_status = bs_zero_adc_start_offset;
          }
          break;
        }
        case bs_zero_adc_start_offset: {
          if (m_adc.status() == meas_status_success) {
            m_adc.start();
            m_balance_status = bs_zero_adc_read_offset;
          }
          break;
        }
        case bs_zero_adc_read_offset: {
          if (m_adc.status() == meas_status_success) {
            m_adc.get_param(irs::adc_offset, &m_adc_offset);
            m_balance_status = bs_zero_adc_wait_offset;
          }
          break;
        }
        case bs_zero_adc_wait_offset: {
          if (m_adc.status() == meas_status_success) {
            irs::mlog() << irsm("Смещение нуля = ") 
              << (convert_adc(m_adc_offset) * 1.0e6) << irsm(" мкВ") << endl;
            m_balance_status = bs_zero_adc_set_mode_fullscale;
          }
          break;
        }
        case bs_zero_adc_set_mode_fullscale: {
          if (m_adc.status() == meas_status_success) {
            m_adc_mode = irs::adc_mode_internal_full_scale;
            m_adc.set_param(irs::adc_mode, m_adc_mode);
            irs::mlog() << irsm("Режим АЦП = Int Fullscale") << endl;
            m_balance_status = bs_zero_adc_start_fullscale;
          }
          break;
        }
        case bs_zero_adc_start_fullscale: {
          if (m_adc.status() == meas_status_success) {
            m_adc.start();
            m_balance_status = bs_zero_adc_read_fullscale;
          }
          break;
        }
        case bs_zero_adc_read_fullscale: {
          if (m_adc.status() == meas_status_success) {
            m_adc.get_param(irs::adc_full_scale, &m_adc_fullscale);
            m_balance_status = bs_zero_adc_wait_fullscale;
          }
          break;
        }
        case bs_zero_adc_wait_fullscale: {
          if (m_adc.status() == meas_status_success) {
            irs::mlog() << irsm("Полная шкала = ") 
              << convert_adc(m_adc_fullscale) << irsm(" В") << endl;
            m_relay_zero = 0;
            m_balance_status = bs_zero_adc_set_mode_single;
          }
          break;
        }
        case bs_zero_adc_set_mode_single: {
          if (m_adc.status() == meas_status_success && 
              m_relay_zero.status() == irs_st_ready) {
//            m_adc_offset = m_adc.get_value();
//            irs::mlog() << irsm("Смещение АЦП = ") << m_adc_offset << endl;
            m_adc_mode = irs::adc_mode_single_conversion;
            m_adc.set_param(irs::adc_mode, m_adc_mode);
            m_balance_status = bs_zero_adc_wait;
          }
          break;
        }
        case bs_zero_adc_wait: {
          if (m_adc.status() == meas_status_success) {
            m_balance_status = m_balance_target_status;
          }
          break;
        }
        case bs_dac_prepare: {
          if (m_dac.ready()) {
            m_dac_code = m_initial_dac_code;
            m_dac_step = m_initial_dac_step;
            m_current_iteration = 0;
            m_iteration_count = 19;
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
            if (can_dec_gain()) {
              m_adc_gain--;
              m_balance_status = bs_zero_relay_on;
              m_balance_target_status = bs_adc_start;
              irs::mlog() << irsm("Усиление АЦП уменьшено") << endl;
            } else if (can_inc_gain()) {
              m_adc_gain++;
              m_balance_status = bs_zero_relay_on;
              m_balance_target_status = bs_adc_start;
              irs::mlog() << irsm("Усиление АЦП увеличено") << endl;
            } else {
              m_balance_status = bs_balance;
            }
          }
          break;
        }
        case bs_balance: {
          if (m_current_iteration < m_iteration_count) {
            m_current_iteration++;
            irs::mlog() << irsm("Итерация № ") << m_current_iteration << endl;
            irs::mlog() << irsm("Шаг ЦАПа = ") << m_dac_step << endl;
            if (m_adc_value < 0) {
              m_dac_code += m_dac_step;
            } else {
              m_dac_code -= m_dac_step;
            }
            m_dac_step /= 2.;
            m_balance_status = bs_dac_set;
          } else {
            m_balance_status = bs_elab_prepare;
//            if (m_etalon_polarity == ep_pos) {
//              m_balance_status = bs_elab_prepare;
//              m_checked_dac_code = static_cast<double>(m_dac_code);
//            } else {
//              if (m_balancing_coil == bc_checked) {
//                m_checked_dac_code = static_cast<double>(m_dac_code);
//                m_balancing_coil = bc_etalon;
//                //m_balance_status = bs_dac_prepare;
//                m_relay_chon = 0;
//                m_relay_eton = 1;
//                m_balance_status = bs_coil_change;
//              } else {
//                m_etalon_dac_code = static_cast<double>(m_dac_code);
//                m_balance_status = bs_elab_prepare;
//              }
//            }
          }
          break;
        }
        case bs_coil_change_prepare: {
          if (m_relay_prot.status() == irs_st_ready) {
            m_relay_chon = 0;
            m_relay_eton = 1;
            m_balance_status = bs_coil_change;
          }
          break;
        }
        case bs_coil_change: {
          if (m_relay_chon.status() == irs_st_ready 
            && m_relay_eton.status() == irs_st_ready) {
            //m_balance_status = bs_dac_prepare;
              m_balance_status = bs_zero_dac_on;
            //m_balance_status = bs_zero_adc_set_channel;
            m_balance_target_status = bs_dac_prepare;
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
        case bs_elab_prepare: {
          irs::mlog() << irsm("----------- Уточнение -----------") << endl;
          m_current_iteration = 0;
          m_iteration_count = 3;
          m_int_dac_code = m_dac.get_int_code();
          m_int_dac_code--;
          //m_elab_vector.clear();
          //m_balance_status = bs_elab_set_adc_channel_2;
          m_balance_status = bs_elab_relay_on;
          break;
        }
        case bs_elab_set_adc_channel_2: {
          if (m_adc.status() == meas_status_success) {
            m_adc_channel = 1;
            m_adc.set_param(irs::adc_channel, m_adc_channel);
            m_balance_status = bs_elab_start_adc_channel_2;
          }
          break;
        }
        case bs_elab_start_adc_channel_2: {
          if (m_adc.status() == meas_status_success) {
            m_adc.start();
            m_balance_status = bs_elab_wait_adc_channel_2;
          }
          break;
        }
        case bs_elab_wait_adc_channel_2: {
          if (m_adc.status() == meas_status_success) {
            m_adc_value_ch2 = convert_adc(m_adc.get_value());
            irs::mlog() << irsm("АЦП канал ") 
              << static_cast<irs_u32>(m_adc_channel) << irsm(" = ")
              << m_adc_value_ch2 << irsm(" В") << endl;
            m_balance_status = bs_elab_set_adc_channel_3;
          }
          break;
        }
        case bs_elab_set_adc_channel_3: {
          if (m_adc.status() == meas_status_success) {
            m_adc_channel = 2;
            m_adc.set_param(irs::adc_channel, m_adc_channel);
            m_balance_status = bs_elab_start_adc_channel_3;
          }
          break;
        }
        case bs_elab_start_adc_channel_3: {
          if (m_adc.status() == meas_status_success) {
            m_adc.start();
            m_balance_status = bs_elab_wait_adc_channel_3;
          }
          break;
        }
        case bs_elab_wait_adc_channel_3: {
          if (m_adc.status() == meas_status_success) {
            m_adc_value_ch3 = convert_adc(m_adc.get_value());
            irs::mlog() << irsm("АЦП канал ") 
              << static_cast<irs_u32>(m_adc_channel) << irsm(" = ")
              << m_adc_value_ch3 << irsm(" В") << endl;
            m_balance_status = bs_elab_set_adc_channel_1;
          }
          break;
        }
        case bs_elab_set_adc_channel_1: {
          if (m_adc.status() == meas_status_success) {
            m_adc_channel = 0;
            m_adc.set_param(irs::adc_channel, m_adc_channel);
            irs::mlog() << irsm("Канал АЦП = ") 
              << static_cast<irs_u32>(m_adc_channel + 1) << endl;
            m_balance_status = bs_elab_wait_adc_channel_1;
          }
          break;
        }
        case bs_elab_wait_adc_channel_1: {
          if (m_adc.status() == meas_status_success) {
            m_balance_status = bs_elab_relay_on;
          }
          break;
        }
        case bs_elab_relay_on: {
          if (m_relay_zero.status() == irs_st_ready) {
            m_relay_zero = 1;
            m_balance_status = bs_elab_relay_wait;
          }
          break;
        }
        case bs_elab_relay_wait: {
          if (m_relay_zero.status() == irs_st_ready) {
            m_balance_status = bs_elab_adc_set_mode_offset;
          }
          break;
        }
        case bs_elab_adc_set_mode_offset: {
          if (m_adc.status() == meas_status_success) {
            m_adc_mode = irs::adc_mode_system_zero_scale;
            m_adc.set_param(irs::adc_mode, m_adc_mode);
            irs::mlog() << irsm("Режим АЦП = System Zero") << endl;
            m_balance_status = bs_elab_adc_start_offset;
          }
          break;
        }
        case bs_elab_adc_start_offset: {
          if (m_adc.status() == meas_status_success) {
            m_adc.start();
            m_balance_status = bs_elab_adc_read_offset;
          }
          break;
        }
        case bs_elab_adc_read_offset: {
          if (m_adc.status() == meas_status_success) {
            m_adc.get_param(irs::adc_offset, &m_adc_offset);
            m_balance_status = bs_elab_adc_wait_offset;
          }
          break;
        }
        case bs_elab_adc_wait_offset: {
          if (m_adc.status() == meas_status_success) {
            irs::mlog() << irsm("Смещение нуля = ") 
              << (convert_adc(m_adc_offset) * 1.0e6) << irsm(" мкВ") << endl;
            m_relay_zero = 0;
            //m_relay_prot = 0;
            m_balance_status = bs_elab_adc_set_mode_single;
          }
          break;
        }
        case bs_elab_adc_set_mode_single: {
          if (m_adc.status() == meas_status_success && m_dac.ready()) {
            m_adc_mode = irs::adc_mode_single_conversion;
            m_adc.set_param(irs::adc_mode, m_adc_mode);
            m_balance_status = bs_elab_dac_prepare;
          }
          break;
        }
        case bs_elab_dac_prepare: {
          if (m_relay_zero.status() == irs_st_ready &&
              m_relay_prot.status() == irs_st_ready) {
            m_balance_status = bs_elab_dac_set;
          }
          break;
        }
        case bs_elab_dac_set: {
          m_dac.set_int_code(m_int_dac_code);
          m_dac_code = m_dac.get_code();
          m_balance_status = bs_elab_adc_start;
          break;
        }
        case bs_elab_adc_start: {
          if (m_dac.ready()) {
            m_adc.start();
            m_balance_status = bs_elab_adc_wait;
          }
          break;
        }
        case bs_elab_adc_wait: {
          if (m_adc.status() == meas_status_success) {
            m_adc_value = convert_adc(m_adc.get_value());
            elab_point_t elab_point;
            elab_point.dac = m_dac_code;
            elab_point.adc = m_adc_value;
            m_elab_vector.push_back(elab_point);
            irs::mlog() << irsm("АЦП = ") << m_adc_value << endl;
            if ((m_current_iteration + 1) < m_iteration_count) {
              m_current_iteration++;
              m_int_dac_code++;
              m_balance_status = bs_elab_dac_set;
            } else {
              if (m_etalon_polarity == ep_pos) {
                m_balance_status = bs_report;
              } else {
                if (m_balancing_coil == bc_checked) {
                  irs::mlog() << irsm("------------ Эталон -------------") 
                    << endl;
                  m_balancing_coil = bc_etalon;
                  m_relay_prot = 1;
                  m_balance_status = bs_coil_change_prepare;
                } else {
                  m_balance_status = bs_report;
                }
              }
            }
          }
          break;
        }
        case bs_report: {
          irs::mlog() << irsm("---------------------------------") << endl;
          for (size_t i = 0; i < m_elab_vector.size(); i++) {
            irs_i32 int_dac_code = 
              static_cast<irs_i32>(m_elab_vector[i].dac * pow(2., 19));
            irs::mlog() << (i + 1) << irsm(": ")
              << int_dac_code << irsm(" : ")
              << (m_elab_vector[i].adc * 1.0e6) << irsm(" мкВ") << endl;
          }
          if (m_etalon_polarity == ep_neg) {
            if (m_etalon_dac_code == 0.0 || m_checked == 0.0) {
              //  error
              m_eth_data.result = 0.;
              m_eth_data.result_error = 0.;
              irs::mlog() << irsm("Ошибка входных данных") << endl;
            } else {
              double dcp = 1. + m_checked_dac_code;
              double dcm = 1. - m_checked_dac_code;
              double dep = 1. + m_etalon_dac_code;
              double dem = 1. - m_etalon_dac_code;
              //m_result = m_etalon * m_checked_dac_code / m_etalon_dac_code;
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
            //double d = m_checked_dac_code;// / pow(2, 32);
            size_t left = 0;
            for (size_t i = 0; i+1 < m_elab_vector.size(); i++) {
              if (m_elab_vector[i].adc * m_elab_vector[i+1].adc < 0.) {
                left = i;
                break;
              }
            }
            double delta_dac = 0.5 * (m_adc_value_ch3 - m_adc_value_ch2) 
              / (m_adc_value_ch2 + m_adc_value_ch3);
            irs::mlog() << irsm("Канал 2 = ") << (m_adc_value_ch2 * 3.) 
              << irsm(" В") << endl;
            irs::mlog() << irsm("Канал 3 = ") << (m_adc_value_ch3 * 3.) 
              << irsm(" В") << endl;
            irs::mlog() << irsm("Нелинейность = ") << delta_dac * pow(2.,19)
                << endl;
            
            double x1 = m_elab_vector[left].adc;
            double y1 = m_elab_vector[left].dac;
            double x2 = m_elab_vector[left+1].adc;
            double y2 = m_elab_vector[left+1].dac;
            double k = (y2 - y1) / (x2 - x1);
            double b = y2 - k * x2;
            double d = b - delta_dac;
            irs::mlog() << irsm("Недокорректированный код = ") << b * pow(2.,19)
                << endl;
            irs::mlog() << irsm("Скорректированный код = ") << d * pow(2.,19)
                << endl;
            
            if (d <= -0.5 || d >= 0.5 || m_checked == 0.0) {
              //  error
              m_eth_data.result = 0.;
              m_eth_data.result_error = 0.;
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
          m_eth_data.result = m_result;
          m_eth_data.result_error = m_result_error;
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
