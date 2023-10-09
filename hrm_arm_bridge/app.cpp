#include <irspch.h>

#include "app.h"

#include <irslimits.h>

#include <irsfinal.h>

hrm::app_t::app_t(cfg_t* ap_cfg, version_t a_version, bool* ap_buf_ready):
  mp_cfg(ap_cfg),
  m_eth_data(),
  mp_buf_ready(ap_buf_ready),
  m_version(a_version),
  m_buzzer(&mp_cfg->buzzer),
  m_lcd_drv(irslcd_4x20, mp_cfg->lcd_port, mp_cfg->lcd_rs_pin,
    mp_cfg->lcd_e_pin),
  m_keyboard_drv(),
  //m_encoder_drv(mp_cfg->enc_a, mp_cfg->enc_b, mp_cfg->encoder_timer_address),

  m_lcd_drv_service(),
  m_buzzer_kb_event(),
  m_hot_kb_event(),
  m_menu_kb_event(),
  m_keyboard_event_gen(),
  mp_menu(),
  m_escape_pressed_event(),
  m_mxnet_server(
    mp_cfg->connector_hardflow,
    m_eth_data.connect(&m_mxnet_server, 0)/sizeof(irs_u32)),
  m_show_network_params(true),
  m_eeprom(&mp_cfg->spi_aux, &mp_cfg->ee_cs, 4096, true, 0, 64,
    irs::make_cnt_s(1)),
  m_eeprom_data(&m_eeprom),
  m_init_eeprom(&m_eeprom, &m_eeprom_data),
  m_adc_ad4630(&mp_cfg->spi_dac,
    &mp_cfg->ad4630_rst,
    &mp_cfg->ad4630_cs,
    &mp_cfg->ad4630_busy,
    &mp_cfg->ad4630_pwr,
    &mp_cfg->ad4630_point_control,
    &mp_cfg->pulse_gen,
    &mp_cfg->adc_ready,
    &mp_cfg->spi_dma_reader),
  m_n_avg(0),
  m_t_adc(0),
  m_ef_smooth(0.0),
  m_n_adc(2000),
  m_current_div_relay(start_div_relay),
  m_bridge_voltage_dac(&mp_cfg->spi_aux, &mp_cfg->bridge_voltage_dac_cs,
    min_bridge_voltage, max_bridge_voltage, bridge_voltage_trans_coef),
  m_eth_timer(irs::make_cnt_ms(100)),
  m_blink_timer(irs::make_cnt_ms(500)),
  m_service_timer(irs::make_cnt_ms(1000)),
  m_blink(false),
  m_relay_bridge_pos(
    &mp_cfg->relay_bridge_pos_off,
    &mp_cfg->relay_bridge_pos_on,
    irst("BPOS"),
    0,
    irs::make_cnt_ms(100)),
  m_relay_bridge_neg(
    &mp_cfg->relay_bridge_neg_off,
    &mp_cfg->relay_bridge_neg_on,
    irst("BNEG"),
    0,
    irs::make_cnt_ms(100)),
  m_relay_prot(&mp_cfg->relay_prot, irst("PROT"), 1),
  //  Relays AD4630
  m_relay_divp(
    &mp_cfg->relay_bridge_divp_off,
    &mp_cfg->relay_bridge_divp_on,
    irst("DIVP"),
    0,
    irs::make_cnt_ms(100)),
  m_relay_divn(
    &mp_cfg->relay_bridge_divn_off,
    &mp_cfg->relay_bridge_divn_on,
    irst("DIVN"),
    0,
    irs::make_cnt_ms(100)),
  m_relay_divsw(
    &mp_cfg->relay_divsw_off,
    &mp_cfg->relay_divsw_on,
    irst("DIVSW"),
    0,
    irs::make_cnt_ms(100)),
  m_mode(md_free),
  m_free_status(fs_prepare),
  m_balance_status(bs_prepare),
  m_current_iteration(0),
  m_iteration_count(20),
  m_elab_mode(em_fast_2points),
  m_balance_polarity(bp_neg),
  m_checked(1.),
  m_etalon(1.),
  m_result(0.),
  m_result_error(0.),
  m_checked_code(0.),
  m_checked_balanced_code(0.0),
  m_etalon_code(0.),
  m_etalon_balanced_code(0.0),
  m_relay_after_pause(irs::make_cnt_ms(100)),
  m_prepare_pause(5),
  m_prepare_current_time(0),
  m_analog_vector(),
  m_exp_cnt(1),
  m_exp_vector(),
  m_no_prot(false),
  m_wild_relays(false),
  m_balanced_sko(0.),
  m_manual_status(ms_prepare),
  m_current_adc_point(0),
  m_prepare_pause_timer(0),
  m_exp_time(0),
  m_prev_exp_time(0),
  m_sum_time(0),
  m_remaining_time(),
  m_remaining_time_calculator(),
  m_is_exp(false),
  m_exp_timer(irs::make_cnt_ms(1000)),
  m_prepare_pause_completed(false),
  m_relay_pause_timer(m_relay_after_pause),
  m_min_after_pause(irs::make_cnt_ms(100)),
  m_min_bridge_voltage(12.0),
  m_max_bridge_voltage(200.0),
  m_min_bridge_voltage_speed(1.0),
  m_max_bridge_voltage_speed(200.0),
  m_min_bridge_voltage_after_pause_ms(0.0),
  m_max_bridge_voltage_after_pause_ms(10000.0),
  m_bridge_voltage(m_min_bridge_voltage),
  m_bridge_voltage_reduced(m_min_bridge_voltage),
  m_bridge_voltage_speed(m_min_bridge_voltage_speed),
  m_bridge_voltage_after_pause_ms(m_max_bridge_voltage_after_pause_ms),
  m_bridge_voltage_reduce_after_switching(true),
  m_device_condition_controller(
    &mp_cfg->fan_ac_on,
    &mp_cfg->fan_dc_ls,
    &mp_cfg->fan_dc_hs,
    &mp_cfg->fan_dc_sen,
    &m_eth_data.th_dac,
    &m_eth_data.th_box_ldo,
    &m_eth_data.th_box_adc,
    &m_eth_data.th_mcu,
    &m_eth_data.th_ext_1,
    &m_eth_data.th_ext_2,
    &m_eth_data.volt_box_neg,
    &m_eth_data.volt_box_pos,
    &m_eth_data.fan_mode,
    &m_eeprom_data.fan_mode,
    &m_eth_data.fan_status,
    &m_eth_data.fan_ac_speed,
    &m_eeprom_data.fan_ac_speed,
    &m_eth_data.fan_dc_speed,
    &m_eeprom_data.fan_dc_speed,
    &m_eth_data.fan_dc_speed_sense
  ),
  //  treg adc
  m_treg_operating_duty_time_interval_s(1.0),
  m_treg_operating_duty_deviation(0.05),
  m_treg_pwm_max_code_float(0.26),
  m_treg_polarity_map(peltier_t::default_polarity_map),
  m_treg_temperature_setpoint(25.0),
  m_treg_termosensor(&m_eth_data.th_box_adc, 15.0, 35.0),
  m_treg_peltier_parameters(
    &m_treg_termosensor,
    &mp_cfg->peltier_pwm1_gen,
    mp_cfg->peltier_pwm1_channel,
    &mp_cfg->peltier_pol1,
    m_treg_operating_duty_time_interval_s,
    m_treg_operating_duty_deviation,
    m_treg_pwm_max_code_float,
    peltier_t::default_polarity_map,
    &m_eth_data.treg_ref,
    &m_eth_data.treg_result,
    &m_eth_data.treg_k,
    &m_eth_data.treg_ki,
    &m_eth_data.treg_kd,
    &m_eth_data.treg_iso_k,
    &m_eth_data.treg_iso_t,
    &m_eth_data.treg_pwm_rate_slope,
    &m_eth_data.treg_pid_out,
    &m_eth_data.treg_amplitude_code_float,
    &m_eth_data.treg_enabled,
    &m_eth_data.treg_pid_reg_enabled,
    &m_eth_data.treg_polarity_pin_bit_data),
  m_treg_peltier(m_treg_peltier_parameters),
  m_treg_sync_parameters(
    &m_eth_data.treg_ref,
    &m_eth_data.treg_k,
    &m_eth_data.treg_ki,
    &m_eth_data.treg_kd,
    &m_eth_data.treg_iso_k,
    &m_eth_data.treg_iso_t,
    &m_eth_data.treg_pwm_rate_slope,
    &m_eth_data.treg_enabled,
    &m_eth_data.treg_pid_reg_enabled,
    &m_eth_data.treg_polarity_pin_bit_data,
    &m_eeprom_data.treg_ref,
    &m_eeprom_data.treg_k,
    &m_eeprom_data.treg_ki,
    &m_eeprom_data.treg_kd,
    &m_eeprom_data.treg_iso_k,
    &m_eeprom_data.treg_iso_t,
    &m_eeprom_data.treg_pwm_rate_slope,
    &m_eeprom_data.treg_enabled,
    &m_eeprom_data.treg_pid_reg_enabled,
    &m_eeprom_data.treg_polarity_pin_bit_data
  ),
  // treg dac
  m_treg_dac_operating_duty_time_interval_s(1.0),
  m_treg_dac_operating_duty_deviation(0.05),
  m_treg_dac_pwm_max_code_float(0.26),
  m_treg_dac_polarity_map(peltier_t::default_polarity_map),
  m_treg_dac_temperature_setpoint(25.0),
  m_treg_dac_termosensor(&m_eth_data./*th_dac*/th_box_ldo, 15.0, 45.0),
  m_treg_dac_peltier_parameters(
    &m_treg_dac_termosensor,
    &mp_cfg->peltier_pwm2_gen,
    mp_cfg->peltier_pwm2_channel,
    &mp_cfg->peltier_pol2,
    m_treg_dac_operating_duty_time_interval_s,
    m_treg_dac_operating_duty_deviation,
    m_treg_dac_pwm_max_code_float,
    peltier_t::default_polarity_map,
    &m_eth_data.treg_dac_ref,
    &m_eth_data.treg_dac_result,
    &m_eth_data.treg_dac_k,
    &m_eth_data.treg_dac_ki,
    &m_eth_data.treg_dac_kd,
    &m_eth_data.treg_dac_iso_k,
    &m_eth_data.treg_dac_iso_t,
    &m_eth_data.treg_dac_pwm_rate_slope,
    &m_eth_data.treg_dac_pid_out,
    &m_eth_data.treg_dac_amplitude_code_float,
    &m_eth_data.treg_dac_enabled,
    &m_eth_data.treg_dac_pid_reg_enabled,
    &m_eth_data.treg_dac_polarity_pin_bit_data),
  m_treg_dac_peltier(m_treg_dac_peltier_parameters),
  m_treg_dac_sync_parameters(
    &m_eth_data.treg_dac_ref,
    &m_eth_data.treg_dac_k,
    &m_eth_data.treg_dac_ki,
    &m_eth_data.treg_dac_kd,
    &m_eth_data.treg_dac_iso_k,
    &m_eth_data.treg_dac_iso_t,
    &m_eth_data.treg_dac_pwm_rate_slope,
    &m_eth_data.treg_dac_enabled,
    &m_eth_data.treg_dac_pid_reg_enabled,
    &m_eth_data.treg_dac_polarity_pin_bit_data,
    &m_eeprom_data.treg_dac_ref,
    &m_eeprom_data.treg_dac_k,
    &m_eeprom_data.treg_dac_ki,
    &m_eeprom_data.treg_dac_kd,
    &m_eeprom_data.treg_dac_iso_k,
    &m_eeprom_data.treg_dac_iso_t,
    &m_eeprom_data.treg_dac_pwm_rate_slope,
    &m_eeprom_data.treg_dac_enabled,
    &m_eeprom_data.treg_dac_pid_reg_enabled,
    &m_eeprom_data.treg_dac_polarity_pin_bit_data
  ),
  m_balance_action(ba_idle)
{
  init_keyboard_drv();
  init_encoder_drv();

  mxip_t ip = mxip_t::zero_ip();
  ip.val[0] = m_eeprom_data.ip_0;
  ip.val[1] = m_eeprom_data.ip_1;
  ip.val[2] = m_eeprom_data.ip_2;
  ip.val[3] = m_eeprom_data.ip_3;
  
  mxip_t mask = mxip_t::zero_ip();
  mask.val[0] = m_eeprom_data.mask_0;
  mask.val[1] = m_eeprom_data.mask_1;
  mask.val[2] = m_eeprom_data.mask_2;
  mask.val[3] = m_eeprom_data.mask_3;

  mxip_t gateway = mxip_t::zero_ip();
  gateway.val[0] = m_eeprom_data.gateway_0;
  gateway.val[1] = m_eeprom_data.gateway_1;
  gateway.val[2] = m_eeprom_data.gateway_2;
  gateway.val[3] = m_eeprom_data.gateway_3;

  const bool dhcp_enabled = m_eeprom_data.dhcp_on;

  mp_cfg->network_config.set(ip, mask, gateway, dhcp_enabled);

  irs::mlog() << setprecision(8);

  m_exp_cnt = m_eeprom_data.exp_cnt;
  m_eth_data.exp_cnt = m_exp_cnt;

  m_etalon = m_eeprom_data.etalon;
  m_eth_data.etalon = m_etalon;

  m_checked = m_eeprom_data.checked;
  m_eth_data.checked = m_checked;

  m_eth_data.ratio = m_eeprom_data.ratio;

  m_eth_data.checked_prev = m_eeprom_data.checked_prev;

  m_relay_after_pause = irs::make_cnt_ms(m_eeprom_data.relay_pause_ms);
  m_eth_data.relay_pause_ms = m_eeprom_data.relay_pause_ms;

  m_prepare_pause = m_eeprom_data.prepare_pause;
  m_eth_data.prepare_pause = m_prepare_pause;

  //m_no_prot = false;//
  m_no_prot = m_eeprom_data.no_prot;
  m_eth_data.no_prot = m_no_prot;

  m_wild_relays = m_eeprom_data.wild_relays;
  m_eth_data.wild_relays = m_wild_relays;
  m_relay_bridge_pos.set_wild(m_wild_relays);
  m_relay_bridge_neg.set_wild(m_wild_relays);
  m_relay_divp.set_wild(m_wild_relays);
  m_relay_divn.set_wild(m_wild_relays);
  m_relay_divsw.set_wild(m_wild_relays);
  
  m_eth_data.bridge_voltage = m_eeprom_data.bridge_voltage;
  m_bridge_voltage = m_eeprom_data.bridge_voltage;
  m_eth_data.bridge_voltage_reduced = m_eeprom_data.bridge_voltage_reduced;
  m_bridge_voltage_reduced = m_eeprom_data.bridge_voltage_reduced;
  m_eth_data.bridge_voltage_speed = m_eeprom_data.bridge_voltage_speed;
  m_bridge_voltage_speed = m_eeprom_data.bridge_voltage_speed;
  m_eth_data.bridge_voltage_after_pause_ms 
    = m_eeprom_data.bridge_voltage_after_pause_ms;
  m_bridge_voltage_after_pause_ms = m_eeprom_data.bridge_voltage_after_pause_ms;
  m_bridge_voltage_dac.set_voltage(m_bridge_voltage);
  m_bridge_voltage_reduce_after_switching 
    = m_eeprom_data.bridge_voltage_reduce_after_switching;
  m_eth_data.bridge_voltage_reduce_after_switching
    = m_eeprom_data.bridge_voltage_reduce_after_switching;
  
  m_relay_after_pause = irs::make_cnt_ms(m_eeprom_data.relay_pause_ms);

  m_eth_data.ip_0 = m_eeprom_data.ip_0;
  m_eth_data.ip_1 = m_eeprom_data.ip_1;
  m_eth_data.ip_2 = m_eeprom_data.ip_2;
  m_eth_data.ip_3 = m_eeprom_data.ip_3;

  m_eth_data.mask_0 = m_eeprom_data.mask_0;
  m_eth_data.mask_1 = m_eeprom_data.mask_1;
  m_eth_data.mask_2 = m_eeprom_data.mask_2;
  m_eth_data.mask_3 = m_eeprom_data.mask_3;

  m_eth_data.gateway_0 = m_eeprom_data.gateway_0;
  m_eth_data.gateway_1 = m_eeprom_data.gateway_1;
  m_eth_data.gateway_2 = m_eeprom_data.gateway_2;
  m_eth_data.gateway_3 = m_eeprom_data.gateway_3;

  m_eth_data.dhcp_on = m_eeprom_data.dhcp_on;

  m_relay_bridge_pos.set_after_pause(m_min_after_pause);
  m_relay_bridge_neg.set_after_pause(m_min_after_pause);
  m_relay_prot.set_after_pause(m_min_after_pause);
  m_relay_divp.set_after_pause(m_min_after_pause);
  m_relay_divn.set_after_pause(m_min_after_pause);
  m_relay_divsw.set_after_pause(m_min_after_pause);

  //  ЖКИ и клавиатура
  m_lcd_drv_service.connect(&m_lcd_drv);
  m_keyboard_event_gen.connect(&m_keyboard_drv);
  //m_keyboard_event_gen.connect_encoder(&m_encoder_drv);
  m_keyboard_event_gen.add_event(&m_menu_kb_event);
  m_keyboard_event_gen.add_event(&m_buzzer_kb_event);
  m_keyboard_event_gen.add_event(&m_hot_kb_event);

  m_keyboard_event_gen.set_antibounce_time(irs::make_cnt_ms(20));
  m_keyboard_event_gen.set_antibounce_encoder_time(irs::make_cnt_ms(20));
  m_keyboard_event_gen.set_defence_time(irs::make_cnt_ms(500));
  m_keyboard_event_gen.set_rep_time(irs::make_cnt_ms(50));

  mp_menu.reset(new menu_t(&m_lcd_drv_service,
    &m_menu_kb_event, &m_eth_data));
  m_buzzer.bzz();
  //
  m_eth_data.version_info = m_version.software;
  //
  m_eth_data.show_options = m_eeprom_data.show_options;
  //
  m_bridge_voltage_dac.show();
  //
  m_n_avg = m_eeprom_data.n_avg;
  m_t_adc = m_eeprom_data.t_adc;
  m_ef_smooth = m_eeprom_data.ef_smooth;
  m_n_adc = m_eeprom_data.n_adc;
  m_eth_data.n_avg = m_n_avg;
  m_eth_data.t_adc = m_t_adc;
  m_eth_data.ef_smooth = m_ef_smooth;
  m_adc_ad4630.set_ef_smooth(m_ef_smooth);
  m_adc_ad4630.set_avg(m_n_avg);
  m_adc_ad4630.set_t(m_t_adc);
}

void hrm::app_t::init_keyboard_drv()
{
  m_keyboard_drv.add_horizontal_pins(&mp_cfg->key_drv_horizontal_pins);
  m_keyboard_drv.add_vertical_pins(&mp_cfg->key_drv_vertical_pins);
  irs::set_default_keys(&m_keyboard_drv);
}

void hrm::app_t::init_encoder_drv()
{
  //m_encoder_drv.add_press_down_pin(&mp_cfg->enc_sw);
  //irs::set_default_keys(&m_encoder_drv);
}

void hrm::app_t::tick()
{ 
  m_adc_ad4630.tick();
  mp_cfg->tick();
  m_mxnet_server.tick();
  m_eeprom.tick();
  m_bridge_voltage_dac.tick();
  
  if (m_show_network_params) {
    mxip_t ip = mxip_t::zero_ip();
    mxip_t mask = mxip_t::zero_ip();
    mxip_t gateway = mxip_t::zero_ip();
    bool dhcp_enabled = false;
    mp_cfg->network_config.get(&ip, &mask, &gateway, &dhcp_enabled);
    if (ip.val[0] != 0 || ip.val[1] != 0 || ip.val[2] != 0 || ip.val[3] != 0) {
      m_show_network_params = false;
      
      show_network_params_t show_network_params(&mp_cfg->network_config);
      
      m_buzzer.bzz();
    }
  }
  m_relay_bridge_pos.tick();
  m_relay_bridge_neg.tick();
  m_relay_prot.tick();
  m_relay_divp.tick();
  m_relay_divn.tick();
  m_relay_divsw.tick();

  m_buzzer.tick();

  mp_menu->tick();

  m_lcd_drv.tick();
  m_keyboard_event_gen.tick();
  
  m_device_condition_controller.tick();
  m_treg_peltier.tick();
  m_treg_dac_peltier.tick();

  if (m_buzzer_kb_event.check()) {
    m_buzzer.bzz();
  }

  if (m_eth_timer.check()) {
    m_eth_data.counter++;
    //  Обновление сетевых переменных от программы - пользователю
    if (m_mode != md_manual) {
      //  Реле
      if (m_eth_data.relay_bridge_pos != m_relay_bridge_pos) {
        m_eth_data.relay_bridge_pos = m_relay_bridge_pos;
      }
      if (m_eth_data.relay_bridge_neg != m_relay_bridge_neg) {
        m_eth_data.relay_bridge_neg = m_relay_bridge_neg;
      }
      if (m_eth_data.relay_prot != m_relay_prot) {
        m_eth_data.relay_prot = m_relay_prot;
      }
      if (m_eth_data.relay_divp != m_relay_divp) {
        m_eth_data.relay_divp = m_relay_divp;
      }
      if (m_eth_data.relay_divn != m_relay_divn) {
        m_eth_data.relay_divn = m_relay_divn;
      }
      if (m_eth_data.relay_divsw != m_relay_divsw) {
        m_eth_data.relay_divsw = m_relay_divsw;
      }
    }
    if (m_adc_ad4630.new_data()) {
      //  Continious conversion
      m_eth_data.voltage1 = m_adc_ad4630.voltage1();
      m_eth_data.voltage2 = m_adc_ad4630.voltage2();
      m_eth_data.voltage_ef1 = m_adc_ad4630.voltage_ef1();
      m_eth_data.voltage_ef2 = m_adc_ad4630.voltage_ef2();
      m_eth_data.result = m_adc_ad4630.voltage_ef1() - m_adc_ad4630.voltage_ef2();
      //m_adc_ad4630.start_single_conversion();
    }
    m_eth_data.adc_error_cnt = m_adc_ad4630.error_cnt();
    if (m_eth_data.n_avg != m_n_avg) {
      m_n_avg = m_adc_ad4630.set_avg(m_eth_data.n_avg);
      m_eth_data.n_avg = m_n_avg;
      m_eeprom_data.n_avg = m_n_avg;
    }
    if (m_mode != md_balance) {
      if (m_eth_data.t_adc != m_t_adc) {
        m_t_adc = m_adc_ad4630.set_t(m_eth_data.t_adc);
        m_eth_data.t_adc = m_t_adc;
        m_eeprom_data.t_adc = m_t_adc;
      }
      if (m_eth_data.ef_smooth != m_ef_smooth) {
        m_ef_smooth = m_adc_ad4630.set_ef_smooth(m_eth_data.ef_smooth);
        m_eth_data.ef_smooth = m_ef_smooth;
        m_eeprom_data.ef_smooth = m_ef_smooth;
      }
      if (m_eth_data.n_adc != m_n_adc) {
        m_n_adc = m_adc_ad4630.correct_n_adc(m_eth_data.n_adc);
        m_eth_data.n_adc = m_n_adc;
        m_eeprom_data.n_adc = m_n_adc;
      }
      if (m_eth_data.ef_preset != 0) {
        m_eth_data.ef_preset = 0;
        m_adc_ad4630.ef_preset();
      }
    }
    //
    if (m_eth_data.current_mode != m_mode) {
      m_eth_data.current_mode = m_mode;
    }
    if (m_bridge_voltage_dac.get_voltage() 
        != m_eth_data.bridge_voltage_current) {
      m_eth_data.bridge_voltage_current = m_bridge_voltage_dac.get_voltage();
    }
    if ((m_mode == md_free) && (m_free_status == fs_idle)) {
      if (m_eth_data.apply_network_options == 1) {
        m_eth_data.apply_network_options = 0;
        m_eth_data.disable_reading_network_options = 0;
        reset_network_config();
      }
      if (!m_eth_data.disable_reading_network_options) {
        network_config_to_eth_data();
      }
    }

    if (m_eth_data.exp_cnt != m_eeprom_data.exp_cnt) {
      m_eeprom_data.exp_cnt = m_eth_data.exp_cnt;
    }
    if (m_eth_data.etalon != m_eeprom_data.etalon) {
      m_eeprom_data.etalon = m_eth_data.etalon;
    }
    if (m_eth_data.checked != m_eeprom_data.checked) {
      m_eeprom_data.checked = m_eth_data.checked;
    }
    if (m_eth_data.ratio != m_eeprom_data.ratio) {
      m_eeprom_data.ratio = m_eth_data.ratio;
    }
    if (m_eth_data.checked_prev != m_eeprom_data.checked_prev) {
      m_eeprom_data.checked_prev = m_eth_data.checked_prev;
    }
    if (m_eth_data.relay_pause_ms != m_eeprom_data.relay_pause_ms) {
      m_eeprom_data.relay_pause_ms = m_eth_data.relay_pause_ms;
    }
    if (m_eth_data.prepare_pause != m_eeprom_data.prepare_pause) {
      m_eeprom_data.prepare_pause = m_eth_data.prepare_pause;
    }
    if (m_eth_data.no_prot != m_eeprom_data.no_prot) {
      m_eeprom_data.no_prot = m_eth_data.no_prot;
    }
    if (m_eth_data.wild_relays != m_eeprom_data.wild_relays) {
      m_eeprom_data.wild_relays = m_eth_data.wild_relays;
      m_wild_relays = m_eth_data.wild_relays;
    }
    switch (m_balance_action) {
      case ba_prepare: m_eth_data.balance_action = 1; break;
      case ba_prepare_pause: m_eth_data.balance_action = 2; break;
      case ba_balance_neg: m_eth_data.balance_action = 3; break;
      case ba_elab_neg: m_eth_data.balance_action = 4; break;
      case ba_balance_pos: m_eth_data.balance_action = 5; break;
      case ba_elab_pos: m_eth_data.balance_action = 6; break;
      default: m_eth_data.balance_action = 0;
    }
    
    sync_first_to_second(&m_eth_data.ip_0, &m_eeprom_data.ip_0);
    sync_first_to_second(&m_eth_data.ip_1, &m_eeprom_data.ip_1);
    sync_first_to_second(&m_eth_data.ip_2, &m_eeprom_data.ip_2);
    sync_first_to_second(&m_eth_data.ip_3, &m_eeprom_data.ip_3);

    sync_first_to_second(&m_eth_data.mask_0, &m_eeprom_data.mask_0);
    sync_first_to_second(&m_eth_data.mask_1, &m_eeprom_data.mask_1);
    sync_first_to_second(&m_eth_data.mask_2, &m_eeprom_data.mask_2);
    sync_first_to_second(&m_eth_data.mask_3, &m_eeprom_data.mask_3);

    sync_first_to_second(&m_eth_data.gateway_0, &m_eeprom_data.gateway_0);
    sync_first_to_second(&m_eth_data.gateway_1, &m_eeprom_data.gateway_1);
    sync_first_to_second(&m_eth_data.gateway_2, &m_eeprom_data.gateway_2);
    sync_first_to_second(&m_eth_data.gateway_3, &m_eeprom_data.gateway_3);

    sync_first_to_second(&m_eth_data.dhcp_on, &m_eeprom_data.dhcp_on);
    
    m_treg_sync_parameters.sync();
    m_treg_dac_sync_parameters.sync();
    
    if (m_eth_data.show_last_result == 1) {
      m_eth_data.show_last_result = 0;
      show_last_result();
    }
    //  Version info
    if (m_eth_data.version_info != m_version.software) {
      m_eth_data.version_info = m_version.software;
    }
    //  Show options
    if (m_eth_data.show_options != m_eeprom_data.show_options) {
      m_eeprom_data.show_options = m_eth_data.show_options;
    }
    //  Rem time
    m_eth_data.exp_percentage = m_remaining_time_calculator.get_percentage();
  }
  if (m_blink_timer.check()) {
    if (m_blink) {
      mp_cfg->led_blink.set();
    } else {
      mp_cfg->led_blink.clear();
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
      m_remaining_time = irs::bound<irs_u32>(m_remaining_time - 1,
        0, m_remaining_time);
      m_remaining_time_calculator.secund_tick();
      m_eth_data.sum_time = m_sum_time;
      m_eth_data.remaining_time = 
        m_remaining_time_calculator.get_remaining_time();//m_remaining_time;
    }
  }
  switch (m_mode) {
    case md_free: {
      switch (m_free_status) {
        case fs_prepare: {
          //  reset to default
          irs::mlog() << endl;
          //
          m_relay_bridge_pos = 0;
          m_relay_bridge_neg = 0;
          m_relay_prot = 1;
          m_relay_divp = 0;
          m_relay_divn = 0;
          m_relay_divsw = 0;
          m_relay_bridge_pos.set_wild(false);
          m_relay_bridge_neg.set_wild(false);
          m_relay_divp.set_wild(false);
          m_relay_divn.set_wild(false);
          m_relay_divsw.set_wild(false);
          //
          m_eth_data.apply = 0;
          m_eth_data.prepare_pause = m_prepare_pause;
          m_service_timer.start();
          //
          m_eth_data.n_avg = m_n_avg;
          m_eth_data.t_adc = m_t_adc;
          m_eth_data.ef_smooth = m_ef_smooth;
          m_eth_data.n_adc = m_n_adc;
          //
          m_adc_ad4630.start_continious();
          
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
            m_eth_data.complete = 0;
            switch (m_eth_data.mode) {
              case md_manual: {
                m_mode = md_manual;
                m_free_status = fs_prepare;
                break;
              }
              case md_balance: {
                mp_menu->show_experiment_progress();
                m_mode = md_balance;
                m_free_status = fs_prepare;
                break;
              }
              default: {
                m_eth_data.mode = md_free;
              }
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
          m_relay_bridge_pos.set_after_pause(m_min_after_pause);
          m_relay_bridge_neg.set_after_pause(m_min_after_pause);
          m_relay_prot.set_after_pause(m_min_after_pause);
          m_relay_divp.set_after_pause(m_min_after_pause);
          m_relay_divn.set_after_pause(m_min_after_pause);
          m_relay_divsw.set_after_pause(m_min_after_pause);
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
            //  Bridge relays
            if (m_eth_data.relay_bridge_pos != m_relay_bridge_pos) {
              m_relay_bridge_pos = m_eth_data.relay_bridge_pos;
            }
            if (m_eth_data.relay_bridge_neg != m_relay_bridge_neg) {
              m_relay_bridge_neg = m_eth_data.relay_bridge_neg;
            }
            if (m_eth_data.relay_prot != m_relay_prot) {
              m_relay_prot = m_eth_data.relay_prot;
            }
            //  Relays AD4630
            if (m_eth_data.relay_divp != m_relay_divp) {
              m_relay_divp = m_eth_data.relay_divp;
            }
            if (m_eth_data.relay_divn != m_relay_divn) {
              m_relay_divn = m_eth_data.relay_divn;
            }
            if (m_eth_data.relay_divsw != m_relay_divsw) {
              m_relay_divsw = m_eth_data.relay_divsw;
            }
            //  Wild mode for bridge relays
            if (m_wild_relays != m_relay_bridge_pos.wild()) {
              m_relay_bridge_pos.set_wild(m_wild_relays);
            }
            if (m_wild_relays != m_relay_bridge_neg.wild()) {
              m_relay_bridge_neg.set_wild(m_wild_relays);
            }
            //  Wild mode for relays AD4630
            if (m_wild_relays != m_relay_divp.wild()) {
              m_relay_divp.set_wild(m_wild_relays);
            }
            if (m_wild_relays != m_relay_divn.wild()) {
              m_relay_divn.set_wild(m_wild_relays);
            }
            if (m_wild_relays != m_relay_divsw.wild()) {
              m_relay_divsw.set_wild(m_wild_relays);
            }
            //  Напряжение моста
            if (m_eth_data.bridge_voltage != m_bridge_voltage) {
              double bridge_voltage = 
                irs::bound<double>(m_eth_data.bridge_voltage, 
                m_min_bridge_voltage, m_max_bridge_voltage);
              if (m_eth_data.bridge_voltage != bridge_voltage) {
                m_eth_data.bridge_voltage = bridge_voltage;
              }
              m_bridge_voltage = bridge_voltage;
              m_eeprom_data.bridge_voltage = bridge_voltage;
              m_bridge_voltage_dac.set_voltage(m_bridge_voltage);
            }
            if (m_eth_data.bridge_voltage_reduced != m_bridge_voltage_reduced) {
              double bridge_voltage_reduced = 
                irs::bound<double>(m_eth_data.bridge_voltage_reduced, 
                m_min_bridge_voltage, m_max_bridge_voltage);
              if (m_eth_data.bridge_voltage_reduced 
                != bridge_voltage_reduced) {
                m_eth_data.bridge_voltage_reduced = bridge_voltage_reduced;
              }
              m_bridge_voltage_reduced = bridge_voltage_reduced;
              m_eeprom_data.bridge_voltage_reduced = bridge_voltage_reduced;
            }
            if (m_eth_data.bridge_voltage_speed != m_bridge_voltage_speed) {
              double bridge_voltage_speed = 
                irs::bound<double>(m_eth_data.bridge_voltage_speed, 
                m_min_bridge_voltage_speed, m_max_bridge_voltage_speed);
              if (m_eth_data.bridge_voltage_speed != bridge_voltage_speed) {
                m_eth_data.bridge_voltage_speed = bridge_voltage_speed;
              }
              m_bridge_voltage_speed = bridge_voltage_speed;
              m_eeprom_data.bridge_voltage_speed = bridge_voltage_speed;
              m_bridge_voltage_dac.set_speed(m_bridge_voltage_speed);
            }
            if (m_eth_data.bridge_voltage_after_pause_ms 
                != m_bridge_voltage_after_pause_ms) {
              double bridge_voltage_after_pause_ms = 
                irs::bound<double>(m_eth_data.bridge_voltage_after_pause_ms,
                m_min_bridge_voltage_after_pause_ms,
                m_max_bridge_voltage_after_pause_ms);
              if (m_eth_data.bridge_voltage_after_pause_ms 
                != bridge_voltage_after_pause_ms) {
                m_eth_data.bridge_voltage_after_pause_ms 
                = bridge_voltage_after_pause_ms;
              }
              m_bridge_voltage_after_pause_ms = bridge_voltage_after_pause_ms;
              m_eeprom_data.bridge_voltage_after_pause_ms 
                = bridge_voltage_after_pause_ms;
              m_bridge_voltage_dac.set_after_pause_ms(
                m_bridge_voltage_after_pause_ms);
            }
          } break;
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
          
          irs::mlog() << defaultfloat;

          m_etalon = m_eth_data.etalon;
          m_checked = m_eth_data.checked;

          m_balance_polarity = bp_neg;

          m_relay_after_pause = irs::make_cnt_ms(m_eth_data.relay_pause_ms);
          m_relay_bridge_pos.set_after_pause(m_relay_after_pause);
          m_relay_bridge_neg.set_after_pause(m_relay_after_pause);
          m_relay_prot.set_after_pause(m_relay_after_pause);
          m_relay_divp.set_after_pause(m_relay_after_pause);
          m_relay_divn.set_after_pause(m_relay_after_pause);
          m_relay_divsw.set_after_pause(m_relay_after_pause);

          m_wild_relays = m_eth_data.wild_relays;
          m_eeprom_data.wild_relays = m_wild_relays;
          if (m_wild_relays) {
            m_relay_bridge_pos.set_wild(true);
            m_relay_bridge_neg.set_wild(true);
            m_relay_divp.set_wild(true);
            m_relay_divn.set_wild(true);
            m_relay_divsw.set_wild(true);
          }

          m_relay_pause_timer.set(m_relay_after_pause);
          
          m_current_div_relay = 0;
          
          m_exp_vector.clear();
          m_analog_point.clear();
          m_analog_vector.clear();
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
          m_eth_data.no_prot = m_no_prot;
          
          //  Параметры АЦП в режиме уравновешивания
          m_adc_ad4630.stop_continious();
          m_adc_ad4630.set_avg(m_n_avg);
          m_adc_ad4630.set_t(m_t_adc);

          m_prepare_pause = m_eth_data.prepare_pause;
          m_prepare_pause_timer.set(irs::make_cnt_ms(1000));

          m_exp_time = 0;
          m_sum_time = 0;
          m_eth_data.sum_time = 0;
          m_remaining_time_calculator.reset();
          m_remaining_time_calculator.start(m_prepare_pause);
          
          //  Обслуживание ЦАП напряжения моста
          m_bridge_voltage = m_eth_data.bridge_voltage;
          m_bridge_voltage = irs::bound(m_bridge_voltage, 
            m_min_bridge_voltage, m_max_bridge_voltage);
          if (m_bridge_voltage != m_eeprom_data.bridge_voltage) {
            m_eeprom_data.bridge_voltage = m_bridge_voltage;
          }
          m_bridge_voltage_reduced = m_eth_data.bridge_voltage_reduced;
          m_bridge_voltage_reduced = irs::bound(
            m_bridge_voltage_reduced, 
            m_min_bridge_voltage, m_max_bridge_voltage);
          if (m_bridge_voltage_reduced != 
            m_eeprom_data.bridge_voltage_reduced) {
            m_eeprom_data.bridge_voltage_reduced = m_bridge_voltage_reduced;
          }
          if (m_bridge_voltage_reduced > m_bridge_voltage) {
            m_bridge_voltage_reduced = m_bridge_voltage;
          }
          m_bridge_voltage_speed = m_eth_data.bridge_voltage_speed;
          m_bridge_voltage_speed = irs::bound(m_bridge_voltage_speed, 
            m_min_bridge_voltage_speed, m_max_bridge_voltage_speed);
          if (m_bridge_voltage_speed != m_eeprom_data.bridge_voltage_speed) {
            m_eeprom_data.bridge_voltage_speed = m_bridge_voltage_speed;
          }
          m_bridge_voltage_after_pause_ms
            = m_eth_data.bridge_voltage_after_pause_ms;
          m_bridge_voltage_after_pause_ms = irs::bound(
            m_bridge_voltage_after_pause_ms,
            m_min_bridge_voltage_after_pause_ms,
            m_max_bridge_voltage_after_pause_ms);
          if (m_bridge_voltage_after_pause_ms != 
            m_eeprom_data.bridge_voltage_after_pause_ms) {
            m_eeprom_data.bridge_voltage_after_pause_ms 
              = m_bridge_voltage_after_pause_ms;
          }
          m_bridge_voltage_reduce_after_switching
            = m_eth_data.bridge_voltage_reduce_after_switching;
          if (m_bridge_voltage_reduce_after_switching !=
            m_eeprom_data.bridge_voltage_reduce_after_switching) {
            m_eeprom_data.bridge_voltage_reduce_after_switching
              = m_bridge_voltage_reduce_after_switching;
          }
          
          m_bridge_voltage_dac.set_after_pause_ms(
            m_bridge_voltage_after_pause_ms);
          m_bridge_voltage_dac.set_speed(m_bridge_voltage_speed);
          
          double current_bridge_dac_voltage = 0.0;
          if (m_bridge_voltage_reduce_after_switching) {
            m_bridge_voltage_dac.set_voltage(m_bridge_voltage_reduced);
            current_bridge_dac_voltage = m_bridge_voltage_reduced;
          } else {
            m_bridge_voltage_dac.set_voltage(m_bridge_voltage);
            current_bridge_dac_voltage = m_bridge_voltage;
          }
          irs::mlog() << irsm("Напряжение моста = ");
          irs::mlog() << fixed << defaultfloat << setprecision(4);
          irs::mlog() << m_bridge_voltage;
          irs::mlog() << irsm(" В, установлено ");
          irs::mlog() << current_bridge_dac_voltage;
          irs::mlog() << irsm(" В") << endl;
          
          irs::mlog() << setprecision(8);
          
          m_device_condition_controller.set_idle(true);
            
          m_balance_action = ba_prepare;
          m_remaining_time_calculator.change_balance_action(m_balance_action);
          m_remaining_time_calculator.
            set_balance_points_count(/*default_balance_points*/10);
          
          m_prepare_pause_completed = false;
          
          m_balance_status = bs_prepare_set_relays;
          break;
        }
        case bs_prepare_set_relays: {
          if (m_adc_ad4630.status() == irs_st_ready 
              && m_bridge_voltage_dac.ready()) {
            m_relay_bridge_neg = 1;
            m_balance_status = bs_prepare_set_pause;
          }
          break;
        }
        case bs_prepare_set_pause: {
          if (bridge_relays_ready()) {
            m_is_exp = false;
            m_prepare_pause_timer.start();
            m_prepare_current_time = m_prepare_pause;
            
            irs::mlog() << endl << irsm("Предварительная пауза ");
            irs::mlog() << m_eth_data.prepare_pause << irsm(" c") << endl;
            
            m_balance_action = ba_prepare_pause;
            m_remaining_time_calculator.change_balance_action(
              m_balance_action);
            
            m_adc_ad4630.start_continious();
            
            m_balance_status = bs_prepare_pause;
          }
          break;
        }
        case bs_prepare_pause: {
          if (m_prepare_pause_timer.check()) {
            if (m_prepare_current_time > 0) {
              m_prepare_pause_timer.start();
              m_prepare_current_time--;
              irs::mlog() << irsm(".") << flush;
              if (m_prepare_current_time % 60 == 0) {
                irs::mlog() << endl;
              }
            } else {
              m_adc_ad4630.stop_continious();
              m_prepare_pause_completed = true;
              irs::mlog() << endl;
              m_balance_action = ba_balance_neg;
              m_remaining_time_calculator.change_balance_action(
                m_balance_action);
              m_balance_status = bs_meas_vcom_bridge_off;
            }
          }
          break;
        }
        case bs_meas_vcom_bridge_off: {
          if (m_adc_ad4630.ready()) {
            m_relay_bridge_neg = 0;
            m_balance_status = bs_meas_vcom_prot_off;
          }
          break;
        }
        case bs_meas_vcom_prot_off: {
          if (bridge_relays_ready()) {
            m_relay_prot = 0;
            m_balance_status = bs_meas_vcom_prepare;
          }
          break;
        }
        case bs_meas_vcom_prepare: {
          if (bridge_relays_ready() && m_adc_ad4630.ready()) {
            m_adc_ad4630.start_continious(m_n_adc);
            m_balance_status = bs_meas_vcom;
          }
          break;
        }
        case bs_meas_vcom: {
          if (m_adc_ad4630.ready()) {
            m_analog_point.vcom1 = m_adc_ad4630.voltage1();
            m_analog_point.vcom2 = m_adc_ad4630.voltage2();
            irs::mlog() << irsm("Напряжения смещения каналов АЦП:") << endl;
            irs::mlog() << irsm("vcom1: ") << m_analog_point.vcom1;
            irs::mlog() << irsm(" В") << endl;
            irs::mlog() << irsm("vcom2: ") << m_analog_point.vcom2;
            irs::mlog() << irsm(" В") << endl;
            m_balance_status = bs_meas_vref_switch_on;
          }
          break;
        }
        case bs_meas_vref_switch_on: {
          if (bridge_relays_ready()) {
            m_relay_divsw = 1;
            m_balance_status = bs_meas_vref_prepare;
          }
          break;
        }
        case bs_meas_vref_prepare: {
          if (bridge_relays_ready() && m_adc_ad4630.ready()) {
            m_adc_ad4630.start_continious(m_n_adc);
            m_balance_status = bs_meas_vref;
          }
          break;
        }
        case bs_meas_vref: {
          if (m_adc_ad4630.ready()) {
            m_analog_point.vref1 = m_adc_ad4630.voltage1();
            m_analog_point.vref2 = m_adc_ad4630.voltage2();
            irs::mlog() << irsm("Напряжения ИОН каналов АЦП:") << endl;
            irs::mlog() << irsm("vref1: ") << m_analog_point.vref1;
            irs::mlog() << irsm(" В") << endl;
            irs::mlog() << irsm("vref2: ") << m_analog_point.vref2;
            irs::mlog() << irsm(" В") << endl;
            m_balance_status = bs_meas_vref_switch_off;
          }
          break;
        }
        case bs_meas_vref_switch_off: {
          if (bridge_relays_ready()) {
            m_relay_divsw = 0;
            m_balance_status = bs_main_prot_on;
          }
          break;
        }
        case bs_main_prot_on: {
          if (bridge_relays_ready()) {
            m_relay_prot = 1;
            m_balance_status = bs_main_change_polarity;
          }
          break;
        }
        case bs_main_change_polarity: {
          if (bridge_relays_ready()) {
            switch (m_balance_polarity) {
              case bp_neg: {
                m_relay_bridge_neg = 1;
                irs::mlog() << irsm("------------- (-) ---------------") <<endl;
                break;
              }
              case bp_pos: {
                m_relay_bridge_pos = 1;
                irs::mlog() << irsm("------------- (+) ---------------") <<endl;
                break;
              }
            }
            m_current_div_relay = 0;
            m_balance_status = bs_main_set_prot;
          }
          break;
        }
        case bs_main_set_prot: {
          if (bridge_relays_ready()) {
            //  Отключение защитного реле, если не нужно
            if (m_no_prot) {
              m_relay_prot = 0;
            }
            m_balance_status = bs_main_set_div_relays;
          }
          break;
        }
        case bs_main_set_div_relays: {
          if (bridge_relays_ready()) {
            m_relay_divn = decode_relay_divn(m_current_div_relay);
            m_relay_divp = decode_relay_divp(m_current_div_relay);
            m_balance_status = bs_main_adc_prepare;
          }
          break;
        }
        case bs_main_adc_prepare: {
          if (bridge_relays_ready() && m_adc_ad4630.ready()) {
            m_adc_ad4630.start_continious(m_n_adc);
            m_balance_status = bs_main_adc_read;
          }
          break;
        }
        case bs_main_adc_read: {
          if (m_adc_ad4630.ready()) {
            double v1 = m_adc_ad4630.voltage1();
            double v2 = m_adc_ad4630.voltage2();
            m_analog_point.v1.push_back(v1);
            m_analog_point.v2.push_back(v2);
            int N = m_current_div_relay;
            irs::mlog() << irsm("------------- (") <<  N;
            irs::mlog() << irsm(") ---------------") <<endl;
            irs::mlog() << irsm("V1_");
            irs::mlog() << decode_relay_divp(m_current_div_relay);
            irs::mlog() << decode_relay_divn(m_current_div_relay);
            irs::mlog() << irsm(" = ") << v1 << irsm(" В") << endl;
            irs::mlog() << irsm("V2_");
            irs::mlog() << decode_relay_divp(m_current_div_relay);
            irs::mlog() << decode_relay_divn(m_current_div_relay);
            irs::mlog() << irsm(" = ") << v2 << irsm(" В") << endl;
            m_balance_status = bs_main_div_change;
          }
          break;
        }
        case bs_main_div_change: {
          if (bridge_relays_ready()) {
            if (m_current_div_relay < stop_div_relay) {
              //  Next DIV relays combination
              //m_current_div_relay++;
              m_current_div_relay = stop_div_relay; //  Use only "0" and "3"
              m_balance_status = bs_main_set_div_relays;
            } else {
              m_balance_status = bs_ending_prot_on;
            }
          }
          break;
        }
        case bs_ending_prot_on: {
          if (bridge_relays_ready()) {
            m_relay_prot = 1;
            m_balance_status = bs_ending_bridge_off;
          }
          break;
        }
        case bs_ending_bridge_off: {
          if (bridge_relays_ready()) {
            switch (m_balance_polarity) {
              case bp_neg: {
                m_relay_bridge_neg = 0;
                m_balance_polarity = bp_pos;
                m_balance_status = bs_main_change_polarity;
                break;
              }
              case bp_pos: {
                m_relay_bridge_pos = 0;
                m_balance_status = bs_ending_ready;
                break;
              }
            }
          }
          break;
        }
        case bs_ending_ready: {
          if (bridge_relays_ready()) {
            m_balance_status = bs_report;
          }
          break;
        }
        case bs_report: {
          irs::mlog() << irsm("------------ REPORT -------------") << endl;
          m_buzzer.bzz(2);
          m_prev_exp_time = m_exp_time;
          m_eth_data.prev_exp_time = m_prev_exp_time;
          m_exp_time = 0;
          m_eth_data.exp_time = m_exp_time;
          
          irs::mlog() << endl;
          exp_t exp;
          
          exp.exp_time = m_prev_exp_time;
          exp.temperature_ext = m_eth_data.th_ext_1;
          exp.temperature_dac = m_eth_data.th_box_ldo;
          exp.temperature_adc = m_eth_data.th_box_adc;
          //  --------------------------------------------------------------
          const irs::string_t sign[] = {irsm("-"), irsm("+")};
          size_t L = m_analog_point.v1.size() / 2;
          irs::mlog() << setprecision(8);
          irs::mlog() << irsm("vcom  ") << m_analog_point.vcom1;
          irs::mlog() << irsm(" ") << m_analog_point.vcom2 << endl;
          irs::mlog() << irsm("vref  ") << m_analog_point.vref1;
          irs::mlog() << irsm(" ") << m_analog_point.vref2 << endl;
          for (irs_u8 j = 0; j <= 1; j++) {
            for (irs_u8 i = 0; i < L; i++) {
              irs::mlog() << irsm("V_");
              irs::mlog() << decode_relay_divp(i);
              irs::mlog() << decode_relay_divn(i);
              irs::mlog() << irsm(sign[j]);
              irs::mlog() << irsm(" ") << m_analog_point.v1[i + j * L];
              irs::mlog() << irsm(" ") << m_analog_point.v2[i + j * L] << endl;
            }
          }
          irs::mlog() << endl; 
          //  --------------------------------------------------------------
          double b = m_analog_point.vcom1 - m_analog_point.vcom2;
          double k = m_analog_point.vref1 / (m_analog_point.vref2 - b);
          double K1 = (4.096 - m_analog_point.vcom1) 
            / (m_analog_point.vref1 - m_analog_point.vcom1);
//          double K2 = (4.096 - m_analog_point.vcom2) 
//            / (m_analog_point.vref2 - m_analog_point.vcom2);
          vector<double> u1v;
          vector<double> u2v;
          u1v.clear();
          u2v.clear();
          double u2 = 0.0;
          for (size_t i = 0; i < m_analog_point.v1.size(); i++) {
            u1v.push_back(m_analog_point.vcom1 
              + K1 * (m_analog_point.v1[i] - m_analog_point.vcom1));
            u2 = m_analog_point.v2[i] * k - b;
            u2v.push_back(m_analog_point.vcom1 
              + K1 * (u2c - m_analog_point.vcom1));
          }
          vector<double> Dv;
          Dv.clear();
          Dv.push_back(u1v[0] / );
          
          //  --------------------------------------------------------------
//          double a1 = m_analog_point.v1_neg_div1 - m_analog_point.vcom1;
//          double b1 = m_analog_point.v1_neg_div2 - m_analog_point.vcom1;
//          double a2 = m_analog_point.v2_neg_div1 - m_analog_point.vcom2;
//          double b2 = m_analog_point.v2_neg_div2 - m_analog_point.vcom2;
//          double d = m_analog_point.vcom2 - m_analog_point.vcom1;
//          double k2 = d * (a1 - b1) / (b2 * b1 - a2 * a1);
//          double k1 = (d + k2 * b2) / a1;
//          double u1 = m_analog_point.vcom1 
//            + k1 * (m_analog_point.v1_neg_div1 - m_analog_point.vcom1);
//          double u2 = m_analog_point.vcom2
//            + k2 * (m_analog_point.v2_neg_div1 - m_analog_point.vcom2);
//          double Dn = u1 / u2;
//          a1 = m_analog_point.v1_pos_div1 - m_analog_point.vcom1;
//          b1 = m_analog_point.v1_pos_div2 - m_analog_point.vcom1;
//          a2 = m_analog_point.v2_pos_div1 - m_analog_point.vcom2;
//          b2 = m_analog_point.v2_pos_div2 - m_analog_point.vcom2;
//          k2 = d * (a1 - b1) / (b2 * b1 - a2 * a1);
//          k1 = (d + k2 * b2) / a1;
//          u1 = m_analog_point.vcom1 
//            + k1 * (m_analog_point.v1_pos_div1 - m_analog_point.vcom1);
//          u2 = m_analog_point.vcom2
//            + k2 * (m_analog_point.v2_pos_div1 - m_analog_point.vcom2);
//          double Dp = u1 / u2;
          //  ---------------  OLD FORMULA RESULT  -------------------------
//          m_result = (1.0 - Dn + Dp);
          m_result = 1.0;//= (1.0 + Dn - Dp);
          //
          irs::mlog() << irsm("Результат") << endl;
          irs::mlog() << setw(14) << setprecision(14);
          irs::mlog() << m_result << endl;
          
          exp.result_old = m_result;
          
          exp.target_sko_adc_neg = 0;
          exp.target_sko_adc_pos = 0;
          exp.target_sko_dac_neg = 0;
          exp.target_sko_dac_pos = 0;
          exp.neg_n = 0;
          exp.pos_n = 0;
          
          m_exp_vector.push_back(exp);
            
//          m_eth_data.result = exp.result_old * m_eth_data.etalon;//  Result OLD
//          m_eth_data.ratio = m_result;
//          m_result *= m_etalon;
//          m_result_error = ((m_result - m_checked) / m_checked) * 100.;
//          irs::mlog() << setprecision(7);
//          irs::mlog() << irsm("Отношение ") << m_eth_data.ratio
//            << endl;
//          irs::mlog() << irsm("Результат ") << m_result
//            << irsm(" Ом") << endl;
//          irs::mlog() << irsm("Отклонение ") << m_result_error
//            << irsm(" %") << endl;
          m_balance_status = bs_next_exp;
          break;
        }
        case bs_next_exp: {
          m_eth_data.current_exp_cnt--;
          
          if (m_exp_vector.size() >= m_exp_cnt) {
            m_eth_data.complete = 1;
            mp_menu->show_experiment_result();
            m_balance_status = bs_final_report;
          } else {
            irs::mlog() << irsm("---------------------------------") << endl;
            irs::mlog() << irsm("Эксперимент № ");
            irs::mlog() << m_exp_vector.size() + 1;
            irs::mlog() << irsm(" из ") << static_cast<int>(m_exp_cnt) << endl;
            irs::mlog() << irsm("---------------------------------") << endl;

            m_balance_polarity = bp_neg;
            if (m_bridge_voltage_reduce_after_switching) {
              m_bridge_voltage_dac.set_voltage(m_bridge_voltage_reduced);
            }

            m_balance_status = bs_meas_vcom_bridge_off;
          }
          break;
        }
        case bs_final_report: {
//          show_last_result();
          m_is_exp = false;
          m_buzzer.bzz(3);
          m_balance_status = bs_prepare;
          m_mode = md_free;
          break;
        }
      }
      if (m_eth_data.reset == 1) {
        irs::mlog() << irsm("---------------------------------") << endl;
        irs::mlog() << irsm("------------- Сброс -------------") << endl;
        m_eth_data.reset = 0;
        m_balance_action = ba_idle;
        m_remaining_time_calculator.change_balance_action(
          m_balance_action);
        m_adc_ad4630.stop_continious();
        if (m_mode == md_balance) {
          mp_menu->show_experiment_options();
          m_balance_status = bs_final_report;
        } else {
          m_balance_status = bs_prepare;
          m_mode = md_free;
          m_eth_data.mode = md_free;
        }
      }
      break;
    }
  }
}

void hrm::app_t::reset_network_config()
{
  mxip_t ip = mxip_t::zero_ip();
  ip.val[0] = m_eth_data.ip_0;
  ip.val[1] = m_eth_data.ip_1;
  ip.val[2] = m_eth_data.ip_2;
  ip.val[3] = m_eth_data.ip_3;

  mxip_t mask = mxip_t::zero_ip();
  mask.val[0] = m_eth_data.mask_0;
  mask.val[1] = m_eth_data.mask_1;
  mask.val[2] = m_eth_data.mask_2;
  mask.val[3] = m_eth_data.mask_3;

  mxip_t gateway = mxip_t::zero_ip();
  gateway.val[0] = m_eth_data.gateway_0;
  gateway.val[1] = m_eth_data.gateway_1;
  gateway.val[2] = m_eth_data.gateway_2;
  gateway.val[3] = m_eth_data.gateway_3;

  const bool dhcp_enabled = m_eth_data.dhcp_on;

  mp_cfg->network_config.set(ip, mask, gateway, dhcp_enabled);
}

void hrm::app_t::network_config_to_eth_data()
{
  mxip_t ip = mxip_t::zero_ip();
  mxip_t mask = mxip_t::zero_ip();
  mxip_t gateway = mxip_t::zero_ip();
  bool dhcp_enabled = false;
  mp_cfg->network_config.get(&ip, &mask, &gateway, &dhcp_enabled);

  m_eth_data.ip_0 = ip.val[0];
  m_eth_data.ip_1 = ip.val[1];
  m_eth_data.ip_2 = ip.val[2];
  m_eth_data.ip_3 = ip.val[3];

  m_eth_data.mask_0 = mask.val[0];
  m_eth_data.mask_1 = mask.val[1];
  m_eth_data.mask_2 = mask.val[2];
  m_eth_data.mask_3 = mask.val[3];

  m_eth_data.gateway_0 = gateway.val[0];
  m_eth_data.gateway_1 = gateway.val[1];
  m_eth_data.gateway_2 = gateway.val[2];
  m_eth_data.gateway_3 = gateway.val[3];

  m_eth_data.dhcp_on = dhcp_enabled;
}

//double hrm::app_t::calc_elab_code(vector<elab_point_t>* ap_elab_vector,
//  balancing_coil_t a_balancing_coil, etalon_polarity_t a_etpol)
//{
//  size_t shift = 0;
//  size_t cnt = 0;
//  if (ap_elab_vector->size() <= 2) {
//    if (a_etpol == ep_neg) {
//      if (a_balancing_coil == bc_etalon) {
//        irs::mlog() << irsm("Эталон") << endl;
//        shift = 1;
//      } else {
//        irs::mlog() << irsm("Поверяемая") << endl;
//        shift = 0;
//      }
//    }
//    irs_i32 int_dac_code =
//      static_cast<irs_i32>((*ap_elab_vector)[shift].dac * pow(2., 19));
//    irs::mlog() << irsm("Код без уточнения = ") << int_dac_code << endl;
//    return (*ap_elab_vector)[shift].dac;
//  } else {
//    if (a_etpol == ep_neg) {
//      cnt = ap_elab_vector->size() / 2;
//      if (a_balancing_coil == bc_etalon) {
//        irs::mlog() << irsm("Эталон") << endl;
//        shift = ap_elab_vector->size() / 2;
//      } else {
//        irs::mlog() << irsm("Поверяемая") << endl;
//      }
//    } else {
//      cnt = ap_elab_vector->size();
//    }
//
//    for (size_t i = 0; i < cnt; i++) {
//      irs_i32 int_dac_code =
//        static_cast<irs_i32>((*ap_elab_vector)[i + shift].dac * pow(2., 19));
//      irs::mlog() << (i + 1) << irsm(": ") << setw(12)
//        << int_dac_code << irsm(" : ") << setw(12)
//        << ((*ap_elab_vector)[i + shift].adc * 1.0e6) << irsm(" мкВ")
//        << irsm(": СКО ") << setw(12)
//        << ((*ap_elab_vector)[i + shift].sko * 1.0e6) << irsm(" мкВ") << endl;
//    }
//    size_t left = 0;
//    for (size_t i = 0; i+1 < cnt; i++) {
//      if ((*ap_elab_vector)[i+shift].adc
//          * (*ap_elab_vector)[i+shift+1].adc < 0.
//      ) {
//        left = i;
//        break;
//      }
//    }
//
//    double x1 = (*ap_elab_vector)[left + shift].adc;
//    double y1 = (*ap_elab_vector)[left + shift].dac;
//    double x2 = (*ap_elab_vector)[left + shift + 1].adc;
//    double y2 = (*ap_elab_vector)[left + shift + 1].dac;
//    double k = (y2 - y1) / (x2 - x1);
//    double b = y2 - k * x2;
//    double int_result = b * pow(2.,19);
//
//    //  mnk
//    double sum_x = 0.;
//    double sum_x2 = 0.;
//    double sum_y = 0.;
//    double sum_xy = 0.;
//    double n = static_cast<double>(cnt);
//    for (size_t i = 0; i < cnt; i++) {
//      sum_x += (*ap_elab_vector)[i+shift].adc;
//      sum_x2 += (*ap_elab_vector)[i+shift].adc * (*ap_elab_vector)[i+shift].adc;
//      sum_y += (*ap_elab_vector)[i+shift].dac;
//      sum_xy += (*ap_elab_vector)[i+shift].adc * (*ap_elab_vector)[i+shift].dac;
//    }
//    double k_mnk = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
//    double b_mnk = (sum_y - k_mnk * sum_x) / n;
//    double int_result_mnk = b_mnk * pow(2.,19);
//
//    irs::mlog() << irsm("Уточнённый код = ") << int_result << endl;
//    irs::mlog() << irsm("Уточнённый код МНК = ") << int_result_mnk << endl;
//    irs::mlog() << irsm("---------------------------------") << endl;
//    return b_mnk;
//  }
//}

void hrm::app_t::print_elab_result(vector<elab_point_t>* ap_elab_vector,
  size_t a_num, size_t a_cnt)
{
  size_t shift = a_cnt * a_num;
  for (size_t i = 0; i < a_cnt; i++) {
    dac_value_t dac_code = (*ap_elab_vector)[i + shift].dac;
    irs::mlog() << (i + 1) << irsm(": ") << setw(12)
      << dac_code << irsm(" : ") << setw(12)
      << ((*ap_elab_vector)[i + shift].adc * 1.0e6) << irsm(" мкВ")
      << irsm(": СКО ") << setw(12)
      << ((*ap_elab_vector)[i + shift].sko * 1.0e6) << irsm(" мкВ") << endl;
  }
}

double hrm::app_t::only_calc_elab_code(vector<elab_point_t>* ap_elab_vector,
  size_t a_num, size_t a_cnt)
{
  double result = 0.0;
  size_t shift = a_num * a_cnt;
  if (a_cnt <= 1) {
    result = (*ap_elab_vector)[shift].dac;
  } else {
    double sum_x = 0.;
    double sum_x2 = 0.;
    double sum_y = 0.;
    double sum_xy = 0.;
    double n = static_cast<double>(a_cnt);
    for (size_t i = 0; i < a_cnt; i++) {
      sum_x += (*ap_elab_vector)[i+shift].adc;
      sum_x2 += (*ap_elab_vector)[i+shift].adc*(*ap_elab_vector)[i+shift].adc;
      sum_y += (*ap_elab_vector)[i+shift].dac;
      sum_xy += (*ap_elab_vector)[i+shift].adc*(*ap_elab_vector)[i+shift].dac;
    }
    double k_mnk = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    double b_mnk = (sum_y - k_mnk * sum_x) / n;
    result = b_mnk;
  }
  return result;
}

double hrm::app_t::calc_elab_code_2point(vector<elab_point_t>* ap_elab_vector,
  size_t a_shift)
{
  double result = 0.0;
  size_t shift = a_shift * 2;
  double y1 = (*ap_elab_vector)[shift].adc;
  double y2 = (*ap_elab_vector)[shift + 1].adc;
  double x1 = (*ap_elab_vector)[shift].dac;
  double x2 = (*ap_elab_vector)[shift + 1].dac;
  if (x1 == x2 || y1 == y2) {
    result = 0.0;
  } else {
    double k = (y2 - y1) / (x2 - x1);
    result = x2 - y2 / k;
  }
  return result;
}

hrm::init_eeprom_t::init_eeprom_t(irs::eeprom_at25128_data_t* ap_eeprom,
  eeprom_data_t* ap_eeprom_data)
{
  while (!ap_eeprom->connected()) ap_eeprom->tick();
  if (ap_eeprom->error()) {
    if (ap_eeprom_data != IRS_NULL) {
      ap_eeprom_data->reset_to_default();
      irs::mlog() << irsm("Ahtung!!! EEPROM reset to default!!!") << endl;
    } else {
      irs::mlog() << irsm("Ahtung!!! EEPROM error!!!") << endl;
    }
  }
}

hrm::init_eeprom_t::~init_eeprom_t()
{
}

//void hrm::app_t::update_elab_pid_koefs(double a_td, double a_adc, 
//  double a_dac, double a_sens = 1.0)
//{
//  m_elab_pid.k = m_elab_pid_kp * a_sens;
//  if (a_td > 0.0) {
//    m_elab_pid.ki = m_elab_pid_ki * a_td;
//  } else {
//    m_elab_pid.ki = 0.0;
//  }
//  m_elab_pid.kd = m_elab_pid_kd / a_td;
//  m_elab_iso.fd.x1 = a_adc;
//  m_elab_iso.fd.y1 = a_adc;
//  
//  if (a_td > 0.0) {
//    m_elab_iso.fd.t = m_elab_iso_t / a_td;
//  } else {
//    m_elab_iso.fd.t = 0.0;
//  }
//  m_elab_pid_fade.x1 = a_dac;
//  m_elab_pid_fade.y1 = a_dac;
//  //m_elab_pid_fade.t = m_elab_pid_fade_t * m_adc.get_reference_frequency();
//  
//  if (a_td > 0.0) {
//    m_elab_pid_fade.t = m_elab_pid_fade_t / a_td;
//  } else {
//    m_elab_pid_fade.t = 0.0;
//  }
//  
//  pid_reg_sync(&m_elab_pid, a_adc, a_dac);
//}
//
//
//hrm::dac_value_t hrm::app_t::norm(hrm::dac_value_t a_in)
//{
//  dac_value_t out = 0.0;
//  if (a_in > 0.0) {
//    out = a_in / (pow(2.0, 19) - 1.0); 
//  } else {
//    out = a_in / pow(2.0, 19);
//  }
//  return out;
//}
//
//hrm::dac_value_t hrm::app_t::denorm(hrm::dac_value_t a_in)
//{
//  dac_value_t out = 0.0;
//  if (a_in > 0.0) {
//    out = a_in * (pow(2.0, 19) - 1.0); 
//  } else {
//    out = a_in * pow(2.0, 19);
//  }
//  return out;
//}

//void hrm::app_t::adc_params_load_from_eeprom()
//{
//  //  ADC free / voltage
//  m_eth_data.adc_free_vx_gain = m_eeprom_data.adc_free_vx_gain;
//  m_eth_data.adc_free_vx_filter = m_eeprom_data.adc_free_vx_filter;
//  m_eth_data.adc_free_vx_channel = m_eeprom_data.adc_free_vx_channel;
//  m_eth_data.adc_free_vx_cnv_cnt = m_eeprom_data.adc_free_vx_cnv_cnt;
//  m_eth_data.adc_free_vx_additional_gain 
//    = m_eeprom_data.adc_free_vx_additional_gain;
//  m_eth_data.adc_free_vx_ref = m_eeprom_data.adc_free_vx_ref;
//  m_eth_data.adc_free_vx_cont_cnv_cnt = m_eeprom_data.adc_free_vx_cont_cnv_cnt;
//  m_eth_data.adc_free_vx_impf_iterations_cnt
//    = m_eeprom_data.adc_free_vx_impf_iterations_cnt;
//  m_eth_data.adc_free_vx_impf_type = m_eeprom_data.adc_free_vx_impf_type;
//  m_eth_data.adc_free_vx_cont_mode = m_eeprom_data.adc_free_vx_cont_mode;
//  m_eth_data.adc_free_vx_cont_sko = m_eeprom_data.adc_free_vx_cont_sko;
//  
//  m_adc_free_vx_param_data.gain = m_eeprom_data.adc_free_vx_gain;
//  m_adc_free_vx_param_data.filter = m_eeprom_data.adc_free_vx_filter;
//  m_adc_free_vx_param_data.channel = m_eeprom_data.adc_free_vx_channel;
//  m_adc_free_vx_param_data.cnv_cnt = m_eeprom_data.adc_free_vx_cnv_cnt;
//  m_adc_free_vx_param_data.additional_gain 
//    = m_eeprom_data.adc_free_vx_additional_gain;
//  m_adc_free_vx_param_data.ref = m_eeprom_data.adc_free_vx_ref;
//  m_adc_free_vx_param_data.cont_cnv_cnt
//    = m_eeprom_data.adc_free_vx_cont_cnv_cnt;
//  m_adc_free_vx_param_data.impf_iterations_cnt
//    = m_eeprom_data.adc_free_vx_impf_iterations_cnt;
//  m_adc_free_vx_param_data.impf_type 
//    = convert_to_impf_type(m_eeprom_data.adc_free_vx_impf_type);
//  m_adc_free_vx_param_data.cont_mode 
//    = convert_to_cont_mode(m_eeprom_data.adc_free_vx_cont_mode);
//  m_adc_free_vx_param_data.cont_sko = m_eeprom_data.adc_free_vx_cont_sko;
//  //  ADC PID
//  m_eth_data.adc_pid_gain = m_eeprom_data.adc_pid_gain;
//  m_eth_data.adc_pid_filter = m_eeprom_data.adc_pid_filter;
//  m_eth_data.adc_pid_channel = m_eeprom_data.adc_pid_channel;
//  m_eth_data.adc_pid_cnv_cnt = m_eeprom_data.adc_pid_cnv_cnt;
//  m_eth_data.adc_pid_additional_gain 
//    = m_eeprom_data.adc_pid_additional_gain;
//  m_eth_data.adc_pid_ref = m_eeprom_data.adc_pid_ref;
//  m_eth_data.adc_pid_cont_cnv_cnt = m_eeprom_data.adc_pid_cont_cnv_cnt;
//  m_eth_data.adc_pid_impf_iterations_cnt
//    = m_eeprom_data.adc_pid_impf_iterations_cnt;
//  m_eth_data.adc_pid_impf_type = m_eeprom_data.adc_pid_impf_type;
//  m_eth_data.adc_pid_cont_mode = m_eeprom_data.adc_pid_cont_mode;
//  m_eth_data.adc_pid_cont_sko = m_eeprom_data.adc_pid_cont_sko;
//  
//  m_adc_pid_param_data.gain = m_eeprom_data.adc_pid_gain;
//  m_adc_pid_param_data.filter = m_eeprom_data.adc_pid_filter;
//  m_adc_pid_param_data.channel = m_eeprom_data.adc_pid_channel;
//  m_adc_pid_param_data.cnv_cnt = m_eeprom_data.adc_pid_cnv_cnt;
//  m_adc_pid_param_data.additional_gain 
//    = m_eeprom_data.adc_pid_additional_gain;
//  m_adc_pid_param_data.ref = m_eeprom_data.adc_pid_ref;
//  m_adc_pid_param_data.cont_cnv_cnt
//    = m_eeprom_data.adc_pid_cont_cnv_cnt;
//  m_adc_pid_param_data.impf_iterations_cnt
//    = m_eeprom_data.adc_pid_impf_iterations_cnt;
//  m_adc_pid_param_data.impf_type 
//    = convert_to_impf_type(m_eeprom_data.adc_pid_impf_type);
//  m_adc_pid_param_data.cont_mode 
//    = convert_to_cont_mode(m_eeprom_data.adc_pid_cont_mode);
//  m_adc_pid_param_data.cont_sko = m_eeprom_data.adc_pid_cont_sko;
//  //  ADC manual
//  m_eth_data.adc_manual_gain = m_eeprom_data.adc_manual_gain;
//  m_eth_data.adc_manual_filter = m_eeprom_data.adc_manual_filter;
//  m_eth_data.adc_manual_channel = m_eeprom_data.adc_manual_channel;
//  m_eth_data.adc_manual_cnv_cnt = m_eeprom_data.adc_manual_cnv_cnt;
//  m_eth_data.adc_manual_additional_gain 
//    = m_eeprom_data.adc_manual_additional_gain;
//  m_eth_data.adc_manual_ref = m_eeprom_data.adc_manual_ref;
//  m_eth_data.adc_manual_cont_cnv_cnt = m_eeprom_data.adc_manual_cont_cnv_cnt;
//  m_eth_data.adc_manual_impf_iterations_cnt
//    = m_eeprom_data.adc_manual_impf_iterations_cnt;
//  m_eth_data.adc_manual_impf_type = m_eeprom_data.adc_manual_impf_type;
//  m_eth_data.adc_manual_cont_mode = m_eeprom_data.adc_manual_cont_mode;
//  m_eth_data.adc_manual_cont_sko = m_eeprom_data.adc_manual_cont_sko;
//  
//  m_adc_manual_param_data.gain = m_eeprom_data.adc_manual_gain;
//  m_adc_manual_param_data.filter = m_eeprom_data.adc_manual_filter;
//  m_adc_manual_param_data.channel = m_eeprom_data.adc_manual_channel;
//  m_adc_manual_param_data.cnv_cnt = m_eeprom_data.adc_manual_cnv_cnt;
//  m_adc_manual_param_data.additional_gain 
//    = m_eeprom_data.adc_manual_additional_gain;
//  m_adc_manual_param_data.ref = m_eeprom_data.adc_manual_ref;
//  m_adc_manual_param_data.cont_cnv_cnt
//    = m_eeprom_data.adc_manual_cont_cnv_cnt;
//  m_adc_manual_param_data.impf_iterations_cnt
//    = m_eeprom_data.adc_manual_impf_iterations_cnt;
//  m_adc_manual_param_data.impf_type 
//    = convert_to_impf_type(m_eeprom_data.adc_manual_impf_type);
//  m_adc_manual_param_data.cont_mode 
//    = convert_to_cont_mode(m_eeprom_data.adc_manual_cont_mode);
//  m_adc_manual_param_data.cont_sko = m_eeprom_data.adc_manual_cont_sko;
//  //  ADC balance
//  m_eth_data.adc_balance_gain = m_eeprom_data.adc_balance_gain;
//  m_eth_data.adc_balance_filter = m_eeprom_data.adc_balance_filter;
//  m_eth_data.adc_balance_channel = m_eeprom_data.adc_balance_channel;
//  m_eth_data.adc_balance_cnv_cnt = m_eeprom_data.adc_balance_cnv_cnt;
//  m_eth_data.adc_balance_additional_gain 
//    = m_eeprom_data.adc_balance_additional_gain;
//  m_eth_data.adc_balance_ref = m_eeprom_data.adc_balance_ref;
//  m_eth_data.adc_balance_cont_cnv_cnt = m_eeprom_data.adc_balance_cont_cnv_cnt;
//  m_eth_data.adc_balance_impf_iterations_cnt
//    = m_eeprom_data.adc_balance_impf_iterations_cnt;
//  m_eth_data.adc_balance_impf_type = m_eeprom_data.adc_balance_impf_type;
//  m_eth_data.adc_balance_cont_mode = m_eeprom_data.adc_balance_cont_mode;
//  m_eth_data.adc_balance_cont_sko = m_eeprom_data.adc_balance_cont_sko;
//  
//  m_adc_balance_param_data.gain = m_eeprom_data.adc_balance_gain;
//  m_adc_balance_param_data.filter = m_eeprom_data.adc_balance_filter;
//  m_adc_balance_param_data.channel = m_eeprom_data.adc_balance_channel;
//  m_adc_balance_param_data.cnv_cnt = m_eeprom_data.adc_balance_cnv_cnt;
//  m_adc_balance_param_data.additional_gain 
//    = m_eeprom_data.adc_balance_additional_gain;
//  m_adc_balance_param_data.ref = m_eeprom_data.adc_balance_ref;
//  m_adc_balance_param_data.cont_cnv_cnt
//    = m_eeprom_data.adc_balance_cont_cnv_cnt;
//  m_adc_balance_param_data.impf_iterations_cnt
//    = m_eeprom_data.adc_balance_impf_iterations_cnt;
//  m_adc_balance_param_data.impf_type 
//    = convert_to_impf_type(m_eeprom_data.adc_balance_impf_type);
//  m_adc_balance_param_data.cont_mode 
//    = convert_to_cont_mode(m_eeprom_data.adc_balance_cont_mode);
//  m_adc_balance_param_data.cont_sko = m_eeprom_data.adc_balance_cont_sko;
//  //  ADC elab
//  m_eth_data.adc_elab_gain = m_eeprom_data.adc_elab_gain;
//  m_eth_data.adc_elab_filter = m_eeprom_data.adc_elab_filter;
//  m_eth_data.adc_elab_channel = m_eeprom_data.adc_elab_channel;
//  m_eth_data.adc_elab_cnv_cnt = m_eeprom_data.adc_elab_cnv_cnt;
//  m_eth_data.adc_elab_additional_gain 
//    = m_eeprom_data.adc_elab_additional_gain;
//  m_eth_data.adc_elab_ref = m_eeprom_data.adc_elab_ref;
//  m_eth_data.adc_elab_cont_cnv_cnt = m_eeprom_data.adc_elab_cont_cnv_cnt;
//  m_eth_data.adc_elab_impf_iterations_cnt
//    = m_eeprom_data.adc_elab_impf_iterations_cnt;
//  m_eth_data.adc_elab_impf_type = m_eeprom_data.adc_elab_impf_type;
//  m_eth_data.adc_elab_cont_mode = m_eeprom_data.adc_elab_cont_mode;
//  m_eth_data.adc_elab_cont_sko = m_eeprom_data.adc_elab_cont_sko;
//  
//  m_adc_elab_param_data.gain = m_eeprom_data.adc_elab_gain;
//  m_adc_elab_param_data.filter = m_eeprom_data.adc_elab_filter;
//  m_adc_elab_param_data.channel = m_eeprom_data.adc_elab_channel;
//  m_adc_elab_param_data.cnv_cnt = m_eeprom_data.adc_elab_cnv_cnt;
//  m_adc_elab_param_data.additional_gain 
//    = m_eeprom_data.adc_elab_additional_gain;
//  m_adc_elab_param_data.ref = m_eeprom_data.adc_elab_ref;
//  m_adc_elab_param_data.cont_cnv_cnt
//    = m_eeprom_data.adc_elab_cont_cnv_cnt;
//  m_adc_elab_param_data.impf_iterations_cnt
//    = m_eeprom_data.adc_elab_impf_iterations_cnt;
//  m_adc_elab_param_data.impf_type 
//    = convert_to_impf_type(m_eeprom_data.adc_elab_impf_type);
//  m_adc_elab_param_data.cont_mode 
//    = convert_to_cont_mode(m_eeprom_data.adc_elab_cont_mode);
//  m_adc_elab_param_data.cont_sko = m_eeprom_data.adc_elab_cont_sko;
//}
//
//void hrm::app_t::adc_params_translate_actual_to_eth()
//{
//  adc_param_data_t actual_params;
//  m_adc.get_params(&actual_params);
//  
//  m_eth_data.adc_gain = actual_params.gain;
//  m_eth_data.adc_filter = actual_params.filter;
//  m_eth_data.adc_channel = actual_params.channel;
//  m_eth_data.adc_cnv_cnt = actual_params.cnv_cnt;
//  m_eth_data.adc_additional_gain 
//    = actual_params.additional_gain;
//  m_eth_data.adc_ref = actual_params.ref;
//  m_eth_data.adc_cont_cnv_cnt = actual_params.cont_cnv_cnt;
//  m_eth_data.adc_impf_iterations_cnt
//    = actual_params.impf_iterations_cnt;
//  m_eth_data.adc_impf_type = actual_params.impf_type;
//  m_eth_data.adc_cont_mode = actual_params.cont_mode;
//  m_eth_data.adc_cont_sko = actual_params.cont_sko;
//}
//
//bool hrm::app_t::adc_params_recieve_and_save_free_vx()
//{
//  bool new_data = false;
//  if (m_eth_data.adc_free_vx_gain != m_adc_free_vx_param_data.gain) {
//    new_data = true;
//    m_adc_free_vx_param_data.gain = m_eth_data.adc_free_vx_gain;
//    m_eeprom_data.adc_free_vx_gain = m_eth_data.adc_free_vx_gain;
//  }
//  if (m_eth_data.adc_free_vx_filter != m_adc_free_vx_param_data.filter) {
//    new_data = true;
//    m_adc_free_vx_param_data.filter = m_eth_data.adc_free_vx_filter;
//    m_eeprom_data.adc_free_vx_filter = m_eth_data.adc_free_vx_filter;
//  }
//  if (m_eth_data.adc_free_vx_channel != m_adc_free_vx_param_data.channel) {
//    new_data = true;
//    m_adc_free_vx_param_data.channel = m_eth_data.adc_free_vx_channel;
//    m_eeprom_data.adc_free_vx_channel = m_eth_data.adc_free_vx_channel;
//  }
//  if (m_eth_data.adc_free_vx_cnv_cnt != m_adc_free_vx_param_data.cnv_cnt) {
//    new_data = true;
//    m_adc_free_vx_param_data.cnv_cnt = m_eth_data.adc_free_vx_cnv_cnt;
//    m_eeprom_data.adc_free_vx_cnv_cnt = m_eth_data.adc_free_vx_cnv_cnt;
//  }
//  if (m_eth_data.adc_free_vx_additional_gain 
//      != m_adc_free_vx_param_data.additional_gain) {
//    new_data = true;
//    m_adc_free_vx_param_data.additional_gain 
//      = m_eth_data.adc_free_vx_additional_gain;
//    m_eeprom_data.adc_free_vx_additional_gain 
//      = m_eth_data.adc_free_vx_additional_gain;
//  }
//  if (m_eth_data.adc_free_vx_ref != m_adc_free_vx_param_data.ref) {
//    new_data = true;
//    m_adc_free_vx_param_data.ref = m_eth_data.adc_free_vx_ref;
//    m_eeprom_data.adc_free_vx_ref = m_eth_data.adc_free_vx_ref;
//  }
//  if (m_eth_data.adc_free_vx_cont_cnv_cnt 
//      != m_adc_free_vx_param_data.cont_cnv_cnt) {
//    new_data = true;
//    m_adc_free_vx_param_data.cont_cnv_cnt 
//      = m_eth_data.adc_free_vx_cont_cnv_cnt;
//    m_eeprom_data.adc_free_vx_cont_cnv_cnt 
//      = m_eth_data.adc_free_vx_cont_cnv_cnt;
//  }
//  if (m_eth_data.adc_free_vx_impf_iterations_cnt 
//      != m_adc_free_vx_param_data.impf_iterations_cnt) {
//    new_data = true;
//    m_adc_free_vx_param_data.impf_iterations_cnt 
//      = m_eth_data.adc_free_vx_impf_iterations_cnt;
//    m_eeprom_data.adc_free_vx_impf_iterations_cnt 
//      = m_eth_data.adc_free_vx_impf_iterations_cnt;
//  }
//  if (m_eth_data.adc_free_vx_impf_type != m_adc_free_vx_param_data.impf_type) {
//    new_data = true;
//    m_adc_free_vx_param_data.impf_type 
//      = convert_to_impf_type(m_eth_data.adc_free_vx_impf_type);
//    m_eeprom_data.adc_free_vx_impf_type = m_eth_data.adc_free_vx_impf_type;
//  }
//  if (m_eth_data.adc_free_vx_cont_mode != m_adc_free_vx_param_data.cont_mode) {
//    new_data = true;
//    m_adc_free_vx_param_data.cont_mode 
//      = convert_to_cont_mode(m_eth_data.adc_free_vx_cont_mode);
//    m_eeprom_data.adc_free_vx_cont_mode = m_eth_data.adc_free_vx_cont_mode;
//  }
//  if (m_eth_data.adc_free_vx_cont_sko != m_adc_free_vx_param_data.cont_sko) {
//    new_data = true;
//    m_adc_free_vx_param_data.cont_sko = m_eth_data.adc_free_vx_cont_sko;
//    m_eeprom_data.adc_free_vx_cont_sko = m_eth_data.adc_free_vx_cont_sko;
//  }
//  return new_data;
//}
//
//bool hrm::app_t::adc_params_recieve_and_save_pid()
//{
//  bool new_data = false;
//  if (m_eth_data.adc_pid_gain != m_adc_pid_param_data.gain) {
//    new_data = true;
//    m_adc_pid_param_data.gain = m_eth_data.adc_pid_gain;
//    m_eeprom_data.adc_pid_gain = m_eth_data.adc_pid_gain;
//  }
//  if (m_eth_data.adc_pid_filter != m_adc_pid_param_data.filter) {
//    new_data = true;
//    m_adc_pid_param_data.filter = m_eth_data.adc_pid_filter;
//    m_eeprom_data.adc_pid_filter = m_eth_data.adc_pid_filter;
//  }
//  if (m_eth_data.adc_pid_channel != m_adc_pid_param_data.channel) {
//    new_data = true;
//    m_adc_pid_param_data.channel = m_eth_data.adc_pid_channel;
//    m_eeprom_data.adc_pid_channel = m_eth_data.adc_pid_channel;
//  }
//  if (m_eth_data.adc_pid_cnv_cnt != m_adc_pid_param_data.cnv_cnt) {
//    new_data = true;
//    m_adc_pid_param_data.cnv_cnt = m_eth_data.adc_pid_cnv_cnt;
//    m_eeprom_data.adc_pid_cnv_cnt = m_eth_data.adc_pid_cnv_cnt;
//  }
//  if (m_eth_data.adc_pid_additional_gain 
//      != m_adc_pid_param_data.additional_gain) {
//    new_data = true;
//    m_adc_pid_param_data.additional_gain 
//      = m_eth_data.adc_pid_additional_gain;
//    m_eeprom_data.adc_pid_additional_gain 
//      = m_eth_data.adc_pid_additional_gain;
//  }
//  if (m_eth_data.adc_pid_ref != m_adc_pid_param_data.ref) {
//    new_data = true;
//    m_adc_pid_param_data.ref = m_eth_data.adc_pid_ref;
//    m_eeprom_data.adc_pid_ref = m_eth_data.adc_pid_ref;
//  }
//  if (m_eth_data.adc_pid_cont_cnv_cnt 
//      != m_adc_pid_param_data.cont_cnv_cnt) {
//    new_data = true;
//    m_adc_pid_param_data.cont_cnv_cnt 
//      = m_eth_data.adc_pid_cont_cnv_cnt;
//    m_eeprom_data.adc_pid_cont_cnv_cnt 
//      = m_eth_data.adc_pid_cont_cnv_cnt;
//  }
//  if (m_eth_data.adc_pid_impf_iterations_cnt 
//      != m_adc_pid_param_data.impf_iterations_cnt) {
//    new_data = true;
//    m_adc_pid_param_data.impf_iterations_cnt 
//      = m_eth_data.adc_pid_impf_iterations_cnt;
//    m_eeprom_data.adc_pid_impf_iterations_cnt 
//      = m_eth_data.adc_pid_impf_iterations_cnt;
//  }
//  if (m_eth_data.adc_pid_impf_type != m_adc_pid_param_data.impf_type) {
//    new_data = true;
//    m_adc_pid_param_data.impf_type 
//      = convert_to_impf_type(m_eth_data.adc_pid_impf_type);
//    m_eeprom_data.adc_pid_impf_type = m_eth_data.adc_pid_impf_type;
//  }
//  if (m_eth_data.adc_pid_cont_mode != m_adc_pid_param_data.cont_mode) {
//    new_data = true;
//    m_adc_pid_param_data.cont_mode 
//      = convert_to_cont_mode(m_eth_data.adc_pid_cont_mode);
//    m_eeprom_data.adc_pid_cont_mode = m_eth_data.adc_pid_cont_mode;
//  }
//  if (m_eth_data.adc_pid_cont_sko != m_adc_pid_param_data.cont_sko) {
//    new_data = true;
//    m_adc_pid_param_data.cont_sko = m_eth_data.adc_pid_cont_sko;
//    m_eeprom_data.adc_pid_cont_sko = m_eth_data.adc_pid_cont_sko;
//  }
//  return new_data;
//}
//
//bool hrm::app_t::reg_params_recieve_and_save_pid()
//{
//  bool new_data = false;
//  if (m_eth_data.elab_pid_sko_meas_time 
//      != m_eeprom_data.elab_pid_sko_meas_time) {
//    new_data = true;
//    m_elab_pid_sko_meas_time = m_eth_data.elab_pid_sko_meas_time;
//    m_eeprom_data.elab_pid_sko_meas_time = m_eth_data.elab_pid_sko_meas_time;
//  }
//  if (m_eth_data.elab_pid_kp != m_elab_pid_kp) {
//    new_data = true;
//    m_elab_pid_kp = m_eth_data.elab_pid_kp;
//    m_eeprom_data.elab_pid_kp = m_eth_data.elab_pid_kp;
//  }
//  if (m_eth_data.elab_pid_ki != m_elab_pid_ki) {
//    new_data = true;
//    m_elab_pid_ki = m_eth_data.elab_pid_ki;
//    m_eeprom_data.elab_pid_ki = m_eth_data.elab_pid_ki;
//  }
//  if (m_eth_data.elab_pid_kd != m_elab_pid_kd) {
//    new_data = true;
//    m_elab_pid_kd = m_eth_data.elab_pid_kd;
//    m_eeprom_data.elab_pid_kd = m_eth_data.elab_pid_kd;
//  }
//  if (m_eth_data.elab_iso_k != m_elab_iso_k) {
//    new_data = true;
//    m_elab_iso_k = m_eth_data.elab_iso_k;
//    m_eeprom_data.elab_iso_k = m_eth_data.elab_iso_k;
//  }
//  if (m_eth_data.elab_iso_t != m_elab_iso_t) {
//    new_data = true;
//    m_elab_iso_t = m_eth_data.elab_iso_t;
//    m_eeprom_data.elab_iso_t = m_eth_data.elab_iso_t;
//  }
//  if (m_eth_data.elab_pid_fade_t != m_elab_pid_fade_t) {
//    new_data = true;
//    m_elab_pid_fade_t = m_eth_data.elab_pid_fade_t;
//    m_eeprom_data.elab_pid_fade_t = m_eth_data.elab_pid_fade_t;
//  }
//  if (m_eth_data.elab_pid_avg_cnt != m_elab_pid_avg_cnt) {
//    new_data = true;
//    m_elab_pid_avg_cnt = m_eth_data.elab_pid_avg_cnt;
//    m_eeprom_data.elab_pid_avg_cnt = m_eth_data.elab_pid_avg_cnt;
//  }
//  if (m_eth_data.elab_pid_target_sko != m_elab_pid_target_sko) {
//    new_data = true;
//    m_elab_pid_target_sko = m_eth_data.elab_pid_target_sko;
//    m_elab_pid_target_sko_norm = norm(m_elab_pid_target_sko);
//    m_eeprom_data.elab_pid_target_sko_norm = m_elab_pid_target_sko_norm;
//    m_eth_data.elab_pid_target_sko_norm = m_elab_pid_target_sko_norm;
//  }
//  if (m_eth_data.elab_pid_target_sko_norm != m_elab_pid_target_sko_norm) {
//    new_data = true;
//    m_elab_pid_target_sko_norm = m_eth_data.elab_pid_target_sko_norm;
//    m_eeprom_data.elab_pid_target_sko_norm = m_elab_pid_target_sko_norm;
//    m_elab_pid_target_sko = denorm(m_elab_pid_target_sko_norm);
//    m_eth_data.elab_pid_target_sko = m_elab_pid_target_sko;
//  }
//  if (m_eth_data.elab_pid_ref != m_eeprom_data.elab_pid_ref) {
//    new_data = true;
//    m_eeprom_data.elab_pid_ref = m_eth_data.elab_pid_ref;
//    m_elab_pid_ref = m_eth_data.elab_pid_ref;
//  }
//  return new_data;
//}
//
//bool hrm::app_t::adc_params_recieve_and_save_manual()
//{
//  bool new_data = false;
//  if (m_eth_data.adc_manual_gain != m_adc_manual_param_data.gain) {
//    new_data = true;
//    m_adc_manual_param_data.gain = m_eth_data.adc_manual_gain;
//    m_eeprom_data.adc_manual_gain = m_eth_data.adc_manual_gain;
//  }
//  if (m_eth_data.adc_manual_filter != m_adc_manual_param_data.filter) {
//    new_data = true;
//    m_adc_manual_param_data.filter = m_eth_data.adc_manual_filter;
//    m_eeprom_data.adc_manual_filter = m_eth_data.adc_manual_filter;
//  }
//  if (m_eth_data.adc_manual_channel != m_adc_manual_param_data.channel) {
//    new_data = true;
//    m_adc_manual_param_data.channel = m_eth_data.adc_manual_channel;
//    m_eeprom_data.adc_manual_channel = m_eth_data.adc_manual_channel;
//  }
//  if (m_eth_data.adc_manual_cnv_cnt != m_adc_manual_param_data.cnv_cnt) {
//    new_data = true;
//    m_adc_manual_param_data.cnv_cnt = m_eth_data.adc_manual_cnv_cnt;
//    m_eeprom_data.adc_manual_cnv_cnt = m_eth_data.adc_manual_cnv_cnt;
//  }
//  if (m_eth_data.adc_manual_additional_gain 
//      != m_adc_manual_param_data.additional_gain) {
//    new_data = true;
//    m_adc_manual_param_data.additional_gain 
//      = m_eth_data.adc_manual_additional_gain;
//    m_eeprom_data.adc_manual_additional_gain 
//      = m_eth_data.adc_manual_additional_gain;
//  }
//  if (m_eth_data.adc_manual_ref != m_adc_manual_param_data.ref) {
//    new_data = true;
//    m_adc_manual_param_data.ref = m_eth_data.adc_manual_ref;
//    m_eeprom_data.adc_manual_ref = m_eth_data.adc_manual_ref;
//  }
//  if (m_eth_data.adc_manual_cont_cnv_cnt 
//      != m_adc_manual_param_data.cont_cnv_cnt) {
//    new_data = true;
//    m_adc_manual_param_data.cont_cnv_cnt 
//      = m_eth_data.adc_manual_cont_cnv_cnt;
//    m_eeprom_data.adc_manual_cont_cnv_cnt 
//      = m_eth_data.adc_manual_cont_cnv_cnt;
//  }
//  if (m_eth_data.adc_manual_impf_iterations_cnt 
//      != m_adc_manual_param_data.impf_iterations_cnt) {
//    new_data = true;
//    m_adc_manual_param_data.impf_iterations_cnt 
//      = m_eth_data.adc_manual_impf_iterations_cnt;
//    m_eeprom_data.adc_manual_impf_iterations_cnt 
//      = m_eth_data.adc_manual_impf_iterations_cnt;
//  }
//  if (m_eth_data.adc_manual_impf_type != m_adc_manual_param_data.impf_type) {
//    new_data = true;
//    m_adc_manual_param_data.impf_type 
//      = convert_to_impf_type(m_eth_data.adc_manual_impf_type);
//    m_eeprom_data.adc_manual_impf_type = m_eth_data.adc_manual_impf_type;
//  }
//  if (m_eth_data.adc_manual_cont_mode != m_adc_manual_param_data.cont_mode) {
//    new_data = true;
//    m_adc_manual_param_data.cont_mode 
//      = convert_to_cont_mode(m_eth_data.adc_manual_cont_mode);
//    m_eeprom_data.adc_manual_cont_mode = m_eth_data.adc_manual_cont_mode;
//  }
//  if (m_eth_data.adc_manual_cont_sko != m_adc_manual_param_data.cont_sko) {
//    new_data = true;
//    m_adc_manual_param_data.cont_sko = m_eth_data.adc_manual_cont_sko;
//    m_eeprom_data.adc_manual_cont_sko = m_eth_data.adc_manual_cont_sko;
//  }
//  return new_data;
//}
//
//bool hrm::app_t::adc_params_recieve_and_save_balance()
//{
//  bool new_data = false;
//  if (m_eth_data.adc_balance_gain != m_adc_balance_param_data.gain) {
//    new_data = true;
//    m_adc_balance_param_data.gain = m_eth_data.adc_balance_gain;
//    m_eeprom_data.adc_balance_gain = m_eth_data.adc_balance_gain;
//  }
//  if (m_eth_data.adc_balance_filter != m_adc_balance_param_data.filter) {
//    new_data = true;
//    m_adc_balance_param_data.filter = m_eth_data.adc_balance_filter;
//    m_eeprom_data.adc_balance_filter = m_eth_data.adc_balance_filter;
//  }
//  if (m_eth_data.adc_balance_channel != m_adc_balance_param_data.channel) {
//    new_data = true;
//    m_adc_balance_param_data.channel = m_eth_data.adc_balance_channel;
//    m_eeprom_data.adc_balance_channel = m_eth_data.adc_balance_channel;
//  }
//  if (m_eth_data.adc_balance_cnv_cnt != m_adc_balance_param_data.cnv_cnt) {
//    new_data = true;
//    m_adc_balance_param_data.cnv_cnt = m_eth_data.adc_balance_cnv_cnt;
//    m_eeprom_data.adc_balance_cnv_cnt = m_eth_data.adc_balance_cnv_cnt;
//  }
//  if (m_eth_data.adc_balance_additional_gain 
//      != m_adc_balance_param_data.additional_gain) {
//    new_data = true;
//    m_adc_balance_param_data.additional_gain 
//      = m_eth_data.adc_balance_additional_gain;
//    m_eeprom_data.adc_balance_additional_gain 
//      = m_eth_data.adc_balance_additional_gain;
//  }
//  if (m_eth_data.adc_balance_ref != m_adc_balance_param_data.ref) {
//    new_data = true;
//    m_adc_balance_param_data.ref = m_eth_data.adc_balance_ref;
//    m_eeprom_data.adc_balance_ref = m_eth_data.adc_balance_ref;
//  }
//  if (m_eth_data.adc_balance_cont_cnv_cnt 
//      != m_adc_balance_param_data.cont_cnv_cnt) {
//    new_data = true;
//    m_adc_balance_param_data.cont_cnv_cnt 
//      = m_eth_data.adc_balance_cont_cnv_cnt;
//    m_eeprom_data.adc_balance_cont_cnv_cnt 
//      = m_eth_data.adc_balance_cont_cnv_cnt;
//  }
//  if (m_eth_data.adc_balance_impf_iterations_cnt 
//      != m_adc_balance_param_data.impf_iterations_cnt) {
//    new_data = true;
//    m_adc_balance_param_data.impf_iterations_cnt 
//      = m_eth_data.adc_balance_impf_iterations_cnt;
//    m_eeprom_data.adc_balance_impf_iterations_cnt 
//      = m_eth_data.adc_balance_impf_iterations_cnt;
//  }
//  if (m_eth_data.adc_balance_impf_type != m_adc_balance_param_data.impf_type) {
//    new_data = true;
//    m_adc_balance_param_data.impf_type 
//      = convert_to_impf_type(m_eth_data.adc_balance_impf_type);
//    m_eeprom_data.adc_balance_impf_type = m_eth_data.adc_balance_impf_type;
//  }
//  if (m_eth_data.adc_balance_cont_mode != m_adc_balance_param_data.cont_mode) {
//    new_data = true;
//    m_adc_balance_param_data.cont_mode 
//      = convert_to_cont_mode(m_eth_data.adc_balance_cont_mode);
//    m_eeprom_data.adc_balance_cont_mode = m_eth_data.adc_balance_cont_mode;
//  }
//  if (m_eth_data.adc_balance_cont_sko != m_adc_balance_param_data.cont_sko) {
//    new_data = true;
//    m_adc_balance_param_data.cont_sko = m_eth_data.adc_balance_cont_sko;
//    m_eeprom_data.adc_balance_cont_sko = m_eth_data.adc_balance_cont_sko;
//  }
//  return new_data;
//}
//
//bool hrm::app_t::adc_params_recieve_and_save_elab()
//{
//  bool new_data = false;
//  if (m_eth_data.adc_elab_gain != m_adc_elab_param_data.gain) {
//    new_data = true;
//    m_adc_elab_param_data.gain = m_eth_data.adc_elab_gain;
//    m_eeprom_data.adc_elab_gain = m_eth_data.adc_elab_gain;
//  }
//  if (m_eth_data.adc_elab_filter != m_adc_elab_param_data.filter) {
//    new_data = true;
//    m_adc_elab_param_data.filter = m_eth_data.adc_elab_filter;
//    m_eeprom_data.adc_elab_filter = m_eth_data.adc_elab_filter;
//  }
//  if (m_eth_data.adc_elab_channel != m_adc_elab_param_data.channel) {
//    new_data = true;
//    m_adc_elab_param_data.channel = m_eth_data.adc_elab_channel;
//    m_eeprom_data.adc_elab_channel = m_eth_data.adc_elab_channel;
//  }
//  if (m_eth_data.adc_elab_cnv_cnt != m_adc_elab_param_data.cnv_cnt) {
//    new_data = true;
//    m_adc_elab_param_data.cnv_cnt = m_eth_data.adc_elab_cnv_cnt;
//    m_eeprom_data.adc_elab_cnv_cnt = m_eth_data.adc_elab_cnv_cnt;
//  }
//  if (m_eth_data.adc_elab_additional_gain 
//      != m_adc_elab_param_data.additional_gain) {
//    new_data = true;
//    m_adc_elab_param_data.additional_gain 
//      = m_eth_data.adc_elab_additional_gain;
//    m_eeprom_data.adc_elab_additional_gain 
//      = m_eth_data.adc_elab_additional_gain;
//  }
//  if (m_eth_data.adc_elab_ref != m_adc_elab_param_data.ref) {
//    new_data = true;
//    m_adc_elab_param_data.ref = m_eth_data.adc_elab_ref;
//    m_eeprom_data.adc_elab_ref = m_eth_data.adc_elab_ref;
//  }
//  if (m_eth_data.adc_elab_cont_cnv_cnt 
//      != m_adc_elab_param_data.cont_cnv_cnt) {
//    new_data = true;
//    m_adc_elab_param_data.cont_cnv_cnt 
//      = m_eth_data.adc_elab_cont_cnv_cnt;
//    m_eeprom_data.adc_elab_cont_cnv_cnt 
//      = m_eth_data.adc_elab_cont_cnv_cnt;
//  }
//  if (m_eth_data.adc_elab_impf_iterations_cnt 
//      != m_adc_elab_param_data.impf_iterations_cnt) {
//    new_data = true;
//    m_adc_elab_param_data.impf_iterations_cnt 
//      = m_eth_data.adc_elab_impf_iterations_cnt;
//    m_eeprom_data.adc_elab_impf_iterations_cnt 
//      = m_eth_data.adc_elab_impf_iterations_cnt;
//  }
//  if (m_eth_data.adc_elab_impf_type != m_adc_elab_param_data.impf_type) {
//    new_data = true;
//    m_adc_elab_param_data.impf_type 
//      = convert_to_impf_type(m_eth_data.adc_elab_impf_type);
//    m_eeprom_data.adc_elab_impf_type = m_eth_data.adc_elab_impf_type;
//  }
//  if (m_eth_data.adc_elab_cont_mode != m_adc_elab_param_data.cont_mode) {
//    new_data = true;
//    m_adc_elab_param_data.cont_mode 
//      = convert_to_cont_mode(m_eth_data.adc_elab_cont_mode);
//    m_eeprom_data.adc_elab_cont_mode = m_eth_data.adc_elab_cont_mode;
//  }
//  if (m_eth_data.adc_elab_cont_sko != m_adc_elab_param_data.cont_sko) {
//    new_data = true;
//    m_adc_elab_param_data.cont_sko = m_eth_data.adc_elab_cont_sko;
//    m_eeprom_data.adc_elab_cont_sko = m_eth_data.adc_elab_cont_sko;
//  }
//  return new_data;
//}
//
//hrm::adc_value_t hrm::app_t::adc_vx_to_th(adc_value_t a_voltage)
//{
//  return (a_voltage - 0.4) / 0.0195;
//}

void hrm::app_t::show_experiment_parameters()
{
  irs::mlog() << irsm("----------- Параметры эксперимента -----------");
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("no_prot ");
  irs::mlog() << setw(7) << left << m_no_prot;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_gain ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_elab_param_data.gain);
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_gain ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_balance_param_data.gain);
  
  irs::mlog() << endl;
 
  irs::mlog() << setprecision(0);
  irs::mlog() << setw(18) << left << irsm("dac_pause_ms ");
  irs::mlog() << setw(7) << left;// << setprecision(0);
//  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_dac_after_pause);
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_filter ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_elab_param_data.filter);
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_filter ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_balance_param_data.filter);
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("relay_pause_ms ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_relay_after_pause);
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_cnv_cnt ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_elab_param_data.cnv_cnt;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_cnv_cnt ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_balance_param_data.cnv_cnt;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("prepare_pause ");
  irs::mlog() << setw(7) << left << m_prepare_pause;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_cnv_cnt ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_elab_param_data.cont_cnv_cnt;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_cont_cnv_cnt ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_balance_param_data.cont_cnv_cnt;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("dac_elab_pause_ms ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_dac_elab_pause);
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_impf_iterations_cnt ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_elab_param_data.impf_iterations_cnt;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_impf_iterations_cnt ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_balance_param_data.impf_iterations_cnt;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("elab_mode ");
  irs::mlog() << setw(7) << left << m_elab_mode;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_impf_type ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_elab_param_data.impf_type;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_impf_type ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_balance_param_data.impf_type;
  
  irs::mlog() << endl;
  
  //irs::mlog() << setw(25) << left << irsm(". .");
  irs::mlog() << setw(18) << left << irsm("prev_exp_time ");
  irs::mlog() << setw(7) << left << m_eth_data.prev_exp_time;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_mode ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_elab_param_data.cont_mode;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_cont_mode ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_balance_param_data.cont_mode;
  
  irs::mlog() << endl;
  
  irs::mlog() << setprecision(1);
  
  irs::mlog() << setw(18) << left << irsm("adc_max_value_prot ");
//  irs::mlog() << setw(7) << left << m_adc_max_value_prot;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_sko ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_elab_param_data.cont_sko * 1.0e6;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_cont_sko ");
  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_balance_param_data.cont_sko * 1.0e6;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("adc_max_value_no_prot ");
//  irs::mlog() << setw(7) << left << m_adc_max_value_no_prot;
  
  irs::mlog() << setw(32) << left << irsm("adc_adaptive_elab_sko ");
//  irs::mlog() << setw(7) << left << m_adc_adaptive_elab_param_data.cont_sko * 1.0e6;
  
  irs::mlog() << setw(32) << left << irsm("adc_adaptive_balance_sko ");
//  irs::mlog() << setw(7) << left << m_adc_adaptive_balance_param_data.cont_sko * 1.0e6;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("use_adc_adaptive_sko ");
  irs::mlog() << setw(7) << left << m_eth_data.use_adc_adaptive_sko;
  
  irs::mlog() << setprecision(2);
  
  irs::mlog() << setw(32) << left << irsm("adaptive_sko_elab_multiplier ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_eth_data.adaptive_sko_elab_multiplier;
  
  irs::mlog() << setw(32) << left << irsm("adaptive_sko_balance_multiplier ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_eth_data.adaptive_sko_balance_multiplier;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("bac_old_coefficient ");
//  irs::mlog() << setw(7) << left << m_bac_old_coefficient;
  
  irs::mlog() << setw(28) << left << irsm("bac_new_coefficient ");
//  irs::mlog() << setw(7) << left << m_bac_new_coefficient;
  
  irs::mlog() << fixed;
  irs::mlog() << defaultfloat << setprecision(8);
  irs::mlog() << setw(28) << left << irsm("dac_hv_correction ");
  irs::mlog() << setw(8) << left;
//  irs::mlog() << m_dac_hv_correction;
  
  irs::mlog() << endl;
}

//void hrm::app_t::show_experiment_parameters_pid()
//{
//  irs::mlog() << irsm("----------- Параметры эксперимента -----------");
//  irs::mlog() << endl;
//  
//  irs::mlog() << setw(18) << left << irsm("no_prot ");
//  irs::mlog() << setw(7) << left << m_no_prot;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_gain ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_pid_param_data.gain);
//  
//  irs::mlog() << setw(32) << left << irsm("elab_pid_sko_meas_time ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_elab_pid_sko_meas_time);
//  
//  irs::mlog() << endl;
// 
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("dac_pause_ms ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_dac_after_pause);
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_filter ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_pid_param_data.filter);
//  
//  irs::mlog() << setprecision(2);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_kp ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_kp;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("relay_pause_ms ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_relay_after_pause);
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_cnv_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.cnv_cnt;
//  
//  irs::mlog() << setprecision(2);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_ki ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_ki;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("prepare_pause ");
//  irs::mlog() << setw(7) << left << m_prepare_pause;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(32) << left << irsm("adc_pid_cont_cnv_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.cont_cnv_cnt;
//  
//  irs::mlog() << setprecision(2);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_kd ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_kd;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("dac_elab_pause_ms ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_dac_elab_pause);
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(32) << left << irsm("adc_pid_impf_iterations_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.impf_iterations_cnt;
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(32) << left << irsm("elab_iso_k ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_iso_k;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("elab_mode ");
//  irs::mlog() << setw(7) << left << m_elab_mode;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_impf_type ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.impf_type;
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(32) << left << irsm("elab_iso_t ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_iso_t;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("prev_exp_time ");
//  irs::mlog() << setw(7) << left << m_eth_data.prev_exp_time;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_cont_mode ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.cont_mode;
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_fade_t ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_fade_t;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(1);
//  
//  irs::mlog() << setw(18) << left << irsm("adc_max_value_prot ");
//  irs::mlog() << setw(7) << left << m_adc_max_value_prot;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_cont_sko ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.cont_sko * 1.0e6;
//  
//  irs::mlog() << setw(32) << left << irsm("elab_pid_avg_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_avg_cnt;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setw(18) << left << irsm("adc_max_value_no_prot ");
//  irs::mlog() << setw(7) << left << m_adc_max_value_no_prot;
//  
//  irs::mlog() << setw(32) << left << irsm("pid_ready_condition ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << pid_ready_condition_to_u32(m_pid_ready_condition);
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_target_sko ");
//  irs::mlog() << setw(7) << left << m_elab_pid_target_sko;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << defaultfloat;
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(18) << left << irsm("bac_old_coefficient ");
//  irs::mlog() << setw(7) << left << m_bac_old_coefficient;
//  
//  irs::mlog() << defaultfloat;
//  
//  irs::mlog() << setw(30) << left << irsm("elab_pid_ref ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_ref;
//  
//  irs::mlog() << fixed << setprecision(2);
//  irs::mlog() << setw(29) << left << irsm("pid_sensivity ");
//  irs::mlog() << setw(10) << left;
//  irs::mlog() << m_pid_sensivity;
//  
//  irs::mlog() << fixed;
//  irs::mlog() << defaultfloat << setprecision(8);
//  irs::mlog() << setw(28) << left << irsm("dac_hv_correction ");
//  irs::mlog() << setw(8) << left;
//  irs::mlog() << m_dac_hv_correction;
//    
//  irs::mlog() << endl;
//  
//  irs::mlog() << defaultfloat;
//  
//  irs::mlog() << setw(18) << left << irsm("meas_sensivity ");
//  irs::mlog() << setw(7) << left << m_pid_meas_sensivity;
//  
//  irs::mlog() << setw(18) << left << irsm("imm_coef ");
//  irs::mlog() << setw(7) << left << m_imm_coef;
//  
//  irs::mlog() << endl;
//}
//
//void hrm::app_t::show_experiment_parameters_pid_linear()
//{
//  irs::mlog() << irsm("----------- Параметры эксперимента -----------");
//  irs::mlog() << endl;
//  
//  irs::mlog() << setw(18) << left << irsm("no_prot ");
//  irs::mlog() << setw(7) << left << m_no_prot;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_gain ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_pid_param_data.gain);
//  
//  irs::mlog() << setw(32) << left << irsm("adc_elab_gain ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_adaptive_elab_param_data.gain);
//  
//  irs::mlog() << setw(32) << left << irsm("elab_pid_sko_meas_time ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_elab_pid_sko_meas_time);
//  
//  irs::mlog() << endl;
// 
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("dac_pause_ms ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_dac_after_pause);
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_filter ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_pid_param_data.filter);
//  
//  irs::mlog() << setw(32) << left << irsm("adc_elab_filter ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << static_cast<int>(m_adc_adaptive_elab_param_data.filter);
//  
//  irs::mlog() << setprecision(2);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_kp ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_kp;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("relay_pause_ms ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_relay_after_pause);
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_cnv_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.cnv_cnt;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_elab_cnv_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_adaptive_elab_param_data.cnv_cnt;
//  
//  irs::mlog() << setprecision(2);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_ki ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_ki;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("prepare_pause ");
//  irs::mlog() << setw(7) << left << m_prepare_pause;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(32) << left << irsm("adc_pid_cont_cnv_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.cont_cnv_cnt;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_cnv_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_adaptive_elab_param_data.cont_cnv_cnt;
//  
//  irs::mlog() << setprecision(2);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_kd ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_kd;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("dac_elab_pause_ms ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_dac_elab_pause);
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(32) << left << irsm("adc_pid_impf_iterations_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.impf_iterations_cnt;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(32) << left << irsm("adc_elab_impf_iterations_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_adaptive_elab_param_data.impf_iterations_cnt;
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(32) << left << irsm("elab_iso_k ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_iso_k;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("elab_mode ");
//  irs::mlog() << setw(7) << left << m_elab_mode;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_impf_type ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.impf_type;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_elab_impf_type ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_adaptive_elab_param_data.impf_type;
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(32) << left << irsm("elab_iso_t ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_iso_t;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(0);
//  irs::mlog() << setw(18) << left << irsm("prev_exp_time ");
//  irs::mlog() << setw(7) << left << m_eth_data.prev_exp_time;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_cont_mode ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.cont_mode;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_mode ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_adaptive_elab_param_data.cont_mode;
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_fade_t ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_fade_t;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setprecision(1);
//  
//  irs::mlog() << setw(18) << left << irsm("adc_max_value_prot ");
//  irs::mlog() << setw(7) << left << m_adc_max_value_prot;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_pid_cont_sko ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_pid_param_data.cont_sko * 1.0e6;
//  
//  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_sko ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_adc_adaptive_elab_param_data.cont_sko * 1.0e6;
//  
//  irs::mlog() << setw(32) << left << irsm("elab_pid_avg_cnt ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_avg_cnt;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << setw(18) << left << irsm("adc_max_value_no_prot ");
//  irs::mlog() << setw(7) << left << m_adc_max_value_no_prot;
//  
//  irs::mlog() << setw(27) << left << irsm("pid_ready_condition ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << pid_ready_condition_to_u32(m_pid_ready_condition);
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(32) << left << irsm("elab_pid_target_sko ");
//  irs::mlog() << setw(7) << left << m_elab_pid_target_sko;
//  
//  irs::mlog() << endl;
//  
//  irs::mlog() << defaultfloat;
//  irs::mlog() << setprecision(3);
//  irs::mlog() << setw(18) << left << irsm("bac_old_coefficient ");
//  irs::mlog() << setw(7) << left << m_bac_old_coefficient;
//  
//  irs::mlog() << defaultfloat;
//  
//  irs::mlog() << setw(30) << left << irsm("elab_pid_ref ");
//  irs::mlog() << setw(7) << left;
//  irs::mlog() << m_elab_pid_ref;
//  
//  irs::mlog() << fixed << setprecision(2);
//  irs::mlog() << setw(29) << left << irsm("pid_sensivity ");
//  irs::mlog() << setw(10) << left;
//  irs::mlog() << m_pid_sensivity;
//    
//  irs::mlog() << endl;
//  
//  irs::mlog() << defaultfloat;
//  
//  irs::mlog() << setw(18) << left << irsm("meas_sensivity ");
//  irs::mlog() << setw(7) << left << m_pid_meas_sensivity;
//  
//  irs::mlog() << setw(18) << left << irsm("imm_coef ");
//  irs::mlog() << setw(7) << left << m_imm_coef;
//  
//  irs::mlog() << fixed;
//  irs::mlog() << defaultfloat << setprecision(8);
//  irs::mlog() << setw(28) << left << irsm("dac_hv_correction ");
//  irs::mlog() << setw(8) << left;
//  irs::mlog() << m_dac_hv_correction;
//  
//  irs::mlog() << endl;
//}

void hrm::app_t::show_last_result()
{
  irs::mlog() << fixed << endl;
  irs::mlog() << irsm("----------- Результат серии экспериментов");
  irs::mlog() << irsm(" -----------") << endl;
  irs::mlog() << irsm("№    ");
  if (m_eth_data.show_old_result) {
    irs::mlog() << irsm("OLD            ");
  }
  if (m_eth_data.show_new_result) {
    irs::mlog() << irsm("NEW            ");
  }
  if (m_eth_data.show_old_unc_result) {
    irs::mlog() << irsm("OLD_UNCORRECT  ");
  }
  if (m_eth_data.show_new_unc_result) {
    irs::mlog() << irsm("NEW_UNCORRECT  ");
  }
  if (m_eth_data.show_th_ext) {
    irs::mlog() << irsm("t°ext  ");
  }
  if (m_eth_data.show_th_dac) {
    irs::mlog() << irsm("t°dac  ");
  }
  if (m_eth_data.show_th_adc) {
    irs::mlog() << irsm("t°adc ");
  }
  if (m_eth_data.show_th_ldo) {
    irs::mlog() << irsm("t°ldo ");
  }
  if (m_eth_data.show_exp_time) {
    irs::mlog() << irsm("Texp ");
  }
  if (m_eth_data.show_target_sko) {
    irs::mlog() << irsm("tSKO   ");
  }
  if (m_eth_data.show_target_balance_sko) {
    irs::mlog() << irsm("tSKOb   ");
  }
  if (m_eth_data.show_target_elab_sko) {
    irs::mlog() << irsm("tSKOe   ");
  }
  if (m_eth_data.show_pid_target_adc_sko) {
    irs::mlog() << irsm("tSKO-   ");
    irs::mlog() << irsm("tSKO+   ");
  }
  if (m_eth_data.show_pid_dac_sko) {
    irs::mlog() << irsm("dSKO-   ");
    irs::mlog() << irsm("dSKO+   ");
  }
  if (m_eth_data.show_pid_n) {
    irs::mlog() << irsm("N-      ");
    irs::mlog() << irsm("N+      ");
  }
  if (m_eth_data.show_codes) {
    irs::mlog() << irsm("D-   ");
    irs::mlog() << irsm("D+");
  }
  if (m_elab_mode == em_analog) {
    irs::mlog() << irsm("Vc-     ");
    irs::mlog() << irsm("SKOc- ");
    irs::mlog() << irsm("Nc- ");
    irs::mlog() << irsm("Vs-     ");
    irs::mlog() << irsm("SKOs- ");
    irs::mlog() << irsm("Ns- ");
    irs::mlog() << irsm("Vc+     ");
    irs::mlog() << irsm("SKOc+ ");
    irs::mlog() << irsm("Nc+ ");
    irs::mlog() << irsm("Vs+     ");
    irs::mlog() << irsm("SKOs+ ");
    irs::mlog() << irsm("Ns+ ");
    irs::mlog() << irsm("ERR");
  }
  irs::mlog() << endl;
  
  for (size_t i = 0; i < m_exp_vector.size(); i++) {
    irs::mlog() << setprecision(10);
    irs::mlog() << setw(3) << i + 1;irs::mlog() << irsm(" ");
    irs::mlog() << setprecision(12);
    if (m_eth_data.show_old_result) {
      irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].result_old;
    }
//    if (m_eth_data.show_new_result) {
//      irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].result_new;
//    }
//    if (m_eth_data.show_old_unc_result) {
//      irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].result_old_uncorrect;
//    }
//    if (m_eth_data.show_new_unc_result) {
//      irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].result_new_uncorrect;
//    }
    irs::mlog() << setprecision(1);
    if (m_eth_data.show_th_ext) {
      irs::mlog() << irsm(" ") << setw(3) << m_exp_vector[i].temperature_ext;
    }
    if (m_eth_data.show_th_dac) {
      irs::mlog() << irsm("   ") << setw(3) << m_exp_vector[i].temperature_dac;
    }
    if (m_eth_data.show_th_adc) {
      irs::mlog() << irsm("   ") << setw(3) << m_exp_vector[i].temperature_adc;
    }
//    if (m_eth_data.show_th_ldo) {
//      irs::mlog() << irsm("   ") << setw(3) << m_exp_vector[i].temperature_ldo;
//    }
    irs::mlog() << setprecision(0);
    if (m_eth_data.show_exp_time) {
      irs::mlog() << irsm("   ") << setw(5) << m_exp_vector[i].exp_time;
    }
    irs::mlog() << setprecision(2);
    if (m_eth_data.show_target_sko) {
      irs::mlog() << irsm(" ") << setw(7) << m_exp_vector[i].target_sko;
    }
    if (m_eth_data.show_target_balance_sko) {
      irs::mlog() << irsm(" ") << setw(7) << m_exp_vector[i].target_balance_sko;
    }
    if (m_eth_data.show_target_elab_sko) {
      irs::mlog() << irsm(" ") << setw(7) << m_exp_vector[i].target_elab_sko;
    }
    if (m_eth_data.show_pid_target_adc_sko) {
      irs::mlog() << irsm(" ") << setw(7) << m_exp_vector[i].target_sko_adc_neg;
      irs::mlog() << irsm(" ") << setw(7) << m_exp_vector[i].target_sko_adc_pos;
    }
    irs::mlog() << setprecision(4);
    if (m_eth_data.show_pid_dac_sko) {
      irs::mlog() << irsm(" ") << setw(7) << m_exp_vector[i].target_sko_dac_neg;
      irs::mlog() << irsm(" ") << setw(7) << m_exp_vector[i].target_sko_dac_pos;
    }
    if (m_eth_data.show_pid_n) {
      irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].neg_n;
      irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].pos_n;
    }
    if (m_eth_data.show_codes) {
      irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].ch_code;
      irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].et_code;
    }
    if (m_elab_mode == em_analog) {
      irs::mlog() << setprecision(7);
      irs::mlog() << irsm(" ") << setw(9) << m_exp_vector[i].coils_voltage_neg;
      irs::mlog() << setprecision(2);
      irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].coils_sko_neg;
      irs::mlog() << irsm(" ") << setw(3) << m_exp_vector[i].coils_n_neg;
      irs::mlog() << setprecision(7);
      irs::mlog() << irsm(" ") << setw(9) << m_exp_vector[i].source_voltage_neg;
      irs::mlog() << setprecision(2);
      irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].source_sko_neg;
      irs::mlog() << irsm(" ") << setw(3) << m_exp_vector[i].source_n_neg;
      irs::mlog() << setprecision(7);
      irs::mlog() << irsm(" ") << setw(9) << m_exp_vector[i].coils_voltage_pos;
      irs::mlog() << setprecision(2);
      irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].coils_sko_pos;
      irs::mlog() << irsm(" ") << setw(3) << m_exp_vector[i].coils_n_pos;
      irs::mlog() << setprecision(7);
      irs::mlog() << irsm(" ") << setw(9) << m_exp_vector[i].source_voltage_pos;
      irs::mlog() << setprecision(2);
      irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].source_sko_pos;
      irs::mlog() << irsm(" ") << setw(3) << m_exp_vector[i].source_n_pos;
      irs::mlog() << irsm(" ") << setw(3) << m_exp_vector[i].errors_cnt;
    }
    irs::mlog() << endl;
  }
  
  if (m_eth_data.show_intersections) {
    irs::mlog() << endl;
    irs::mlog() << irsm("----------- Пересечения серии экспериментов");
    irs::mlog() << irsm(" -----------") << endl;
    irs::mlog() << irsm("№   ");
    irs::mlog() << irsm("NEG           ");
    irs::mlog() << irsm("NEG_0         ");
    irs::mlog() << irsm("POS           ");
    irs::mlog() << irsm("POS_0         ");
    if (m_elab_mode != em_pid) {
      irs::mlog() << irsm("n0            ");
      irs::mlog() << irsm("num           ");
      irs::mlog() << irsm("den           ");
    }
    irs::mlog() << endl;
    irs::mlog() << left;
    
    for (size_t i = 0; i < m_exp_vector.size(); i++) {
      irs::mlog() << setw(3) << i + 1;
      irs::mlog() << irsm(" ");
      irs::mlog() << setprecision(5);
      irs::mlog() << setw(13) << m_exp_vector[i].ch_code;
      irs::mlog() << irsm(" ");
      irs::mlog() << setprecision(0);
      irs::mlog() << setw(13) << m_exp_vector[i].ch_balanced_code;
      irs::mlog() << irsm(" ");
      irs::mlog() << setprecision(5);
      irs::mlog() << setw(13) << m_exp_vector[i].et_code;
      irs::mlog() << setprecision(0);
      irs::mlog() << irsm(" ");
      irs::mlog() << setw(13) << m_exp_vector[i].et_balanced_code;
//      if (m_elab_mode != em_pid) {
//        irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].n0;
//        irs::mlog() << setprecision(3);
//        irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].num;
//        irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].den;
//      }
      irs::mlog() << endl;
    }
    
    irs::mlog() << setprecision(8);
    irs::mlog() << irsm("-------------------------------------------");
    irs::mlog() << irsm("--------------");
    irs::mlog() << endl;
  }
  show_experiment_parameters();
//  switch (m_elab_mode) {
//    case em_pid: show_experiment_parameters_pid(); break;
//    case em_pid_linear: show_experiment_parameters_pid_linear(); break;
//    default: show_experiment_parameters();
//  };
}

hrm::app_t::elab_mode_t hrm::app_t::convert_u8_to_elab_mode(irs_u8 a_mode)
{
  elab_mode_t return_mode = em_none;
  switch (a_mode) { 
    case 0: return_mode = em_linear;        break;
    case 1: return_mode = em_pid;           break;
    case 2: return_mode = em_fast_2points;  break;
    case 3: return_mode = em_none;          break;
    case 4: return_mode = em_pid_linear;    break;
    case 5: return_mode = em_analog;        break;
    default:return_mode = em_none;
  }
  return return_mode;
}

hrm::app_t::remaining_time_calculator_t::remaining_time_calculator_t():
  m_meas_prepare_time(0),
  m_meas_balance_time(0),
  m_meas_elab_time(0),
  m_balance_action(ba_prepare),
  m_remaining_time(0),
  m_current_time(0),
  m_current_percentage(0),
  m_prepare_pause(0),
  m_balance_points_count(0),
  m_balance_current_point(0)
{
}

void hrm::app_t::remaining_time_calculator_t::reset()
{
  m_balance_action = ba_prepare;
  m_meas_prepare_time = 0;
  m_meas_balance_time = 0;
  m_meas_elab_time = 0;
  m_remaining_time = 0;
  m_current_time = 0;
  m_current_percentage = 0;
  m_prepare_pause = 0;
  m_balance_points_count = max_balance_points_count;
  m_balance_current_point = 0;
  irs::mlog() << irsm("rem reset") << endl;
}

void hrm::app_t::remaining_time_calculator_t::start(irs_u32 a_prepare_pause)
{
  m_prepare_pause = a_prepare_pause;
  m_remaining_time = a_prepare_pause + default_exp_time;
  m_current_time = 0;
  m_current_percentage = 0;
  //irs::mlog() << irsm("rem start = ") << m_remaining_time << endl;
}

void hrm::app_t::remaining_time_calculator_t::secund_tick()
{
  //irs::mlog() << irsm("rem = ") << m_remaining_time;
  switch (m_balance_action) {
    case ba_idle: {
      //
      break;
    }
    case ba_prepare: {
      m_remaining_time = irs::bound<irs_u32>(m_remaining_time - 1,
        0, m_remaining_time);
      break;
    }
    case ba_prepare_pause: {
      m_current_time++;
      m_remaining_time = irs::bound<irs_u32>(m_remaining_time - 1,
        0, m_remaining_time);
      m_current_percentage = m_current_time * default_prepare_pause_percentage /
        m_prepare_pause;
      m_current_percentage = irs::bound<irs_u32>(m_current_percentage, 
        0, default_prepare_pause_percentage);
      break;
    }
    case ba_balance_neg: {
      m_current_time++;
      m_remaining_time = irs::bound<irs_u32>(m_remaining_time - 1,
        (default_exp_time * 3) / 4, m_remaining_time);
      m_current_percentage = min_balance_neg +
        m_balance_current_point * default_balance_percentage /
        m_balance_points_count;
      m_current_percentage = irs::bound<irs_u32>(m_current_percentage, 
        min_balance_neg, max_balance_neg);
      break;
    }
    case ba_elab_neg: {
      m_current_time++;
      m_remaining_time = irs::bound<irs_u32>(m_remaining_time - 1,
        ((default_exp_time * 2) / 4) + m_meas_balance_time, m_remaining_time);
      m_current_percentage = min_elab_neg +
        m_balance_current_point * default_elab_percentage /
        elab_points_count;
      m_current_percentage = irs::bound<irs_u32>(m_current_percentage, 
        min_elab_neg, max_elab_neg);
      break;
    }
    case ba_balance_pos: {
      m_current_time++;
      m_remaining_time = irs::bound<irs_u32>(m_remaining_time - 1,
        ((default_exp_time * 1) / 4) + m_meas_balance_time + m_meas_elab_time,
        m_remaining_time);
      m_current_percentage = min_balance_pos +
        m_balance_current_point * default_balance_percentage /
        m_balance_points_count;
      m_current_percentage = irs::bound<irs_u32>(m_current_percentage, 
        min_balance_pos, max_balance_pos);
      break;
    }
    case ba_elab_pos: {
      m_current_time++;
      m_remaining_time = irs::bound<irs_u32>(m_remaining_time - 1,
        m_meas_balance_time + m_meas_balance_time + m_meas_elab_time,
        m_remaining_time);
      m_current_percentage = min_elab_pos +
        m_balance_current_point * default_elab_percentage /
        elab_points_count;
      m_current_percentage = irs::bound<irs_u32>(m_current_percentage, 
        min_elab_pos, max_elab_pos);
      break;
    }
  }
  //irs::mlog() << irsm(" ") << m_remaining_time << endl;
}

irs_u32 hrm::app_t::remaining_time_calculator_t::get_remaining_time()
{
  return m_remaining_time;
}

irs_u32 hrm::app_t::remaining_time_calculator_t::get_percentage()
{
  return m_current_percentage;
}

void hrm::app_t::remaining_time_calculator_t::change_balance_action(
  balance_action_t a_balance_action)
{
  m_balance_action = a_balance_action;
  switch (m_balance_action) {
    case ba_idle: {
      //
      break;
    }
    case ba_prepare: {
      //
      break;
    }
    case ba_prepare_pause: {
      //
      break;
    }
    case ba_balance_neg: {
      m_meas_prepare_time = m_current_time;
      irs::mlog() << irsm("Измеренное время подготовки ");
      irs::mlog() << m_meas_prepare_time << irsm(" с") << endl;
      break;
    }
    case ba_elab_neg: {
      m_meas_balance_time = m_current_time - m_meas_prepare_time;
      irs::mlog() << irsm("Измеренное время уравновешивания ");
      irs::mlog() << m_meas_balance_time << irsm(" с") << endl;
      break;
    }
    case ba_balance_pos: {
      m_meas_elab_time = 
        m_current_time - m_meas_prepare_time - m_meas_balance_time;
      irs::mlog() << irsm("Измеренное время уточнения ");
      irs::mlog() << m_meas_elab_time << irsm(" с") << endl;
      break;
    }
    case ba_elab_pos: {
      //
      break;
    }
  }
}

void hrm::app_t::remaining_time_calculator_t::set_balance_points_count(
  size_t a_balance_points_count)
{
  m_balance_points_count = a_balance_points_count;
}

void hrm::app_t::remaining_time_calculator_t::set_current_balance_point(
  size_t a_point)
{
  m_balance_current_point = a_point;
}

//------------------------------------------------------------------------------

//hrm::app_t::adaptive_sko_calc_t::adaptive_sko_calc_t(eth_data_t& ap_eth_data, 
//  eeprom_data_t& ap_ee_data):
//  mp_eth_data(ap_eth_data),
//  mp_ee_data(ap_ee_data),
//  m_target_sko(0.0),
//  m_used(false),
//  m_target_balance_sko(0.0),
//  m_target_elab_sko(0.0),
//  m_adaptive_sko_balance_multiplier(0.0),
//  m_adaptive_sko_elab_multiplier(0.0),
//  m_sko_calc(default_len, default_len)
//{
//  mp_eth_data.use_adc_adaptive_sko = mp_ee_data.use_adc_adaptive_sko;
//  if (mp_eth_data.use_adc_adaptive_sko) {
//    m_used = true;
//  } else {
//    m_used = false;
//  }
//  m_adaptive_sko_balance_multiplier = 
//    mp_ee_data.adaptive_sko_balance_multiplier;
//  m_adaptive_sko_elab_multiplier = 
//    mp_ee_data.adaptive_sko_elab_multiplier;
//  mp_eth_data.adaptive_sko_balance_multiplier = 
//    mp_ee_data.adaptive_sko_balance_multiplier;
//  mp_eth_data.adaptive_sko_elab_multiplier = 
//    mp_ee_data.adaptive_sko_elab_multiplier;
//}
//
//void hrm::app_t::adaptive_sko_calc_t::reset(size_t a_len)
//{
//  if (mp_eth_data.use_adc_adaptive_sko) {
//    m_used = true;
//  } else {
//    m_used = false;
//  }
//  m_started = true;
//  m_target_sko = 0.0;
//  m_sko_calc.clear();
//  m_sko_calc.resize(a_len);
//}
//
//void hrm::app_t::adaptive_sko_calc_t::stop()
//{
//  m_started = false;
//}
//
//void hrm::app_t::adaptive_sko_calc_t::add(adc_value_t a_sko)
//{
//  if (m_started) {
//    m_sko_calc.add(a_sko);
//    m_target_sko = m_sko_calc.average();
//  }
//}
//
//hrm::adc_value_t hrm::app_t::adaptive_sko_calc_t::get_target_sko()
//{
//  return m_target_sko;
//}
//
//hrm::adc_value_t hrm::app_t::adaptive_sko_calc_t::get_target_balance_sko()
//{
//  return m_target_sko * m_adaptive_sko_balance_multiplier;
//}
//
//hrm::adc_value_t hrm::app_t::adaptive_sko_calc_t::get_target_elab_sko()
//{
//  return m_target_sko * m_adaptive_sko_elab_multiplier;
//}
//
//void hrm::app_t::adaptive_sko_calc_t::sync_parameters()
//{
//  if (mp_eth_data.use_adc_adaptive_sko != mp_ee_data.use_adc_adaptive_sko) {
//    mp_ee_data.use_adc_adaptive_sko = mp_eth_data.use_adc_adaptive_sko;
//    if (mp_eth_data.use_adc_adaptive_sko) {
//      m_used = true;
//    } else {
//      m_used = false;
//    }
//  }
//  if (mp_eth_data.adaptive_sko_elab_multiplier 
//      != mp_ee_data.adaptive_sko_elab_multiplier) {
//    m_adaptive_sko_elab_multiplier 
//      = static_cast<double>(mp_eth_data.adaptive_sko_elab_multiplier);
//    mp_ee_data.adaptive_sko_elab_multiplier 
//      = mp_eth_data.adaptive_sko_elab_multiplier;
//    
//    irs::mlog() << irsm("eth.adaptive_sko_elab_multiplier = ");
//    irs::mlog() << mp_eth_data.adaptive_sko_elab_multiplier << endl;
//    irs::mlog() << irsm("ee.adaptive_sko_elab_multiplier = ");
//    irs::mlog() << mp_ee_data.adaptive_sko_elab_multiplier << endl;
//    irs::mlog() << irsm("adaptive_sko_elab_multiplier = ");
//    irs::mlog() << m_adaptive_sko_elab_multiplier << endl;
////    irs::mlog() << irsm("TEST = ");
////    irs::mlog() << mp_eth_data.test << endl;
//  }
//  if (mp_eth_data.adaptive_sko_balance_multiplier 
//      != mp_ee_data.adaptive_sko_balance_multiplier) {
//    m_adaptive_sko_balance_multiplier 
//      = mp_eth_data.adaptive_sko_balance_multiplier;
//    mp_ee_data.adaptive_sko_balance_multiplier 
//      = mp_eth_data.adaptive_sko_balance_multiplier;
//  }
//}

//void hrm::app_t::show_pid_params()
//{
//  irs::mlog() << irsm("---------------------------------") << endl;
//  //irs::mlog() << setprecision(7);
//  irs::mlog() << defaultfloat << fixed;
//  irs::mlog() << setprecision(4);
//  irs::mlog() << irsm("Td = ");
//  irs::mlog() << m_elab_pid_td << irsm(" с") << endl;
//  
//  irs::mlog() << setprecision(2);
//  irs::mlog() << irsm("Fd = ");
//  irs::mlog() << 1.0 / m_elab_pid_td << irsm(" Гц") << endl;
//  
//  irs::mlog() << setprecision(3);
//  irs::mlog() << irsm("Kp = ") << m_elab_pid.k << endl;
//  irs::mlog() << irsm("Ki = ") << m_elab_pid.ki << endl;
//  irs::mlog() << irsm("Kd = ") << m_elab_pid.kd << endl;
//  irs::mlog() << irsm("Kiso = ") << m_elab_iso_k << endl;
//  irs::mlog() << irsm("Tiso = ") << m_elab_iso_t << endl;
//  irs::mlog() << setprecision(1);
//  irs::mlog() << irsm("Tfade = ") << m_elab_pid_fade_t << endl;
//  irs::mlog() << irsm("Navg = ") << m_elab_pid_avg_cnt << endl;
//  irs::mlog() << irsm("dSKO = ") << m_elab_pid_target_sko << endl;
//  irs::mlog() << setprecision(5);
//  irs::mlog() << irsm("Vmax = ") << m_elab_pid_ref << endl;
//}
