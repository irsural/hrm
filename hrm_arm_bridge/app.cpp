#include <irspch.h>

#include "app.h"

#include <irslimits.h>

#include <irsfinal.h>

hrm::app_t::app_t(cfg_t* ap_cfg):
  mp_cfg(ap_cfg),
  m_eth_data(),
  m_mxnet_server(
    mp_cfg->connector_hardflow,
    m_eth_data.connect(&m_mxnet_server, 0)/sizeof(irs_u32)),
  m_eeprom(&mp_cfg->spi_aux, &mp_cfg->ee_cs, 4096, true, 0, 64,
    irs::make_cnt_s(1)),
  m_eeprom_data(&m_eeprom),
  m_init_eeprom(&m_eeprom, &m_eeprom_data),
  m_lcd_drv(irslcd_4x20, mp_cfg->lcd_port, mp_cfg->lcd_rs_pin,
    mp_cfg->lcd_e_pin),
  m_keyboard_drv(),
  m_encoder_drv(mp_cfg->enc_a, mp_cfg->enc_b, mp_cfg->encoder_timer_address),

  m_lcd_drv_service(),
  m_buzzer_kb_event(),
  m_hot_kb_event(),
  m_menu_kb_event(),
  m_keyboard_event_gen(),

  m_raw_dac(
    &mp_cfg->spi_dac,
    &mp_cfg->dac_cs,
    &mp_cfg->dac_ldac,
    &mp_cfg->dac_clr,
    &mp_cfg->dac_reset,
    0),
  m_dac(&m_raw_dac),
  m_adc(&mp_cfg->spi_adc, &mp_cfg->adc_cs, &mp_cfg->adc_exti),
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
//  m_relay_bridge_pos(
//    &mp_cfg->relay_bridge_pos_on,
//    irst("BPOS"),
//    0),
//  m_relay_bridge_neg(
//    &mp_cfg->relay_bridge_neg_on,
//    irst("BNEG"),
//    0),
  m_relay_prot(&mp_cfg->relay_prot, irst("PROT"), 1),
  m_mode(md_free),
  m_free_status(fs_prepare),
  m_balance_status(bs_prepare),
  m_current_iteration(0),
  m_iteration_count(20),
  m_elab_mode(em_fast_2points),
  m_elab_iteration_count(3),
  m_elab_step(1),
  m_dac_code(0.0),
  m_dac_step(0.0),
  m_balanced_dac_code(0.0),
  m_initial_dac_code(0.0),
  m_initial_dac_step(pow(2.0, 18)),//m_dac.max_code()),
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
  m_dac_after_pause(irs::make_cnt_ms(100)),
  m_dac_elab_pause(irs::make_cnt_ms(101)),
  m_prepare_pause(5),
  m_elab_vector(),
  m_fast_elab_vector(),
  m_fast_elab_dac_step(0.0),
  m_fast_elab_dac_step_2(0.0),
  m_fast_elab_dac_step_3(0.0),
  m_exp_cnt(1),
  m_exp_vector(),
  m_no_prot(false),
  m_wild_relays(false),
  m_balanced_sko(0.),
  //m_adc_experiment_gain(m_min_adc_gain),
  //m_adc_experiment_filter(m_slow_adc_filter),
  m_manual_status(ms_prepare),
  m_scan_status(ss_prepare),
  m_dac_center_scan(0.),
  m_current_adc_point(0),
  m_prepare_pause_timer(0),
  m_exp_time(0),
  m_prev_exp_time(0),
  m_sum_time(0),
  m_remaining_time(),
  m_is_exp(false),
  m_exp_timer(irs::make_cnt_ms(1000)),
  m_optimize_balance(false),
  m_adc_fade(0.0),
  m_voltage(0.0),
  m_temperature(0.0),
  m_meas_temperature(false),
  m_max_unsaturated_voltage(0.0),
  m_max_unsaturated_dac_code(0.0),
  m_auto_elab_step(false),
  m_dac_step_amplitude(0.0),
  m_current_elab(0),
  m_pos_current_elab(0),
  m_min_elab_cnt(1),
  m_max_elab_cnt(5),
  m_ok_elab_cnt(0),
  m_elab_result_vector(),
  m_prev_elab_vector(),
  m_prev_elab_cnt(2),
  m_elab_polarity(bp_neg),
  m_elab_step_multiplier(1.0),
  m_elab_max_delta(1.0),
  m_new_adc_param_free_vx(false),
  m_new_adc_param_free_th(false),
  m_new_adc_param_manual(false),
  m_new_adc_param_balance(false),
  m_new_adc_param_elab(false),
  m_relay_pause_timer(m_relay_after_pause),
  m_buzzer(&mp_cfg->buzzer),
  mp_menu(),
  m_escape_pressed_event(),
  m_r_standard_type(r_standard_type_original),
  m_termostat(&mp_cfg->aux1),
  m_elab_pid_on(false),
  m_elab_pid_kp(0.0),
  m_elab_pid_ki(0.0),
  m_elab_pid_kd(0.0),
  m_elab_iso_k(0.0),
  m_elab_iso_t(0.0),
  m_min_after_pause(irs::make_cnt_ms(100)),
  m_elab_pid_fade_t(0.0),
  m_elab_pid_sko(m_eeprom_data.elab_pid_avg_cnt, 
    m_eeprom_data.elab_pid_avg_cnt),
  m_elab_pid_target_sko(0.0),
  m_elab_pid_target_sko_norm(0.0),
  m_elab_pid_ref(0.0),
  m_adc_max_value_prot(0.4),
  m_adc_max_value_no_prot(1.0),
  m_bac_old_coefficient(0.0),
  m_bac_new_coefficient(1.0),
  m_bac_new_int_coefficient(1),
  m_bac_new_int_multiplier(1000.0),
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
  m_treg_operating_duty_time_interval_s(1.0),
  m_treg_operating_duty_deviation(0.05),
  m_treg_pwm_max_code_float(1.0),
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

  show_network_params_t show_network_params(&mp_cfg->network_config);

  irs::mlog() << setprecision(8);

  m_exp_cnt = m_eeprom_data.exp_cnt;
  m_eth_data.exp_cnt = m_exp_cnt;

  m_etalon = m_eeprom_data.etalon;
  m_eth_data.etalon = m_etalon;

  m_checked = m_eeprom_data.checked;
  m_eth_data.checked = m_checked;

  m_eth_data.ratio = m_eeprom_data.ratio;

  m_eth_data.checked_prev = m_eeprom_data.checked_prev;

  m_dac.set_lin(m_eeprom_data.dac_lin);
  m_eth_data.dac_lin = m_dac.get_lin();

  m_elab_iteration_count = m_eeprom_data.elab_cnt;
  m_eth_data.elab_cnt = m_elab_iteration_count;

  m_dac_after_pause = irs::make_cnt_ms(m_eeprom_data.dac_pause_ms);
  m_eth_data.dac_pause_ms = m_eeprom_data.dac_pause_ms;
  m_dac.set_after_pause(m_min_after_pause);

  m_dac_elab_pause = irs::make_cnt_ms(m_eeprom_data.dac_elab_pause_ms);
  m_eth_data.dac_elab_pause_ms = m_eeprom_data.dac_elab_pause_ms;

  m_relay_after_pause = irs::make_cnt_ms(m_eeprom_data.relay_pause_ms);
  m_eth_data.relay_pause_ms = m_eeprom_data.relay_pause_ms;

  m_dac_center_scan = m_eeprom_data.dac_center_scan;
  m_eth_data.dac_center_scan = m_dac_center_scan;

  m_dac_center_scan = m_eeprom_data.int_dac_center_scan;
  m_eth_data.dac_center_scan = m_dac_center_scan;

  m_prepare_pause = m_eeprom_data.prepare_pause;
  m_eth_data.prepare_pause = m_prepare_pause;
  // Загрузка параметров АЦП из eeprom и выгрузка в ethernet
  adc_params_load_from_eeprom();
  //  Параметры уточнения
  m_eth_data.elab_pid_kp = m_eeprom_data.elab_pid_kp;
  m_elab_pid_kp = m_eeprom_data.elab_pid_kp;
  m_eth_data.elab_pid_ki = m_eeprom_data.elab_pid_ki;
  m_elab_pid_ki = m_eeprom_data.elab_pid_ki;
  m_eth_data.elab_pid_kd = m_eeprom_data.elab_pid_kd;
  m_elab_pid_kd = m_eeprom_data.elab_pid_kd;
  m_elab_pid.min = -1.0;
  m_elab_pid.max = 1.0;
  m_elab_pid.prev_e = 0.0;
  m_elab_pid.pp_e = 0.0;
  m_elab_pid.prev_out = 0.0;
  m_elab_pid.block_int = 0;
  m_elab_pid.block_int_ext = 0;
  m_elab_pid.int_val = 0.0;
  m_elab_pid.k_d_pid = 0.1;
  m_eth_data.elab_iso_k = m_eeprom_data.elab_iso_k;
  m_elab_iso_k = m_eeprom_data.elab_iso_k;
  m_eth_data.elab_iso_t = m_eeprom_data.elab_iso_t;
  m_elab_iso_t = m_eeprom_data.elab_iso_t;
  m_elab_iso.k = m_elab_iso_k;
  m_elab_iso.fd.x1 = 0.0;
  m_elab_iso.fd.y1 = 0.0;
  m_elab_iso.fd.t = m_elab_iso_t * m_adc.get_reference_frequency();
  m_eth_data.elab_pid_fade_t = m_eeprom_data.elab_pid_fade_t;
  m_elab_pid_fade_t = m_eeprom_data.elab_pid_fade_t;
  m_elab_pid_fade.x1 = 0.0;
  m_elab_pid_fade.y1 = 0.0;
  m_elab_pid_fade.t =  m_elab_pid_fade_t * m_adc.get_reference_frequency();
  m_eth_data.elab_pid_avg_cnt = m_eeprom_data.elab_pid_avg_cnt;
  m_elab_pid_target_sko_norm = m_eeprom_data.elab_pid_target_sko_norm;
  m_elab_pid_target_sko = denorm(m_elab_pid_target_sko_norm);
  m_eth_data.elab_pid_target_sko_norm = m_elab_pid_target_sko_norm;
  m_eth_data.elab_pid_target_sko = m_elab_pid_target_sko;
  m_elab_pid_ref = m_eeprom_data.elab_pid_ref;
  m_eth_data.elab_pid_ref = m_elab_pid_ref;

  m_adc_fade_data.t = m_eeprom_data.adc_filter_constant;
  m_eth_data.adc_filter_constant = m_adc_fade_data.t;

  //m_adc.set_skip_cnt(m_eeprom_data.adc_average_skip_cnt);
  //m_eth_data.adc_average_skip_cnt = m_eeprom_data.adc_average_skip_cnt;

  m_dac.set_voltage_pos(m_eeprom_data.dac_voltage_pos);
  m_eth_data.dac_voltage_pos = m_dac.voltage_pos();

  m_dac.set_voltage_neg(m_eeprom_data.dac_voltage_neg);
  m_eth_data.dac_voltage_neg = m_dac.voltage_neg();

  m_no_prot = m_eeprom_data.no_prot;
  m_eth_data.no_prot = m_no_prot;

  m_optimize_balance = m_eeprom_data.optimize_balance;
  m_eth_data.optimize_balance = m_optimize_balance;

  m_wild_relays = m_eeprom_data.wild_relays;
  m_eth_data.wild_relays = m_wild_relays;

  m_auto_elab_step = m_eeprom_data.auto_elab_step;
  m_eth_data.auto_elab_step = m_auto_elab_step;

  m_elab_step = m_eeprom_data.elab_step;
  m_eth_data.elab_step = m_elab_step;

  m_min_elab_cnt = m_eeprom_data.min_elab_cnt;
  m_eth_data.min_elab_cnt = m_min_elab_cnt;

  m_max_elab_cnt = m_eeprom_data.max_elab_cnt;
  m_eth_data.max_elab_cnt = m_max_elab_cnt;

  m_elab_step_multiplier = m_eeprom_data.elab_step_multiplier;
  m_eth_data.elab_step_multiplier = m_elab_step_multiplier;

  m_eth_data.adc_simply_show = m_eeprom_data.adc_simply_show;
  m_eth_data.adc_show_points = m_eeprom_data.adc_show_points;
  m_adc.set_show_points(m_eth_data.adc_show_points);

  m_elab_max_delta = m_eeprom_data.elab_max_delta;
  m_eth_data.elab_max_delta = m_elab_max_delta;
  
  m_elab_mode = convert_u8_to_elab_mode(m_eeprom_data.elab_mode);
  m_eth_data.elab_mode = m_eeprom_data.elab_mode;

  m_adc_max_value_prot = m_eeprom_data.adc_max_value_prot;
  m_eth_data.adc_max_value_prot = m_eeprom_data.adc_max_value_prot;
  
  m_adc_max_value_no_prot = m_eeprom_data.adc_max_value_no_prot;
  m_eth_data.adc_max_value_no_prot = m_eeprom_data.adc_max_value_no_prot;
  
  m_bac_old_coefficient = m_eeprom_data.bac_old_coefficient;
  m_eth_data.bac_old_coefficient = m_eeprom_data.bac_old_coefficient;
  
  m_bac_new_coefficient = m_eeprom_data.bac_new_coefficient;
  m_eth_data.bac_new_coefficient = m_eeprom_data.bac_new_coefficient;
  
  m_bac_new_int_coefficient = m_eeprom_data.bac_new_int_coefficient;
  m_eth_data.bac_new_int_coefficient = m_eeprom_data.bac_new_int_coefficient;
  
  m_bac_new_int_multiplier = m_eeprom_data.bac_new_int_multiplier;
  m_eth_data.bac_new_int_multiplier = m_eeprom_data.bac_new_int_multiplier;
  
  m_adc_fade_data.x1 = 0.0;
  m_adc_fade_data.y1 = 0.0;
  m_adc_fade_data.t = m_eeprom_data.adc_filter_constant;
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
  
  m_eth_data.termostat_off_pause = m_eeprom_data.termostat_off_pause;
  m_termostat.set_after_pause(m_min_after_pause);

  //  ЖКИ и клавиатура
  m_lcd_drv_service.connect(&m_lcd_drv);
  m_keyboard_event_gen.connect(&m_keyboard_drv);
  m_keyboard_event_gen.connect_encoder(&m_encoder_drv);
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
  irs::mlog() << irsm("sizeof(adc_value_t) = ");
  irs::mlog() << sizeof(adc_value_t) << endl;
}

void hrm::app_t::init_keyboard_drv()
{
  m_keyboard_drv.add_horizontal_pins(&mp_cfg->key_drv_horizontal_pins);
  m_keyboard_drv.add_vertical_pins(&mp_cfg->key_drv_vertical_pins);
  irs::set_default_keys(&m_keyboard_drv);
}

void hrm::app_t::init_encoder_drv()
{
  m_encoder_drv.add_press_down_pin(&mp_cfg->enc_sw);
  irs::set_default_keys(&m_encoder_drv);
}

void hrm::app_t::tick()
{ 
  mp_cfg->tick();
  m_mxnet_server.tick();
  m_eeprom.tick();
  m_raw_dac.tick();
  m_dac.tick();
  m_adc.tick();

  m_relay_bridge_pos.tick();
  m_relay_bridge_neg.tick();
  m_relay_prot.tick();

  m_buzzer.tick();

  mp_menu->tick();

  m_lcd_drv.tick();
  m_keyboard_event_gen.tick();
  
  m_termostat.tick();
  m_device_condition_controller.tick();
  m_treg_peltier.tick();

  if (m_buzzer_kb_event.check()) {
    m_buzzer.bzz();
  }

  if (m_eth_timer.check()) {
    m_eth_data.counter++;
    //  Приём параметров АЦП из ethernet, сохранение в eeprom и запись в
    //  структуры для каждого режима работы
    m_new_adc_param_free_vx = adc_params_recieve_and_save_free_vx();
    m_new_adc_param_free_th = adc_params_recieve_and_save_free_th();
    m_new_adc_param_manual = adc_params_recieve_and_save_manual();
    m_new_adc_param_balance = adc_params_recieve_and_save_balance();
    m_new_adc_param_elab = adc_params_recieve_and_save_elab();
    //  Передача текущих параметров АЦП в ethernet
    adc_params_translate_actual_to_eth();
    //  Обновление сетевых переменных от программы - пользователю
    if (m_mode != md_manual) {
      //  ЦАП
      if (m_eth_data.dac_normalize_code != m_dac.get_normalize_code()) {
        m_eth_data.dac_normalize_code = m_dac.get_normalize_code();
      }
      if (m_eth_data.dac_code != m_dac.get_code()) {
        m_eth_data.dac_code = m_dac.get_code();
      }
      if (m_eth_data.dac_on != m_dac.is_on()) {
        m_eth_data.dac_on = m_dac.is_on();
      }
      if (m_eth_data.dac_lin != m_dac.get_lin()) {
        m_eth_data.dac_lin = m_dac.get_lin();
      }
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
    }
    if (m_adc.new_result()) {
      m_adc.result(&m_adc_result_data);
      m_eth_data.adc_sko = abs(m_adc_result_data.sko * 1e6);
      //m_eth_data.adc_impf = m_adc_result_data.impf;
      //m_eth_data.adc_sko_impf = abs(m_adc_result_data.impf_sko * 1e6);
      //m_eth_data.adc_min = m_adc_result_data.min;
      //m_eth_data.adc_max = m_adc_result_data.max;
      m_eth_data.adc_raw = m_adc_result_data.raw;
      m_eth_data.adc_meas_freq = m_adc_result_data.measured_frequency;
      m_eth_data.adc_point_time = m_adc_result_data.point_time * 1e6;
      
      if (m_mode != md_free && m_free_status != fs_idle) {
        m_voltage = m_adc_result_data.avg;
        //double iso_out = isodr(&m_elab_iso, m_voltage);
        m_eth_data.elab_iso_out = m_adc_result_data.unnormalized_value;//iso_out;
      }
    }
    if (m_eth_data.adc_value != m_voltage) {
      m_eth_data.adc_value = m_voltage;
      m_adc_fade = fade(&m_adc_fade_data, m_voltage);
      m_eth_data.adc_filter_value = m_adc_fade;
    }
    if (m_eth_data.adc_clear_filter) {
      m_eth_data.adc_clear_filter = 0;
      m_adc_fade_data.x1 = m_voltage;
      m_adc_fade_data.y1 = m_voltage;
      m_adc_fade = m_voltage;
      m_eth_data.adc_filter_value = m_adc_fade;
    }
    if (m_eth_data.current_mode != m_mode) {
      m_eth_data.current_mode = m_mode;
    }
    if (m_eth_data.adc_temperature != m_temperature) {
      m_eth_data.adc_temperature = m_temperature;
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
    if (m_eth_data.dac_lin != m_eeprom_data.dac_lin) {
      m_eeprom_data.dac_lin = m_eth_data.dac_lin;
    }
    if (m_eth_data.elab_cnt != m_eeprom_data.elab_cnt) {
      m_eeprom_data.elab_cnt = m_eth_data.elab_cnt;
    }
    if (m_eth_data.dac_pause_ms != m_eeprom_data.dac_pause_ms) {
      m_eeprom_data.dac_pause_ms = m_eth_data.dac_pause_ms;
    }
    if (m_eth_data.relay_pause_ms != m_eeprom_data.relay_pause_ms) {
      m_eeprom_data.relay_pause_ms = m_eth_data.relay_pause_ms;
    }
    if (m_eth_data.dac_center_scan != m_eeprom_data.dac_center_scan) {
      m_eeprom_data.dac_center_scan = m_eth_data.dac_center_scan;
    }
    if (m_eth_data.int_dac_center_scan != m_eeprom_data.int_dac_center_scan) {
      m_eeprom_data.int_dac_center_scan = m_eth_data.int_dac_center_scan;
    }
    if (m_eth_data.prepare_pause != m_eeprom_data.prepare_pause) {
      m_eeprom_data.prepare_pause = m_eth_data.prepare_pause;
    }
    if (m_eth_data.adc_average_cnt != m_eeprom_data.adc_average_cnt) {
      m_eeprom_data.adc_average_cnt = m_eth_data.adc_average_cnt;
    }
    if (m_eth_data.adc_filter_constant != m_adc_fade_data.t) {
      m_adc_fade_data.t = m_eth_data.adc_filter_constant;
      m_eeprom_data.adc_filter_constant = m_eth_data.adc_filter_constant;
    }
    if (m_eth_data.adc_average_skip_cnt != m_eeprom_data.adc_average_skip_cnt) {
      m_eeprom_data.adc_average_skip_cnt = m_eth_data.adc_average_skip_cnt;
    }
    if (m_eth_data.adc_experiment_gain != m_eeprom_data.adc_experiment_gain) {
      m_eeprom_data.adc_experiment_gain = m_eth_data.adc_experiment_gain;
    }
    if (m_eth_data.adc_experiment_filter != m_eeprom_data.adc_experiment_filter) {
      m_eeprom_data.adc_experiment_filter = m_eth_data.adc_experiment_filter;
    }
    if (m_eth_data.dac_voltage_pos != m_eeprom_data.dac_voltage_pos) {
      m_eeprom_data.dac_voltage_pos = m_eth_data.dac_voltage_pos;
    }
    if (m_eth_data.dac_voltage_neg != m_eeprom_data.dac_voltage_neg) {
      m_eeprom_data.dac_voltage_neg = m_eth_data.dac_voltage_neg;
    }
    if (m_eth_data.no_prot != m_eeprom_data.no_prot) {
      m_eeprom_data.no_prot = m_eth_data.no_prot;
    }
    if (m_eth_data.optimize_balance != m_eeprom_data.optimize_balance) {
      m_eeprom_data.optimize_balance = m_eth_data.optimize_balance;
    }
    if (m_eth_data.wild_relays != m_eeprom_data.wild_relays) {
      m_eeprom_data.wild_relays = m_eth_data.wild_relays;
    }
    if (m_eth_data.auto_elab_step != m_eeprom_data.auto_elab_step) {
      m_eeprom_data.auto_elab_step = m_eth_data.auto_elab_step;
    }
    if (m_eth_data.elab_step != m_eeprom_data.elab_step) {
      m_eeprom_data.elab_step = m_eth_data.elab_step;
    }
    if (m_eth_data.min_elab_cnt != m_eeprom_data.min_elab_cnt) {
      m_eeprom_data.min_elab_cnt = m_eth_data.min_elab_cnt;
    }
    if (m_eth_data.max_elab_cnt != m_eeprom_data.max_elab_cnt) {
      m_eeprom_data.max_elab_cnt = m_eth_data.max_elab_cnt;
    }
    if (m_eth_data.dac_elab_pause_ms != m_eeprom_data.dac_elab_pause_ms) {
      m_eeprom_data.dac_elab_pause_ms = m_eth_data.dac_elab_pause_ms;
    }
    if (m_eth_data.elab_step_multiplier != m_eeprom_data.elab_step_multiplier) {
      m_eeprom_data.elab_step_multiplier = m_eth_data.elab_step_multiplier;
    }
    if (m_eth_data.adc_simply_show != m_eeprom_data.adc_simply_show) {
      m_eeprom_data.adc_simply_show = m_eth_data.adc_simply_show;
    }
    if (m_eth_data.adc_show_points != m_eeprom_data.adc_show_points) {
      m_eeprom_data.adc_show_points = m_eth_data.adc_show_points;
      m_adc.set_show_points(m_eth_data.adc_show_points);
    }
    if (m_eth_data.use_impf != m_eeprom_data.use_impf) {
      m_eeprom_data.use_impf = m_eth_data.use_impf;
    }
    if (m_eth_data.elab_max_delta != m_eeprom_data.elab_max_delta) {
      m_eeprom_data.elab_max_delta = m_eth_data.elab_max_delta;
    }
    if (m_eth_data.elab_mode != m_eeprom_data.elab_mode) {
      m_eeprom_data.elab_mode = m_eth_data.elab_mode;
    }
    if (m_eth_data.elab_pid_kp != m_eeprom_data.elab_pid_kp) {
      m_eeprom_data.elab_pid_kp = m_eth_data.elab_pid_kp;
    }
    if (m_eth_data.elab_pid_ki != m_eeprom_data.elab_pid_ki) {
      m_eeprom_data.elab_pid_ki = m_eth_data.elab_pid_ki;
    }
    if (m_eth_data.elab_pid_kd != m_eeprom_data.elab_pid_kd) {
      m_eeprom_data.elab_pid_kd = m_eth_data.elab_pid_kd;
    }
    if (m_eth_data.elab_iso_k != m_eeprom_data.elab_iso_k) {
      m_eeprom_data.elab_iso_k = m_eth_data.elab_iso_k;
    }
    if (m_eth_data.elab_iso_t != m_eeprom_data.elab_iso_t) {
      m_eeprom_data.elab_iso_t = m_eth_data.elab_iso_t;
    }
    if (m_eth_data.elab_pid_fade_t != m_eeprom_data.elab_pid_fade_t) {
      m_eeprom_data.elab_pid_fade_t = m_eth_data.elab_pid_fade_t;
    }
    if (m_eth_data.elab_pid_avg_cnt != m_eeprom_data.elab_pid_avg_cnt) {
      m_eeprom_data.elab_pid_avg_cnt = m_eth_data.elab_pid_avg_cnt;
    }
    if (m_eth_data.elab_pid_target_sko != m_elab_pid_target_sko) {
      m_elab_pid_target_sko = m_eth_data.elab_pid_target_sko;
      m_elab_pid_target_sko_norm = norm(m_elab_pid_target_sko);
      m_eeprom_data.elab_pid_target_sko_norm = m_elab_pid_target_sko_norm;
      m_eth_data.elab_pid_target_sko_norm = m_elab_pid_target_sko_norm;
    }
    if (m_eth_data.elab_pid_target_sko_norm != m_elab_pid_target_sko_norm) {
      m_elab_pid_target_sko_norm = m_eth_data.elab_pid_target_sko_norm;
      m_eeprom_data.elab_pid_target_sko_norm = m_elab_pid_target_sko_norm;
      m_elab_pid_target_sko = denorm(m_elab_pid_target_sko_norm);
      m_eth_data.elab_pid_target_sko = m_elab_pid_target_sko;
    }
    if (m_eth_data.elab_pid_ref != m_eeprom_data.elab_pid_ref) {
      m_eeprom_data.elab_pid_ref = m_eth_data.elab_pid_ref;
      m_elab_pid_ref = m_eth_data.elab_pid_ref;
    }
    if (m_eth_data.adc_max_value_prot != m_eeprom_data.adc_max_value_prot) {
      m_eeprom_data.adc_max_value_prot = m_eth_data.adc_max_value_prot;
      m_adc_max_value_prot = m_eth_data.adc_max_value_prot;
      if (m_relay_prot == 1) {
        m_adc.set_max_value(m_adc_max_value_prot);
      }
    }
    if (m_eth_data.adc_max_value_no_prot != m_eeprom_data.adc_max_value_no_prot) {
      m_eeprom_data.adc_max_value_no_prot = m_eth_data.adc_max_value_no_prot;
      m_adc_max_value_no_prot = m_eth_data.adc_max_value_no_prot;
      if (m_relay_prot == 0) {
        m_adc.set_max_value(m_adc_max_value_no_prot);
      }
    }
    if (m_eth_data.use_impf != m_eeprom_data.use_impf) {
      m_eeprom_data.use_impf = m_eth_data.use_impf;
    }
    if (m_eth_data.bac_old_coefficient != m_eeprom_data.bac_old_coefficient) {
      m_eeprom_data.bac_old_coefficient = m_eth_data.bac_old_coefficient;
      m_bac_old_coefficient = m_eth_data.bac_old_coefficient;
    }
    if (m_eth_data.bac_new_coefficient != m_eeprom_data.bac_new_coefficient) {
      m_eeprom_data.bac_new_coefficient = m_eth_data.bac_new_coefficient;
      m_bac_new_coefficient = m_eth_data.bac_new_coefficient;
    }
    if (m_eth_data.bac_new_int_coefficient != 
      m_eeprom_data.bac_new_int_coefficient) {
      m_eeprom_data.bac_new_int_coefficient = m_eth_data.bac_new_int_coefficient;
      m_bac_new_int_coefficient = m_eth_data.bac_new_int_coefficient;
    }
    if (m_eth_data.bac_new_int_multiplier != 
      m_eeprom_data.bac_new_int_multiplier) {
      m_eeprom_data.bac_new_int_multiplier = m_eth_data.bac_new_int_multiplier;
      m_bac_new_int_multiplier = m_eth_data.bac_new_int_multiplier;
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
    
    //m_eth_data.external_temperature = m_ext_th_data.temperature_code *
      //m_ext_th.get_conv_koef();
    //m_eth_data.adc_meas_freq = m_adc.get_adc_frequency();

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
    
    m_eth_data.termostat_is_off = m_termostat.is_off();
    if (m_eth_data.termostat_off_pause != m_eeprom_data.termostat_off_pause) {
      m_eeprom_data.termostat_off_pause = m_eth_data.termostat_off_pause;
    }
    
    m_treg_sync_parameters.sync();
    
    if (m_adc.status() == irs_st_ready) {
      m_eth_data.adc_meas_process = 0;
    } else {
      m_eth_data.adc_meas_process = 1;
    }
    
    if (m_eth_data.show_last_result == 1) {
      m_eth_data.show_last_result = 0;
      show_last_result();
    }
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
      m_eth_data.sum_time = m_sum_time;
      m_eth_data.remaining_time = m_remaining_time;
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
          m_adc.set_max_value(m_adc_max_value_prot);
          //
          m_dac.show();
          m_dac.on();
          m_dac.set_code(0);
          //
          m_adc.hide();
          //
          //m_relay_bridge_pos.set_wild(false);
          //m_relay_bridge_neg.set_wild(false);
          //
          m_eth_data.apply = 0;
          m_eth_data.prepare_pause = m_prepare_pause;
          m_service_timer.start();
          m_meas_temperature = false;
          m_termostat.set_off(false);
          m_eth_data.termostat_off = 0;
          m_elab_pid_on = false;
          m_eth_data.elab_pid_on = false;
          m_eth_data.elab_pid_reset = 0;
          m_eth_data.elab_pid_sko_ready = 0;
          
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
              if (m_meas_temperature) {
                m_temperature = adc_vx_to_th(m_adc_result_data.avg);
                m_adc.set_params(&m_adc_free_vx_param_data);
                m_adc.start_conversion();
                m_meas_temperature = false;
              } else {
                m_voltage = m_adc_result_data.avg;
                m_adc.set_params(&m_adc_free_th_param_data);
                m_adc.start_conversion();
                m_meas_temperature = true;
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
          m_dac.set_after_pause(m_min_after_pause);
          m_termostat.set_after_pause(m_min_after_pause);
          m_meas_temperature = false;
          m_manual_status = ms_adc_setup;
          break;
        }
        case ms_adc_setup: {
          if (m_adc.status() == irs_st_ready) {
            m_adc.set_params(&m_adc_manual_param_data);
            update_elab_pid_koefs();
            m_manual_status = ms_check_user_changes;
          }
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
            if (m_eth_data.relay_prot != m_relay_prot) {
              m_relay_prot = m_eth_data.relay_prot;
              if (m_eth_data.relay_prot == 1) {
                m_adc.set_max_value(m_adc_max_value_prot);
              } else {
                m_adc.set_max_value(m_adc_max_value_no_prot);  
              }
            }
            //  ЦАП
            if (m_eth_data.dac_on != m_dac.is_on()) {
              if (m_eth_data.dac_on) {
                m_dac.on();
                m_adc.continious_pause();
              } else {
                m_dac.off();
                m_adc.continious_pause();
              }
            }
            if (m_eth_data.dac_code != m_dac.get_code()) {
              m_dac.set_code(m_eth_data.dac_code);
              m_adc.continious_pause();
              m_eth_data.dac_normalize_code = m_dac.get_normalize_code();
              if (m_eth_data.dac_code != m_dac.get_code()) {
                m_eth_data.dac_code = m_dac.get_code();
              }
            }
            if (m_eth_data.dac_normalize_code != m_dac.get_normalize_code()) {
              m_dac.set_normalize_code(m_eth_data.dac_normalize_code);
              m_adc.continious_pause();
              m_eth_data.dac_code = m_dac.get_code();
              if (m_eth_data.dac_normalize_code != m_dac.get_normalize_code()) {
                m_eth_data.dac_normalize_code = m_dac.get_normalize_code();
              }
            }
            if (m_eth_data.dac_lin != m_dac.get_lin()) {
              m_dac.set_lin(m_eth_data.dac_lin);
              m_adc.continious_pause();
            }
            //  АЦП
            if (m_new_adc_param_manual) {
              m_new_adc_param_manual = false;
              m_adc.set_params(&m_adc_manual_param_data);
            }
            if (m_eth_data.adc_simply_show) {
              m_adc.show();
            } else {
              m_adc.hide();
            }
            if (m_eth_data.elab_iso_k != m_elab_iso_k) {
              m_elab_iso_k = m_eth_data.elab_iso_k;
              update_elab_pid_koefs();
            }
            if (m_eth_data.test_impf == 1) {
              m_adc.test_impf();
              m_eth_data.test_impf = 0;
            }
            if (m_adc.status() == irs_st_ready) {
              m_adc.start_conversion();
            }
            if (m_eth_data.start_adc_sequence == 1) {
              m_eth_data.start_adc_sequence = 0;
              m_manual_status = ms_adc_show;
            } else if (m_eth_data.elab_pid_on == 1) {
              m_manual_status = ms_pid_start;
            }
            //
            bool thst_is_off = false;
            if (m_eth_data.termostat_off) {
              thst_is_off = true;
            }
            if (thst_is_off != m_termostat.is_off()) {
              m_termostat.set_off(thst_is_off);
            }
          } break;
        }
        case ms_adc_show: {
          if (m_adc.status() == irs_st_ready) {
            if (m_eth_data.adc_simply_show) {
              m_adc.show();
            } else {
              m_adc.show();
            }
            m_current_iteration = 1;
            m_iteration_count = m_eth_data.exp_cnt;
            m_adc.start_conversion();
            m_manual_status = ms_adc_hide;
          }
        } break;
        case ms_adc_hide: {
          if (m_adc.status() == irs_st_ready) {
            if (m_current_iteration < m_iteration_count) {
              m_current_iteration++;
              m_eth_data.current_exp_cnt = m_current_iteration;
            } else {
              m_adc.hide();
              m_manual_status = ms_check_user_changes;
            }
            m_adc.start_conversion();
          }
        } break;
        case ms_pid_start: {
          m_elab_pid_on = true;
          m_elab_pid_kp = m_eth_data.elab_pid_kp;
          m_elab_pid_ki = m_eth_data.elab_pid_ki;
          m_elab_pid_kd = m_eth_data.elab_pid_kd;
          m_elab_iso_k = m_eth_data.elab_iso_k;
          m_elab_iso_t = m_eth_data.elab_iso_t;
          m_elab_pid_fade_t = m_eth_data.elab_pid_fade_t;
          update_elab_pid_koefs();
          irs::mlog() << irsm("PID ON") << endl;
          m_manual_status = ms_pid_process;
          break;
        }
        case ms_pid_process: {
          if (m_eth_data.reset == 1) {
            m_eth_data.reset = 0;
            m_manual_status = ms_prepare;
            irs::mlog() << irsm("PID OFF") << endl;
            m_mode = md_free;
            m_eth_data.mode = md_free;
          } else if (m_eth_data.elab_pid_on == 0) {
            m_manual_status = ms_check_user_changes;
            m_elab_pid_on = false;
            irs::mlog() << irsm("PID OFF") << endl;
          } else {
            //  ЦАП
            if (m_eth_data.dac_normalize_code != m_dac.get_normalize_code()) {
              m_eth_data.dac_normalize_code = m_dac.get_normalize_code();
            }
            if (m_eth_data.dac_code != m_dac.get_code()) {
              m_eth_data.dac_code = m_dac.get_code();
            }
            //  АЦП
            if (m_new_adc_param_manual) {
              m_new_adc_param_manual = false;
              m_adc.set_params(&m_adc_manual_param_data);
              update_elab_pid_koefs();
            }
            if (m_eth_data.elab_pid_kp != m_elab_pid_kp) {
              m_elab_pid_kp = m_eth_data.elab_pid_kp;
              update_elab_pid_koefs();
            }
            if (m_eth_data.elab_pid_ki != m_elab_pid_ki) {
              m_elab_pid_ki = m_eth_data.elab_pid_ki;
              update_elab_pid_koefs();
            }
            if (m_eth_data.elab_pid_kd != m_elab_pid_kd) {
              m_elab_pid_kd = m_eth_data.elab_pid_kd;
              update_elab_pid_koefs();
            }
            if (m_eth_data.elab_iso_k != m_elab_iso_k) {
              m_elab_iso_k = m_eth_data.elab_iso_k;
              update_elab_pid_koefs();
            }
            if (m_eth_data.elab_iso_t != m_elab_iso_t) {
              m_elab_iso_t = m_eth_data.elab_iso_t;
              update_elab_pid_koefs();
            }
            if (m_eth_data.elab_pid_fade_t != m_elab_pid_fade_t) {
              m_elab_pid_fade_t = m_eth_data.elab_pid_fade_t;
              update_elab_pid_koefs();
            }
            if (m_eth_data.elab_pid_avg_cnt != m_elab_pid_sko.size()) {
              m_elab_pid_sko.resize(m_eth_data.elab_pid_avg_cnt);
              m_elab_pid_sko.resize_average(m_eth_data.elab_pid_avg_cnt);
              //update_elab_pid_koefs();
            }
            bool thst_is_off = false;
            if (m_eth_data.termostat_off) {
              thst_is_off = true;
            }
            if (thst_is_off != m_termostat.is_off()) {
              m_termostat.set_off(thst_is_off);
            }
            if (m_eth_data.elab_pid_reset == 1) {
              m_eth_data.elab_pid_reset = 0;
              irs::mlog() << irsm("PID RESET") << endl;
              m_manual_status = ms_pid_reset;
            }
            if (m_adc.status() == irs_st_ready) {
              m_adc.start_conversion();
              double iso_out = isodr(&m_elab_iso, m_voltage);
              m_eth_data.elab_iso_out = iso_out;
              double dac = pid_reg(&m_elab_pid, m_elab_pid_ref - iso_out);
              m_dac.set_normalize_code(dac);
              double dac_out = fade(&m_elab_pid_fade, dac);
              m_eth_data.elab_pid_fade_out = denorm(dac_out);
              m_eth_data.elab_pid_int = m_elab_pid.int_val;
              m_elab_pid_sko.add(dac);
              m_eth_data.elab_pid_avg = denorm(m_elab_pid_sko.average());
              m_eth_data.elab_pid_sko = denorm(m_elab_pid_sko);
              m_eth_data.elab_pid_avg_norm = m_elab_pid_sko.average();
              m_eth_data.elab_pid_sko_norm = m_elab_pid_sko;
              if (m_elab_pid_sko < m_eth_data.elab_pid_target_sko) {
                m_eth_data.elab_pid_sko_ready = 1;
              } else {
                m_eth_data.elab_pid_sko_ready = 0;
              }
            }
          }
          break;
        }
        case ms_pid_reset: {
          if (m_dac.ready()) {
            m_dac.set_code(0);
            update_elab_pid_koefs();
            m_elab_pid.int_val = 0.0;
            m_elab_pid.pp_e = 0.0;
            m_elab_pid.prev_e = 0.0;
            m_elab_pid.prev_out = 0.0;
            m_elab_iso.fd.x1 = 0.0;
            m_elab_iso.fd.y1 = 0.0;
            m_elab_pid_fade.x1 = 0.0;
            m_elab_pid_fade.y1 = 0.0;
            m_elab_pid_sko.clear();
            m_manual_status = ms_pid_reset_wait;
          }
          break;
        }
        case ms_pid_reset_wait: {
          if (m_dac.ready()) {
            irs::mlog() << irsm("PID READY") << endl;
            m_manual_status = ms_pid_process;
          }
          break;
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

          m_elab_iteration_count = 100 * m_eth_data.elab_cnt;
          if (m_elab_iteration_count < 3) {
            m_elab_iteration_count = 3;
            m_eth_data.elab_cnt = m_elab_iteration_count;
          }
          m_exp_cnt = m_eth_data.exp_cnt;
          m_dac_center_scan = m_eth_data.dac_center_scan;

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
            m_dac_code = m_dac_center_scan;
            m_dac_code -= static_cast<dac_value_t>(m_elab_iteration_count / 2);
            irs::mlog() << irsm("Число точек = ") << m_elab_iteration_count;
            irs::mlog() << endl;
            irs::mlog() << irsm("Начальный код = ") << m_dac_code << endl;
            m_scan_status = ss_first_dac_set;
          }
          break;
        }
        case ss_first_dac_set: {
          if (m_dac.ready()) {
            m_dac.set_code(m_dac_code);
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
            if (m_new_adc_param_manual) {
              m_new_adc_param_manual = false;
              m_adc.set_params(&m_adc_manual_param_data);
            }
            m_adc.show();
            m_scan_status = ss_start_adc;
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_adc.start_conversion();
            }
          }
          break;
        }
        case ss_dac_set: {
          if (m_dac.ready()) {
            m_dac.set_code(m_dac_code);
            m_scan_status = ss_start_adc;
          }
          break;
        }
        case ss_start_adc: {
          if (m_dac.ready() && m_adc.status() == irs_st_ready) {
            m_adc.start_conversion();
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
                m_dac_code += 1.0;
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
        case bs_prepare: {
          irs::mlog() << endl;
          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("----- Режим уравновешивания -----") << endl;
          irs::mlog() << irsm("---------------------------------") << endl;

          m_etalon = m_eth_data.etalon;
          m_checked = m_eth_data.checked;

          m_balance_polarity = bp_neg;

          m_dac_after_pause = irs::make_cnt_ms(m_eth_data.dac_pause_ms);
          m_dac_elab_pause = irs::make_cnt_ms(m_eth_data.dac_elab_pause_ms);
          m_dac.set_after_pause(m_dac_after_pause);
          m_relay_after_pause = irs::make_cnt_ms(m_eth_data.relay_pause_ms);
          m_relay_bridge_pos.set_after_pause(m_relay_after_pause);
          m_relay_bridge_neg.set_after_pause(m_relay_after_pause);
          m_relay_prot.set_after_pause(m_relay_after_pause);

          m_wild_relays = m_eth_data.wild_relays;
          m_eeprom_data.wild_relays = m_wild_relays;
          if (m_wild_relays) {
            //m_relay_bridge_pos.set_wild(true);
            //m_relay_bridge_neg.set_wild(true);
          }

          m_relay_pause_timer.set(m_relay_after_pause);

          m_elab_iteration_count = m_eth_data.elab_cnt;
          m_elab_step = m_eth_data.elab_step;
          m_min_elab_cnt = m_eth_data.min_elab_cnt;
          m_max_elab_cnt = m_eth_data.max_elab_cnt;
          if (m_elab_step != m_eth_data.elab_step) {
            m_eth_data.elab_step = m_elab_step;
            m_eeprom_data.elab_step = m_elab_step;
          }
          m_auto_elab_step = m_eth_data.auto_elab_step;
          m_exp_vector.clear();
          m_elab_vector.clear();
          m_fast_elab_vector.clear();
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
          
          //  Параметры АЦП в режиме уравновешивания
          m_adc.set_params(&m_adc_balance_param_data);
          irs::mlog() << irsm("Параметры АЦП m_adc_balance_param_data") << endl;

          m_prepare_pause = m_eth_data.prepare_pause;
          m_prepare_pause_timer.set(irs::make_cnt_ms(1000));

          m_exp_time = 0;
          m_sum_time = 0;
          m_eth_data.sum_time = 0;
          m_remaining_time = 50; // Пока не реализован расчет времени!!!!!
          m_optimize_balance = m_eth_data.optimize_balance;

          m_max_unsaturated_voltage = 0.0;
          m_max_unsaturated_dac_code = 0.0;

          m_elab_result_vector.clear();
          m_pos_current_elab = 0;
          m_elab_polarity = bp_neg;
          m_elab_step_multiplier = m_eth_data.elab_step_multiplier;
          m_elab_max_delta = m_eth_data.elab_max_delta;
          
          m_termostat.set_off(false);
          m_termostat.set_after_pause(
            DBLTIME_TO_CNT(m_eth_data.termostat_off_pause));
          m_device_condition_controller.set_idle(true);
            
          m_elab_mode = convert_u8_to_elab_mode(m_eth_data.elab_mode);
          
          m_balance_action = ba_prepare;
          
          m_balance_status = bs_set_prot;
          break;
        }
        case bs_set_prot: {
          if (bridge_relays_ready()) {
            //  Отключение защитного реле, если не нужно
            if (m_no_prot) {
              m_relay_prot = 0;
              m_adc.set_max_value(m_adc_max_value_no_prot);
            }
            m_balance_status = bs_set_coils;
          }
          break;
        }
        case bs_set_coils: {
          if (bridge_relays_ready()) {
            //  Включение напряжения на катушки
            m_is_exp = true;
            m_exp_timer.start();
            switch (m_balance_polarity) {
            case bp_neg:
              irs::mlog() << irsm("------------- (-) ---------------") <<endl;
              m_relay_bridge_neg = 1;
              break;
            case bp_pos:
              irs::mlog() << irsm("------------- (+) ---------------") <<endl;
              m_relay_bridge_pos = 1;
              break;
            }
            m_balance_status = bs_coils_wait;
          }
          break;
        }
        case bs_coils_wait: {
          if (bridge_relays_ready()) {
            //  Ожидание готовности реле подачи напряжения
            m_relay_pause_timer.start();
            float pause
              = 0.001 * static_cast<float>(m_eth_data.relay_pause_ms);
            irs::mlog() << irsm("Пауза реле ") << pause << irsm(" c") << endl;
            m_balance_status = bs_coils_relay_pause;
          }
          break;
        }
        case bs_coils_relay_pause: {
          if (m_relay_pause_timer.check()) {
            //  Пауза перед измерениями
            m_balance_status = bs_set_pause;
          }
          break;
        }
        case bs_set_pause: {
          if (m_adc.status() == irs_st_ready) {
            if (m_exp_vector.size() < 1
              && m_balance_polarity == bp_neg) {
              m_is_exp = false;
              m_prepare_pause_timer.start();
              m_prepare_current_time = m_prepare_pause;
              irs::mlog() << endl << irsm("Пауза ") << m_eth_data.prepare_pause;
              irs::mlog() << irsm(" c") << endl;
              m_adc.hide();
              m_dac.hide();
              m_balance_action = ba_prepare_pause;
              m_balance_status = bs_pause;
            } else {
              m_balance_status = bs_adc_prepare;//bs_meas_temperature;
            }
          }
          break;
        }
        case bs_pause: {
          if (m_prepare_pause_timer.check()) {
            if (m_prepare_current_time > 0) {
              m_prepare_pause_timer.start();
              m_prepare_current_time--;
              irs::mlog() << irsm(".") << flush;
              if (m_prepare_current_time % 60 == 0) {
                irs::mlog() << endl;
              }
              m_eth_data.prepare_pause = m_prepare_current_time;
            } else {
              irs::mlog() << endl;
              m_eth_data.prepare_pause = m_prepare_pause;
              m_balance_action = ba_balance_neg;
              m_balance_status = bs_adc_show;
            }
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_adc.start_conversion();
            }
          }
          break;
        }
        case bs_adc_show: {
          if (m_adc.status() == irs_st_ready) {
            m_is_exp = true;
            m_exp_timer.start();
            if (m_eth_data.adc_simply_show) {
              m_adc.show();
            } else {
              m_adc.hide();
            }
            //m_adc.show();
            m_dac.hide();
            m_balance_status = bs_adc_prepare;//bs_meas_temperature;
          }
          break;
        }
        case bs_meas_temperature: {
          if (m_adc.status() == irs_st_ready) {
            m_adc.set_params(&m_adc_free_th_param_data);
            irs::mlog() << irsm("Параметры АЦП m_adc_free_th_param_data")
              << endl;
            m_adc.start_conversion();
            m_balance_status = bs_wait_temperature;
          }
          break;
        }
        case bs_wait_temperature: {
          if (m_adc.status() == irs_st_ready) {
            m_temperature = adc_vx_to_th(m_voltage);
            irs::mlog() << irsm("Температура = ") << m_temperature;
            irs::mlog() << irsm(" °C") << endl;
            m_adc.set_params(&m_adc_balance_param_data);
            irs::mlog() << irsm("Параметры АЦП m_adc_balance_param_data")
              << endl;
            m_balance_status = bs_dac_prepare;
          }
          break;
        }
        case bs_adc_prepare: {
          if (m_adc.status() == irs_st_ready) {
            m_adc.set_params(&m_adc_balance_param_data);
            irs::mlog() << irsm("Параметры АЦП m_adc_balance_param_data")
              << endl;
            m_balance_status = bs_dac_prepare;
          }
          break;
        }
        case bs_dac_prepare: {
          if (m_dac.ready()) {
            m_dac_code = m_initial_dac_code;
            m_dac.set_code(m_initial_dac_code);
            m_dac_step = m_initial_dac_step;
            m_current_iteration = 0;
            m_iteration_count = 20;
            m_dac_step_amplitude = 0.0;
            m_dac.on();
            m_prev_elab_vector.clear();
            m_balance_status = bs_termostat_off_adc_start;
          }
          break;
        }
        case bs_termostat_off_adc_start: {
          if(m_dac.ready()) {
            m_termostat.set_off(true);
            m_device_condition_controller.set_idle(false);
            m_balance_status = bs_adc_start;
          }
          break;
        }
        case bs_adc_start: {
          if (m_adc.status() == irs_st_ready 
              && m_termostat.status() == irs_st_ready) {
            m_adc.start_conversion();
            m_balance_status = bs_adc_wait;
          }
          break;
        }
        case bs_adc_wait: {
          if (m_adc.status() == irs_st_ready) {
            adc_result_data_t adc_result_data;
            m_adc.result(&adc_result_data);
            adc_value_t adc_result = adc_result_data.avg;
            m_termostat.set_off(false);
            m_device_condition_controller.set_idle(true);
            //bool m_prev_balance_completed = false;
            
            if (abs(adc_result) < 0.99 * m_adc.max_value() &&
                m_current_iteration > 0 &&
                m_elab_mode != em_none) {
              if (abs(adc_result) > m_max_unsaturated_voltage) {
                m_max_unsaturated_voltage = abs(adc_result);
                m_max_unsaturated_dac_code = abs(m_dac_code);

//                irs::mlog() << irsm(">>>> ") << setw(8);
//                irs::mlog() << m_max_unsaturated_dac_code;
//                irs::mlog() << irsm(" : ");
//                irs::mlog() << m_max_unsaturated_voltage;
//                irs::mlog() << irsm(" В ");
//                irs::mlog() << abs(adc_result_data.sko * 1e6);
//                irs::mlog() << irsm(" ppm ") << endl;
              }
              elab_point_t elab_point;
              elab_point.dac = m_dac.get_code();
              elab_point.adc = adc_result;
              elab_point.sko = adc_result_data.sko;
              elab_point.avg = adc_result;
              m_prev_elab_vector.push_back(elab_point);

              if (m_prev_elab_vector.size() >= m_prev_elab_cnt) {
                m_balanced_dac_code = floor(0.5 +
                  only_calc_elab_code(&m_prev_elab_vector, 0, m_prev_elab_cnt));

                if (m_prev_elab_vector.size() >= 2) {
                  double divider =
                    m_prev_elab_vector[m_prev_elab_cnt-1].dac -
                    m_prev_elab_vector[m_prev_elab_cnt-2].dac;
                  if (divider != 0) {
                    m_dac_step_amplitude
                      = abs((m_prev_elab_vector[m_prev_elab_cnt-1].adc -
                        m_prev_elab_vector[m_prev_elab_cnt-2].adc) / m_dac_code);
                  }
                }
                //irs::mlog() << irsm(">DAC> ");
                //irs::mlog() << m_dac_step_amplitude << irsm(" В") << endl;

                irs::mlog() << setw(2) << m_current_iteration+1 << irsm(" : ");
                irs::mlog() << setw(8) << m_dac_code << irsm(" : ");
                irs::mlog() << setw(8) << m_dac_step << irsm(" : ");
                irs::mlog() << setw(12) << adc_result << irsm(" : ");
                adc_value_t rel_sko = abs(adc_result_data.sko * 1e6);
                irs::mlog() << setw(8) << rel_sko << irsm(" ppm : ");
                irs::mlog() << adc_result_data.current_point << endl;
                irs::mlog() << irsm("PREV ") << setw(8) << m_balanced_dac_code;
                irs::mlog() << endl;
                //m_prev_balance_completed = true;
                if (elab_point.dac >= 0) {
                  m_elab_polarity = bp_pos;
                  m_balance_action = ba_elab_pos;
                } else {
                  m_elab_polarity = bp_neg;
                  m_balance_action = ba_elab_neg;
                }
                switch (m_elab_mode) {
                  case em_fast_2points: {
                    m_balance_status = bs_fast_elab_prepare;
                    break;
                  }
                  default: {
                    m_balance_status = bs_elab_prepare;
                    break;
                  }
                }
              } else {
                m_balance_status = bs_balance;
              }
            } else {
              m_balance_status = bs_balance;
            }
            // if (m_prev_balance_completed) {
              // m_balance_status = bs_elab_prepare;
            // } else {
              // m_balance_status = bs_balance;
            // }
          }
          break;
        }
        case bs_balance: {
          adc_result_data_t adc_result_data;
          m_adc.result(&adc_result_data);
          adc_value_t adc_result = adc_result_data.avg;
          if (m_current_iteration == 0
              && m_optimize_balance && abs(adc_result) <
                0.9 * m_adc.max_value()) {
            // Корректировка
            double ratio = (pow(2.0, 20) / 12.0) * abs(adc_result);
            double log_ratio = floor(log2(ratio) - 1.0);
            irs_u32 int_log_ratio = static_cast<size_t>(log_ratio);
            m_current_iteration = m_iteration_count - int_log_ratio - 1;
            irs_i32 int_dac_code = static_cast<irs_u32>(pow(2.0,int_log_ratio));
            irs_i32 int_dac_step = int_dac_code;
            if (adc_result > 0.0) {
              int_dac_code = -int_dac_code;
            }
            m_dac_code = static_cast<dac_value_t>(int_dac_code);
            m_dac_step = static_cast<dac_value_t>(int_dac_step);
            irs::mlog() << irsm(">>>OPT>>> ");
            irs::mlog() << int_dac_code;
            irs::mlog() << irsm(" ") << int_dac_step << endl;
          }
          if (m_current_iteration < m_iteration_count) {
            m_current_iteration++;
            irs::mlog() << setw(2) << m_current_iteration << irsm(" : ");
            irs::mlog() << setw(8) << m_dac_code << irsm(" : ");
            irs::mlog() << setw(8) << m_dac_step << irsm(" : ");
            irs::mlog() << setw(12) << adc_result << irsm(" : ");
            adc_value_t rel_sko = abs(adc_result_data.sko * 1e6);
            irs::mlog() << setw(8) << rel_sko << irsm(" ppm : ");
            irs::mlog() << adc_result_data.current_point << endl;
            if (m_dac_step_amplitude > 0.0
                && abs(adc_result) < 0.5 * m_dac_step_amplitude) {
              switch (m_elab_mode) {
                case em_fast_2points: {
                  m_balance_status = bs_fast_elab_prepare;
                  break;
                }
                default: {
                  m_balance_status = bs_elab_prepare;
                  break;
                }
              }
            } else {
              if (adc_result > 0) {
                m_dac_code += m_dac_step;
              } else {
                m_dac_code -= m_dac_step;
              }
              m_dac_step = floor(m_dac_step * 0.5);
              m_balance_status = bs_dac_set;
            }
          } else {
            m_balanced_dac_code = m_dac.get_code();
            switch (m_elab_mode) {
              case em_fast_2points: {
                m_balance_status = bs_fast_elab_prepare;
                break;
              }
              case em_none: {
                m_balance_status = bs_none_elab_prepare;
                break;
              }
              default: {
                m_balance_status = bs_elab_prepare;
                break;
              }
            }
          }
          break;
        }
        case bs_dac_set: {
          if (m_dac.ready()) {
            m_dac.set_code(m_dac_code);
            m_balance_status = bs_termostat_off_dac_wait;
          }
          break;
        }
        case bs_termostat_off_dac_wait: {
          if (m_dac.ready()) {
            m_termostat.set_off(true);
            m_device_condition_controller.set_idle(false);
            m_balance_status = bs_dac_wait;
          }
          break;
        }
        case bs_dac_wait: {
          if (m_termostat.status() == irs_st_ready) {
            m_balance_status = bs_adc_start;
          }
          break;
        }
        case bs_none_elab_prepare: {
          irs::mlog() << irsm("------- Без уточнения -------") << endl;
          irs::mlog() << irsm("Измеренный код ЦАП = ") 
            << m_balanced_dac_code << endl;
            
          elab_result_t elab_result;
          elab_result.polarity = m_balance_polarity;
          elab_result.start_code = static_cast<double>(m_balanced_dac_code);
          elab_result.code = elab_result.start_code;

          m_elab_result_vector.push_back(elab_result);
          m_pos_current_elab++;
          
          if (m_balance_polarity == bp_neg) {
            m_etalon_code = elab_result.code;
            m_etalon_balanced_code = m_balanced_dac_code;
          } else {
            m_checked_code = elab_result.code;
            m_checked_balanced_code = m_balanced_dac_code;
          }
          if (!m_no_prot) {
            m_relay_prot = 1;
            m_adc.set_max_value(m_adc_max_value_prot);
          }
          m_dac.set_after_pause(m_dac_after_pause);
          m_balance_status = bs_coils_off;
          break;
        }
        case bs_fast_elab_prepare: {
          //m_balanced_dac_code = 0.0;
          m_fast_elab_dac_step 
            = abs(round(0.85 * (m_dac_code - m_balanced_dac_code)));
          if (m_balanced_dac_code + m_fast_elab_dac_step > m_dac.max_code()) {
            m_fast_elab_dac_step = m_dac.max_code() - m_balanced_dac_code;
          } else if (m_balanced_dac_code - m_fast_elab_dac_step 
            < m_dac.min_code()) {
            m_fast_elab_dac_step = m_balanced_dac_code - m_dac.min_code();  
          }
          
          m_fast_elab_dac_step_2 
            = abs(round(0.8 * (m_dac_code - m_balanced_dac_code)));
          if (m_balanced_dac_code + m_fast_elab_dac_step_2 > m_dac.max_code()) {
            m_fast_elab_dac_step_2 = m_dac.max_code() - m_balanced_dac_code;
          } else if (m_balanced_dac_code - m_fast_elab_dac_step_2 
            < m_dac.min_code()) {
            m_fast_elab_dac_step_2 = m_balanced_dac_code - m_dac.min_code();  
          }
          
          m_fast_elab_dac_step_3 
            = abs(round(0.9 * (m_dac_code - m_balanced_dac_code)));
          if (m_balanced_dac_code + m_fast_elab_dac_step_3 > m_dac.max_code()) {
            m_fast_elab_dac_step_3 = m_dac.max_code() - m_balanced_dac_code;
          } else if (m_balanced_dac_code - m_fast_elab_dac_step_3 
            < m_dac.min_code()) {
            m_fast_elab_dac_step_3 = m_balanced_dac_code - m_dac.min_code();  
          }
          
          irs::mlog() << irsm("------- Быстрое уточнение -------") << endl;
          //irs::mlog() << irsm("Величина прыжка ЦАП = ") 
            //<< m_fast_elab_dac_step << endl;
          //  Внесение ассиметрии для нормальной работы новой формулы
          switch (m_fast_elab_vector.size()) {
            case 0: {
              m_dac_code = m_balanced_dac_code + m_fast_elab_dac_step_2;
              irs::mlog() << irsm("Величина прыжка ЦАП = ") 
                << m_fast_elab_dac_step_2 << endl;
              break;
            }
            case 2: {
              if (m_dac_code > m_balanced_dac_code) {
                m_dac_code = m_balanced_dac_code + m_fast_elab_dac_step_3;
              } else {
                m_dac_code = m_balanced_dac_code - m_fast_elab_dac_step_3;
              }
              //m_dac_code = m_balanced_dac_code + m_fast_elab_dac_step_3;
              irs::mlog() << irsm("Величина прыжка ЦАП = ") 
                << m_fast_elab_dac_step_3 << endl;
              break;
            }
            default: {
              m_dac_code = m_balanced_dac_code + m_fast_elab_dac_step;
              irs::mlog() << irsm("Величина прыжка ЦАП = ") 
                << m_fast_elab_dac_step << endl;
            }
          }
          if (m_fast_elab_vector.size() < 1)  {
            
          } else {
            
          }
          
          m_adc.set_params(&m_adc_elab_param_data);
          irs::mlog() << irsm("Параметры АЦП m_adc_elab_param_data") << endl;
          m_dac.set_after_pause(m_dac_elab_pause);
          m_current_iteration = 0;
          m_elab_iteration_count = 2;
          //m_fast_elab_vector.clear();
          m_balance_status = bs_fast_elab_dac_set;
          break;
        }
        case bs_fast_elab_dac_set: {
          if (m_dac.ready()) {
            m_dac.set_code(m_dac_code);
            irs::mlog() << setw(2) << (m_current_iteration + 1) << irsm(" : ");
            irs::mlog() << setw(8) << (m_dac_code) << irsm(" : ") << flush;
            m_balance_status = bs_fast_elab_adc_start;
          }
          break;
        }
        case bs_fast_elab_adc_start: {
          if (m_dac.ready()) {
            if (m_adc.status() == irs_st_ready) {
              m_adc.start_conversion();
              m_balance_status = bs_fast_elab_adc_read;
            }
          }
          break;
        }
        case bs_fast_elab_adc_read: {
          if (m_adc.status() == irs_st_ready) {
            adc_result_data_t adc_result_data;
            m_adc.result(&adc_result_data);
            adc_value_t adc_result = adc_result_data.unnormalized_value;
            adc_value_t adc_voltage = adc_result_data.avg;
            elab_point_t elab_point;
            elab_point.dac = m_dac.get_code();
            elab_point.adc = adc_result;
            elab_point.sko = adc_result_data.sko;
            elab_point.avg = adc_result;
            elab_point.num_of_adc_points = adc_result_data.current_point;
            m_fast_elab_vector.push_back(elab_point);
            
            irs::mlog() << setw(12) << (adc_voltage) << irsm(" В : ");
            irs::mlog() << (elab_point.sko * 1e6) << irsm(" ppm : ");
            irs::mlog() << adc_result_data.current_point << endl;
            
            m_balance_status = bs_fast_elab_point_processing;
          }
          break;
        }
        case bs_fast_elab_point_processing: {
          if (m_current_iteration < (m_elab_iteration_count - 1)) {
            m_current_iteration++;
            //m_pos_current_elab++;
            //m_dac_code = m_balanced_dac_code - m_fast_elab_dac_step;
            switch (m_fast_elab_vector.size()) {
              case 1: {
                m_dac_code = m_balanced_dac_code - m_fast_elab_dac_step_2;
                break;
              }
              case 3: {
                if (m_dac_code > m_balanced_dac_code) {
                  m_dac_code = m_balanced_dac_code - m_fast_elab_dac_step_3;
                } else {
                  m_dac_code = m_balanced_dac_code + m_fast_elab_dac_step_3;
                }
                break;
              }
              default: {
                m_dac_code = m_balanced_dac_code - m_fast_elab_dac_step;
              }
            }
            m_balance_status = bs_fast_elab_dac_set;
          } else {
            m_balance_status = bs_fast_elab_result;
          }
          break;
        }
        case bs_fast_elab_result: {
          elab_result_t elab_result;
          elab_result.polarity = m_balance_polarity;
          elab_result.start_code = static_cast<double>(m_balanced_dac_code);
          size_t shift = 0;
          if (m_balance_polarity == bp_pos) {
            shift = 1;
          }
          elab_result.code = calc_elab_code_2point(&m_fast_elab_vector,
            shift);

          m_elab_result_vector.push_back(elab_result);
          m_pos_current_elab++;
          
          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("Измеренный код (старый вариант) = ");
          irs::mlog() << elab_result.code << endl;
          
          if (m_balance_polarity == bp_neg) {
            m_etalon_code = elab_result.code;
            m_etalon_balanced_code = m_balanced_dac_code;
          } else {
            m_checked_code = elab_result.code;
            m_checked_balanced_code = m_balanced_dac_code;
          }
          if (!m_no_prot) {
            m_relay_prot = 1;
            m_adc.set_max_value(m_adc_max_value_prot);
          }
          m_dac.set_after_pause(m_dac_after_pause);
          m_balance_status = bs_coils_off;
          break;
        }
        case bs_elab_prepare: {
          irs::mlog() << irsm("----------- Уточнение -----------") << endl;
          if (m_elab_iteration_count < 2) {
            irs::mlog() << irsm("Уточнения не будет!!! MWA-HA-HA!!") << endl;
            elab_point_t elab_point;
            elab_point.dac = m_balanced_dac_code;
            elab_point.adc = m_voltage;
            elab_point.sko = 0.;
            elab_point.avg = m_voltage;
            m_elab_vector.push_back(elab_point);
            if (m_balance_polarity == bp_neg) {
              m_etalon_code = static_cast<double>(m_balanced_dac_code);
            } else {
              m_checked_code = static_cast<double>(m_balanced_dac_code);
            }
            m_balance_status = bs_coils_off;
          } else {
            if (m_auto_elab_step) {
              irs_i32 dac_swing =
                static_cast<irs_i32>(m_elab_step_multiplier * 2.0 *
                abs(m_max_unsaturated_dac_code - abs(m_balanced_dac_code)));
              m_elab_step = 2 * dac_swing / m_elab_iteration_count;
              irs::mlog() << irsm("Размах значений ЦАП = ");
              irs::mlog() << dac_swing << endl;
              irs::mlog() << irsm("Размах значений АЦП = ");
              irs::mlog() << m_max_unsaturated_voltage * 2.0 << endl;
            }

            irs::mlog() << irsm("Число точек = ");
            irs::mlog() << m_elab_iteration_count << endl;
            irs::mlog() << irsm("Начальный шаг = ");
            irs::mlog() << m_elab_step << endl;

            bool invalid_experiment = false;

            if (m_elab_step > (m_dac.max_code() - m_dac.min_code())) {
              invalid_experiment = true;
            }
            if (m_balanced_dac_code > m_dac.max_code()
                || m_balanced_dac_code < m_dac.min_code()) {
              invalid_experiment = true;
            }
            if (invalid_experiment) {
              irs::mlog()
                << irsm(">>>> Ахтунг! Неправильно померялось! <<<<") << endl;
              m_exp_time = 0;
              m_eth_data.exp_time = m_exp_time;
              m_max_unsaturated_voltage = 0.0;
              m_max_unsaturated_dac_code = 0.0;

              m_elab_vector.clear();
              m_elab_result_vector.clear();
              m_pos_current_elab = 0;

              m_balance_polarity = bp_neg;

              m_relay_bridge_pos = 0;
              m_relay_bridge_neg = 0;
              m_balance_status = bs_set_prot;
            } else {
              m_current_elab = 0;
              m_ok_elab_cnt = 0;
              m_dac_code = m_balanced_dac_code;
              m_dac.set_after_pause(m_dac_elab_pause);

              m_balance_status = bs_elab_start;
            }
          }
          break;
        }
        case bs_elab_start: {
          m_current_iteration = 0;
          dac_value_t shift = static_cast<dac_value_t>(
            (static_cast<irs_i32>((m_elab_iteration_count-1)/2)) * m_elab_step);
          if (m_elab_iteration_count % 2 == 0) {
            dac_value_t step = static_cast<dac_value_t>(m_elab_step / 2);
            if (step == 0.0) {
              step = 1.0;
            }
            shift += step;
          }
          if (m_elab_polarity == bp_pos) {
            m_dac_code += shift;
          } else {
            m_dac_code -= shift;
          }

          irs::mlog() << irsm("----------- Уточнение № ");
          irs::mlog() << m_current_elab + 1;
          irs::mlog() << irsm(" -----------") << endl;
          irs::mlog() << irsm("Шаг = ");
          irs::mlog() << m_elab_step << endl;
          irs::mlog() << irsm("Баланс = ");
          irs::mlog() << m_balanced_dac_code << endl;
          irs::mlog() << irsm("Начальный код = ") << m_dac_code << endl;
          irs::mlog() << irsm("---------------------------------") << endl;

          m_balance_status = bs_elab_relay_on;
          break;
        }
        case bs_elab_relay_on: {
          if (!m_no_prot && m_relay_prot.status() == irs_st_ready) {
            m_relay_prot = 0;
            m_adc.set_max_value(m_adc_max_value_no_prot);
          }
          m_balance_status = bs_elab_relay_wait;
          break;
        }
        case bs_elab_relay_wait: {
          if (m_relay_prot.status() == irs_st_ready) {
            m_balance_status = bs_elab_dac_set;
          }
          break;
        }
        case bs_elab_dac_set: {
          if (m_dac.ready()) {
            m_dac.set_code(m_dac_code);
            m_balance_status = bs_termostat_off_elab_adc_start;
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_adc.start_conversion();
            }
          }
          break;
        }
        case bs_termostat_off_elab_adc_start: {
          if (m_dac.ready()) {
            m_termostat.set_off(true);
            m_device_condition_controller.set_idle(false);
            m_balance_status = bs_elab_adc_start;
          }
          break;
        }
        case bs_elab_adc_start: {
          if (m_termostat.status() == irs_st_ready) {
            m_adc.start_conversion();
            m_balance_status = bs_elab_adc_wait;
          }
          break;
        }
        case bs_elab_adc_wait: {
          if (m_adc.status() == irs_st_ready) {
            m_termostat.set_off(false);
            m_device_condition_controller.set_idle(true);
            elab_point_t elab_point;
            elab_point.dac = m_dac.get_code();
            elab_point.adc = m_voltage;
            elab_point.sko = m_adc_result_data.sko;
            elab_point.avg = m_voltage;
            m_elab_vector.push_back(elab_point);

            irs::mlog() << setw(2) << (m_current_iteration + 1) << irsm(" : ");
            irs::mlog() << setw(8) << (m_dac_code) << irsm(" : ");
            irs::mlog() << setw(12) << (m_voltage) << irsm(" В") << endl;

            if ((m_current_iteration + 1) < m_elab_iteration_count) {
              m_current_iteration++;
              if (m_elab_polarity == bp_pos) {
                m_dac_code -= m_elab_step;
              } else {
                m_dac_code += m_elab_step;
              }
              m_balance_status = bs_elab_dac_set;
            } else {
              m_balance_status = bs_elab_result;
            }
          }
          break;
        }
        case bs_elab_result: {
          elab_result_t elab_result;
          elab_result.polarity = m_balance_polarity;
          elab_result.start_code = static_cast<double>(m_balanced_dac_code);
          elab_result.code = only_calc_elab_code(&m_elab_vector,
            m_pos_current_elab, m_elab_iteration_count);

          m_elab_result_vector.push_back(elab_result);

          double elab_delta_code
            = abs(elab_result.start_code - elab_result.code);
          //if (m_elab_step_multiplier >= 1.0) {
          if (m_auto_elab_step) {
            m_elab_step -= 2 * static_cast<irs_i32>(elab_delta_code + 0.5);
          }
          m_balanced_dac_code = floor(elab_result.code + 0.5);

          irs::mlog() << irsm("---------------------------------") << endl;
          irs::mlog() << irsm("Измеренный код = ");
          irs::mlog() << elab_result.code << endl;
          irs::mlog() << irsm("Изменённый шаг = ") << m_elab_step << endl;

          if (elab_delta_code < m_elab_max_delta && m_current_elab > 0) {
            m_ok_elab_cnt++;
          } else {
            m_ok_elab_cnt = 0;
          }

          if (m_ok_elab_cnt >= m_min_elab_cnt
              || m_current_elab >= m_max_elab_cnt) {
            if (m_balance_polarity == bp_neg) {
              m_etalon_code = elab_result.code;
              m_etalon_balanced_code = m_balanced_dac_code;
            } else {
              m_checked_code = elab_result.code;
              m_checked_balanced_code = m_balanced_dac_code;
            }
            if (!m_no_prot) {
              m_relay_prot = 1;
              m_adc.set_max_value(m_adc_max_value_prot);
            }
            m_dac.set_after_pause(m_dac_after_pause);
            m_balance_status = bs_coils_off;
          } else {
            m_current_elab++;
            m_pos_current_elab++;
            m_dac_code = m_balanced_dac_code;
            m_balance_status = bs_elab_start;
          }
          break;
        }
        case bs_coils_off: {
          if (m_relay_prot.status() == irs_st_ready) {
            m_relay_bridge_pos = 0;
            m_relay_bridge_neg = 0;
            m_balance_status = bs_wait_relays;
          }
          break;
        }
        case bs_wait_relays: {
          if (m_relay_prot.status() == irs_st_ready && bridge_relays_ready()) {
            if (m_balance_polarity == bp_neg) {
              m_balance_polarity = bp_pos;
              m_max_unsaturated_voltage = 0.0;
              m_max_unsaturated_dac_code = 0.0;
              m_pos_current_elab++;
              m_balance_action = ba_balance_pos;
              m_balance_status = bs_set_coils;
            } else {
              m_balance_action = ba_idle;
              m_balance_status = bs_report;
            }
          }
          break;
        }
        case bs_report: {
          irs::mlog() << irsm("---------------------------------") << endl;
          m_buzzer.bzz(2);
          switch (m_elab_mode) {
            case em_fast_2points: {
              irs::mlog() << endl;
              irs::mlog() << irsm("Результаты уточнения") << endl;
              for (size_t i = 0; i < m_elab_result_vector.size(); i++) {
                irs::mlog() << setw(2) << i + 1;
                irs::mlog() << irsm(" <");
                irs::mlog() << setw(1) << m_elab_result_vector[i].polarity;
                irs::mlog() << irsm("> ");
                irs::mlog() << setw(7) << m_elab_result_vector[i].start_code;
                irs::mlog() << irsm(" ");
                irs::mlog() << setw(8) << m_elab_result_vector[i].code;
                irs::mlog() << endl;
              }
              //  ---------------      RESULTS         -------------------------
              irs::mlog() << endl;
              exp_t exp;
              exp.temperature_ext = m_eth_data.th_ext_1;
              exp.temperature_dac = m_eth_data.th_dac;
              exp.temperature_adc = m_eth_data.th_box_adc;
              exp.temperature_ldo = m_eth_data.th_box_ldo;
              //  ---------------  OLD FORMULA RESULT  -------------------------
              m_checked_code /= pow(2., 19);
              m_etalon_code /= pow(2., 19);
              //  m_bac_old_coefficient - смещение в попугаях
              double A = (m_bac_old_coefficient) / pow(2.0, 19);
              m_result = (2.0 - m_checked_code + m_etalon_code - A);
              m_result /= (2.0 + m_checked_code - m_etalon_code + A);
              //
              exp.result_old_uncorrect = (2.0 - m_checked_code + m_etalon_code);
              exp.result_old_uncorrect /= (2.0 + m_checked_code - m_etalon_code);
              
              irs::mlog() << irsm("Старый результат") << endl;
              irs::mlog() << irsm("BAC OLD = ") << m_bac_old_coefficient;
              irs::mlog() << endl;
              irs::mlog() << irsm("A = ") << A << endl;
              irs::mlog() << irsm("RESULT OLD = ");
              irs::mlog() << setw(14) << setprecision(14);
              irs::mlog() << m_result << endl;
              irs::mlog() << irsm("RESULT OLD UNCORRECT = ");
              irs::mlog() << exp.result_old_uncorrect << endl << endl;
              
              exp.result_old = m_result;
              exp.ch_code = m_checked_code;
              exp.et_code = m_etalon_code;
              exp.et_balanced_code = m_etalon_balanced_code;
              exp.ch_balanced_code = m_checked_balanced_code;
              //  --------------------------------------------------------------
              
              //  ---------------  NEW FORMULA RESULT  -------------------------
              irs::mlog() << irsm("Новый результат") << endl;
              //  --------------------------------------------------------------
              //irs::mlog() << irsm("FLOAT") << endl;
              const size_t index_1n = 2;
              const size_t index_2n = 3;
              const size_t index_1p = 0;
              const size_t index_2p = 1;
              adc_value_t m1n = m_fast_elab_vector[index_1n].adc;
              adc_value_t m1p = m_fast_elab_vector[index_1p].adc;
              adc_value_t m2n = m_fast_elab_vector[index_2n].adc;
              adc_value_t m2p = m_fast_elab_vector[index_2p].adc;
              adc_value_t n1n = m_fast_elab_vector[index_1n].dac;
              adc_value_t n1p = m_fast_elab_vector[index_1p].dac;
              adc_value_t n2n = m_fast_elab_vector[index_2n].dac;
              adc_value_t n2p = m_fast_elab_vector[index_2p].dac;
              
              adc_value_t B = m_bac_new_coefficient;
              
              adc_value_t n0 = 0.0;
              adc_value_t num = 0.0;
              adc_value_t denom = 0.0;
              adc_value_t h = pow(2.0, 19);
              
              num = (m2p - m2n) * (n1p - n1n - 1.0) 
                    - (m1p - m1n) * (n2p - n2n - 1.0);
                    
              denom = (2.0 * ((m2p - m2n) - (m1p - m1n)));
              
              n0 = num / denom;
              exp.n0 = n0;
              exp.result_new =  (1.0 + num / denom / h) 
                            / (1.0 - ((1.0 - B) * denom + num) / denom / h);
              exp.result_new_uncorrect = (1.0 + num / denom / h) 
                            / (1.0 - ((1.0 - 0.0) * denom + num) / denom / h);
              exp.num = num;
              exp.den = denom;
                            
//              irs::mlog() << irsm("m1n = ") << m1n << endl;
//              irs::mlog() << irsm("m1p = ") << m1p << endl;
//              irs::mlog() << irsm("m2n = ") << m2n << endl;
//              irs::mlog() << irsm("m2p = ") << m2p << endl;
//              irs::mlog() << irsm("n1n = ") << n1n << endl;
//              irs::mlog() << irsm("n1p = ") << n1p << endl;
//              irs::mlog() << irsm("n2n = ") << n2n << endl;
//              irs::mlog() << irsm("n2p = ") << n2p << endl;
//              irs::mlog() << endl;
//              irs::mlog() << irsm("m2p - m2n = ") << (m2p - m2n) << endl;
//              irs::mlog() << irsm("n1p - n1n = ") << (n1p - n1n) << endl;
//              irs::mlog() << irsm("m1p - m1n = ") << (m1p - m1n) << endl;
//              irs::mlog() << irsm("n2p - n2n = ") << (n2p - n2n) << endl;
//              irs::mlog() << endl;
//              irs::mlog() << irsm("BAC NEW = ");
//              irs::mlog() << m_bac_new_coefficient << endl;
//              irs::mlog() << endl;
//              irs::mlog() << irsm("Denom     = ") << denom << endl;
//              irs::mlog() << irsm("Numer     = ") << num << endl;
//              irs::mlog() << endl;
//              irs::mlog() << irsm("n0 = ") << n0 << endl;
//              irs::mlog() << irsm("Результат NEW = ");
//              irs::mlog() << setw(14) << setprecision(14);
//              irs::mlog() << exp.result_new;
//              irs::mlog() << irsm("Результат NEW UNCORRECT = ");
//              irs::mlog() << exp.result_new_uncorrect;
//              irs::mlog() << endl;
//              irs::mlog() << endl;
              //  --------------------------------------------------------------
              /*irs::mlog() << irsm("INT") << endl;
              adc_value_t mult = m_bac_new_int_multiplier;
              irs::mlog() << irsm("MULT = ") << mult << endl;
              
              irs_i64 M1N = static_cast<irs_i64>(m1n * mult);
              irs_i64 M1P = static_cast<irs_i64>(m1p * mult);
              irs_i64 M2N = static_cast<irs_i64>(m2n * mult);
              irs_i64 M2P = static_cast<irs_i64>(m2p * mult);
              irs_i64 N1N = static_cast<irs_i64>(n1n);
              irs_i64 N1P = static_cast<irs_i64>(n1p);
              irs_i64 N2N = static_cast<irs_i64>(n2n);
              irs_i64 N2P = static_cast<irs_i64>(n2p);
              
              irs_i64 NUM = 0;
              irs_i64 DENOM = 0;
              irs_i64 DENOM_CORR = 0;*/
              //irs_i64 B_INT = m_bac_new_int_coefficient;
              
              /*NUM = (M2P - M2N) * (N1P - N1N - 1) 
                  - (M1P - M1N) * (N2P - N2N - 1);
                    
              DENOM = 2 * ((M2P - M2N) - (M1P - M1N));
              DENOM_CORR = 
                static_cast<irs_i64>(static_cast<adc_value_t>(DENOM) * B);
              
              n0 =  static_cast<adc_value_t>(NUM) 
                  / static_cast<adc_value_t>(DENOM);
              
              irs_i64 HALF = 1 << 19;
              irs_i64 NUM_N0 = NUM + HALF * DENOM;
              irs_i64 DENOM_N0 = (HALF - 1) * DENOM - NUM + DENOM_CORR;*/
              
              //exp.n0_5 = n0;
              //exp.result_5 =  static_cast<adc_value_t>(NUM_N0) 
              //              / static_cast<adc_value_t>(DENOM_N0);
                            
              /*irs::mlog() << irsm("M1N = ") << M1N << endl;
              irs::mlog() << irsm("M1P = ") << M1P << endl;
              irs::mlog() << irsm("M2N = ") << M2N << endl;
              irs::mlog() << irsm("M2P = ") << M2P << endl;
              irs::mlog() << irsm("N1N = ") << N1N << endl;
              irs::mlog() << irsm("N1P = ") << N1P << endl;
              irs::mlog() << irsm("N2N = ") << N2N << endl;
              irs::mlog() << irsm("N2P = ") << N2P << endl;
              irs::mlog() << endl;
              irs::mlog() << irsm("M2P - M2N = ") << (M2P - M2N) << endl;
              irs::mlog() << irsm("N1P - N1N = ") << (N1P - N1N) << endl;
              irs::mlog() << irsm("M1P - M1N = ") << (M1P - M1N) << endl;
              irs::mlog() << irsm("N2P - N2N = ") << (N2P - N2N) << endl;
              irs::mlog() << endl;
              irs::mlog() << irsm("BAC NEW INT = ");
              irs::mlog() << m_bac_new_int_coefficient << endl;
              irs::mlog() << endl;
              irs::mlog() << irsm("DENOM   = ") << DENOM << endl;
              irs::mlog() << irsm("NUM     = ") << NUM << endl;
              irs::mlog() << endl;
              irs::mlog() << irsm("N0 = ") << n0 << endl;
              irs::mlog() << irsm("РЕЗУЛЬТАТ NEW INT ");
              irs::mlog() << setw(12) << exp.result_5 << endl;
              irs::mlog() << endl;
              irs::mlog() << endl;*/
              //  --------------------------------------------------------------
              
              // irs::mlog() << irsm("n0 = ") << n0 << endl;
              // irs::mlog() << irsm("Результат 4 ") << setw(12) << exp.result_4
              // num_5 = ((m_fast_elab_vector[index_2p].adc - m_fast_elab_vector[index_2n].adc)) * (m_fast_elab_vector[index_1p].dac - m_fast_elab_vector[index_1n].dac - 1.0) - ((m_fast_elab_vector[index_1p].adc - m_fast_elab_vector[index_1n].adc)) * (m_fast_elab_vector[index_2p].dac - m_fast_elab_vector[index_2n].dac - 1.0);
              // denom_5 = (2.0 * ((m_fast_elab_vector[index_2p].adc - m_fast_elab_vector[index_2n].adc) - (m_fast_elab_vector[index_1p].adc - m_fast_elab_vector[index_1n].adc)));
              //n0 = (m2p - m2n) * (n1p - n1n - 1.0) - (m1p - m1n) * (n2p - n2n - 1.0);
              //n0 /= (2.0 * ((m2p - m2n) - (m1p - m1n)));
              //irs::mlog() << irsm("B   = ") << B << endl;
              //num_5 = static_cast<adc_value_t>(NUM_5);
              //denom_5 = static_cast<adc_value_t>(DENOM_5);
              //exp.result_5 = (n0 + pow(2.0, 19)) / (pow(2., 19) - 1.0 - n0);
              //exp.result_5 = (num_5 + pow(2.0, 19) * denom_5) / ((pow(2.0, 19) - 1.0) * denom_5 - num_5);
              // irs::mlog() << irsm("Результат 5 via n0") << setw(12) << n0
              // << endl;
              
              m_exp_vector.push_back(exp);
                
              m_fast_elab_vector.clear();
              
              m_eth_data.result = exp.result_old;     //  Result OLD
              //m_eth_data.ratio = exp.result_5;        //  Result INT
              m_eth_data.result_error = exp.result_new; //  Result FLOAT
              m_eth_data.ratio = m_result;
              break;
            }
            case em_none: {
              m_checked_code /= pow(2., 19);
              m_etalon_code /= pow(2., 19);
              m_result = (2. - m_checked_code + m_etalon_code);
              m_result /= (2. + m_checked_code - m_etalon_code);
              irs::mlog() << irsm("Старый результат ") << setw(14) <<
                setprecision(14) << m_result << endl << endl;
              
              exp_t exp;
              exp.result_old = m_result;
              exp.ch_code = m_checked_code;
              exp.et_code = m_etalon_code;
              
              exp.n0 = 0.0;
              exp.result_new = 0.0;

              m_exp_vector.push_back(exp);
                
              m_fast_elab_vector.clear();
              
              m_eth_data.result = exp.result_old;
              //m_eth_data.ratio = exp.result_new;
              m_eth_data.ratio = m_result;
              break;
            }
            default: {
              if (m_elab_iteration_count >= 2) {
                irs::mlog() << irsm("Результаты уточнения") << endl;
                for (size_t i = 0; i < m_elab_result_vector.size(); i++) {
                  irs::mlog() << setw(2) << i + 1;
                  irs::mlog() << irsm(" <");
                  irs::mlog() << setw(1) << m_elab_result_vector[i].polarity;
                  irs::mlog() << irsm("> ");
                  irs::mlog() << setw(7) << m_elab_result_vector[i].start_code;
                  irs::mlog() << irsm(" ");
                  irs::mlog() << setw(8) << m_elab_result_vector[i].code;
                  irs::mlog() << endl;
                }
              }
              //m_checked_code = calc_elab_code(&m_elab_vector, bc_checked);
              //m_etalon_code = calc_elab_code(&m_elab_vector, bc_etalon);
              //m_result = 1. - (m_checked_code - m_etalon_code) / 2.;
              //m_result = m_result * m_etalon;
              m_checked_code /= pow(2., 19);
              m_etalon_code /= pow(2., 19);
              //m_result = (2. - m_checked_code + m_etalon_code);
              //m_result /= (2. + m_checked_code - m_etalon_code);
              
              m_result = (2. - m_checked_code - m_etalon_code);
              m_result /= (2. + m_checked_code + m_etalon_code);
              
              exp_t exp;
              exp.result_old = m_result;
              exp.ch_code = m_checked_code;
              exp.et_code = m_etalon_code;
              
              exp.n0 = 0.0;
              exp.result_new = 0.0;
              
              m_fast_elab_vector.clear();
              
              m_eth_data.result = exp.result_old;
              //m_eth_data.ratio = exp.result_new;
              m_eth_data.ratio = m_result;
              
              m_exp_vector.push_back(exp);
            }
          }
          m_result *= m_etalon;
          m_result_error = ((m_result - m_checked) / m_checked) * 100.;
          irs::mlog() << irsm("Результат ") << m_result
            << irsm(" Ом") << endl;
          irs::mlog() << irsm("Отклонение ") << m_result_error
            << irsm(" %") << endl;
          m_eth_data.result = m_result;
          //m_eth_data.result_error = m_result_error;
          m_balance_status = bs_next_exp;
          break;
        }
        case bs_next_exp: {
          // exp_t exp;
          // exp.result = m_result;
          // exp.error = m_result_error;
          // exp.ch_code = m_checked_code;
          // exp.et_code = m_etalon_code;
          // m_exp_vector.push_back(exp);
          //irs::mlog() << int(m_eth_data.current_exp_cnt) << endl;
          m_eth_data.current_exp_cnt--;
          
          //irs::mlog() << int(m_eth_data.current_exp_cnt) << endl;
          m_prev_exp_time = m_exp_time;
          m_eth_data.prev_exp_time = m_prev_exp_time;
          m_exp_time = 0;
          m_eth_data.exp_time = m_exp_time;
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

            m_max_unsaturated_voltage = 0.0;
            m_max_unsaturated_dac_code = 0.0;

            m_elab_vector.clear();
            m_elab_result_vector.clear();
            m_pos_current_elab = 0;

            m_balance_polarity = bp_neg;

            m_balance_status = bs_set_coils;
          }
          break;
        }
        case bs_final_report: {
          show_last_result();
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

void hrm::app_t::print_voltage(adc_value_t a_value)
{
  if (abs(a_value) < 1.1e-3) {
    irs::mlog() << (a_value * 1.e6) << irsm(" мкВ");
  } else if (abs(a_value < 1.1)) {
    irs::mlog() << (a_value * 1.e3) << irsm(" мВ");
  } else {
    irs::mlog() << a_value << irsm(" В");
  }
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

void hrm::app_t::update_elab_pid_koefs()
{
  m_elab_pid.k = m_elab_pid_kp;
  m_elab_pid.ki = m_elab_pid_ki * m_adc.get_reference_frequency();
  if (m_adc.get_reference_frequency() > 0.0) {
    m_elab_pid.kd = m_elab_pid_kd / m_adc.get_reference_frequency();
  }
  m_elab_iso.fd.x1 = m_voltage;
  m_elab_iso.fd.y1 = m_voltage;
  m_elab_iso.fd.t = m_elab_iso_t * m_adc.get_reference_frequency();
  
  m_elab_pid_fade.x1 = m_dac.get_normalize_code();
  m_elab_pid_fade.y1 = m_dac.get_normalize_code();
  m_elab_pid_fade.t = m_elab_pid_fade_t * m_adc.get_reference_frequency();
  
  pid_reg_sync(&m_elab_pid, m_elab_pid_ref - m_voltage, 
    m_dac.get_normalize_code());
}


hrm::dac_value_t hrm::app_t::norm(hrm::dac_value_t a_in)
{
  dac_value_t out = 0.0;
  if (a_in > 0.0) {
    out = a_in / (pow(2.0, 19) - 1.0); 
  } else {
    out = a_in / pow(2.0, 19);
  }
  return out;
}

hrm::dac_value_t hrm::app_t::denorm(hrm::dac_value_t a_in)
{
  dac_value_t out = 0.0;
  if (a_in > 0.0) {
    out = a_in * (pow(2.0, 19) - 1.0); 
  } else {
    out = a_in * pow(2.0, 19);
  }
  return out;
}

void hrm::app_t::adc_params_load_from_eeprom()
{
  //  ADC free / voltage
  m_eth_data.adc_free_vx_gain = m_eeprom_data.adc_free_vx_gain;
  m_eth_data.adc_free_vx_filter = m_eeprom_data.adc_free_vx_filter;
  m_eth_data.adc_free_vx_channel = m_eeprom_data.adc_free_vx_channel;
  m_eth_data.adc_free_vx_cnv_cnt = m_eeprom_data.adc_free_vx_cnv_cnt;
  m_eth_data.adc_free_vx_additional_gain 
    = m_eeprom_data.adc_free_vx_additional_gain;
  m_eth_data.adc_free_vx_ref = m_eeprom_data.adc_free_vx_ref;
  m_eth_data.adc_free_vx_cont_cnv_cnt = m_eeprom_data.adc_free_vx_cont_cnv_cnt;
  m_eth_data.adc_free_vx_impf_iterations_cnt
    = m_eeprom_data.adc_free_vx_impf_iterations_cnt;
  m_eth_data.adc_free_vx_impf_type = m_eeprom_data.adc_free_vx_impf_type;
  m_eth_data.adc_free_vx_cont_mode = m_eeprom_data.adc_free_vx_cont_mode;
  m_eth_data.adc_free_vx_cont_sko = m_eeprom_data.adc_free_vx_cont_sko;
  
  m_adc_free_vx_param_data.gain = m_eeprom_data.adc_free_vx_gain;
  m_adc_free_vx_param_data.filter = m_eeprom_data.adc_free_vx_filter;
  m_adc_free_vx_param_data.channel = m_eeprom_data.adc_free_vx_channel;
  m_adc_free_vx_param_data.cnv_cnt = m_eeprom_data.adc_free_vx_cnv_cnt;
  m_adc_free_vx_param_data.additional_gain 
    = m_eeprom_data.adc_free_vx_additional_gain;
  m_adc_free_vx_param_data.ref = m_eeprom_data.adc_free_vx_ref;
  m_adc_free_vx_param_data.cont_cnv_cnt
    = m_eeprom_data.adc_free_vx_cont_cnv_cnt;
  m_adc_free_vx_param_data.impf_iterations_cnt
    = m_eeprom_data.adc_free_vx_impf_iterations_cnt;
  m_adc_free_vx_param_data.impf_type 
    = convert_to_impf_type(m_eeprom_data.adc_free_vx_impf_type);
  m_adc_free_vx_param_data.cont_mode 
    = convert_to_cont_mode(m_eeprom_data.adc_free_vx_cont_mode);
  m_adc_free_vx_param_data.cont_sko = m_eeprom_data.adc_free_vx_cont_sko;
  //  ADC free / temperature
  m_eth_data.adc_free_th_gain = m_eeprom_data.adc_free_th_gain;
  m_eth_data.adc_free_th_filter = m_eeprom_data.adc_free_th_filter;
  m_eth_data.adc_free_th_channel = m_eeprom_data.adc_free_th_channel;
  m_eth_data.adc_free_th_cnv_cnt = m_eeprom_data.adc_free_th_cnv_cnt;
  m_eth_data.adc_free_th_additional_gain 
    = m_eeprom_data.adc_free_th_additional_gain;
  m_eth_data.adc_free_th_ref = m_eeprom_data.adc_free_th_ref;
  m_eth_data.adc_free_th_cont_cnv_cnt = m_eeprom_data.adc_free_th_cont_cnv_cnt;
  m_eth_data.adc_free_th_impf_iterations_cnt
    = m_eeprom_data.adc_free_th_impf_iterations_cnt;
  m_eth_data.adc_free_th_impf_type = m_eeprom_data.adc_free_th_impf_type;
  m_eth_data.adc_free_th_cont_mode = m_eeprom_data.adc_free_th_cont_mode;
  m_eth_data.adc_free_th_cont_sko = m_eeprom_data.adc_free_th_cont_sko;
  
  m_adc_free_th_param_data.gain = m_eeprom_data.adc_free_th_gain;
  m_adc_free_th_param_data.filter = m_eeprom_data.adc_free_th_filter;
  m_adc_free_th_param_data.channel = m_eeprom_data.adc_free_th_channel;
  m_adc_free_th_param_data.cnv_cnt = m_eeprom_data.adc_free_th_cnv_cnt;
  m_adc_free_th_param_data.additional_gain 
    = m_eeprom_data.adc_free_th_additional_gain;
  m_adc_free_th_param_data.ref = m_eeprom_data.adc_free_th_ref;
  m_adc_free_th_param_data.cont_cnv_cnt
    = m_eeprom_data.adc_free_th_cont_cnv_cnt;
  m_adc_free_th_param_data.impf_iterations_cnt
    = m_eeprom_data.adc_free_th_impf_iterations_cnt;
  m_adc_free_th_param_data.impf_type 
    = convert_to_impf_type(m_eeprom_data.adc_free_th_impf_type);
  m_adc_free_th_param_data.cont_mode 
    = convert_to_cont_mode(m_eeprom_data.adc_free_th_cont_mode);
  m_adc_free_th_param_data.cont_sko = m_eeprom_data.adc_free_th_cont_sko;
  //  ADC manual
  m_eth_data.adc_manual_gain = m_eeprom_data.adc_manual_gain;
  m_eth_data.adc_manual_filter = m_eeprom_data.adc_manual_filter;
  m_eth_data.adc_manual_channel = m_eeprom_data.adc_manual_channel;
  m_eth_data.adc_manual_cnv_cnt = m_eeprom_data.adc_manual_cnv_cnt;
  m_eth_data.adc_manual_additional_gain 
    = m_eeprom_data.adc_manual_additional_gain;
  m_eth_data.adc_manual_ref = m_eeprom_data.adc_manual_ref;
  m_eth_data.adc_manual_cont_cnv_cnt = m_eeprom_data.adc_manual_cont_cnv_cnt;
  m_eth_data.adc_manual_impf_iterations_cnt
    = m_eeprom_data.adc_manual_impf_iterations_cnt;
  m_eth_data.adc_manual_impf_type = m_eeprom_data.adc_manual_impf_type;
  m_eth_data.adc_manual_cont_mode = m_eeprom_data.adc_manual_cont_mode;
  m_eth_data.adc_manual_cont_sko = m_eeprom_data.adc_manual_cont_sko;
  
  m_adc_manual_param_data.gain = m_eeprom_data.adc_manual_gain;
  m_adc_manual_param_data.filter = m_eeprom_data.adc_manual_filter;
  m_adc_manual_param_data.channel = m_eeprom_data.adc_manual_channel;
  m_adc_manual_param_data.cnv_cnt = m_eeprom_data.adc_manual_cnv_cnt;
  m_adc_manual_param_data.additional_gain 
    = m_eeprom_data.adc_manual_additional_gain;
  m_adc_manual_param_data.ref = m_eeprom_data.adc_manual_ref;
  m_adc_manual_param_data.cont_cnv_cnt
    = m_eeprom_data.adc_manual_cont_cnv_cnt;
  m_adc_manual_param_data.impf_iterations_cnt
    = m_eeprom_data.adc_manual_impf_iterations_cnt;
  m_adc_manual_param_data.impf_type 
    = convert_to_impf_type(m_eeprom_data.adc_manual_impf_type);
  m_adc_manual_param_data.cont_mode 
    = convert_to_cont_mode(m_eeprom_data.adc_manual_cont_mode);
  m_adc_manual_param_data.cont_sko = m_eeprom_data.adc_manual_cont_sko;
  //  ADC balance
  m_eth_data.adc_balance_gain = m_eeprom_data.adc_balance_gain;
  m_eth_data.adc_balance_filter = m_eeprom_data.adc_balance_filter;
  m_eth_data.adc_balance_channel = m_eeprom_data.adc_balance_channel;
  m_eth_data.adc_balance_cnv_cnt = m_eeprom_data.adc_balance_cnv_cnt;
  m_eth_data.adc_balance_additional_gain 
    = m_eeprom_data.adc_balance_additional_gain;
  m_eth_data.adc_balance_ref = m_eeprom_data.adc_balance_ref;
  m_eth_data.adc_balance_cont_cnv_cnt = m_eeprom_data.adc_balance_cont_cnv_cnt;
  m_eth_data.adc_balance_impf_iterations_cnt
    = m_eeprom_data.adc_balance_impf_iterations_cnt;
  m_eth_data.adc_balance_impf_type = m_eeprom_data.adc_balance_impf_type;
  m_eth_data.adc_balance_cont_mode = m_eeprom_data.adc_balance_cont_mode;
  m_eth_data.adc_balance_cont_sko = m_eeprom_data.adc_balance_cont_sko;
  
  m_adc_balance_param_data.gain = m_eeprom_data.adc_balance_gain;
  m_adc_balance_param_data.filter = m_eeprom_data.adc_balance_filter;
  m_adc_balance_param_data.channel = m_eeprom_data.adc_balance_channel;
  m_adc_balance_param_data.cnv_cnt = m_eeprom_data.adc_balance_cnv_cnt;
  m_adc_balance_param_data.additional_gain 
    = m_eeprom_data.adc_balance_additional_gain;
  m_adc_balance_param_data.ref = m_eeprom_data.adc_balance_ref;
  m_adc_balance_param_data.cont_cnv_cnt
    = m_eeprom_data.adc_balance_cont_cnv_cnt;
  m_adc_balance_param_data.impf_iterations_cnt
    = m_eeprom_data.adc_balance_impf_iterations_cnt;
  m_adc_balance_param_data.impf_type 
    = convert_to_impf_type(m_eeprom_data.adc_balance_impf_type);
  m_adc_balance_param_data.cont_mode 
    = convert_to_cont_mode(m_eeprom_data.adc_balance_cont_mode);
  m_adc_balance_param_data.cont_sko = m_eeprom_data.adc_balance_cont_sko;
  //  ADC elab
  m_eth_data.adc_elab_gain = m_eeprom_data.adc_elab_gain;
  m_eth_data.adc_elab_filter = m_eeprom_data.adc_elab_filter;
  m_eth_data.adc_elab_channel = m_eeprom_data.adc_elab_channel;
  m_eth_data.adc_elab_cnv_cnt = m_eeprom_data.adc_elab_cnv_cnt;
  m_eth_data.adc_elab_additional_gain 
    = m_eeprom_data.adc_elab_additional_gain;
  m_eth_data.adc_elab_ref = m_eeprom_data.adc_elab_ref;
  m_eth_data.adc_elab_cont_cnv_cnt = m_eeprom_data.adc_elab_cont_cnv_cnt;
  m_eth_data.adc_elab_impf_iterations_cnt
    = m_eeprom_data.adc_elab_impf_iterations_cnt;
  m_eth_data.adc_elab_impf_type = m_eeprom_data.adc_elab_impf_type;
  m_eth_data.adc_elab_cont_mode = m_eeprom_data.adc_elab_cont_mode;
  m_eth_data.adc_elab_cont_sko = m_eeprom_data.adc_elab_cont_sko;
  
  m_adc_elab_param_data.gain = m_eeprom_data.adc_elab_gain;
  m_adc_elab_param_data.filter = m_eeprom_data.adc_elab_filter;
  m_adc_elab_param_data.channel = m_eeprom_data.adc_elab_channel;
  m_adc_elab_param_data.cnv_cnt = m_eeprom_data.adc_elab_cnv_cnt;
  m_adc_elab_param_data.additional_gain 
    = m_eeprom_data.adc_elab_additional_gain;
  m_adc_elab_param_data.ref = m_eeprom_data.adc_elab_ref;
  m_adc_elab_param_data.cont_cnv_cnt
    = m_eeprom_data.adc_elab_cont_cnv_cnt;
  m_adc_elab_param_data.impf_iterations_cnt
    = m_eeprom_data.adc_elab_impf_iterations_cnt;
  m_adc_elab_param_data.impf_type 
    = convert_to_impf_type(m_eeprom_data.adc_elab_impf_type);
  m_adc_elab_param_data.cont_mode 
    = convert_to_cont_mode(m_eeprom_data.adc_elab_cont_mode);
  m_adc_elab_param_data.cont_sko = m_eeprom_data.adc_elab_cont_sko;
}

void hrm::app_t::adc_params_translate_actual_to_eth()
{
  adc_param_data_t actual_params;
  m_adc.get_params(&actual_params);
  
  m_eth_data.adc_gain = actual_params.gain;
  m_eth_data.adc_filter = actual_params.filter;
  m_eth_data.adc_channel = actual_params.channel;
  m_eth_data.adc_cnv_cnt = actual_params.cnv_cnt;
  m_eth_data.adc_additional_gain 
    = actual_params.additional_gain;
  m_eth_data.adc_ref = actual_params.ref;
  m_eth_data.adc_cont_cnv_cnt = actual_params.cont_cnv_cnt;
  m_eth_data.adc_impf_iterations_cnt
    = actual_params.impf_iterations_cnt;
  m_eth_data.adc_impf_type = actual_params.impf_type;
  m_eth_data.adc_cont_mode = actual_params.cont_mode;
  m_eth_data.adc_cont_sko = actual_params.cont_sko;
}

bool hrm::app_t::adc_params_recieve_and_save_free_vx()
{
  bool new_data = false;
  if (m_eth_data.adc_free_vx_gain != m_adc_free_vx_param_data.gain) {
    new_data = true;
    m_adc_free_vx_param_data.gain = m_eth_data.adc_free_vx_gain;
    m_eeprom_data.adc_free_vx_gain = m_eth_data.adc_free_vx_gain;
  }
  if (m_eth_data.adc_free_vx_filter != m_adc_free_vx_param_data.filter) {
    new_data = true;
    m_adc_free_vx_param_data.filter = m_eth_data.adc_free_vx_filter;
    m_eeprom_data.adc_free_vx_filter = m_eth_data.adc_free_vx_filter;
  }
  if (m_eth_data.adc_free_vx_channel != m_adc_free_vx_param_data.channel) {
    new_data = true;
    m_adc_free_vx_param_data.channel = m_eth_data.adc_free_vx_channel;
    m_eeprom_data.adc_free_vx_channel = m_eth_data.adc_free_vx_channel;
  }
  if (m_eth_data.adc_free_vx_cnv_cnt != m_adc_free_vx_param_data.cnv_cnt) {
    new_data = true;
    m_adc_free_vx_param_data.cnv_cnt = m_eth_data.adc_free_vx_cnv_cnt;
    m_eeprom_data.adc_free_vx_cnv_cnt = m_eth_data.adc_free_vx_cnv_cnt;
  }
  if (m_eth_data.adc_free_vx_additional_gain 
      != m_adc_free_vx_param_data.additional_gain) {
    new_data = true;
    m_adc_free_vx_param_data.additional_gain 
      = m_eth_data.adc_free_vx_additional_gain;
    m_eeprom_data.adc_free_vx_additional_gain 
      = m_eth_data.adc_free_vx_additional_gain;
  }
  if (m_eth_data.adc_free_vx_ref != m_adc_free_vx_param_data.ref) {
    new_data = true;
    m_adc_free_vx_param_data.ref = m_eth_data.adc_free_vx_ref;
    m_eeprom_data.adc_free_vx_ref = m_eth_data.adc_free_vx_ref;
  }
  if (m_eth_data.adc_free_vx_cont_cnv_cnt 
      != m_adc_free_vx_param_data.cont_cnv_cnt) {
    new_data = true;
    m_adc_free_vx_param_data.cont_cnv_cnt 
      = m_eth_data.adc_free_vx_cont_cnv_cnt;
    m_eeprom_data.adc_free_vx_cont_cnv_cnt 
      = m_eth_data.adc_free_vx_cont_cnv_cnt;
  }
  if (m_eth_data.adc_free_vx_impf_iterations_cnt 
      != m_adc_free_vx_param_data.impf_iterations_cnt) {
    new_data = true;
    m_adc_free_vx_param_data.impf_iterations_cnt 
      = m_eth_data.adc_free_vx_impf_iterations_cnt;
    m_eeprom_data.adc_free_vx_impf_iterations_cnt 
      = m_eth_data.adc_free_vx_impf_iterations_cnt;
  }
  if (m_eth_data.adc_free_vx_impf_type != m_adc_free_vx_param_data.impf_type) {
    new_data = true;
    m_adc_free_vx_param_data.impf_type 
      = convert_to_impf_type(m_eth_data.adc_free_vx_impf_type);
    m_eeprom_data.adc_free_vx_impf_type = m_eth_data.adc_free_vx_impf_type;
  }
  if (m_eth_data.adc_free_vx_cont_mode != m_adc_free_vx_param_data.cont_mode) {
    new_data = true;
    m_adc_free_vx_param_data.cont_mode 
      = convert_to_cont_mode(m_eth_data.adc_free_vx_cont_mode);
    m_eeprom_data.adc_free_vx_cont_mode = m_eth_data.adc_free_vx_cont_mode;
  }
  if (m_eth_data.adc_free_vx_cont_sko != m_adc_free_vx_param_data.cont_sko) {
    new_data = true;
    m_adc_free_vx_param_data.cont_sko = m_eth_data.adc_free_vx_cont_sko;
    m_eeprom_data.adc_free_vx_cont_sko = m_eth_data.adc_free_vx_cont_sko;
  }
  return new_data;
}

bool hrm::app_t::adc_params_recieve_and_save_free_th()
{
  bool new_data = false;
  if (m_eth_data.adc_free_th_gain != m_adc_free_th_param_data.gain) {
    new_data = true;
    m_adc_free_th_param_data.gain = m_eth_data.adc_free_th_gain;
    m_eeprom_data.adc_free_th_gain = m_eth_data.adc_free_th_gain;
  }
  if (m_eth_data.adc_free_th_filter != m_adc_free_th_param_data.filter) {
    new_data = true;
    m_adc_free_th_param_data.filter = m_eth_data.adc_free_th_filter;
    m_eeprom_data.adc_free_th_filter = m_eth_data.adc_free_th_filter;
  }
  if (m_eth_data.adc_free_th_channel != m_adc_free_th_param_data.channel) {
    new_data = true;
    m_adc_free_th_param_data.channel = m_eth_data.adc_free_th_channel;
    m_eeprom_data.adc_free_th_channel = m_eth_data.adc_free_th_channel;
  }
  if (m_eth_data.adc_free_th_cnv_cnt != m_adc_free_th_param_data.cnv_cnt) {
    new_data = true;
    m_adc_free_th_param_data.cnv_cnt = m_eth_data.adc_free_th_cnv_cnt;
    m_eeprom_data.adc_free_th_cnv_cnt = m_eth_data.adc_free_th_cnv_cnt;
  }
  if (m_eth_data.adc_free_th_additional_gain 
      != m_adc_free_th_param_data.additional_gain) {
    new_data = true;
    m_adc_free_th_param_data.additional_gain 
      = m_eth_data.adc_free_th_additional_gain;
    m_eeprom_data.adc_free_th_additional_gain 
      = m_eth_data.adc_free_th_additional_gain;
  }
  if (m_eth_data.adc_free_th_ref != m_adc_free_th_param_data.ref) {
    new_data = true;
    m_adc_free_th_param_data.ref = m_eth_data.adc_free_th_ref;
    m_eeprom_data.adc_free_th_ref = m_eth_data.adc_free_th_ref;
  }
  if (m_eth_data.adc_free_th_cont_cnv_cnt 
      != m_adc_free_th_param_data.cont_cnv_cnt) {
    new_data = true;
    m_adc_free_th_param_data.cont_cnv_cnt 
      = m_eth_data.adc_free_th_cont_cnv_cnt;
    m_eeprom_data.adc_free_th_cont_cnv_cnt 
      = m_eth_data.adc_free_th_cont_cnv_cnt;
  }
  if (m_eth_data.adc_free_th_impf_iterations_cnt 
      != m_adc_free_th_param_data.impf_iterations_cnt) {
    new_data = true;
    m_adc_free_th_param_data.impf_iterations_cnt 
      = m_eth_data.adc_free_th_impf_iterations_cnt;
    m_eeprom_data.adc_free_th_impf_iterations_cnt 
      = m_eth_data.adc_free_th_impf_iterations_cnt;
  }
  if (m_eth_data.adc_free_th_impf_type != m_adc_free_th_param_data.impf_type) {
    new_data = true;
    m_adc_free_th_param_data.impf_type 
      = convert_to_impf_type(m_eth_data.adc_free_th_impf_type);
    m_eeprom_data.adc_free_th_impf_type = m_eth_data.adc_free_th_impf_type;
  }
  if (m_eth_data.adc_free_th_cont_mode != m_adc_free_th_param_data.cont_mode) {
    new_data = true;
    m_adc_free_th_param_data.cont_mode 
      = convert_to_cont_mode(m_eth_data.adc_free_th_cont_mode);
    m_eeprom_data.adc_free_th_cont_mode = m_eth_data.adc_free_th_cont_mode;
  }
  if (m_eth_data.adc_free_th_cont_sko != m_adc_free_th_param_data.cont_sko) {
    new_data = true;
    m_adc_free_th_param_data.cont_sko = m_eth_data.adc_free_th_cont_sko;
    m_eeprom_data.adc_free_th_cont_sko = m_eth_data.adc_free_th_cont_sko;
  }
  return new_data;
}

bool hrm::app_t::adc_params_recieve_and_save_manual()
{
  bool new_data = false;
  if (m_eth_data.adc_manual_gain != m_adc_manual_param_data.gain) {
    new_data = true;
    m_adc_manual_param_data.gain = m_eth_data.adc_manual_gain;
    m_eeprom_data.adc_manual_gain = m_eth_data.adc_manual_gain;
  }
  if (m_eth_data.adc_manual_filter != m_adc_manual_param_data.filter) {
    new_data = true;
    m_adc_manual_param_data.filter = m_eth_data.adc_manual_filter;
    m_eeprom_data.adc_manual_filter = m_eth_data.adc_manual_filter;
  }
  if (m_eth_data.adc_manual_channel != m_adc_manual_param_data.channel) {
    new_data = true;
    m_adc_manual_param_data.channel = m_eth_data.adc_manual_channel;
    m_eeprom_data.adc_manual_channel = m_eth_data.adc_manual_channel;
  }
  if (m_eth_data.adc_manual_cnv_cnt != m_adc_manual_param_data.cnv_cnt) {
    new_data = true;
    m_adc_manual_param_data.cnv_cnt = m_eth_data.adc_manual_cnv_cnt;
    m_eeprom_data.adc_manual_cnv_cnt = m_eth_data.adc_manual_cnv_cnt;
  }
  if (m_eth_data.adc_manual_additional_gain 
      != m_adc_manual_param_data.additional_gain) {
    new_data = true;
    m_adc_manual_param_data.additional_gain 
      = m_eth_data.adc_manual_additional_gain;
    m_eeprom_data.adc_manual_additional_gain 
      = m_eth_data.adc_manual_additional_gain;
  }
  if (m_eth_data.adc_manual_ref != m_adc_manual_param_data.ref) {
    new_data = true;
    m_adc_manual_param_data.ref = m_eth_data.adc_manual_ref;
    m_eeprom_data.adc_manual_ref = m_eth_data.adc_manual_ref;
  }
  if (m_eth_data.adc_manual_cont_cnv_cnt 
      != m_adc_manual_param_data.cont_cnv_cnt) {
    new_data = true;
    m_adc_manual_param_data.cont_cnv_cnt 
      = m_eth_data.adc_manual_cont_cnv_cnt;
    m_eeprom_data.adc_manual_cont_cnv_cnt 
      = m_eth_data.adc_manual_cont_cnv_cnt;
  }
  if (m_eth_data.adc_manual_impf_iterations_cnt 
      != m_adc_manual_param_data.impf_iterations_cnt) {
    new_data = true;
    m_adc_manual_param_data.impf_iterations_cnt 
      = m_eth_data.adc_manual_impf_iterations_cnt;
    m_eeprom_data.adc_manual_impf_iterations_cnt 
      = m_eth_data.adc_manual_impf_iterations_cnt;
  }
  if (m_eth_data.adc_manual_impf_type != m_adc_manual_param_data.impf_type) {
    new_data = true;
    m_adc_manual_param_data.impf_type 
      = convert_to_impf_type(m_eth_data.adc_manual_impf_type);
    m_eeprom_data.adc_manual_impf_type = m_eth_data.adc_manual_impf_type;
  }
  if (m_eth_data.adc_manual_cont_mode != m_adc_manual_param_data.cont_mode) {
    new_data = true;
    m_adc_manual_param_data.cont_mode 
      = convert_to_cont_mode(m_eth_data.adc_manual_cont_mode);
    m_eeprom_data.adc_manual_cont_mode = m_eth_data.adc_manual_cont_mode;
  }
  if (m_eth_data.adc_manual_cont_sko != m_adc_manual_param_data.cont_sko) {
    new_data = true;
    m_adc_manual_param_data.cont_sko = m_eth_data.adc_manual_cont_sko;
    m_eeprom_data.adc_manual_cont_sko = m_eth_data.adc_manual_cont_sko;
  }
  return new_data;
}

bool hrm::app_t::adc_params_recieve_and_save_balance()
{
  bool new_data = false;
  if (m_eth_data.adc_balance_gain != m_adc_balance_param_data.gain) {
    new_data = true;
    m_adc_balance_param_data.gain = m_eth_data.adc_balance_gain;
    m_eeprom_data.adc_balance_gain = m_eth_data.adc_balance_gain;
  }
  if (m_eth_data.adc_balance_filter != m_adc_balance_param_data.filter) {
    new_data = true;
    m_adc_balance_param_data.filter = m_eth_data.adc_balance_filter;
    m_eeprom_data.adc_balance_filter = m_eth_data.adc_balance_filter;
  }
  if (m_eth_data.adc_balance_channel != m_adc_balance_param_data.channel) {
    new_data = true;
    m_adc_balance_param_data.channel = m_eth_data.adc_balance_channel;
    m_eeprom_data.adc_balance_channel = m_eth_data.adc_balance_channel;
  }
  if (m_eth_data.adc_balance_cnv_cnt != m_adc_balance_param_data.cnv_cnt) {
    new_data = true;
    m_adc_balance_param_data.cnv_cnt = m_eth_data.adc_balance_cnv_cnt;
    m_eeprom_data.adc_balance_cnv_cnt = m_eth_data.adc_balance_cnv_cnt;
  }
  if (m_eth_data.adc_balance_additional_gain 
      != m_adc_balance_param_data.additional_gain) {
    new_data = true;
    m_adc_balance_param_data.additional_gain 
      = m_eth_data.adc_balance_additional_gain;
    m_eeprom_data.adc_balance_additional_gain 
      = m_eth_data.adc_balance_additional_gain;
  }
  if (m_eth_data.adc_balance_ref != m_adc_balance_param_data.ref) {
    new_data = true;
    m_adc_balance_param_data.ref = m_eth_data.adc_balance_ref;
    m_eeprom_data.adc_balance_ref = m_eth_data.adc_balance_ref;
  }
  if (m_eth_data.adc_balance_cont_cnv_cnt 
      != m_adc_balance_param_data.cont_cnv_cnt) {
    new_data = true;
    m_adc_balance_param_data.cont_cnv_cnt 
      = m_eth_data.adc_balance_cont_cnv_cnt;
    m_eeprom_data.adc_balance_cont_cnv_cnt 
      = m_eth_data.adc_balance_cont_cnv_cnt;
  }
  if (m_eth_data.adc_balance_impf_iterations_cnt 
      != m_adc_balance_param_data.impf_iterations_cnt) {
    new_data = true;
    m_adc_balance_param_data.impf_iterations_cnt 
      = m_eth_data.adc_balance_impf_iterations_cnt;
    m_eeprom_data.adc_balance_impf_iterations_cnt 
      = m_eth_data.adc_balance_impf_iterations_cnt;
  }
  if (m_eth_data.adc_balance_impf_type != m_adc_balance_param_data.impf_type) {
    new_data = true;
    m_adc_balance_param_data.impf_type 
      = convert_to_impf_type(m_eth_data.adc_balance_impf_type);
    m_eeprom_data.adc_balance_impf_type = m_eth_data.adc_balance_impf_type;
  }
  if (m_eth_data.adc_balance_cont_mode != m_adc_balance_param_data.cont_mode) {
    new_data = true;
    m_adc_balance_param_data.cont_mode 
      = convert_to_cont_mode(m_eth_data.adc_balance_cont_mode);
    m_eeprom_data.adc_balance_cont_mode = m_eth_data.adc_balance_cont_mode;
  }
  if (m_eth_data.adc_balance_cont_sko != m_adc_balance_param_data.cont_sko) {
    new_data = true;
    m_adc_balance_param_data.cont_sko = m_eth_data.adc_balance_cont_sko;
    m_eeprom_data.adc_balance_cont_sko = m_eth_data.adc_balance_cont_sko;
  }
  return new_data;
}

bool hrm::app_t::adc_params_recieve_and_save_elab()
{
  bool new_data = false;
  if (m_eth_data.adc_elab_gain != m_adc_elab_param_data.gain) {
    new_data = true;
    m_adc_elab_param_data.gain = m_eth_data.adc_elab_gain;
    m_eeprom_data.adc_elab_gain = m_eth_data.adc_elab_gain;
  }
  if (m_eth_data.adc_elab_filter != m_adc_elab_param_data.filter) {
    new_data = true;
    m_adc_elab_param_data.filter = m_eth_data.adc_elab_filter;
    m_eeprom_data.adc_elab_filter = m_eth_data.adc_elab_filter;
  }
  if (m_eth_data.adc_elab_channel != m_adc_elab_param_data.channel) {
    new_data = true;
    m_adc_elab_param_data.channel = m_eth_data.adc_elab_channel;
    m_eeprom_data.adc_elab_channel = m_eth_data.adc_elab_channel;
  }
  if (m_eth_data.adc_elab_cnv_cnt != m_adc_elab_param_data.cnv_cnt) {
    new_data = true;
    m_adc_elab_param_data.cnv_cnt = m_eth_data.adc_elab_cnv_cnt;
    m_eeprom_data.adc_elab_cnv_cnt = m_eth_data.adc_elab_cnv_cnt;
  }
  if (m_eth_data.adc_elab_additional_gain 
      != m_adc_elab_param_data.additional_gain) {
    new_data = true;
    m_adc_elab_param_data.additional_gain 
      = m_eth_data.adc_elab_additional_gain;
    m_eeprom_data.adc_elab_additional_gain 
      = m_eth_data.adc_elab_additional_gain;
  }
  if (m_eth_data.adc_elab_ref != m_adc_elab_param_data.ref) {
    new_data = true;
    m_adc_elab_param_data.ref = m_eth_data.adc_elab_ref;
    m_eeprom_data.adc_elab_ref = m_eth_data.adc_elab_ref;
  }
  if (m_eth_data.adc_elab_cont_cnv_cnt 
      != m_adc_elab_param_data.cont_cnv_cnt) {
    new_data = true;
    m_adc_elab_param_data.cont_cnv_cnt 
      = m_eth_data.adc_elab_cont_cnv_cnt;
    m_eeprom_data.adc_elab_cont_cnv_cnt 
      = m_eth_data.adc_elab_cont_cnv_cnt;
  }
  if (m_eth_data.adc_elab_impf_iterations_cnt 
      != m_adc_elab_param_data.impf_iterations_cnt) {
    new_data = true;
    m_adc_elab_param_data.impf_iterations_cnt 
      = m_eth_data.adc_elab_impf_iterations_cnt;
    m_eeprom_data.adc_elab_impf_iterations_cnt 
      = m_eth_data.adc_elab_impf_iterations_cnt;
  }
  if (m_eth_data.adc_elab_impf_type != m_adc_elab_param_data.impf_type) {
    new_data = true;
    m_adc_elab_param_data.impf_type 
      = convert_to_impf_type(m_eth_data.adc_elab_impf_type);
    m_eeprom_data.adc_elab_impf_type = m_eth_data.adc_elab_impf_type;
  }
  if (m_eth_data.adc_elab_cont_mode != m_adc_elab_param_data.cont_mode) {
    new_data = true;
    m_adc_elab_param_data.cont_mode 
      = convert_to_cont_mode(m_eth_data.adc_elab_cont_mode);
    m_eeprom_data.adc_elab_cont_mode = m_eth_data.adc_elab_cont_mode;
  }
  if (m_eth_data.adc_elab_cont_sko != m_adc_elab_param_data.cont_sko) {
    new_data = true;
    m_adc_elab_param_data.cont_sko = m_eth_data.adc_elab_cont_sko;
    m_eeprom_data.adc_elab_cont_sko = m_eth_data.adc_elab_cont_sko;
  }
  return new_data;
}

hrm::adc_value_t hrm::app_t::adc_vx_to_th(adc_value_t a_voltage)
{
  return (a_voltage - 0.4) / 0.0195;
}

void hrm::app_t::show_experiment_parameters()
{
  irs::mlog() << irsm("----------- Параметры эксперимента -----------");
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("no_prot ");
  irs::mlog() << setw(7) << left << m_no_prot;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_gain ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << static_cast<int>(m_adc_elab_param_data.gain);
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_gain ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << static_cast<int>(m_adc_balance_param_data.gain);
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("dac_pause_ms ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_dac_after_pause);
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_filter ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << static_cast<int>(m_adc_elab_param_data.filter);
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_filter ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << static_cast<int>(m_adc_balance_param_data.filter);
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("relay_pause_ms ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_relay_after_pause);
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_cnv_cnt ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_elab_param_data.cnv_cnt;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_cnv_cnt ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_balance_param_data.cnv_cnt;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("prepare_pause ");
  irs::mlog() << setw(7) << left << m_prepare_pause;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_cnv_cnt ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_elab_param_data.cont_cnv_cnt;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_cont_cnv_cnt ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_balance_param_data.cont_cnv_cnt;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("dac_elab_pause_ms ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << 1000.0 * CNT_TO_DBLTIME(m_dac_elab_pause);
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_impf_iterations_cnt ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_elab_param_data.impf_iterations_cnt;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_impf_iterations_cnt ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_balance_param_data.impf_iterations_cnt;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("elab_mode ");
  irs::mlog() << setw(7) << left << m_elab_mode;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_impf_type ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_elab_param_data.impf_type;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_impf_type ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_balance_param_data.impf_type;
  
  irs::mlog() << endl;
  
  //irs::mlog() << setw(25) << left << irsm(". .");
  irs::mlog() << setw(18) << left << irsm("prev_exp_time ");
  irs::mlog() << setw(7) << left << m_eth_data.prev_exp_time;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_mode ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_elab_param_data.cont_mode;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_cont_mode ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_balance_param_data.cont_mode;
  
  irs::mlog() << endl;
  
  //irs::mlog() << setw(25) << left << irsm(". .");
  irs::mlog() << setw(18) << left << irsm("adc_max_value_prot ");
  irs::mlog() << setw(7) << left << m_adc_max_value_prot;
  
  irs::mlog() << setw(32) << left << irsm("adc_elab_cont_sko ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_elab_param_data.cont_sko;
  
  irs::mlog() << setw(32) << left << irsm("adc_balance_cont_sko ");
  irs::mlog() << setw(7) << left;
  irs::mlog() << m_adc_balance_param_data.cont_sko;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("adc_max_value_no_prot ");
  irs::mlog() << setw(7) << left << m_adc_max_value_no_prot;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("bac_old_coefficient ");
  irs::mlog() << setw(7) << left << m_bac_old_coefficient;
  
  irs::mlog() << endl;
  
  irs::mlog() << setw(18) << left << irsm("bac_new_coefficient ");
  irs::mlog() << setw(7) << left << m_bac_new_coefficient;
  
  irs::mlog() << endl;
  
  //irs::mlog() << setw(18) << left << irsm("bac_new_int_coefficient ");
  //irs::mlog() << setw(7) << left << m_bac_new_int_coefficient;
  
  //irs::mlog() << endl;
}

void hrm::app_t::show_last_result()
{
  irs::mlog() << endl;
  irs::mlog() << irsm("----------- Результат серии экспериментов");
  irs::mlog() << irsm(" -----------") << endl;
  irs::mlog() << irsm("№    ");
  irs::mlog() << irsm("OLD            ");
  irs::mlog() << irsm("NEW            ");
  irs::mlog() << irsm("OLD_UNCORRECT  ");
  irs::mlog() << irsm("NEW_UNCORRECT  ");
  irs::mlog() << irsm("t°ext  ");
  irs::mlog() << irsm("t°dac  ");
  irs::mlog() << irsm("t°adc  ");
  irs::mlog() << irsm("t°ldo  ");
  irs::mlog() << endl;
  
  for (size_t i = 0; i < m_exp_vector.size(); i++) {
    irs::mlog() << setprecision(10);
    irs::mlog() << setw(3) << i + 1;irs::mlog() << irsm(" ");
    //irs::mlog() << setw(12) << m_exp_vector[i].ch_code * pow(2., 19);
    //irs::mlog() << irsm(" ");
    //irs::mlog() << setw(12) << m_exp_vector[i].et_code * pow(2., 19);
    irs::mlog() << setprecision(12);
    //irs::mlog() << irsm(" ") << setw(14) << m_exp_vector[i].n0;
    irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].result_old;
    irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].result_new;
    irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].result_old_uncorrect;
    irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].result_new_uncorrect;
    irs::mlog() << setprecision(5);
    irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].temperature_ext;
    irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].temperature_dac;
    irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].temperature_adc;
    irs::mlog() << irsm(" ") << setw(5) << m_exp_vector[i].temperature_ldo;
    irs::mlog() << endl;
  }
  
  irs::mlog() << endl;
  irs::mlog() << irsm("----------- Пересечения серии экспериментов");
  irs::mlog() << irsm(" -----------") << endl;
  irs::mlog() << irsm("№     ");
  irs::mlog() << irsm("NEG            ");
  irs::mlog() << irsm("NEG_0          ");
  irs::mlog() << irsm("POS            ");
  irs::mlog() << irsm("POS_0          ");
  irs::mlog() << irsm("n0             ");
  irs::mlog() << irsm("num            ");
  irs::mlog() << irsm("den            ");
  irs::mlog() << endl;
  
  for (size_t i = 0; i < m_exp_vector.size(); i++) {
    //irs::mlog() << setprecision(6);
    irs::mlog() << setw(3) << i + 1;irs::mlog() << irsm(" ");
    irs::mlog() << setprecision(12);
    irs::mlog() << setw(13) << m_exp_vector[i].ch_code * pow(2., 19);
    irs::mlog() << irsm(" ");
    irs::mlog() << setw(13) << m_exp_vector[i].ch_balanced_code;
    irs::mlog() << irsm(" ");
    irs::mlog() << setw(13) << m_exp_vector[i].et_code * pow(2., 19);
    irs::mlog() << irsm(" ");
    irs::mlog() << setw(13) << m_exp_vector[i].et_balanced_code;
    irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].n0;
    irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].num;
    irs::mlog() << irsm(" ") << setw(13) << m_exp_vector[i].den;
    irs::mlog() << endl;
  }
  
  irs::mlog() << setprecision(8);
  irs::mlog() << irsm("-------------------------------------------");
  irs::mlog() << irsm("--------------");
  irs::mlog() << endl;
  
  show_experiment_parameters();
}

hrm::app_t::elab_mode_t hrm::app_t::convert_u8_to_elab_mode(irs_u8 a_mode)
{
  elab_mode_t return_mode = em_none;
  switch (a_mode) { 
    case 0: return_mode = em_linear;        break;
    case 1: return_mode = em_pid;           break;
    case 2: return_mode = em_fast_2points;  break;
    case 3: return_mode = em_none;          break;
    default:return_mode = em_none;
  }
  return return_mode;
}
