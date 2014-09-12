#include <irspch.h>

#include "app.h"

#include <irslimits.h>

#include <irsfinal.h>

hrm::app_t::app_t(cfg_t* ap_cfg):
  mp_cfg(ap_cfg),
  m_eth_data(),
  m_mxnet_server(
    #ifdef LWIP
    mp_cfg->connector_hardflow,
    #else
    mp_cfg->simple_hardflow,
    #endif
    m_eth_data.connect(&m_mxnet_server, 0) / sizeof(irs_u32)),
  m_eeprom(&mp_cfg->spi, &mp_cfg->ee_cs, 1024, true, 0, 64,
    irs::make_cnt_s(1)),
  m_eeprom_data(&m_eeprom),
  m_init_eeprom(&m_eeprom, &m_eeprom_data),
  m_lcd_drv(irslcd_4x20, mp_cfg->lcd_port, mp_cfg->lcd_rs_pin,
    mp_cfg->lcd_e_pin),
  m_keyboard_drv(),
  m_encoder_drv(mp_cfg->encoder_gpio_channel_1, mp_cfg->encoder_gpio_channel_2,
    mp_cfg->encoder_timer_address),

  m_lcd_drv_service(),
  m_buzzer_kb_event(),
  m_hot_kb_event(),
  m_menu_kb_event(),
  m_keyboard_event_gen(),

  m_raw_dac(
    &mp_cfg->spi,
    &mp_cfg->dac_cs,
    &mp_cfg->dac_ldac,
    &mp_cfg->dac_clr,
    &mp_cfg->dac_reset,
    0),
  m_dac(&m_raw_dac),
  m_adc(&mp_cfg->spi, &mp_cfg->adc_cs, &mp_cfg->adc_exti),
  m_ext_th(&mp_cfg->spi_th, &mp_cfg->th_cs, irs::make_cnt_s(1)),
  m_ext_th_data(&m_ext_th),
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
  m_mode(md_free),
  m_free_status(fs_prepare),
  m_balance_status(bs_prepare),
  m_current_iteration(0),
  m_iteration_count(21),
  m_elab_iteration_count(3),
  m_elab_step(1),
  m_dac_code(0.0),
  m_dac_step(0.0),
  m_balanced_dac_code(0.0),
  m_initial_dac_code(0.0),
  m_initial_dac_step(m_dac.max_code()),
  m_balance_polarity(bp_neg),
  m_checked(1.),
  m_etalon(1.),
  m_result(0.),
  m_result_error(0.),
  m_checked_code(0.),
  m_etalon_code(0.),
  m_relay_after_pause(irs::make_cnt_ms(100)),
  m_dac_after_pause(irs::make_cnt_ms(100)),
  m_dac_elab_pause(irs::make_cnt_ms(101)),
  m_prepare_pause(5),
  m_elab_vector(),
  m_exp_cnt(1),
  m_exp_vector(),
  m_no_prot(false),
  m_wild_relays(false),
  m_balanced_sko(0.),
  m_adc_experiment_gain(m_min_adc_gain),
  m_adc_experiment_filter(m_slow_adc_filter),
  m_manual_status(ms_prepare),
  m_scan_status(ss_prepare),
  m_dac_center_scan(0.),
  m_current_adc_point(0),
  m_prepare_pause_timer(0),
  m_exp_time(0),
  m_prev_exp_time(0),
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
  m_relay_pause_timer(m_relay_after_pause),
  m_buzzer(&mp_cfg->buzzer),
  //
  // Меню
  //
  m_menu_timer(irs::make_cnt_ms(40)),
  m_mode_item(),
  m_menu_voltage_ref(0),
  m_voltage_ref_item(&m_menu_voltage_ref, true),
  m_trans_volt_ref_event(),
  m_menu_voltage(0),
  m_voltage_item(&m_menu_voltage, true),

  m_main_creep(
    mp_creep_buffer,
    m_creep_len,
    m_lcd_drv.get_width(),
    m_creep_static_path_len,
    m_creep_message,
    m_creep_time_num,
    m_creep_time_denom),
  m_main_screen(),
  mp_cur_menu(&m_main_screen),
  m_escape_pressed_event()
{

  init_keyboard_drv();
  init_encoder_drv();

  mxip_t ip = mxip_t::zero_ip();
  ip.val[0] = IP_0;
  ip.val[1] = IP_1;
  ip.val[2] = IP_2;
  ip.val[3] = IP_3;

  #ifdef LWIP
  mxip_t mask = mxip_t::zero_ip();
  mask.val[0] = 255;
  mask.val[1] = 255;
  mask.val[2] = 255;
  mask.val[3] = 0;

  mxip_t gateway = mxip_t::zero_ip();
  bool dhcp_enabled = false;
  /*gateway.val[0] = m_eeprom_data.gateway_0;
  gateway.val[1] = m_eeprom_data.gateway_1;
  gateway.val[2] = m_eeprom_data.gateway_2;
  gateway.val[3] = m_eeprom_data.gateway_3;*/
  mp_cfg->network_config.set(ip, mask, gateway, dhcp_enabled);
  #else // !LWIP
  char ip_str[IP_STR_LEN];
  mxip_to_cstr(ip_str, ip);
  mp_cfg->simple_hardflow.set_param("local_addr", ip_str);
  #endif // !LWIP

  irs::mlog() << setprecision(8);

  m_exp_cnt = m_eeprom_data.exp_cnt;
  m_eth_data.exp_cnt = m_exp_cnt;

  m_etalon = m_eeprom_data.etalon;
  m_eth_data.etalon = m_etalon;

  m_checked = m_eeprom_data.checked;
  m_eth_data.checked = m_checked;

  m_dac.set_lin(m_eeprom_data.dac_lin);
  m_eth_data.dac_lin = m_dac.get_lin();

  m_elab_iteration_count = m_eeprom_data.elab_cnt;
  m_eth_data.elab_cnt = m_elab_iteration_count;

  m_dac_after_pause = irs::make_cnt_ms(m_eeprom_data.dac_pause_ms);
  m_eth_data.dac_pause_ms = m_eeprom_data.dac_pause_ms;
  m_dac.set_after_pause(m_dac_after_pause);

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

  m_adc.set_cnv_cnt(m_eeprom_data.adc_average_cnt);
  m_eth_data.adc_average_cnt = m_adc.cnv_cnt();

  m_adc_fade_data.t = m_eeprom_data.adc_filter_constant;
  m_eth_data.adc_filter_constant = m_adc_fade_data.t;

  m_adc.set_skip_cnt(m_eeprom_data.adc_average_skip_cnt);
  m_eth_data.adc_average_skip_cnt = m_eeprom_data.adc_average_skip_cnt;

  m_adc_experiment_gain = m_eeprom_data.adc_experiment_gain;
  m_eth_data.adc_experiment_gain = m_adc_experiment_gain;

  m_adc_experiment_filter = m_eeprom_data.adc_experiment_filter;
  m_eth_data.adc_experiment_filter = m_adc_experiment_filter;

  m_adc.set_additional_gain(m_eeprom_data.adc_additional_gain);
  m_eth_data.adc_additional_gain = m_adc.additional_gain();

  m_adc.set_ref(m_eeprom_data.adc_ref);
  m_eth_data.adc_ref = m_adc.ref();

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

  m_elab_max_delta = m_eeprom_data.elab_max_delta;
  m_eth_data.elab_max_delta = m_elab_max_delta;

  mp_cfg->vben.set();

  m_adc_fade_data.x1 = 0.0;
  m_adc_fade_data.y1 = 0.0;
  m_adc_fade_data.t = m_eeprom_data.adc_filter_constant;
  m_relay_after_pause = irs::make_cnt_ms(m_eeprom_data.relay_pause_ms);

  m_relay_bridge_pos.set_after_pause(m_relay_after_pause);
  m_relay_bridge_neg.set_after_pause(m_relay_after_pause);
  m_relay_prot.set_after_pause(m_relay_after_pause);

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

  //change_mode_on_creep();
  strcpy(mp_enter_msg, "Для входа в меню нажмите '€'");
  strcpy(mp_exit_msg, "Для выхода из меню нажмите 'esc'");

  //  Основной экран
  m_main_screen.set_key_event(&m_menu_kb_event);
  //m_main_screen.set_message(mp_enter_msg);
  //m_main_screen.set_creep(&m_main_creep);
  m_main_screen.set_disp_drv(&m_lcd_drv_service);
  m_main_screen.set_cursor_symbol(0x01);
  //m_main_screen.set_slave_menu(&m_main_menu);

  m_mode_item.set_parametr_string("=U");

  m_voltage_ref_item.set_header("Уст. напряжения");
  m_voltage_ref_item.set_message(mp_exit_msg);
  m_voltage_ref_item.set_str(mp_user_str, "Uo", "В", 10, 5);
  m_voltage_ref_item.set_max_value(670.0);
  m_voltage_ref_item.set_min_value(0.0);
  m_voltage_ref_item.add_change_event(&m_trans_volt_ref_event);
  m_voltage_ref_item.set_key_type(IMK_DIGITS);

  m_voltage_item.set_header("Уст. напряжения");
  m_voltage_item.set_message(mp_exit_msg);
  m_voltage_item.set_str(mp_user_str, "U1", "В", 10, 5);
  m_voltage_item.set_max_value(670.0);
  m_voltage_item.set_min_value(0.0);
  m_voltage_item.add_change_event(&m_trans_volt_ref_event);
  m_voltage_item.set_key_type(IMK_DIGITS);

  m_main_screen.creep_stop();
  m_main_screen.add(&m_mode_item, 0, 0, IMM_FULL);
  m_main_screen.add(&m_voltage_ref_item, 0, 1, IMM_FULL);
  m_main_screen.add(&m_voltage_item, 0, 2, IMM_FULL);
  m_main_screen.add(&m_voltage_item, 0, 3, IMM_FULL);



  m_buzzer.bzz();
}

void hrm::app_t::init_keyboard_drv()
{
  m_keyboard_drv.add_horizontal_pins(&mp_cfg->key_drv_horizontal_pins);
  m_keyboard_drv.add_vertical_pins(&mp_cfg->key_drv_vertical_pins);
  irs::set_default_keys(&m_keyboard_drv);
}

void hrm::app_t::init_encoder_drv()
{
  m_encoder_drv.add_press_down_pin(&mp_cfg->key_encoder);
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
  m_ext_th.tick();

  m_relay_bridge_pos.tick();
  m_relay_bridge_neg.tick();
  m_relay_prot.tick();

  m_buzzer.tick();

  menu_check();

  if (m_menu_timer.check()) {
    mp_cur_menu->draw(&mp_cur_menu);
  }
  m_lcd_drv.tick();
  m_keyboard_event_gen.tick();

  if (m_buzzer_kb_event.check()) {
    m_buzzer.bzz();
  }

  if (m_eth_timer.check()) {
    m_eth_data.counter++;
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
      //  АЦП
      if (m_eth_data.adc_filter != m_adc.filter()) {
        m_eth_data.adc_filter = m_adc.filter();
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
      if (m_eth_data.relay_prot != m_relay_prot) {
        m_eth_data.relay_prot = m_relay_prot;
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

    if (m_eth_data.exp_cnt != m_eeprom_data.exp_cnt) {
      m_eeprom_data.exp_cnt = m_eth_data.exp_cnt;
    }
    if (m_eth_data.etalon != m_eeprom_data.etalon) {
      m_eeprom_data.etalon = m_eth_data.etalon;
    }
    if (m_eth_data.checked != m_eeprom_data.checked) {
      m_eeprom_data.checked = m_eth_data.checked;
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
    if (m_eth_data.adc_additional_gain != m_adc.additional_gain()) {
      m_adc.set_additional_gain(m_eth_data.adc_additional_gain);
      m_eeprom_data.adc_additional_gain = m_eth_data.adc_additional_gain;
    }
    if (m_eth_data.adc_ref != m_adc.ref()) {
      m_adc.set_ref(m_eth_data.adc_ref);
      m_eeprom_data.adc_ref = m_eth_data.adc_ref;
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
    if (m_eth_data.elab_max_delta != m_eeprom_data.elab_max_delta) {
      m_eeprom_data.elab_max_delta = m_eth_data.elab_max_delta;
    }
    m_eth_data.external_temperature = m_ext_th_data.temperature_code *
      m_ext_th.get_conv_koef();
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
      m_eth_data.sum_time = m_sum_time;
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
          //
          m_dac.show();
          m_dac.on();
          m_dac.set_code(0);
          //
          m_adc.hide();
          //
          m_relay_bridge_pos.set_wild(false);
          m_relay_bridge_neg.set_wild(false);
          //
          m_eth_data.apply = 0;
          m_eth_data.prepare_pause = m_prepare_pause;
          m_service_timer.start();
          m_meas_temperature = false;
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
              if (m_meas_temperature) {
                m_temperature =
                  (-m_adc.avg() * m_adc.additional_gain()-0.4) / 0.0195;
                m_adc.set_channel(ac_voltage);
                m_adc.set_gain(m_adc_experiment_gain);
                m_adc.set_filter(m_adc_experiment_filter);
                m_adc.set_cnv_cnt(m_eth_data.adc_average_cnt);
                m_adc.start_conversion();
                m_meas_temperature = false;
              } else {
                m_voltage = m_adc.avg();
                m_adc.set_channel(ac_temperature);
                m_adc.set_gain(m_min_adc_gain);
                m_adc.set_filter(m_slow_adc_filter);
                m_adc.set_cnv_cnt(1);
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
              m_eth_data.dac_normalize_code = m_dac.get_normalize_code();
              if (m_eth_data.dac_code != m_dac.get_code()) {
                m_eth_data.dac_code = m_dac.get_code();
              }
            }
            if (m_eth_data.dac_normalize_code != m_dac.get_normalize_code()) {
              m_dac.set_normalize_code(m_eth_data.dac_normalize_code);
              m_eth_data.dac_code = m_dac.get_code();
              if (m_eth_data.dac_normalize_code != m_dac.get_normalize_code()) {
                m_eth_data.dac_normalize_code = m_dac.get_normalize_code();
              }
            }
            if (m_eth_data.dac_lin != m_dac.get_lin()) {
              m_dac.set_lin(m_eth_data.dac_lin);
            }
            //  АЦП
            if (m_eth_data.adc_filter != m_adc.filter()) {
              m_adc.set_filter(m_eth_data.adc_filter);
            }
            if (m_eth_data.adc_channel != m_adc.channel()) {
              m_adc.set_channel(m_eth_data.adc_channel);
            }
            if (m_eth_data.adc_gain != m_adc.gain()) {
              m_adc.set_gain(m_eth_data.adc_gain);
            }
            if (m_adc.status() == irs_st_ready) {
              if (m_meas_temperature) {
                m_meas_temperature = false;
              } else {
                m_voltage = m_adc.avg();
              }
              m_adc.start_conversion();
            }
            if (m_eth_data.adc_average_cnt != m_adc.cnv_cnt()) {
              m_adc.set_cnv_cnt(m_eth_data.adc_average_cnt);
            }
            if (m_eth_data.start_adc_sequence == 1) {
              m_eth_data.start_adc_sequence = 0;
              m_manual_status = ms_adc_show;
            }
          } break;
        }
        case ms_adc_show: {
          if (m_adc.status() == irs_st_ready) {
            if (m_eth_data.adc_simply_show) {
              m_adc.show_simply();
            } else {
              m_adc.show();
            }
            m_current_iteration = 1;
            m_iteration_count = m_eth_data.exp_cnt;
            m_voltage = m_adc.avg();
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
            m_voltage = m_adc.avg();
            m_adc.start_conversion();
          }
        } break;
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
            m_adc.set_gain(m_adc_experiment_gain);
            m_adc.set_filter(m_adc_experiment_filter);
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
            m_voltage = m_adc.avg();
            m_adc.start_conversion();
            m_scan_status = ss_adc_wait;
          }
          break;
        }
        case ss_adc_wait: {
          if (m_adc.status() == irs_st_ready) {
            m_voltage = m_adc.avg();
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
            m_relay_bridge_pos.set_wild(true);
            m_relay_bridge_neg.set_wild(true);
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
          m_adc.set_cnv_cnt(m_eth_data.adc_average_cnt);
          m_adc.set_skip_cnt(m_eth_data.adc_average_skip_cnt);

          m_adc_experiment_gain = m_eth_data.adc_experiment_gain;
          if (m_adc_experiment_gain > m_max_adc_gain) {
            m_adc_experiment_gain = m_max_adc_gain;
            m_eth_data.adc_experiment_gain = m_max_adc_gain;
          }
          m_adc_experiment_filter = m_eth_data.adc_experiment_filter;
          if (m_adc_experiment_filter > m_slow_adc_filter) {
            m_adc_experiment_filter = m_slow_adc_filter;
            m_eth_data.adc_experiment_filter = m_slow_adc_filter;
          }

          m_prepare_pause = m_eth_data.prepare_pause;
          m_prepare_pause_timer.set(irs::make_cnt_ms(1000));

          m_exp_time = 0;
          m_sum_time = 0;
          m_optimize_balance = m_eth_data.optimize_balance;

          m_adc.set_additional_gain(m_eth_data.adc_additional_gain);
          m_adc.set_ref(m_eth_data.adc_ref);

          m_max_unsaturated_voltage = 0.0;
          m_max_unsaturated_dac_code = 0.0;

          m_elab_result_vector.clear();
          m_pos_current_elab = 0;
          m_elab_polarity = bp_neg;
          m_elab_step_multiplier = m_eth_data.elab_step_multiplier;
          m_elab_max_delta = m_eth_data.elab_max_delta;

          m_balance_status = bs_set_prot;
          break;
        }
        case bs_set_prot: {
          if (bridge_relays_ready()) {
            if (m_no_prot) {
              m_relay_prot = 0;
            }
            m_balance_status = bs_set_coils;
          }
          break;
        }
        case bs_set_coils: {
          if (bridge_relays_ready()) {
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
              m_adc.set_channel(ac_voltage);
              m_adc.set_gain(m_adc_experiment_gain);
              m_adc.set_filter(m_adc_experiment_filter);
              m_adc.hide();
              m_dac.hide();
              m_balance_status = bs_pause;
            } else {
              m_balance_status = bs_meas_temperature;
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
              m_eth_data.prepare_pause = m_prepare_current_time;
            } else {
              irs::mlog() << endl;
              m_eth_data.prepare_pause = m_prepare_pause;
              m_balance_status = bs_adc_show;
            }
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_voltage = m_adc.avg();
              m_adc.start_conversion();
            }
          }
          break;
        }
        case bs_adc_show: {
          if (m_adc.status() == irs_st_ready) {
            m_is_exp = true;
            m_exp_timer.start();
            m_adc.set_gain(m_adc_experiment_gain);
            m_adc.set_filter(m_adc_experiment_filter);
            m_adc.hide();
            m_dac.hide();
            m_balance_status = bs_meas_temperature;
          }
          break;
        }
        case bs_meas_temperature: {
          if (m_adc.status() == irs_st_ready) {
            m_voltage = m_adc.avg();
            m_adc.set_cnv_cnt(1);
            m_adc.set_channel(ac_temperature);
            m_adc.set_gain(m_min_adc_gain);
            m_adc.set_filter(m_slow_adc_filter);
            m_adc.start_conversion();
            m_balance_status = bs_wait_temperature;
          }
          break;
        }
        case bs_wait_temperature: {
          if (m_adc.status() == irs_st_ready) {
            m_temperature =
              (-m_adc.avg() * m_adc.additional_gain()-0.4) / 0.0195;
            irs::mlog() << irsm("Температура = ") << m_temperature;
            irs::mlog() << irsm(" °C") << endl;
            m_adc.set_channel(ac_voltage);
            m_adc.set_gain(m_adc_experiment_gain);
            m_adc.set_cnv_cnt(m_eth_data.adc_average_cnt);
            m_adc.set_filter(m_adc_experiment_filter);
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
            m_iteration_count = 21;
            m_dac_step_amplitude = 0.0;
            m_dac.on();
            m_prev_elab_vector.clear();
            m_balance_status = bs_adc_start;
          }
          break;
        }
        case bs_adc_start: {
          if (m_adc.status() == irs_st_ready && m_dac.ready()) {
            m_adc.start_conversion();
            m_balance_status = bs_adc_wait;
          }
          break;
        }
        case bs_adc_wait: {
          if (m_adc.status() == irs_st_ready) {
            bool m_prev_balance_completed = false;
            m_voltage = m_adc.avg();
            if (abs(m_voltage) < 0.99 * m_adc.max_value() &&
                m_current_iteration > 0) {
              if (abs(m_voltage) > m_max_unsaturated_voltage) {
                m_max_unsaturated_voltage = abs(m_voltage);
                m_max_unsaturated_dac_code = abs(m_dac_code);
                adc_value_t rel_sko = abs(m_adc.sko() / m_voltage) * 1e6;

                irs::mlog() << irsm(">>>> ") << setw(8);
                irs::mlog() << m_max_unsaturated_dac_code;
                irs::mlog() << irsm(" : ");
                irs::mlog() << m_max_unsaturated_voltage;
                irs::mlog() << irsm(" В ");
                irs::mlog() << rel_sko << irsm(" ppm ") << endl;
              }
              elab_point_t elab_point;
              elab_point.dac = m_dac.get_code();
              elab_point.adc = m_voltage;
              elab_point.sko = m_adc.sko();
              elab_point.avg = m_voltage;
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
                irs::mlog() << irsm(">DAC> ");
                irs::mlog() << m_dac_step_amplitude << irsm(" В") << endl;

                irs::mlog() << setw(2) << m_current_iteration+1 << irsm(" : ");
                irs::mlog() << setw(8) << m_dac_code << irsm(" : ");
                irs::mlog() << setw(8) << m_dac_step << irsm(" : ");
                irs::mlog() << setw(12) << m_voltage << irsm(" : ");
                adc_value_t rel_sko = abs(m_adc.sko() / m_adc.avg()) * 1e6;
                irs::mlog() << setw(8) << rel_sko << irsm(" ppm") << endl;
                irs::mlog() << irsm("PREV ") << setw(8) << m_balanced_dac_code;
                irs::mlog() << endl;
                m_prev_balance_completed = true;
                if (elab_point.dac >= 0) {
                  m_elab_polarity = bp_pos;
                } else {
                  m_elab_polarity = bp_neg;
                }
              }
            }
            if (m_prev_balance_completed) {
              m_balance_status = bs_elab_prepare;
            } else {
              m_balance_status = bs_balance;
            }
          }
          break;
        }
        case bs_balance: {
          if (m_current_iteration == 0
              && m_optimize_balance && abs(m_voltage) <
                0.9 * m_adc.max_value()) {
            // Корректировка
            double ratio = (pow(2.0, 20) / 12.0) * abs(m_voltage);
            double log_ratio = floor(log2(ratio) - 1.0);
            irs_u32 int_log_ratio = static_cast<size_t>(log_ratio);
            m_current_iteration = m_iteration_count - int_log_ratio - 1;
            irs_i32 int_dac_code = static_cast<irs_u32>(pow(2.0,int_log_ratio));
            irs_i32 int_dac_step = int_dac_code;
            if (m_voltage > 0.0) {
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
            irs::mlog() << setw(12) << m_voltage << irsm(" : ");
            adc_value_t rel_sko = abs(m_adc.sko() / m_adc.avg()) * 1e6;
            irs::mlog() << setw(8) << rel_sko << irsm(" ppm") << endl;
            if (m_dac_step_amplitude > 0.0
                && abs(m_voltage) < 0.5 * m_dac_step_amplitude) {
              m_balance_status = bs_elab_prepare;
            } else {
              if (m_voltage < 0) {
                m_dac_code += m_dac_step;
              } else {
                m_dac_code -= m_dac_step;
              }
              m_dac_step = floor(m_dac_step * 0.5);
              m_balance_status = bs_dac_set;
            }
          } else {
            m_balanced_dac_code = m_dac.get_code();
            m_balance_status = bs_elab_prepare;
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

            if (m_elab_step > m_dac.max_code()
                || m_elab_step < m_dac.min_code()) {
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
          double shift = 0.0;
          if (m_elab_iteration_count % 2 == 0) {
            shift = floor(0.5 + ((m_elab_iteration_count-1)/2) * m_elab_step);
            dac_value_t step = floor(0.5 + m_elab_step / 2.0);
            if (step == 0.0) {
              step = 1.0;
            }
            shift += step;
          } else {
            shift = floor(0.5 + (((m_elab_iteration_count-1)/2) * m_elab_step));
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
            m_balance_status = bs_elab_adc_start;
          } else {
            if (m_adc.status() == irs_st_ready) {
              m_voltage = m_adc.avg();
              m_adc.start_conversion();
            }
          }
          break;
        }
        case bs_elab_adc_start: {
          if (m_dac.ready()) {
            m_adc.start_conversion();
            m_balance_status = bs_elab_adc_wait;
          }
          break;
        }
        case bs_elab_adc_wait: {
          if (m_adc.status() == irs_st_ready) {
            m_voltage = m_adc.avg();
            elab_point_t elab_point;
            elab_point.dac = m_dac.get_code();
            elab_point.adc = m_voltage;
            elab_point.sko = m_adc.sko();
            elab_point.avg = m_adc.avg();
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
          if (m_elab_step_multiplier >= 1.0) {
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
            } else {
              m_checked_code = elab_result.code;
            }
            if (!m_no_prot) {
              m_relay_prot = 1;
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
              m_balance_status = bs_set_coils;
            } else {
              m_balance_status = bs_report;
            }
          }
          break;
        }
        case bs_report: {
          irs::mlog() << irsm("---------------------------------") << endl;
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
        }
        case bs_next_exp: {
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
          irs::mlog() << irsm("----------- Результат -----------") << endl;
          for (size_t i = 0; i < m_exp_vector.size(); i++) {
            irs::mlog() << setprecision(5);
            irs::mlog() << setw(3) << i + 1;irs::mlog() << irsm(" ");
            irs::mlog() << setw(12) << m_exp_vector[i].ch_code * pow(2., 19);
            irs::mlog() << irsm(" ");
            irs::mlog() << setw(12) << m_exp_vector[i].et_code * pow(2., 19);
            irs::mlog() << setprecision(12);
            irs::mlog() << irsm(" ") << setw(14) << m_exp_vector[i].result;
            irs::mlog() << irsm(" ") << setw(14) << m_exp_vector[i].error;
            irs::mlog() << irsm(" %");
            irs::mlog() << endl;
          }
          irs::mlog() << setprecision(8);
          irs::mlog() << irsm("-------------------------------------------");
          irs::mlog() << irsm("--------------");
          irs::mlog() << endl;
          m_is_exp = false;
          m_balance_status = bs_prepare;
          m_mode = md_free;
          break;
        }
      }
      if (m_eth_data.reset == 1) {
        irs::mlog() << irsm("---------------------------------") << endl;
        irs::mlog() << irsm("------------- Сброс -------------") << endl;
        m_eth_data.reset = 0;
        if (m_mode == md_balance) {
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

void hrm::app_t::menu_check()
{
  if (mp_cur_menu == &m_main_screen) {
    irskey_t key = m_hot_kb_event.check();
    /*if (key != irskey_none) {
      int i = 0;
      irs::mlog() << key << endl;
    }*/
    switch (key) {
      case irskey_0: {
        //m_out_mode = out_mode_actual_value;
        //mp_source->set(calibrator::source_t::mode_dc_current, 0);
      } break;
      case irskey_1: {
        //m_out_mode = out_mode_relative_deviation;
      } break;
      case irskey_2: {
        //m_out_mode = out_mode_sko;
      } break;
      case irskey_3: {
       // m_out_mode = out_mode_average;
      } break;
      case irskey_up: {
        /*if (m_dc_ac_mode == mode_dc) {
          m_dc_ac_mode = mode_ac;
        } else {
          m_dc_ac_mode = mode_dc;
        }
        update_mode_item();*/
      } break;
      case irskey_down: {
        mp_cur_menu = &m_voltage_ref_item;
        m_voltage_ref_item.set_master_menu(&m_main_screen);
        m_voltage_ref_item.show();
      } break;
      case irskey_backspace: {
        //mp_cur_menu = &m_freq_trim;
        //m_freq_trim.set_master_menu(&m_main_screen);
        //m_freq_trim.show();
        /*if (m_ui_mode == mode_u) {
          m_ui_mode = mode_i;
        } else {
          m_ui_mode = mode_u;
        }
        update_mode_item();*/
      } break;
      case irskey_escape: {
        m_escape_pressed_event.exec();
      } break;
    }
  }
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
    } else {
      irs::mlog() << irsm("Ahtung!!! EEPROM error!!!") << endl;
    }
  }
}

hrm::init_eeprom_t::~init_eeprom_t()
{
}
