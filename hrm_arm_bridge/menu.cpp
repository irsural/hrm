#include <irspch.h>

#include <irslimits.h>

#include "menu.h"

#include <irsfinal.h>

// class form_t
hrm::form_t::form_t():
  r_min(0.1),
  r_max(9999999999999.9),
  m_menu_timer(irs::make_cnt_ms(40)),
  mp_parent_form_maker(NULL),
  m_command(command_no)
{
}

hrm::form_t::~form_t()
{
}

void hrm::form_t::set_parent_form_maker(form_maker_base_t* ap_maker)
{
  mp_parent_form_maker = ap_maker;
}

hrm::form_maker_base_t* hrm::form_t::get_parent_form_maker()
{
  return mp_parent_form_maker;
}

hrm::form_t::command_t hrm::form_t::get_command() const
{
  return m_command;
}

void hrm::form_t::set_command(command_t a_command)
{
  m_command = a_command;
}

void hrm::form_t::tick()
{
  if (m_menu_timer.check()) {
    draw();
  }
}

// class form_maker_base_t
hrm::form_maker_base_t::form_maker_base_t():
  mp_form(),
  mp_parent_form_maker(NULL)
{
}
hrm::form_maker_base_t::~form_maker_base_t()
{
}
void hrm::form_maker_base_t::destroy()
{
  mp_form.reset();
}
hrm::form_t* hrm::form_maker_base_t::get_form()
{
  return mp_form.get();
}
void hrm::form_maker_base_t::reset_form(irs::handle_t<form_t> ap_form)
{
  mp_form = ap_form;
}
void hrm::form_maker_base_t::set_parent_form_maker(form_maker_base_t* ap_maker)
{
  mp_parent_form_maker = ap_maker;
}
hrm::form_maker_base_t* hrm::form_maker_base_t::get_parent_form_maker()
{
  return mp_parent_form_maker;
}

// class screensaver_t
hrm::screensaver_t::screensaver_t(
    mxdisplay_drv_service_t *ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event,
    eth_data_t* ap_eth_data):
  form_t(),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),
  m_str_1_item(),
  m_str_2_item(),
  m_str_3_item(),
  m_str_4_item(),
  m_main_screen(),
  mp_cur_menu(&m_main_screen),
  m_timeout(irs::make_cnt_s(5))
{
  m_str_1_item.set_parametr_string("    Высокоомная");
  m_str_2_item.set_parametr_string("     установка");
  m_str_3_item.set_parametr_string("     ООО \"РЭС\"");
  m_str_4_item.set_parametr_string("  www.irsural.ru");

  m_main_screen.set_disp_drv(ap_lcd_drv_service);
  m_main_screen.set_key_event(ap_menu_kb_event);
  m_main_screen.set_cursor_symbol(0x01);
  m_main_screen.creep_stop();
  m_main_screen.add(&m_str_1_item, 0, 0, IMM_FULL);
  m_main_screen.add(&m_str_2_item, 0, 1, IMM_FULL);
  m_main_screen.add(&m_str_3_item, 0, 2, IMM_FULL);
  m_main_screen.add(&m_str_4_item, 0, 3, IMM_FULL);

  m_timeout.start();
}

void hrm::screensaver_t::tick()
{
  form_t::tick();
  if (m_timeout.check()) {
    set_command(command_show_experiment_options_dialog);
  }
}

void hrm::screensaver_t::draw()
{
  mp_cur_menu->draw(&mp_cur_menu);
}

// class experiment_options_dialog_t
hrm::experiment_options_dialog_t::experiment_options_dialog_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event,
    eth_data_t* ap_eth_data):
  form_t(),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),
  m_mode_item(),
  m_hint_item(),
  m_r_standard(mp_eth_data->etalon),
  m_r_standard_item(&m_r_standard, true),
  m_r_standard_changed_event(),

  m_menu_r_standart_type(r_standard_type_original),
  m_r_standart_type_item(&m_menu_r_standart_type, true),
  m_r_standart_type_changed_event(),
  m_main_screen(),
  mp_cur_menu(&m_main_screen)
{
  m_mode_item.set_parametr_string("Ввод эталона");
  m_hint_item.set_parametr_string("€-Пуск");

  m_r_standard_item.set_header("Ввод эталона");
  m_r_standard_item.set_str(mp_user_str, "Rэ ", "Ом", r_width, r_precision,
    irs::num_mode_general);
  m_r_standard_item.set_max_value(r_max);
  m_r_standard_item.set_min_value(r_min);
  m_r_standard_item.add_change_event(&m_r_standard_changed_event);
  m_r_standard_item.set_key_type(IMK_DIGITS);

  m_r_standart_type_item.set_header("Тип Rэ");
  m_r_standart_type_item.set_str("        МЭС", "   Имитатор");
  m_r_standart_type_item.add_change_event(&m_r_standart_type_changed_event);

  m_main_screen.set_disp_drv(ap_lcd_drv_service);
  m_main_screen.set_key_event(ap_menu_kb_event);
  m_main_screen.set_cursor_symbol(0x01);
  m_main_screen.creep_stop();
  m_main_screen.add(&m_mode_item, 0, 0, IMM_FULL);
  m_main_screen.add(&m_r_standard_item, 0, 1, IMM_FULL);
  m_main_screen.add(&m_r_standart_type_item, 0, 2, IMM_FULL);
  m_main_screen.add(&m_hint_item, 0, 3, IMM_FULL);
}

void hrm::experiment_options_dialog_t::tick()
{
  form_t::tick();
  menu_check();
}

void hrm::experiment_options_dialog_t::draw()
{
  mp_cur_menu->draw(&mp_cur_menu);
}

void hrm::experiment_options_dialog_t::menu_check()
{
  if(m_r_standard_changed_event.check()) {
    mp_eth_data->etalon = m_r_standard;
  }

  if (mp_cur_menu == &m_main_screen) {
    if (mp_eth_data->etalon != m_r_standard) {
      m_r_standard = mp_eth_data->etalon;
    }
    irskey_t key = mp_menu_kb_event->check();
    switch (key) {
      case irskey_5: {
        mp_cur_menu = &m_r_standard_item;
        m_r_standard_item.set_master_menu(&m_main_screen);
        m_r_standard_item.show();
      } break;
      case irskey_6: {
        mp_cur_menu = &m_r_standart_type_item;
        m_r_standart_type_item.set_master_menu(&m_main_screen);
        m_r_standart_type_item.show();
      } break;
      case irskey_enter: {
        mp_eth_data->mode = md_balance;
        mp_eth_data->apply = 1;
        set_command(command_show_experiment_progress);
      } break;
      case irskey_backspace: {
        set_command(command_show_options);
      } break;
    }
  }
}

// class experiment_progress_t
hrm::experiment_progress_t::experiment_progress_t(
    mxdisplay_drv_service_t *ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event,
    eth_data_t* ap_eth_data):
  form_t(),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),
  m_mode_item(),
  m_elapsed_time_item(),
  m_remaining_time_item(),
  m_progress_bar_item(0x01, lcd_width, 1.f),
  m_update_items(irs::make_cnt_s(1)),
  m_main_screen(),
  mp_cur_menu(&m_main_screen)
{
  m_mode_item.set_parametr_string("Идет измерение");

  m_elapsed_time_item.prefix = irst("Прошло ");
  set_str_item(mp_eth_data->sum_time, &m_elapsed_time_item);

  m_remaining_time_item.prefix = irst("Осталось ");
  set_str_item(mp_eth_data->remaining_time, &m_remaining_time_item);

  m_main_screen.set_disp_drv(ap_lcd_drv_service);
  m_main_screen.set_key_event(ap_menu_kb_event);
  m_main_screen.set_cursor_symbol(0x01);
  m_main_screen.creep_stop();
  m_main_screen.add(&m_mode_item, 0, 0, IMM_FULL);
  m_main_screen.add(&m_elapsed_time_item.menu_item, 0, 1, IMM_FULL);
  m_main_screen.add(&m_remaining_time_item.menu_item, 0, 2, IMM_FULL);
  m_main_screen.add(&m_progress_bar_item, 0, 3, IMM_FULL);
}

void hrm::experiment_progress_t::tick()
{
  form_t::tick();
  menu_check();
}

void hrm::experiment_progress_t::draw()
{
  mp_cur_menu->draw(&mp_cur_menu);
}

void hrm::experiment_progress_t::menu_check()
{
  if (mp_cur_menu == &m_main_screen) {
    const irskey_t key = mp_menu_kb_event->check();
    switch (key) {
      case irskey_escape: {
        set_command(command_show_confirmation_stop_experiment);
      } break;
    }
  }
  if (m_update_items.check()) {
    set_str_item(mp_eth_data->sum_time, &m_elapsed_time_item);
    set_str_item(mp_eth_data->remaining_time, &m_remaining_time_item);
    update_progress();
  }
}

void hrm::experiment_progress_t::set_str_item(
  double a_time_s, menu_str_item_t* ap_menu_str_item)
{
  const irs::millisecond_t time_ms = irs::round<double, irs::millisecond_t>(
    a_time_s*1000);
  const bool show_ms = false;
  const string_type time_str = irs::ms_to_strtime(time_ms, show_ms);

  const string_type prefix = ap_menu_str_item->prefix;

  irs::ostringstream_t ostr;
  IRS_LIB_ASSERT(prefix.size() < lcd_width);
  ostr << prefix << setw(lcd_width - prefix.size()) << right << time_str;
  string_type msg = ostr.str();

  msg.resize(min<size_type>(msg.size(),
    IRS_ARRAYSIZE(ap_menu_str_item->cstr) - 1));
  strcpy(ap_menu_str_item->cstr, msg.c_str());
  ap_menu_str_item->menu_item.set_parametr_string(ap_menu_str_item->cstr);

}

void hrm::experiment_progress_t::update_progress()
{
  const double elapsed_time = mp_eth_data->sum_time;
  const double remaining_time = mp_eth_data->remaining_time;
  float progress = 0;
  if ((elapsed_time == 0) && (remaining_time == 0)) {
    progress = 0.f;
  }  else {
    progress = static_cast<float>(elapsed_time/(elapsed_time + remaining_time));
  }
  m_progress_bar_item.set_value(progress);
}

// class experiment_result_t
hrm::experiment_result_t::experiment_result_t(
    mxdisplay_drv_service_t *ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event,
    eth_data_t* ap_eth_data):
  form_t(),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),
  m_mode_item(),
  m_r_standard(mp_eth_data->etalon),
  m_r_standard_item(&m_r_standard, item_read_only),
  m_r_verifiable(mp_eth_data->result),
  m_r_verifiable_item(&m_r_verifiable, item_read_only),
  m_hint_item(),
  m_main_screen(),
  mp_cur_menu(&m_main_screen)
{
  m_mode_item.set_parametr_string("Результат:");

  m_r_standard_item.set_str(mp_r_standard_str, "Rэ ", "Ом",
    r_width, r_precision, irs::num_mode_general);
  m_r_standard_item.set_max_value(r_max);
  m_r_standard_item.set_min_value(r_min);
  m_r_standard_item.set_key_type(IMK_DIGITS);

  m_r_verifiable_item.set_str(mp_r_verifiable_str, "Rп ", "Ом",
    r_width, r_precision, irs::num_mode_general);
  m_r_verifiable_item.set_max_value(r_max);
  m_r_verifiable_item.set_min_value(r_min);
  m_r_standard_item.set_key_type(IMK_DIGITS);

  m_hint_item.set_parametr_string("1 - Расчет погрешн.");

  m_main_screen.set_disp_drv(ap_lcd_drv_service);
  m_main_screen.set_key_event(ap_menu_kb_event);
  m_main_screen.set_cursor_symbol(0x01);
  m_main_screen.creep_stop();
  m_main_screen.add(&m_mode_item, 0, 0, IMM_FULL);
  m_main_screen.add(&m_r_standard_item, 0, 1, IMM_FULL);
  m_main_screen.add(&m_r_verifiable_item, 0, 2, IMM_FULL);
  m_main_screen.add(&m_hint_item, 0, 3, IMM_FULL);
}

void hrm::experiment_result_t::tick()
{
  form_t::tick();
  m_r_standard = mp_eth_data->etalon;
  //m_r_verifiable = mp_eth_data->result;
  const double ration = mp_eth_data->ratio;
  const double r_etalon = mp_eth_data->etalon;
  m_r_verifiable = ration*r_etalon;
  menu_check();
}

void hrm::experiment_result_t::draw()
{
  mp_cur_menu->draw(&mp_cur_menu);
}

void hrm::experiment_result_t::menu_check()
{
  if (mp_cur_menu == &m_main_screen) {
    const irskey_t key = mp_menu_kb_event->check();

    switch (key) {
      case irskey_enter:
      case irskey_escape: {
        form_maker_base_t* parent_form_maker = get_parent_form_maker();
        if (dynamic_cast<form_maker_t<options_dialog_t>* >(
            parent_form_maker)) {
          set_command(command_show_options);
        } else {
          set_command(command_show_experiment_options_dialog);
        }
      } break;
      case irskey_1: {
        set_command(command_show_calculation_errors_dialog);
      } break;
    }
  }
}

// class confirmation_stop_experiment_t
hrm::confirmation_stop_experiment_t::confirmation_stop_experiment_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event,
    eth_data_t* ap_eth_data):
  form_t(),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),
  m_mode_item(),
  m_ok_item(),
  m_cancel_item(),
  m_main_screen(),
  mp_cur_menu(&m_main_screen)
{
  m_mode_item.set_parametr_string("Прервать измерение?");
  m_ok_item.set_parametr_string("€-Да");
  m_cancel_item.set_parametr_string("esc-Нет");

  m_main_screen.set_disp_drv(ap_lcd_drv_service);
  m_main_screen.set_key_event(ap_menu_kb_event);
  m_main_screen.set_cursor_symbol(0x01);
  m_main_screen.creep_stop();
  m_main_screen.add(&m_mode_item, 0, 0, IMM_FULL);
  m_main_screen.add(&m_ok_item, 0, 1, IMM_FULL);
  m_main_screen.add(&m_cancel_item, 0, 2, IMM_FULL);
}

void hrm::confirmation_stop_experiment_t::tick()
{
  form_t::tick();
  menu_check();
}

void hrm::confirmation_stop_experiment_t::draw()
{
  mp_cur_menu->draw(&mp_cur_menu);
}

void hrm::confirmation_stop_experiment_t::menu_check()
{
  if (mp_cur_menu == &m_main_screen) {
    irskey_t key = mp_menu_kb_event->check();
    switch (key) {
      case irskey_enter: {
        mp_eth_data->reset = 1;
        set_command(command_show_experiment_options_dialog);
      } break;
      case irskey_escape: {
        set_command(command_show_experiment_progress);
      } break;
    }
  }
}

// class calculation_errors_dialog_t
hrm::calculation_errors_dialog_t::calculation_errors_dialog_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event,
    eth_data_t* ap_eth_data):
  form_t(),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),

  m_r_nominal(1),
  m_r_nominal_item(&m_r_nominal, item_edited),
  m_r_nominal_changed_event(),

  m_r_prev_user(1),
  m_r_prev_user_item(&m_r_prev_user, item_edited),
  m_r_prev_user_changed_event(),

  m_deviation_min(-99.999),
  m_deviation_max(99.9999),
  m_deviation(0),
  m_deviation_item(&m_deviation, item_read_only),

  m_instability_min(-99.999),
  m_instability_max(99.9999),
  m_instability(0),
  m_instability_item(&m_instability, item_read_only),

  m_r_standard(mp_eth_data->etalon),
  m_r_standard_item(&m_r_standard, true),
  m_r_standard_changed_event(),

  m_main_screen(),
  mp_cur_menu(&m_main_screen),
  m_recalculation_period_timer(irs::make_cnt_s(0.1))
{
  m_r_nominal_item.set_header("Ввод номинала");

  m_r_nominal_item.set_str(mp_user_str,
    "Rн ", "Ом", r_width, r_precision, irs::num_mode_general);
  m_r_nominal_item.set_max_value(r_max);
  m_r_nominal_item.set_min_value(r_min);
  m_r_nominal_item.add_change_event(&m_r_nominal_changed_event);
  m_r_nominal_item.set_key_type(IMK_DIGITS);

  m_r_prev_user_item.set_header("Ввод предыдущ. знач.");
  m_r_prev_user_item.set_str(mp_user_str,
    "Rпп", "Ом", r_width, r_precision, irs::num_mode_general);
  m_r_prev_user_item.set_max_value(r_max);
  m_r_prev_user_item.set_min_value(r_min);
  m_r_prev_user_item.add_change_event(&m_r_prev_user_changed_event);
  m_r_prev_user_item.set_key_type(IMK_DIGITS);

  m_deviation_item.set_str(mp_user_str, "Откл. ном.", "%",
    deviation_width, deviation_precision, irs::num_mode_fixed);
  m_deviation_item.set_max_value(m_deviation_max);
  m_deviation_item.set_min_value(m_deviation_min);
  m_deviation_item.set_key_type(IMK_DIGITS);

  m_instability_item.set_str(mp_user_str, "Нестаб.   ", "%",
    instability_width, instability_precision, irs::num_mode_fixed);
  m_instability_item.set_max_value(m_instability_max);
  m_instability_item.set_min_value(m_instability_min);
  m_instability_item.set_key_type(IMK_DIGITS);

  m_r_standard_item.set_disp_drv(ap_lcd_drv_service);
  m_r_standard_item.set_key_event(ap_menu_kb_event);
  m_r_standard_item.set_cursor_symbol(0x01);
  m_r_standard_item.set_header("Ввод эталона");
  m_r_standard_item.set_str(mp_user_str, "Rэ ", "Ом",
    r_width, r_precision, irs::num_mode_general);
  m_r_standard_item.set_max_value(r_max);
  m_r_standard_item.set_min_value(r_min);
  m_r_standard_item.add_change_event(&m_r_standard_changed_event);
  m_r_standard_item.set_key_type(IMK_DIGITS);

  m_main_screen.set_disp_drv(ap_lcd_drv_service);
  m_main_screen.set_key_event(ap_menu_kb_event);
  m_main_screen.set_cursor_symbol(0x01);
  m_main_screen.creep_stop();
  m_main_screen.add(&m_r_nominal_item, 0, 0, IMM_FULL);
  m_main_screen.add(&m_r_prev_user_item, 0, 1, IMM_FULL);
  m_main_screen.add(&m_deviation_item, 0, 2, IMM_FULL);
  m_main_screen.add(&m_instability_item, 0, 3, IMM_FULL);

  calc_errors();
}

void hrm::calculation_errors_dialog_t::tick()
{
  form_t::tick();
  menu_check();
}

void hrm::calculation_errors_dialog_t::draw()
{
  mp_cur_menu->draw(&mp_cur_menu);
}

void hrm::calculation_errors_dialog_t::menu_check()
{
  if(m_r_nominal_changed_event.check()) {
    mp_eth_data->checked = m_r_nominal;
  }
  if(m_r_prev_user_changed_event.check()) {
    mp_eth_data->prev_user_result = m_r_prev_user;
  }

  if(m_r_standard_changed_event.check()) {
    mp_eth_data->etalon = m_r_standard;
  }

  if (mp_cur_menu == &m_main_screen) {
    sync_first_to_second(&mp_eth_data->checked, &m_r_nominal);
    sync_first_to_second(&mp_eth_data->prev_user_result, &m_r_prev_user);
    sync_first_to_second(&mp_eth_data->etalon, &m_r_standard);

    if (m_recalculation_period_timer.check()) {
      calc_errors();
    }
    irskey_t key = mp_menu_kb_event->check();
    switch (key) {
      case irskey_5: {
        mp_cur_menu = &m_r_standard_item;
        m_r_standard_item.set_master_menu(&m_main_screen);
        m_r_standard_item.show();
      } break;
      case irskey_8: {
        mp_cur_menu = &m_r_nominal_item;
        m_r_nominal_item.set_master_menu(&m_main_screen);
        m_r_nominal_item.show();
      } break;
      case irskey_9: {
        mp_cur_menu = &m_r_prev_user_item;
        m_r_prev_user_item.set_master_menu(&m_main_screen);
        m_r_prev_user_item.show();
      } break;
      case irskey_escape: {
        set_command(command_show_prev_form);
      } break;
    }
  }
}

void hrm::calculation_errors_dialog_t::calc_errors()
{
  const double r_nominal = m_r_nominal;
  const double r_verifiable_prev = m_r_prev_user;
  const double ration = mp_eth_data->ratio;
  const double r_etalon = mp_eth_data->etalon;
  const double r_verifiable = ration*r_etalon;

  if (r_nominal != 0) {
    m_deviation = (r_verifiable - r_nominal)/r_nominal*100;
    m_deviation = irs::bound(m_deviation, m_deviation_min, m_deviation_max);
  } else {
    m_deviation = m_deviation_max;
  }
  if (r_verifiable_prev != 0) {
    m_instability = (r_verifiable - r_verifiable_prev)/r_verifiable_prev*100;
    m_instability = irs::bound(m_instability,
      m_instability_min, m_instability_max);
  } else {
    m_instability = m_instability_max;
  }
}

// class network_options_t
hrm::network_options_t::network_options_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data):
  form_t(),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),
  m_parent_menu(),

  m_ip_item(m_menu_ip, mp_user_str, item_edited),
  m_trans_ip_event(),

  m_mask_item(m_menu_mask, mp_user_str, item_edited),
  m_trans_mask_event(),

  m_gateway_item(m_menu_gateway, mp_user_str, item_edited),
  m_trans_gateway_event(),

  m_menu_dhcp(static_cast<bool>(mp_eth_data->dhcp_on)),
  m_dhcp_item(&m_menu_dhcp, item_edited),
  m_trans_dhcp_event(),

  m_network_menu(),
  mp_cur_menu(&m_network_menu),
  m_options_changed(false)
{
  m_parent_menu.set_disp_drv(ap_lcd_drv_service);
  m_parent_menu.set_key_event(ap_menu_kb_event);
  m_parent_menu.creep_stop();

  m_ip_item.set_header("IP адрес");
  m_ip_item.add_change_event(&m_trans_ip_event);

  m_mask_item.set_header("Маска");
  m_mask_item.add_change_event(&m_trans_mask_event);

  m_gateway_item.set_header("Шлюз");
  m_gateway_item.add_change_event(&m_trans_gateway_event);

  m_dhcp_item.set_header("DHCP");
  m_dhcp_item.set_str("Включено", "Выключено");
  m_dhcp_item.add_change_event(&m_trans_dhcp_event);

  m_network_menu.set_disp_drv(ap_lcd_drv_service);
  m_network_menu.set_key_event(ap_menu_kb_event);
  m_network_menu.set_cursor_symbol(0x01);
  m_network_menu.set_master_menu(&m_parent_menu);
  m_network_menu.set_header(irst("Настройки сети"));

  m_network_menu.set_header(irst("Параметры сети"));

  m_network_menu.add(&m_ip_item);
  m_network_menu.add(&m_mask_item);
  m_network_menu.add(&m_gateway_item);
  m_network_menu.add(&m_dhcp_item);
}

void hrm::network_options_t::tick()
{
  form_t::tick();
  menu_check();
}

void hrm::network_options_t::draw()
{
  mp_cur_menu->draw(&mp_cur_menu);
}

void hrm::network_options_t::menu_check()
{
  if (m_trans_ip_event.check()) {
    mp_eth_data->ip_0 = m_menu_ip[0];
    mp_eth_data->ip_1 = m_menu_ip[1];
    mp_eth_data->ip_2 = m_menu_ip[2];
    mp_eth_data->ip_3 = m_menu_ip[3];
    m_options_changed = true;
  }

  if (m_trans_mask_event.check()) {
    mp_eth_data->mask_0 = m_menu_mask[0];
    mp_eth_data->mask_1 = m_menu_mask[1];
    mp_eth_data->mask_2 = m_menu_mask[2];
    mp_eth_data->mask_3 = m_menu_mask[3];
    m_options_changed = true;
  }

  if (m_trans_gateway_event.check()) {
    mp_eth_data->gateway_0 = m_menu_gateway[0];
    mp_eth_data->gateway_1 = m_menu_gateway[1];
    mp_eth_data->gateway_2 = m_menu_gateway[2];
    mp_eth_data->gateway_3 = m_menu_gateway[3];
    m_options_changed = true;
  }

  if (m_trans_dhcp_event.check()) {
    mp_eth_data->dhcp_on = m_menu_dhcp;
    m_options_changed = true;
  }
  if (mp_cur_menu == &m_network_menu) {
    sync_first_to_second(&mp_eth_data->ip_0, &m_menu_ip[0]);
    sync_first_to_second(&mp_eth_data->ip_1, &m_menu_ip[1]);
    sync_first_to_second(&mp_eth_data->ip_2, &m_menu_ip[2]);
    sync_first_to_second(&mp_eth_data->ip_3, &m_menu_ip[3]);

    sync_first_to_second(&mp_eth_data->mask_0, &m_menu_mask[0]);
    sync_first_to_second(&mp_eth_data->mask_1, &m_menu_mask[1]);
    sync_first_to_second(&mp_eth_data->mask_2, &m_menu_mask[2]);
    sync_first_to_second(&mp_eth_data->mask_3, &m_menu_mask[3]);

    sync_first_to_second(&mp_eth_data->gateway_0, &m_menu_gateway[0]);
    sync_first_to_second(&mp_eth_data->gateway_1, &m_menu_gateway[1]);
    sync_first_to_second(&mp_eth_data->gateway_2, &m_menu_gateway[2]);
    sync_first_to_second(&mp_eth_data->gateway_3, &m_menu_gateway[3]);

    sync_first_to_second(&mp_eth_data->dhcp_on, &m_menu_dhcp);
  } else if (mp_cur_menu == &m_parent_menu) {
    if (m_options_changed) {
      m_options_changed = false;
      mp_eth_data->apply_network_options = 1;
    }
    set_command(command_show_prev_form);
  }
}

// class options_dialog_t
hrm::options_dialog_t::options_dialog_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event,
    eth_data_t* ap_eth_data):
  form_t(),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),

  m_parent_menu(),

  m_prepare_pause_value(mp_eth_data->prepare_pause),
  m_prepare_pause_item(&m_prepare_pause_value, item_edited),
  m_prepare_pause_event(),

  m_thermostat_temperature_value(40),
  m_thermostat_temperature_item(&m_thermostat_temperature_value, item_edited),
  m_thermostat_temperature_event(),

  m_result_item(),
  m_calculation_errors_item(),
  m_network_options_item(),

  m_main_menu(),
  mp_cur_menu(&m_main_menu)
{
  m_parent_menu.set_disp_drv(ap_lcd_drv_service);
  m_parent_menu.set_key_event(ap_menu_kb_event);
  m_parent_menu.creep_stop();

  m_prepare_pause_item.set_header("Пауза после термос.");
  m_prepare_pause_item.set_str(mp_user_str, "Пауза", "с", 6, 0);
  m_prepare_pause_item.set_max_value(100000);
  m_prepare_pause_item.set_min_value(0.1);
  m_prepare_pause_item.add_change_event(&m_prepare_pause_event);
  m_prepare_pause_item.set_key_type(IMK_DIGITS);

  m_thermostat_temperature_item.set_header("Темп. термостата");
  m_thermostat_temperature_item.set_str(mp_user_str, "Темп.", "град.",
    2, 0);
  m_thermostat_temperature_item.set_max_value(99);
  m_thermostat_temperature_item.set_min_value(1);
  m_thermostat_temperature_item.add_change_event(
    &m_thermostat_temperature_event);
  m_thermostat_temperature_item.set_key_type(IMK_DIGITS);

  m_result_item.set_disp_drv(ap_lcd_drv_service);
  m_result_item.set_key_event(ap_menu_kb_event);
  m_result_item.creep_stop();
  m_result_item.set_header(irst("Результат"));

  m_calculation_errors_item.set_disp_drv(ap_lcd_drv_service);
  m_calculation_errors_item.set_key_event(ap_menu_kb_event);
  m_calculation_errors_item.creep_stop();
  m_calculation_errors_item.set_header(irst("Расчет погрешностей"));

  m_network_options_item.set_disp_drv(ap_lcd_drv_service);
  m_network_options_item.set_key_event(ap_menu_kb_event);
  m_network_options_item.creep_stop();
  m_network_options_item.set_header(irst("Настройки сети"));

  m_main_menu.set_disp_drv(ap_lcd_drv_service);
  m_main_menu.set_key_event(ap_menu_kb_event);
  m_main_menu.set_cursor_symbol(0x01);
  m_main_menu.set_master_menu(&m_parent_menu);
  m_main_menu.set_header(irst("Настройки"));
  m_main_menu.add(&m_prepare_pause_item);
  m_main_menu.add(&m_thermostat_temperature_item);
  m_main_menu.add(&m_result_item);
  m_main_menu.add(&m_calculation_errors_item);
  m_main_menu.add(&m_network_options_item);
}

void hrm::options_dialog_t::tick()
{
  form_t::tick();
  menu_check();
}

void hrm::options_dialog_t::draw()
{
  mp_cur_menu->draw(&mp_cur_menu);
}

void hrm::options_dialog_t::menu_check()
{
  if(m_prepare_pause_event.check()) {
    mp_eth_data->prepare_pause =
      irs::round<double, irs_i32>(m_prepare_pause_value);
  }
  if (mp_cur_menu == &m_main_menu) {
    if (mp_eth_data->prepare_pause != m_prepare_pause_value) {
      m_prepare_pause_value = mp_eth_data->prepare_pause;
    }
  } else if (mp_cur_menu == &m_parent_menu) {
    set_command(command_show_experiment_options_dialog);
  } else if (mp_cur_menu == &m_result_item) {
    set_command(command_show_experiment_result);
  } else if (mp_cur_menu == &m_calculation_errors_item) {
    set_command(command_show_calculation_errors_dialog);
  } else if (mp_cur_menu == &m_network_options_item) {
    set_command(command_show_network_options);
  }
}

hrm::options_dialog_t::size_type
hrm::options_dialog_t::get_current_item() const
{
  return m_main_menu.get_current_item();
}

void hrm::options_dialog_t::set_current_item(size_type a_item_index)
{
  m_main_menu.set_current_item(a_item_index);
}

// class menu_t
hrm::menu_t::menu_t(mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data):
  mp_lcd_drv_service(ap_lcd_drv_service),
  mp_menu_kb_event(ap_menu_kb_event),
  mp_eth_data(ap_eth_data),
  mp_form(NULL),
  mp_form_maker(),
  m_screensaver_maker(),
  m_experiment_options_dialog_maker(),
  m_experiment_progress_maker(),
  m_experiment_result_maker(),
  m_confirmation_stop_experiment_maker(),
  m_calculation_errors_dialog_maker(),
  m_options_dialog_maker()

{
  show_screensaver();
}

void hrm::menu_t::show_screensaver()
{
  reset(&m_screensaver_maker, mp_form_maker);
}

void hrm::menu_t::show_experiment_options()
{
  reset(&m_experiment_options_dialog_maker, mp_form_maker);
}

void hrm::menu_t::show_experiment_progress()
{
  reset(&m_experiment_progress_maker, mp_form_maker);
}

void hrm::menu_t::show_experiment_result()
{
  reset(&m_experiment_result_maker, mp_form_maker);
}

void hrm::menu_t::show_confirmation_stop_experiment()
{
  reset(&m_confirmation_stop_experiment_maker, mp_form_maker);
}

void hrm::menu_t::show_calculation_errors()
{
  reset(&m_calculation_errors_dialog_maker, mp_form_maker);
}

void hrm::menu_t::show_network_options()
{
  reset(&m_network_options_maker, mp_form_maker);
}

void hrm::menu_t::show_options()
{
  reset(&m_options_dialog_maker, mp_form_maker);
}

void hrm::menu_t::tick()
{
  mp_form->tick();
  check_commands();
}

void hrm::menu_t::check_commands()
{
  const form_t::command_t command = mp_form->get_command();
  switch (command) {
    case form_t::command_no: {
      // Ничего не делаем
    } break;
    case form_t::command_show_prev_form: {
      show_prev_form();
    } break;
    case form_t::command_show_screensaver: {
      show_screensaver();
    } break;
    case form_t::command_show_experiment_options_dialog: {
      show_experiment_options();
    } break;
    case form_t::command_show_experiment_progress: {
      show_experiment_progress();
    } break;
    case form_t::command_show_experiment_result: {
      show_experiment_result();
    } break;
    case form_t::command_show_confirmation_stop_experiment: {
      show_confirmation_stop_experiment();
    } break;
    case form_t::command_show_calculation_errors_dialog: {
      show_calculation_errors();
    } break;
    case form_t::command_show_network_options: {
      show_network_options();
    } break;
    case form_t::command_show_options: {
      show_options();
    } break;
  }
}

void hrm::menu_t::show_prev_form()
{
  form_maker_base_t* parent_form_maker = mp_form_maker->get_parent_form_maker();
  if (parent_form_maker) {
    mp_form_maker->destroy();
    mp_form = parent_form_maker->make(mp_lcd_drv_service, mp_menu_kb_event,
      mp_eth_data);
    mp_form_maker = parent_form_maker;
  } else {
    show_experiment_options();
  }
}

void hrm::menu_t::reset(form_maker_base_t* ap_maker,
  form_maker_base_t* ap_parent_form_maker)
{
  if (mp_form_maker) {
    mp_form_maker->destroy();
  }
  ap_maker->set_parent_form_maker(ap_parent_form_maker);
  mp_form = ap_maker->make(mp_lcd_drv_service, mp_menu_kb_event,
    mp_eth_data);
  mp_form_maker = ap_maker;
}
