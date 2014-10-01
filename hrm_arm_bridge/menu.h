#ifndef MENU_H
#define MENU_H

#include <irsdefs.h>

#include <irsmenu.h>

#include <hrm_bridge_data.h>

#include "cfg.h"
#include "privatecfg.h"
#include "utils.h"

#include <irsfinal.h>

namespace hrm {

enum r_standard_type_t {
  r_standard_type_original,
  r_standard_type_imitator
};

template <class first_t, class second_t>
bool sync_first_to_second(first_t* ap_first, second_t* ap_second)
{
  if ((*ap_first) != (*ap_second)) {
    *ap_second = *ap_first;
    return true;
  }
  return false;
}

class form_maker_base_t;
class menu_t;

class form_t
{
public:
  typedef size_t size_type;
  enum command_t {
    command_no,
    command_show_parent_form,
    command_show_prev_form,
    command_show_experiment_options_dialog,
    command_show_experiment_progress,
    command_show_experiment_result,
    command_show_confirmation_stop_experiment,
    command_show_calculation_errors_dialog,
    command_show_network_options,
    command_show_options
  };
  form_t();
  virtual ~form_t();
  virtual void set_parent_form_maker(form_maker_base_t* ap_maker);
  virtual command_t get_command() const;
  virtual void tick();
protected:
  enum { lcd_width = 20 };
  enum { item_edited = 1 };
  enum { item_read_only = 0 };
  enum { r_width = 13 };
  enum { r_precision = 8 };
  const double r_min;
  const double r_max;
  virtual void draw() = 0;
  form_maker_base_t* get_parent_form_maker();
  virtual void set_command(command_t a_command);
private:
  irs::loop_timer_t m_menu_timer;
  form_maker_base_t* mp_parent_form_maker;
  command_t m_command;
};

class form_maker_base_t
{
public:
  typedef size_t size_type;
  form_maker_base_t();
  virtual ~form_maker_base_t();
  virtual form_t* make(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event,
    eth_data_t* ap_eth_data) = 0;
  virtual void destroy();
  virtual form_t* get_form();
  virtual void set_parent_form_maker(form_maker_base_t* ap_maker);
  virtual form_maker_base_t* get_parent_form_maker();
protected:
  void reset_form(irs::handle_t<form_t> ap_form);
private:
  irs::handle_t<form_t> mp_form;
  form_maker_base_t* mp_parent_form_maker;
};

template <class form_type>
class form_maker_t: public form_maker_base_t
{
public:
  virtual form_t* make(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data)
  {
    irs::handle_t<form_t> form(new form_type(
      ap_lcd_drv_service, ap_menu_kb_event, ap_eth_data));
    form_maker_base_t* maker = get_parent_form_maker();
    form->set_parent_form_maker(maker);
    reset_form(form);
    return form.get();
  }
};

class experiment_options_dialog_t: public form_t
{
public:
  experiment_options_dialog_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data);
  virtual void tick();
private:
  virtual void draw();
  void menu_check();
  mxkey_event_t* mp_menu_kb_event;
  eth_data_t* mp_eth_data;

  irs_menu_string_item_t m_mode_item;
  irs_menu_string_item_t m_hint_item;

  double m_r_standard;
  irs_menu_double_item_t m_r_standard_item;
  mxfact_event_t m_r_standard_changed_event;

  irs_bool m_menu_r_standart_type;
  irs_menu_bool_item_t m_r_standart_type_item;
  mxfact_event_t m_r_standart_type_changed_event;

  static const irs_u8 m_user_str_len = 30;
  char mp_user_str[m_user_str_len + 1];
  irs_advanced_tablo_t m_main_screen;

  irs_menu_base_t* mp_cur_menu;
  command_t m_command;
};

class experiment_progress_t: public form_t
{
public:
  typedef size_t size_type;
  typedef irs::string_t string_type;
  enum command_t {
    command_stop
  };
  experiment_progress_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data);
  virtual void tick();
private:
  struct  menu_str_item_t;
  virtual void draw();
  void set_str_item(double a_time_s, menu_str_item_t* ap_menu_str_item);
  void update_progress();
  void menu_check();
  mxkey_event_t* mp_menu_kb_event;
  eth_data_t* mp_eth_data;

  struct menu_str_item_t
  {
    irs_menu_string_item_t menu_item;
    char cstr[lcd_width + 1];
    string_type prefix;
  };


  irs_menu_string_item_t m_mode_item;

  menu_str_item_t m_elapsed_time_item;
  menu_str_item_t m_remaining_time_item;

  irs_menu_progress_bar_t  m_progress_bar_item;

  irs::loop_timer_t m_update_items;

  char mp_elapsed_time_str[lcd_width + 1];
  char mp_remaining_time_str[lcd_width + 1];
  irs_advanced_tablo_t m_main_screen;

  irs_menu_base_t* mp_cur_menu;
};

class experiment_result_t: public form_t
{
public:
  typedef size_t size_type;
  typedef irs::string_t string_type;

  enum command_t {
    command_stop
  };
  experiment_result_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data);
  virtual void tick();
  irs::generator_events_1_t<command_t>* on_command();
private:
  virtual void draw();
  void menu_check();
  mxkey_event_t* mp_menu_kb_event;
  eth_data_t* mp_eth_data;
  mode_t m_mode;

  irs_menu_string_item_t m_mode_item;

  double m_r_standard;
  irs_menu_double_item_t m_r_standard_item;
  enum { r_standard_str_len = 30 };
  char mp_r_standard_str[r_standard_str_len + 1];

  double m_r_verifiable;
  irs_menu_double_item_t m_r_verifiable_item;
  enum { r_verifiable_str_len = 30 };
  char mp_r_verifiable_str[r_verifiable_str_len + 1];

  irs_menu_string_item_t m_hint_item;

  irs_advanced_tablo_t m_main_screen;

  irs_menu_base_t* mp_cur_menu;
};

class confirmation_stop_experiment_t: public form_t
{
public:
  confirmation_stop_experiment_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data);
  virtual void tick();
private:
  virtual void draw();
  void menu_check();
  mxkey_event_t* mp_menu_kb_event;
  eth_data_t* mp_eth_data;

  irs_menu_string_item_t m_mode_item;
  irs_menu_string_item_t m_ok_item;
  irs_menu_string_item_t m_cancel_item;

  irs_advanced_tablo_t m_main_screen;

  irs_menu_base_t* mp_cur_menu;
  command_t m_command;
};

class calculation_errors_dialog_t: public form_t
{
public:
  calculation_errors_dialog_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data);
  virtual void tick();
private:
  virtual void draw();
  void menu_check();
  void calc_errors();
  enum { deviation_width = 7 };
  enum { deviation_precision = 4 };
  enum { instability_width = 7 };
  enum { instability_precision = 4 };

  mxkey_event_t* mp_menu_kb_event;
  eth_data_t* mp_eth_data;

  double m_r_nominal;
  irs_menu_double_item_t m_r_nominal_item;
  mxfact_event_t m_r_nominal_changed_event;

  double m_r_prev_user;
  irs_menu_double_item_t m_r_prev_user_item;
  mxfact_event_t m_r_prev_user_changed_event;

  const double m_deviation_min;
  const double m_deviation_max;

  double m_deviation;
  irs_menu_double_item_t m_deviation_item;

  const double m_instability_min;
  const double m_instability_max;

  double m_instability;
  irs_menu_double_item_t m_instability_item;

  enum { buffer_str_len = 30 };
  char mp_r_nominal_str[buffer_str_len + 1];
  char mp_r_prev_user_str[buffer_str_len + 1];
  char mp_deviation_str[buffer_str_len + 1];
  char mp_instability_str[buffer_str_len + 1];

  irs_advanced_tablo_t m_main_screen;

  irs_menu_base_t* mp_cur_menu;
  command_t m_command;
  irs::loop_timer_t m_recalculation_period_timer;
};

class network_options_t: public form_t
{
public:
  network_options_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data);
  virtual void tick();
  size_type get_current_item() const;
  void set_current_item(size_type a_item_index);
private:
  virtual void draw();
  void menu_check();

  mxkey_event_t* mp_menu_kb_event;
  eth_data_t* mp_eth_data;

  enum { buffer_str_len = 30 };
  char mp_user_str[buffer_str_len + 1];

  irs_advanced_tablo_t m_parent_menu;

  irs_u8 m_menu_ip[4];
  irs_menu_ip_item_t m_ip_item;
  mxfact_event_t m_trans_ip_event;

  irs_u8 m_menu_mask[4];
  irs_menu_ip_item_t m_mask_item;
  mxfact_event_t m_trans_mask_event;

  irs_u8 m_menu_gateway[4];
  irs_menu_ip_item_t m_gateway_item;
  mxfact_event_t m_trans_gateway_event;

  irs_bool m_menu_dhcp;
  irs_menu_bool_item_t m_dhcp_item;
  mxfact_event_t m_trans_dhcp_event;
  irs_advanced_menu_t m_network_menu;
  irs_menu_base_t* mp_cur_menu;

  bool m_options_changed;
};

class options_dialog_t: public form_t
{
public:
  options_dialog_t(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data);
  virtual void tick();
  size_type get_current_item() const;
  void set_current_item(size_type a_item_index);
private:
  virtual void draw();
  void menu_check();
  mxkey_event_t* mp_menu_kb_event;
  eth_data_t* mp_eth_data;

  enum { buffer_str_len = 30 };
  char mp_user_str[buffer_str_len + 1];

  irs_advanced_tablo_t m_parent_menu;

  double m_prepare_pause_value;
  irs_menu_double_item_t m_prepare_pause_item;
  mxfact_event_t m_prepare_pause_event;

  double m_thermostat_temperature_value;
  irs_menu_double_item_t m_thermostat_temperature_item;
  mxfact_event_t m_thermostat_temperature_event;

  irs_advanced_tablo_t m_result_item;
  irs_advanced_tablo_t m_calculation_errors_item;

  irs_advanced_tablo_t m_network_options_item;

  irs_advanced_menu_t m_main_menu;
  irs_menu_base_t* mp_cur_menu;
};

template <>
class form_maker_t<options_dialog_t>: public form_maker_base_t
{
public:
  form_maker_t():
    m_current_item(0)
  {
  }
  virtual form_t* make(
    mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data)
  {
    irs::handle_t<form_t> form(new options_dialog_t(
      ap_lcd_drv_service, ap_menu_kb_event, ap_eth_data));
    form_maker_base_t* maker = get_parent_form_maker();
    form->set_parent_form_maker(maker);
    options_dialog_t* settings_dialog =
      static_cast<options_dialog_t*>(form.get());
    settings_dialog->set_current_item(m_current_item);
    reset_form(form);
    return form.get();
  }
  virtual void destroy()
  {
    if (get_form()) {
      options_dialog_t* form = static_cast<options_dialog_t*>(get_form());
      m_current_item = form->get_current_item();
      form_maker_base_t::destroy();
    }
  }
private:
  size_type m_current_item;
};

class menu_t
{
public:
  menu_t(mxdisplay_drv_service_t* ap_lcd_drv_service,
    mxkey_event_t* ap_menu_kb_event, eth_data_t* ap_eth_data);
  void show_experiment_options();
  void show_experiment_progress();
  void show_experiment_result();
  void show_confirmation_stop_experiment();
  void show_calculation_errors();
  void show_network_options();
  void show_options();
  void tick();
private:
  menu_t();
  void show_prev_form();
  void reset(form_maker_base_t* ap_maker,
    form_maker_base_t *ap_parent_form_maker);
  void check_commands();
  mxdisplay_drv_service_t* mp_lcd_drv_service;
  mxkey_event_t* mp_menu_kb_event;
  eth_data_t* mp_eth_data;
  form_t* mp_form;

  form_maker_base_t* mp_form_maker;
  form_maker_t<experiment_options_dialog_t> m_experiment_options_dialog_maker;
  form_maker_t<experiment_progress_t> m_experiment_progress_maker;
  form_maker_t<experiment_result_t> m_experiment_result_maker;
  form_maker_t<confirmation_stop_experiment_t>
    m_confirmation_stop_experiment_maker;
  form_maker_t<calculation_errors_dialog_t> m_calculation_errors_dialog_maker;
  form_maker_t<network_options_t> m_network_options_maker;
  form_maker_t<options_dialog_t> m_options_dialog_maker;
};

} // namespace hrm

#endif // MENU_H
