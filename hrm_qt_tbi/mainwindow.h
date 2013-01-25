#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <irsdefs.h>

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QProgressBar>
#include <QSettings>

#include <hrm_defs.h>
#include "cfg.h"
#include "app.h"

#include <irsfinal.h>

namespace Ui {
class MainWindow;
}

class MainWindow: public QMainWindow
{
  Q_OBJECT
  
public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private slots:
  void tick();

  void on_TestLedCheckBox_clicked();

  void on_SetCodeButton_clicked();

  void on_AbortButton_clicked();

  void on_ClearButton_clicked();

  void on_SaveSettingsButton_clicked();

  void on_ActionButton_clicked();

private:
  Ui::MainWindow *ui;
  hrm::cfg_t m_cfg;
  hrm::app_t m_app;
  QTimer m_timer;
  QLabel m_core_temperature_label;

  irs::string_t m_text_line_string;
  irs::string_t m_multimeter_string;
  bool m_r2r_comparator;
  bool m_led;
  irs_u32 m_code_value;
  irs::string_t m_connected_string;
  irs::string_t m_core_temperature_string;

  bool m_multimeter_box_enabled;
  bool m_balance_box_enabled;
  bool m_calibre_box_enabled;
  bool m_code_box_enabled;
  bool m_led_enabled;
  bool m_connected_label_enabled;
  bool m_core_temperature_label_enabled;

  int m_time_value;

  void collect_balance_params(hrm::app_t::balance_params_t* ap_params);
  void save_balance_params(hrm::app_t::balance_params_t* ap_params);
};

#endif // MAINWINDOW_H
