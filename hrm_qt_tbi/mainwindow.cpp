#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  m_cfg(),
  m_app(m_cfg),
  m_timer(this),
  m_core_temperature_label(this),
  m_text_line_string(),
  m_multimeter_string(),
  m_led(false),
  m_code_value(0),
  m_connected_string(),
  m_core_temperature_string(),
  m_multimeter_box_enabled(true),
  m_balance_box_enabled(true),
  m_calibre_box_enabled(true),
  m_code_box_enabled(true),
  m_led_enabled(true),
  m_connected_label_enabled(true),
  m_core_temperature_label_enabled(true),
  m_time_value(0)
{
  ui->setupUi(this);

  connect(&m_timer, SIGNAL(timeout()), this, SLOT(tick()));
  m_timer.start(10);

  //m_progress_bar.set;
  //ui->statusBar->addPermanentWidget(&m_progress_bar);
  ui->statusBar->addPermanentWidget(&m_core_temperature_label);

  QSettings settings("hrm_settings.ini", QSettings::IniFormat);
  settings.beginGroup("Balance");

  //irs::ostringstream_t stream;
  //stream << fixed << setprecision(0) << params.pause;
  ui->BalancePauseSpinBox->setValue(settings.value("Pause", 10).toDouble());
  ui->BalanceAdaptivePauseCheckBox->
    setChecked(settings.value("Adaptive", true).toBool());
  ui->BalanceMeasCntSpinBox->setValue(settings.value("MeasCount", 1).toInt());
  ui->BalanceExpCntSpinBox->setValue(settings.value("ExpCount", 1).toInt());
  ui->CoilEtalonSpinBox->setValue(settings.value("EtalonValue",
    1e6).toDouble() / 1e6);
  ui->CoilCheckedSpinBox->setValue(settings.value("CheckedValue",
    1e6).toDouble() / 1e6);
  if (settings.value("BalanceMode", "bm_multimeter")
    == "bm_multimeter") {
    ui->BalanceUseMultimeterButton->setChecked(true);
  } else {
    ui->BalanceUseComparatorButton->setChecked(true);
  }
  settings.endGroup();
  settings.beginGroup("Calibre");
  ui->CalibreSubtractBiasCheckBox->
    setChecked(settings.value("SubtractBias").toBool());
  ui->CalibreCheckingWeight->setChecked(settings.value("CheckWeight").toBool());
  vector<double> calibre_vector(settings.beginReadArray("CalibreVector"));
  for (irs_u8 i = 0; i < calibre_vector.size(); i++) {
    settings.setArrayIndex(i);
    calibre_vector[i] = settings.value("Point", 1.).toDouble();
  }
  m_app.set_calibre_vector(&calibre_vector);
  settings.endArray();
  settings.endGroup();
  ui->NPLCSpinBox->setValue(settings.value("NPLC").toDouble());
  ui->LSBCountSpinBox->setValue(settings.value("LSBCount").toInt());
  ui->LSBMeasCountSpinBox->setValue(settings.value("LSBMeasCount").toInt());
  ui->BufferTestDigitsSpinBox->
    setValue(settings.value("BufferTestDigits").toInt());
  ui->SourceVoltageSpinBox->
    setValue(settings.value("SourceVoltage").toDouble());
  ui->R2RScanStartDigitSpinBox->
    setValue(settings.value("R2RScanStartDigit").toInt());
  ui->R2RScanDigitsSpinBox->
    setValue(settings.value("R2RScanDigits").toInt());
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::tick()
{
  if (m_app.get_connected_label_text(&m_connected_string)) {
    ui->statusBar->showMessage(QString::fromStdWString(m_connected_string));
  }
  if (m_app.get_core_temperature_label_text(&m_core_temperature_string)) {
    m_core_temperature_label.setText(
      QString::fromStdWString(m_core_temperature_string));
  }
  if (m_app.get_multimeter_value(&m_multimeter_string)) {
    ui->MultimeterLabel->setText(QString::fromStdWString(m_multimeter_string));
  }
  if (m_app.get_r2r_comparator_value(&m_r2r_comparator)) {
    if (m_r2r_comparator) {
      ui->ComparatorLabel->setText("+");
    } else {
      ui->ComparatorLabel->setText("-");
    }
  }
  if (m_app.get_led_check_box_checked(&m_led)) {
    ui->TestLedCheckBox->setChecked(m_led);
  }
  if (m_app.get_code_spin_box_value(&m_code_value)) {
    ui->CodeSpinBox->setValue(m_code_value);
  }
  if (m_app.get_new_log_line(&m_text_line_string)) {
    ui->MemoLog->append(QString::fromStdWString(m_text_line_string));
  }
  if (m_app.get_new_report_line(&m_text_line_string)) {
    ui->MemoReport->append(QString::fromStdWString(m_text_line_string));
  }
  if (m_app.get_multimeter_box_enabled() != m_multimeter_box_enabled) {
    m_multimeter_box_enabled = m_app.get_multimeter_box_enabled();
    ui->MultimeterBox->setEnabled(m_multimeter_box_enabled);
  }
  if (m_app.get_balance_box_enabled() != m_balance_box_enabled) {
    m_balance_box_enabled = m_app.get_balance_box_enabled();
    ui->BalanceBox->setEnabled(m_balance_box_enabled);
  }
  if (m_app.get_code_box_enabled() != m_code_box_enabled) {
    m_code_box_enabled = m_app.get_code_box_enabled();
    ui->CodeBox->setEnabled(m_code_box_enabled);
  }
  if (m_app.get_led_enabled() != m_led_enabled) {
    m_led_enabled = m_app.get_led_enabled();
    ui->TestLedCheckBox->setEnabled(m_led_enabled);
  }
  if (m_app.get_connected_label_enabled() != m_connected_label_enabled) {
    m_connected_label_enabled = m_app.get_connected_label_enabled();
    ui->statusBar->setEnabled(m_connected_label_enabled);
  }
  if (m_app.get_core_temperature_label_enabled() !=
    m_core_temperature_label_enabled) {
    m_core_temperature_label_enabled =
    m_app.get_core_temperature_label_enabled();
    m_core_temperature_label.setEnabled(m_core_temperature_label_enabled);
  }
  if (m_app.get_time_value(&m_time_value)) {
    ui->progressBar->setValue(m_time_value);
  }
  m_app.tick();
}

void MainWindow::on_TestLedCheckBox_clicked()
{
  m_app.led_check_box_pressed(ui->TestLedCheckBox->checkState());
}

void MainWindow::on_SetCodeButton_clicked()
{
  m_app.code_set_button_pressed(ui->CodeSpinBox->value());
}

void MainWindow::on_AbortButton_clicked()
{
  m_app.abort_button_pressed();
}

void MainWindow::on_ClearButton_clicked()
{
  ui->MemoReport->clear();
  ui->MemoLog->clear();
}

void MainWindow::on_SaveSettingsButton_clicked()
{
  hrm::app_t::balance_params_t params;
  collect_balance_params(&params);
  save_balance_params(&params);
}

void MainWindow::collect_balance_params(hrm::app_t::balance_params_t *ap_params)
{
  ap_params->pause = ui->BalancePauseSpinBox->value();
  ap_params->adaptive = ui->BalanceAdaptivePauseCheckBox->checkState();
  ap_params->meas_count = ui->BalanceMeasCntSpinBox->value();
  ap_params->exp_count = ui->BalanceExpCntSpinBox->value();
  ap_params->etalon_value = ui->CoilEtalonSpinBox->value() * 1e6;
  ap_params->checked_value = ui->CoilCheckedSpinBox->value() * 1e6;
  if (ui->BalanceUseComparatorButton->isChecked()) {
    ap_params->balance_mode = hrm::bm_comparator;
  } else {
    ap_params->balance_mode = hrm::bm_multimeter;
  }
  ap_params->subtract_bias = ui->CalibreSubtractBiasCheckBox->checkState();
  ap_params->check_weight = ui->CalibreCheckingWeight->checkState();
  ap_params->nplc = ui->NPLCSpinBox->value();
  ap_params->lsb_count = ui->LSBCountSpinBox->value();
  ap_params->lsb_meas_count = ui->LSBMeasCountSpinBox->value();
  ap_params->buffer_test_digits = ui->BufferTestDigitsSpinBox->value();
  ap_params->source_voltage = ui->SourceVoltageSpinBox->value();
  if (ui->CalibreRadioButton->isChecked()) {
    ap_params->target_action = hrm::st_calibre;
  } else if (ui->BalanceRadioButton->isChecked()) {
    ap_params->target_action = hrm::st_balance;
  } else if (ui->BalanceEmulateRadioButton->isChecked()) {
    ap_params->target_action = hrm::st_balance_emulate;
  } else if (ui->BufferTestRadioButton->isChecked()) {
    ap_params->target_action = hrm::st_buffer_test;
  } else if (ui->R2RScanRadioButton->isChecked()) {
    ap_params->target_action = hrm::st_r2r_scan;
  } else if (ui->R2RScanEmulateRadioButton->isChecked()) {
    ap_params->target_action = hrm::st_r2r_scan_emulate;
  } else if (ui->OneMeasRadioButton->isChecked()) {
    ap_params->target_action = hrm::st_one_meas;
  } else {
    ap_params->target_action = hrm::st_idle;
  }
  ap_params->r2r_scan_start_digit = ui->R2RScanStartDigitSpinBox->value();
  ap_params->r2r_scan_digits = ui->R2RScanDigitsSpinBox->value();
}

void MainWindow::save_balance_params(hrm::app_t::balance_params_t* ap_params)
{
  QSettings settings("hrm_settings.ini", QSettings::IniFormat);
  settings.beginGroup("Balance");
  settings.setValue("Pause", ap_params->pause);
  settings.setValue("Adaptive", ap_params->adaptive);
  settings.setValue("MeasCount", ap_params->meas_count);
  settings.setValue("ExpCount", ap_params->exp_count);
  settings.setValue("EtalonValue", ap_params->etalon_value);
  settings.setValue("CheckedValue", ap_params->checked_value);
  if (ap_params->balance_mode == hrm::bm_multimeter) {
    settings.setValue("BalanceMode", "bm_multimeter");
  } else {
    settings.setValue("BalanceMode", "bm_comparator");
  }
  settings.endGroup();
  settings.beginGroup("Calibre");
  settings.setValue("SubtractBias", ap_params->subtract_bias);
  settings.setValue("CheckWeight", ap_params->check_weight);
  settings.endGroup();
  vector<double> calibre_vector;
  bool exist_bias = false;
  if (m_app.get_calibre_vector(&calibre_vector, &exist_bias)) {
    settings.beginGroup("Calibre");
    settings.beginWriteArray("CalibreVector");
    for (irs_u8 i = 0; i < calibre_vector.size() - 1; i++) {
      settings.setArrayIndex(i);
      settings.setValue("Point", calibre_vector[i]);
    }
    if (exist_bias) {
      settings.endArray();
      settings.setValue("Bias", calibre_vector[calibre_vector.size() - 1]);
    } else {
      settings.setArrayIndex(calibre_vector.size() - 1);
      settings.setValue("Point", calibre_vector[calibre_vector.size() - 1]);
      settings.endArray();

    }
    settings.endGroup();
  }
  settings.setValue("NPLC", ap_params->nplc);
  settings.setValue("LSBCount", ap_params->lsb_count);
  settings.setValue("LSBMeasCount", ap_params->lsb_meas_count);
  settings.setValue("BufferTestDigits", ap_params->buffer_test_digits);
  settings.setValue("SourceVoltage", ap_params->source_voltage);
  settings.setValue("R2RScanStartDigit", ap_params->r2r_scan_start_digit);
  settings.setValue("R2RScanDigits", ap_params->r2r_scan_digits);
}

void MainWindow::on_ActionButton_clicked()
{
  hrm::app_t::balance_params_t params;
  collect_balance_params(&params);
  save_balance_params(&params);
  m_app.action_button_pressed(&params);
}
