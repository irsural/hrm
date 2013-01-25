#include <irsdefs.h>

#include <irsstrm.h>
#include <irsmcutil.h>
#include <irsinit.h>
#include <armcfg.h>
#include <irserror.h>

#include "app.h"

#include <irsfinal.h>

enum { HRM_rev = 2, mxsrclib_rev = 666 };

void app_start(hrm::cfg_t* ap_cfg);

void main()
{
  pll_on();

  static hard_fault_event_t hard_fault_event(GPIO_PORTD, 0);
  
  static irs::arm::com_buf log_buf(1, 10, 115200);
  irs::mlog().rdbuf(&log_buf);
  #ifdef HRM_DEBUG
  static irs::mc_error_handler_t error_handler(GPIO_PORTD, 0, &irs::mlog());
  #endif // HRM_DEBUG
  irs::mlog() << endl;
  irs::mlog() << endl;
  irs::mlog() << irsm("--------- INITIALIZATION --------") << endl;
  irs::mlog() << irsm("HRM rev. ") << HRM_rev << irsm(", ");
  irs::mlog() << irsm("mxsrclib rev. ") << mxsrclib_rev << endl;

  irs::arm::io_pin_t m_memory_chip_select_pin(GPIO_PORTD, 7, irs::io_t::dir_out,
    irs::io_pin_on);
  
  static hrm::cfg_t cfg;
  app_start(&cfg);
}

void app_start(hrm::cfg_t* ap_cfg)
{
  static hrm::app_t app(ap_cfg);
  
  irs::mlog() << irsm("------------- START -------------") << endl;
  while(true) {
    #ifdef HRM_DEBUG
    static const counter_t period = irs::make_cnt_s(1);
    static irs::measure_time_t period_update_measure_time;
    static double tick_time;
    static int count = 0;
    static irs::measure_time_t tick_measure_time;
    tick_measure_time.start();
    #endif // HRM_DEBUG
    app.tick();
    static irs::blink_t F0_blink(GPIO_PORTD, 1, irs::make_cnt_ms(100));
    F0_blink(); // ������� ����������� �� ����� arm
    
    #ifdef HRM_DEBUG
    count++;
    tick_time += tick_measure_time.get();
    if (period_update_measure_time.get_cnt() >= period) {
      IRS_DBG_RAW_MSG(CNT_TO_DBLTIME(counter_get()) <<
        " ����� ������ main tick  " <<
        tick_time/period_update_measure_time.get() << " count= " <<
        count/period_update_measure_time.get()<< endl);

      tick_time = 0;
      count = 0;
      period_update_measure_time.start();
    }

    #endif // HRM_DEBUG
  }
}