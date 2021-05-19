#include <irsdefs.h>

#include <irsstrm.h>
#include <irsmcutil.h>
#include <irsinit.h>
#include <armcfg.h>
#include <irserror.h>
#include <irslocale.h>

#include "app.h"

#include <irsfinal.h>

enum { 
  hardware_rev = 3,
  software_rev = 97,
  mxsrclib_rev = 1361,
  extern_libs_rev = 26
};

void app_start(hrm::cfg_t* ap_cfg, irs_u32 a_version);

void main()
{
  // Ethernet PHY clock 50 MHz
  RCC_CFGR_bit.MCO1PRE = 5; //  101 = Div 3
  RCC_CFGR_bit.MCO1 = 3;    //  11 = PLL
  irs::clock_enable(PA8);
  irs::gpio_moder_alternate_function_enable(PA8);
  irs::gpio_alternate_function_select(PA8, GPIO_AF_MCO);
  GPIOA_OSPEEDR_bit.OSPEEDR8 = 3; //100 MHz High speed on 30 pF
  //pll_on();
  irs::param_pll_t param_pll;
  param_pll.freq_quartz = 25000000;
  param_pll.PLLM = 25;  // Делитель на входе PLL, Fvco = 1 MHz
  param_pll.PLLN = 300; // Множитель внутри PLL, Fpll = 300 MHz
  param_pll.PLLP = 0;   // 0 это деление на 2, делитель для ядра 150 МГц
  param_pll.PLLQ = 3;   // Делитель для USB 48 МГц
  // APB Low speed prescaler (APB1). 101: AHB clock divided by 4
  param_pll.PPRE1 = 5;
  // APB high-speed prescaler (APB2). 101: 100: AHB clock divided by 2
  param_pll.PPRE2 = 4;
  param_pll.FLASH_STATE = 3;
  param_pll.HSEBYP = 0; //HSE clock bypass External crystal/ceramic resonator
  irs::pll_on(param_pll);

  static hard_fault_event_t hard_fault_event(GPIO_PORTD, 9);  //  Red LED

  static irs::arm::com_buf log_buf(1, 10, 115200);
  irs::loc();
  irs::mlog().rdbuf(&log_buf);
  irs::mlog() << endl;
  irs::mlog() << endl;
  irs::mlog() << irsm("--------- INITIALIZATION --------") << endl;
  irs::mlog() << irsm("hardware rev. ") << hardware_rev << endl;
  irs::mlog() << irsm("software rev. ") << software_rev << endl;
  irs::mlog() << irsm("mxsrclib rev. ") << mxsrclib_rev << endl;
  irs::mlog() << irsm("extern_libs rev. ") << extern_libs_rev << endl;
  irs::mlog() << endl;

  irs::pause(irs::make_cnt_s(1));
  
  static hrm::cfg_t cfg;
  app_start(&cfg, software_rev);
}

void app_start(hrm::cfg_t* ap_cfg, irs_u32 a_version)
{
  static hrm::app_t app(ap_cfg, a_version);

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
    static irs::blink_t green_led_blink(GPIO_PORTD, 8, irs::make_cnt_ms(100));
    green_led_blink(); // Мигание зелёным светодиодом на плате arm

    #ifdef HRM_DEBUG
    count++;
    tick_time += tick_measure_time.get();
    if (period_update_measure_time.get_cnt() >= period) {
      IRS_DBG_RAW_MSG(CNT_TO_DBLTIME(counter_get()) <<
        " Время вызова main tick  " <<
        tick_time/period_update_measure_time.get() << " count= " <<
        count/period_update_measure_time.get()<< endl);

      tick_time = 0;
      count = 0;
      period_update_measure_time.start();
    }

    #endif // HRM_DEBUG
  }
}
