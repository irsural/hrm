#ifndef datah
#define datah

#include <irsdefs.h>

#include <mxdata.h>

#include "privatecfg.h"

#include <irsfinal.h>

namespace hrm {

enum mode_t {
  md_free = 0,
  md_manual = 1,
  md_balance = 2,
  md_scan = 3
};

typedef double  adc_value_t;
typedef double  dac_value_t;

struct eth_data_t {
  irs::conn_data_t<irs_u32> counter;        //  4 byte
  //  command
  irs::conn_data_t<irs_u8> mode;
  irs::conn_data_t<irs_u8> range;
  irs::conn_data_t<irs_u8> exp_cnt;
  irs::bit_data_t etpol;
  irs::bit_data_t change_coils_polarity;
  irs::bit_data_t no_zero_balance;
  irs::bit_data_t no_zero_elab;
  irs::bit_data_t inc_elab_voltage;
  irs::bit_data_t no_prot;
  irs::bit_data_t reset;
  irs::bit_data_t apply;
  irs::bit_data_t complete; // Измерения завершены
  //  status
  irs::conn_data_t<irs_u8> current_mode;    //  1 byte
  irs::conn_data_t<irs_u8> current_range;   //  1 byte
  irs::conn_data_t<irs_u8> current_exp_cnt; //  1 byte
  irs::bit_data_t current_etpol;
  //  values
  irs::conn_data_t<double> etalon;          //  8 byte
  irs::conn_data_t<double> checked;         //  8 byte
  irs::conn_data_t<double> result;          //  8 byte
  irs::conn_data_t<double> result_error;    //  8 byte
  irs::conn_data_t<double> ratio;           //  8 byte
  irs::conn_data_t<double> prev_user_result;
  //  manual
  irs::bit_data_t relay_1g;
  irs::bit_data_t relay_100m;
  irs::bit_data_t relay_10m;
  irs::bit_data_t relay_1m;
  irs::bit_data_t relay_100k;
  irs::bit_data_t relay_chon;
  irs::bit_data_t relay_eton;
  irs::bit_data_t relay_prot;
  irs::bit_data_t relay_zero;
  irs::bit_data_t relay_etpol;
  irs::bit_data_t relay_bridge_pos;
  irs::bit_data_t relay_bridge_neg;
  irs::bit_data_t relay_gain;
  irs::bit_data_t relay_voltage;
  irs::bit_data_t optimize_balance;
  irs::conn_data_t<irs_u8> dac_lin;
  irs::conn_data_t<irs_u8> elab_cnt;
  irs::bit_data_t dac_on;
  irs::conn_data_t<dac_value_t> dac_normalize_code;
  irs::conn_data_t<irs_i32> dac_code;
  irs::conn_data_t<irs_u8> adc_filter;
  irs::conn_data_t<irs_u8> adc_mode;
  irs::conn_data_t<irs_u8> adc_channel;
  irs::conn_data_t<irs_u8> adc_gain;
  irs::conn_data_t<adc_value_t> adc_value;
  irs::conn_data_t<adc_value_t> adc_zero;
  irs::conn_data_t<adc_value_t> adc_temperature;
  irs::conn_data_t<irs_u32> dac_pause_ms;
  irs::conn_data_t<irs_u32> relay_pause_ms;
  irs::conn_data_t<dac_value_t> dac_center_scan;
  irs::conn_data_t<irs_i32> int_dac_center_scan;
  irs::conn_data_t<irs_u32> prepare_pause;
  irs::conn_data_t<irs_u32> adc_average_cnt;
  irs::conn_data_t<irs_u32> exp_time;
  irs::conn_data_t<irs_u32> prev_exp_time;
  irs::conn_data_t<irs_u32> sum_time;
  irs::conn_data_t<irs_u32> remaining_time;
  irs::conn_data_t<irs_u32> options;
  irs::bit_data_t wild_relays;
  irs::bit_data_t adc_clear_filter;
  irs::bit_data_t start_adc_sequence;
  irs::bit_data_t auto_elab_step;
  irs::bit_data_t adc_simply_show;
  irs::bit_data_t termostat_off;
  irs::bit_data_t termostat_is_off;
  irs::bit_data_t adc_meas_process;
  irs::conn_data_t<irs_u8> elab_mode;
  irs::bit_data_t elab_pid_on;
  irs::bit_data_t elab_pid_sync;
  irs::conn_data_t<adc_value_t> adc_filter_constant;
  irs::conn_data_t<adc_value_t> adc_filter_value;
  irs::conn_data_t<irs_u32> adc_average_skip_cnt;
  irs::conn_data_t<irs_u32> adc_experiment_gain;
  irs::conn_data_t<irs_u32> adc_experiment_filter;
  irs::conn_data_t<adc_value_t> adc_additional_gain;
  irs::conn_data_t<adc_value_t> adc_ref;
  irs::conn_data_t<dac_value_t> dac_voltage_pos;
  irs::conn_data_t<dac_value_t> dac_voltage_neg;
  irs::conn_data_t<irs_i32> elab_step;
  irs::conn_data_t<irs_i32> min_elab_cnt;
  irs::conn_data_t<irs_i32> max_elab_cnt;
  irs::conn_data_t<irs_u32> dac_elab_pause_ms;
  irs::conn_data_t<dac_value_t> elab_step_multiplier;
  irs::conn_data_t<double> elab_max_delta;
  irs::conn_data_t<double> external_temperature;
  irs::conn_data_t<double> termostat_off_pause;
  irs::conn_data_t<double> elab_pid_kp;
  irs::conn_data_t<double> elab_pid_ki;
  irs::conn_data_t<double> elab_pid_kd;
  irs::conn_data_t<double> elab_pid_int;
  irs::conn_data_t<irs_u8> ip_0;  //  1 byte
  irs::conn_data_t<irs_u8> ip_1;  //  1 byte
  irs::conn_data_t<irs_u8> ip_2;  //  1 byte
  irs::conn_data_t<irs_u8> ip_3;  //  1 byte
  irs::conn_data_t<irs_u8> mask_0;  //  1 byte
  irs::conn_data_t<irs_u8> mask_1;  //  1 byte
  irs::conn_data_t<irs_u8> mask_2;  //  1 byte
  irs::conn_data_t<irs_u8> mask_3;  //  1 byte
  irs::conn_data_t<irs_u8> gateway_0;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_1;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_2;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_3;     //  1 byte
  irs::bit_data_t dhcp_on;                //  1 byte
  irs::bit_data_t disable_reading_network_options; // 1 bit
  irs::bit_data_t apply_network_options;  //  1 bit
  //------------------------------------------

  eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    index = counter.connect(ap_data, index);

    index = mode.connect(ap_data, index);
    index = range.connect(ap_data, index);
    index = exp_cnt.connect(ap_data, index);
    etpol.connect(ap_data, index, 0);
    change_coils_polarity.connect(ap_data, index, 1);
    no_zero_balance.connect(ap_data, index, 2);
    no_zero_elab.connect(ap_data, index, 3);
    inc_elab_voltage.connect(ap_data, index, 4);
    no_prot.connect(ap_data, index, 5);
    reset.connect(ap_data, index, 6);
    apply.connect(ap_data, index, 7);
    index++;
    complete.connect(ap_data, index, 0);
    index++;

    index = current_mode.connect(ap_data, index);
    index = current_range.connect(ap_data, index);
    index = current_exp_cnt.connect(ap_data, index);
    current_etpol.connect(ap_data, index, 0);
    index++;
    //index+=3;

    index = etalon.connect(ap_data, index);
    index = checked.connect(ap_data, index);
    index = result.connect(ap_data, index);
    index = result_error.connect(ap_data, index);
    index = ratio.connect(ap_data, index);
    index = prev_user_result.connect(ap_data, index);

    relay_1g.connect(ap_data, index, 0);
    relay_100m.connect(ap_data, index, 1);
    relay_10m.connect(ap_data, index, 2);
    relay_1m.connect(ap_data, index, 3);
    relay_100k.connect(ap_data, index, 4);
    relay_chon.connect(ap_data, index, 5);
    relay_eton.connect(ap_data, index, 6);
    relay_prot.connect(ap_data, index, 7);
    index++;

    relay_zero.connect(ap_data, index, 0);
    relay_etpol.connect(ap_data, index, 1);
    dac_on.connect(ap_data, index, 2);
    relay_bridge_pos.connect(ap_data, index, 3);
    relay_bridge_neg.connect(ap_data, index, 4);
    relay_gain.connect(ap_data, index, 5);
    relay_voltage.connect(ap_data, index, 6);

    optimize_balance.connect(ap_data, index, 7);
    index++;

    index = dac_lin.connect(ap_data, index);
    index = elab_cnt.connect(ap_data, index);

    index = dac_normalize_code.connect(ap_data, index);
    index = dac_code.connect(ap_data, index);
    index = adc_filter.connect(ap_data, index);
    index = adc_mode.connect(ap_data, index);
    index = adc_channel.connect(ap_data, index);
    index = adc_gain.connect(ap_data, index);
    index = adc_value.connect(ap_data, index);
    index = adc_zero.connect(ap_data, index);
    index = adc_temperature.connect(ap_data, index);
    index = dac_pause_ms.connect(ap_data, index);
    index = relay_pause_ms.connect(ap_data, index);
    index = dac_center_scan.connect(ap_data, index);
    index = int_dac_center_scan.connect(ap_data, index);
    index = prepare_pause.connect(ap_data, index);
    index = adc_average_cnt.connect(ap_data, index);
    index = exp_time.connect(ap_data, index);
    index = prev_exp_time.connect(ap_data, index);
    index = sum_time.connect(ap_data, index);
    index = remaining_time.connect(ap_data, index);

    wild_relays.connect(ap_data, index, 0);
    adc_clear_filter.connect(ap_data, index, 1);
    start_adc_sequence.connect(ap_data, index, 2);
    auto_elab_step.connect(ap_data, index, 3);
    adc_simply_show.connect(ap_data, index, 4);
    termostat_off.connect(ap_data, index, 5);
    termostat_is_off.connect(ap_data, index, 6);
    adc_meas_process.connect(ap_data, index, 7);
    elab_mode.connect(ap_data, index);
    index = options.connect(ap_data, index);

    index= adc_filter_constant.connect(ap_data, index);
    index= adc_filter_value.connect(ap_data, index);

    index = adc_average_skip_cnt.connect(ap_data, index);
    index = adc_experiment_gain.connect(ap_data, index);
    index = adc_experiment_filter.connect(ap_data, index);

    index = adc_additional_gain.connect(ap_data, index);
    index = adc_ref.connect(ap_data, index);
    index = dac_voltage_pos.connect(ap_data, index);
    index = dac_voltage_neg.connect(ap_data, index);
    index = elab_step.connect(ap_data, index);
    index = min_elab_cnt.connect(ap_data, index);
    index = max_elab_cnt.connect(ap_data, index);
    index = dac_elab_pause_ms.connect(ap_data, index);
    index = elab_step_multiplier.connect(ap_data, index);
    index = elab_max_delta.connect(ap_data, index);
    index = external_temperature.connect(ap_data, index);
    index = termostat_off_pause.connect(ap_data, index);
    index = elab_pid_kp.connect(ap_data, index);
    index = elab_pid_ki.connect(ap_data, index);
    index = elab_pid_kd.connect(ap_data, index);
    index = elab_pid_int.connect(ap_data, index);

    index = ip_0.connect(ap_data, index);
    index = ip_1.connect(ap_data, index);
    index = ip_2.connect(ap_data, index);
    index = ip_3.connect(ap_data, index);

    index = mask_0.connect(ap_data, index);
    index = mask_1.connect(ap_data, index);
    index = mask_2.connect(ap_data, index);
    index = mask_3.connect(ap_data, index);

    index = gateway_0.connect(ap_data, index);
    index = gateway_1.connect(ap_data, index);
    index = gateway_2.connect(ap_data, index);
    index = gateway_3.connect(ap_data, index);

    dhcp_on.connect(ap_data, index, 0);
    disable_reading_network_options.connect(ap_data, index, 1);
    apply_network_options.connect(ap_data, index, 2);

    return index;
  }
};  //  eth_data_t

struct eeprom_data_t {
  irs::conn_data_t<irs_u32> exp_cnt;
  irs::conn_data_t<double> etalon;          //  8 byte
  irs::conn_data_t<double> checked;         //  8 byte
  irs::conn_data_t<irs_u32> dac_lin;
  irs::conn_data_t<irs_u32> elab_cnt;
  irs::conn_data_t<irs_u32> dac_pause_ms;
  irs::conn_data_t<irs_u32> relay_pause_ms;
  irs::conn_data_t<dac_value_t> dac_center_scan;
  irs::conn_data_t<irs_i32> int_dac_center_scan;
  irs::conn_data_t<irs_u32> prepare_pause;
  irs::conn_data_t<irs_u32> adc_average_cnt;
  irs::conn_data_t<adc_value_t> adc_filter_constant;
  irs::conn_data_t<irs_u32> adc_average_skip_cnt;
  irs::conn_data_t<irs_u32> adc_experiment_gain;
  irs::conn_data_t<irs_u32> adc_experiment_filter;
  irs::conn_data_t<adc_value_t> adc_additional_gain;
  irs::conn_data_t<adc_value_t> adc_ref;
  irs::conn_data_t<dac_value_t> dac_voltage_pos;
  irs::conn_data_t<dac_value_t> dac_voltage_neg;
  irs::conn_data_t<irs_u32> options;
  irs::bit_data_t no_prot;
  irs::bit_data_t optimize_balance;
  irs::bit_data_t wild_relays;
  irs::bit_data_t auto_elab_step;
  irs::bit_data_t adc_simply_show;
  irs::conn_data_t<irs_u8> elab_mode;
  irs::conn_data_t<irs_i32> elab_step;
  irs::conn_data_t<irs_i32> min_elab_cnt;
  irs::conn_data_t<irs_i32> max_elab_cnt;
  irs::conn_data_t<irs_u32> dac_elab_pause_ms;
  irs::conn_data_t<dac_value_t> elab_step_multiplier;
  irs::conn_data_t<dac_value_t> elab_max_delta;
  irs::conn_data_t<double> ratio;
  irs::conn_data_t<double> prev_user_result;
  irs::conn_data_t<irs_u8> ip_0;          //  1 byte
  irs::conn_data_t<irs_u8> ip_1;          //  1 byte
  irs::conn_data_t<irs_u8> ip_2;          //  1 byte
  irs::conn_data_t<irs_u8> ip_3;          //  1 byte
  irs::conn_data_t<irs_u8> mask_0;        //  1 byte
  irs::conn_data_t<irs_u8> mask_1;        //  1 byte
  irs::conn_data_t<irs_u8> mask_2;        //  1 byte
  irs::conn_data_t<irs_u8> mask_3;        //  1 byte
  irs::conn_data_t<irs_u8> gateway_0;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_1;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_2;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_3;     //  1 byte
  irs::bit_data_t dhcp_on;                //  1 bit
  irs::conn_data_t<double> termostat_off_pause;
  irs::conn_data_t<double> elab_pid_kp;
  irs::conn_data_t<double> elab_pid_ki;
  irs::conn_data_t<double> elab_pid_kd;

  eeprom_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }
  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;
    index = exp_cnt.connect(ap_data, index);
    index = etalon.connect(ap_data, index);
    index = checked.connect(ap_data, index);
    index = dac_lin.connect(ap_data, index);
    index = elab_cnt.connect(ap_data, index);
    index = dac_pause_ms.connect(ap_data, index);
    index = relay_pause_ms.connect(ap_data, index);
    index = dac_center_scan.connect(ap_data, index);
    index = int_dac_center_scan.connect(ap_data, index);
    index = prepare_pause.connect(ap_data, index);
    index = adc_average_cnt.connect(ap_data, index);
    index = adc_filter_constant.connect(ap_data, index);
    index = adc_average_skip_cnt.connect(ap_data, index);
    index = adc_experiment_gain.connect(ap_data, index);
    index = adc_experiment_filter.connect(ap_data, index);
    index = adc_additional_gain.connect(ap_data, index);
    index = adc_ref.connect(ap_data, index);
    index = dac_voltage_pos.connect(ap_data, index);
    index = dac_voltage_neg.connect(ap_data, index);
    no_prot.connect(ap_data, index, 0);
    optimize_balance.connect(ap_data, index, 1);
    wild_relays.connect(ap_data, index, 2);
    auto_elab_step.connect(ap_data, index, 3);
    adc_simply_show.connect(ap_data, index, 4);
    elab_mode.connect(ap_data, index);
    index = options.connect(ap_data, index);
    index = elab_step.connect(ap_data, index);
    index = min_elab_cnt.connect(ap_data, index);
    index = max_elab_cnt.connect(ap_data, index);
    index = dac_elab_pause_ms.connect(ap_data, index);
    index = elab_step_multiplier.connect(ap_data, index);
    index = elab_max_delta.connect(ap_data, index);
    index = ratio.connect(ap_data, index);
    index = prev_user_result.connect(ap_data, index);
    index = ip_0.connect(ap_data, index);
    index = ip_1.connect(ap_data, index);
    index = ip_2.connect(ap_data, index);
    index = ip_3.connect(ap_data, index);
    index = mask_0.connect(ap_data, index);
    index = mask_1.connect(ap_data, index);
    index = mask_2.connect(ap_data, index);
    index = mask_3.connect(ap_data, index);
    index = gateway_0.connect(ap_data, index);
    index = gateway_1.connect(ap_data, index);
    index = gateway_2.connect(ap_data, index);
    index = gateway_3.connect(ap_data, index);
    index = dhcp_on.connect(ap_data, index, 0);
    index+=4;
    index = termostat_off_pause.connect(ap_data, index);
    index = elab_pid_kp.connect(ap_data, index);
    index = elab_pid_ki.connect(ap_data, index);
    index = elab_pid_kd.connect(ap_data, index);
    return index;
  }
  inline void reset_to_default()
  {
    exp_cnt = 1;
    etalon = 1.;
    checked = 1.;
    dac_lin = 0;
    elab_cnt = 3;
    dac_pause_ms = 100;
    relay_pause_ms = 100;
    dac_center_scan = 0.;
    int_dac_center_scan = 0;
    prepare_pause = 5;
    adc_average_cnt = 0;
    adc_filter_constant = 100.;
    adc_average_skip_cnt = 0;
    adc_experiment_gain = 7;
    adc_experiment_filter = 15;
    adc_additional_gain = 83.;
    adc_ref = 4.096;
    dac_voltage_pos = 12.288;
    dac_voltage_neg = -12.288;
    no_prot = 0;
    optimize_balance = 0;
    wild_relays = 0;
    auto_elab_step = 0;
    elab_step = 1;
    min_elab_cnt = 1;
    max_elab_cnt = 5;
    dac_elab_pause_ms = 101;
    elab_step_multiplier = 1.0;
    adc_simply_show = 0;
    elab_max_delta = 1.0;
    ratio = 1.0;
    prev_user_result = 1.0;
    ip_0 = IP_0;
    ip_1 = IP_1;
    ip_2 = IP_2;
    ip_3 = IP_3;

    mask_0 = HRM_MASK_0;
    mask_1 = HRM_MASK_1;
    mask_2 = HRM_MASK_2;
    mask_3 = HRM_MASK_3;

    gateway_0 = HRM_GATEWAY_0;
    gateway_1 = HRM_GATEWAY_1;
    gateway_2 = HRM_GATEWAY_2;
    gateway_3 = HRM_GATEWAY_3;

    dhcp_on = HRM_DHCP_ON;
    
    termostat_off_pause = 3.1459;
    elab_pid_kp = 1.0;
    elab_pid_ki = 0.0;
    elab_pid_kd = 0.0;
    elab_mode = 0;
  }
};

} // namespace hrm

#endif // datah
