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
enum adc_default_params_t {
  adc_default_gain = 0,
  adc_default_filter = 15,
  adc_default_channel_voltage = 0,
  adc_default_channel_temperature = 1,
  adc_default_cnv_cnt_temperature = 3,
  adc_default_cnv_cnt_voltage = 10,
  adc_default_cont_cnv_cnt = 15,
  adc_default_impf_iterations_cnt = 1,
  adc_default_impf_type = 0,
  adc_default_cont_mode = 0
};

enum fan_mode_t {
  fan_already_off = 0,
  fan_already_on = 1,
  fan_idle_on = 2
};

enum fan_status_t {
  fan_off_now = 0,
  fan_on_now = 1
};

enum balance_action_t {
  ba_idle = 0,
  ba_prepare = 1,
  ba_prepare_pause = 2,
  ba_balance_neg = 3,
  ba_elab_neg = 4,
  ba_balance_pos = 5,
  ba_elab_pos = 6
};

typedef double  adc_value_t;
typedef double  dac_value_t;
typedef double  th_value_t;

const adc_value_t adc_additional_gain = 1.0;
const adc_value_t adc_ref = 4.096;
const adc_value_t adc_default_cont_sko = 1.0e-6;

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
  irs::conn_data_t<double> checked_prev;    //  8 byte
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
  irs::conn_data_t<irs_u8> old_adc_filter;
  irs::conn_data_t<irs_u8> old_adc_mode;
  irs::conn_data_t<irs_u8> old_adc_channel;
  irs::conn_data_t<irs_u8> old_adc_gain;
  irs::conn_data_t<adc_value_t> adc_value;
  irs::conn_data_t<adc_value_t> adc_impf;
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
  irs::bit_data_t elab_pid_reset;
  irs::bit_data_t elab_pid_sko_ready;
  irs::bit_data_t use_impf;
  irs::bit_data_t test_impf;
  irs::bit_data_t adc_continious;
  irs::bit_data_t adc_show_points;
  irs::bit_data_t show_last_result;
  irs::conn_data_t<irs_u8> balance_action;
  irs::conn_data_t<irs_u8> reserve_1;
  irs::conn_data_t<irs_u16> reserve_2;
  irs::conn_data_t<adc_value_t> adc_filter_constant;
  irs::conn_data_t<adc_value_t> adc_filter_value;
  irs::conn_data_t<irs_u32> adc_average_skip_cnt;
  irs::conn_data_t<irs_u32> adc_experiment_gain;
  irs::conn_data_t<irs_u32> adc_experiment_filter;
  irs::conn_data_t<adc_value_t> old_adc_additional_gain;
  irs::conn_data_t<adc_value_t> old_adc_ref;
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
  irs::conn_data_t<double> elab_iso_k;
  irs::conn_data_t<double> elab_iso_t;
  irs::conn_data_t<double> elab_iso_out;
  irs::conn_data_t<double> elab_pid_fade_out;
  irs::conn_data_t<double> elab_pid_fade_t;
  irs::conn_data_t<double> elab_pid_avg;
  irs::conn_data_t<double> elab_pid_sko;
  irs::conn_data_t<double> elab_pid_avg_norm;
  irs::conn_data_t<double> elab_pid_sko_norm;
  irs::conn_data_t<irs_u32> elab_pid_avg_cnt;
  irs::conn_data_t<double> elab_pid_target_sko;
  irs::conn_data_t<double> elab_pid_target_sko_norm;
  irs::conn_data_t<double> elab_pid_ref;
  irs::conn_data_t<double> adc_meas_freq;
  irs::conn_data_t<double> adc_point_time;
  irs::conn_data_t<double> adc_sko;
  irs::conn_data_t<double> adc_sko_impf;
  irs::conn_data_t<double> adc_max_value_prot;
  irs::conn_data_t<double> adc_max_value_no_prot;
  irs::conn_data_t<double> adc_impf_alternate;
  irs::conn_data_t<double> adc_impf_alternate_sko;
  irs::conn_data_t<irs_i32> adc_raw;
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
  //  ADC actual params
  irs::conn_data_t<irs_u8> adc_gain;                  //  1
  irs::conn_data_t<irs_u8> adc_filter;                //  1
  irs::conn_data_t<irs_u8> adc_channel;               //  1
  irs::conn_data_t<irs_u32> adc_cnv_cnt;              //  4
  irs::conn_data_t<double> adc_additional_gain;       //  8
  irs::conn_data_t<double> adc_ref;                   //  8
  irs::conn_data_t<irs_u32> adc_cont_cnv_cnt;         //  4
  irs::conn_data_t<irs_u32> adc_impf_iterations_cnt;  //  4
  irs::conn_data_t<irs_u8> adc_impf_type;             //  1
  irs::conn_data_t<irs_u8> adc_cont_mode;             //  1
  irs::conn_data_t<double> adc_cont_sko;              //  8
  //  ADC free / voltage
  irs::conn_data_t<irs_u8> adc_free_vx_gain;          //  1
  irs::conn_data_t<irs_u8> adc_free_vx_filter;        //  1
  irs::conn_data_t<irs_u8> adc_free_vx_channel;       //  1
  irs::conn_data_t<irs_u32> adc_free_vx_cnv_cnt;      //  4
  irs::conn_data_t<double> adc_free_vx_additional_gain;       //  8
  irs::conn_data_t<double> adc_free_vx_ref;           //  8
  irs::conn_data_t<irs_u32> adc_free_vx_cont_cnv_cnt; //  4
  irs::conn_data_t<irs_u32> adc_free_vx_impf_iterations_cnt;  //  4
  irs::conn_data_t<irs_u8> adc_free_vx_impf_type;     //  1
  irs::conn_data_t<irs_u8> adc_free_vx_cont_mode;     //  1
  irs::conn_data_t<double> adc_free_vx_cont_sko;      //  8
  //  ADC free / temperature
  irs::conn_data_t<irs_u8> adc_free_th_gain;          //  1
  irs::conn_data_t<irs_u8> adc_free_th_filter;        //  1
  irs::conn_data_t<irs_u8> adc_free_th_channel;       //  1
  irs::conn_data_t<irs_u32> adc_free_th_cnv_cnt;      //  4
  irs::conn_data_t<double> adc_free_th_additional_gain;       //  8
  irs::conn_data_t<double> adc_free_th_ref;           //  8
  irs::conn_data_t<irs_u32> adc_free_th_cont_cnv_cnt; //  4
  irs::conn_data_t<irs_u32> adc_free_th_impf_iterations_cnt;  //  4
  irs::conn_data_t<irs_u8> adc_free_th_impf_type;     //  1
  irs::conn_data_t<irs_u8> adc_free_th_cont_mode;     //  1
  irs::conn_data_t<double> adc_free_th_cont_sko;      //  8
  //  ADC manual
  irs::conn_data_t<irs_u8> adc_manual_gain;           //  1
  irs::conn_data_t<irs_u8> adc_manual_filter;         //  1
  irs::conn_data_t<irs_u8> adc_manual_channel;        //  1
  irs::conn_data_t<irs_u32> adc_manual_cnv_cnt;       //  4
  irs::conn_data_t<double> adc_manual_additional_gain;        //  8
  irs::conn_data_t<double> adc_manual_ref;            //  8
  irs::conn_data_t<irs_u32> adc_manual_cont_cnv_cnt;  //  4
  irs::conn_data_t<irs_u32> adc_manual_impf_iterations_cnt;   //  4
  irs::conn_data_t<irs_u8> adc_manual_impf_type;      //  1
  irs::conn_data_t<irs_u8> adc_manual_cont_mode;      //  1
  irs::conn_data_t<double> adc_manual_cont_sko;       //  8
  //  ADC balance
  irs::conn_data_t<irs_u8> adc_balance_gain;          //  1
  irs::conn_data_t<irs_u8> adc_balance_filter;        //  1
  irs::conn_data_t<irs_u8> adc_balance_channel;       //  1
  irs::conn_data_t<irs_u32> adc_balance_cnv_cnt;      //  4
  irs::conn_data_t<double> adc_balance_additional_gain;       //  8
  irs::conn_data_t<double> adc_balance_ref;           //  8
  irs::conn_data_t<irs_u32> adc_balance_cont_cnv_cnt; //  4
  irs::conn_data_t<irs_u32> adc_balance_impf_iterations_cnt;  //  4
  irs::conn_data_t<irs_u8> adc_balance_impf_type;     //  1
  irs::conn_data_t<irs_u8> adc_balance_cont_mode;     //  1
  irs::conn_data_t<double> adc_balance_cont_sko;      //  8
  //  ADC elab
  irs::conn_data_t<irs_u8> adc_elab_gain;             //  1
  irs::conn_data_t<irs_u8> adc_elab_filter;           //  1
  irs::conn_data_t<irs_u8> adc_elab_channel;          //  1
  irs::conn_data_t<irs_u32> adc_elab_cnv_cnt;         //  4
  irs::conn_data_t<double> adc_elab_additional_gain;          //  8
  irs::conn_data_t<double> adc_elab_ref;              //  8
  irs::conn_data_t<irs_u32> adc_elab_cont_cnv_cnt;    //  4
  irs::conn_data_t<irs_u32> adc_elab_impf_iterations_cnt;     //  4
  irs::conn_data_t<irs_u8> adc_elab_impf_type;        //  1
  irs::conn_data_t<irs_u8> adc_elab_cont_mode;        //  1
  irs::conn_data_t<double> adc_elab_cont_sko;         //  8
  //  Bridge Assimetry Correction
  irs::conn_data_t<double> bac_old_coefficient;       //  8
  irs::conn_data_t<double> bac_new_coefficient;       //  8
  irs::conn_data_t<irs_i32> bac_new_int_coefficient;  //  4
  irs::conn_data_t<double> bac_new_int_multiplier;    //  8
  //  OnBoard temperature sensors, voltages and fans
  irs::conn_data_t<th_value_t> th_dac;                //  8
  irs::conn_data_t<th_value_t> th_box_ldo;            //  8
  irs::conn_data_t<th_value_t> th_box_adc;            //  8
  irs::conn_data_t<th_value_t> th_mcu;                //  8
  irs::conn_data_t<th_value_t> th_ext_1;              //  8
  irs::conn_data_t<th_value_t> th_ext_2;              //  8
  irs::conn_data_t<th_value_t> volt_box_neg;          //  8
  irs::conn_data_t<th_value_t> volt_box_pos;          //  8
  irs::conn_data_t<irs_u8> fan_mode;                  //  1
  irs::conn_data_t<irs_u8> fan_status;                //  1
  irs::conn_data_t<irs_u8> fan_ac_speed;              //  1
  irs::conn_data_t<irs_u8> fan_dc_speed;              //  1
  irs::conn_data_t<float> fan_dc_speed_sense;         //  4
  //  Termoregulator
  irs::conn_data_t<th_value_t> treg_ref;              //  8
  irs::conn_data_t<th_value_t> treg_result;           //  8
  irs::conn_data_t<th_value_t> treg_k;                //  8
  irs::conn_data_t<th_value_t> treg_ki;               //  8
  irs::conn_data_t<th_value_t> treg_kd;               //  8
  irs::conn_data_t<th_value_t> treg_iso_k;            //  8
  irs::conn_data_t<th_value_t> treg_iso_t;            //  8
  irs::conn_data_t<th_value_t> treg_pwm_rate_slope;   //  8
  irs::conn_data_t<th_value_t> treg_pid_out;          //  8
  irs::conn_data_t<th_value_t> treg_amplitude_code_float;   //  8
  irs::conn_data_t<irs_u32> treg_options;             //  4
  irs::bit_data_as_bool_t treg_enabled;
  irs::bit_data_as_bool_t treg_pid_reg_enabled;
  irs::bit_data_as_bool_t treg_polarity_pin_bit_data;
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
    index = checked_prev.connect(ap_data, index);

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
    index = old_adc_filter.connect(ap_data, index);
    index = old_adc_mode.connect(ap_data, index);
    index = old_adc_channel.connect(ap_data, index);
    index = old_adc_gain.connect(ap_data, index);
    index = adc_value.connect(ap_data, index);
    index = adc_impf.connect(ap_data, index);
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

    wild_relays.connect(ap_data,            index, 0);
    adc_clear_filter.connect(ap_data,       index, 1);
    start_adc_sequence.connect(ap_data,     index, 2);
    auto_elab_step.connect(ap_data,         index, 3);
    adc_simply_show.connect(ap_data,        index, 4);
    termostat_off.connect(ap_data,          index, 5);
    termostat_is_off.connect(ap_data,       index, 6);
    adc_meas_process.connect(ap_data,       index, 7);
    elab_mode.connect(ap_data,              index + 1);
    elab_pid_on.connect(ap_data,            index + 2, 0);
    elab_pid_sync.connect(ap_data,          index + 2, 1);
    elab_pid_reset.connect(ap_data,         index + 2, 2);
    elab_pid_sko_ready.connect(ap_data,     index + 2, 3);
    use_impf.connect(ap_data,               index + 2, 4);
    test_impf.connect(ap_data,              index + 2, 5);
    adc_continious.connect(ap_data,         index + 2, 6);
    adc_show_points.connect(ap_data,        index + 2, 7);
    show_last_result.connect(ap_data,       index + 3, 0);
    index = options.connect(ap_data,        index);
    
    index= balance_action.connect(ap_data, index);
    index= reserve_1.connect(ap_data, index);
    index= reserve_2.connect(ap_data, index);

    index= adc_filter_constant.connect(ap_data, index);
    index= adc_filter_value.connect(ap_data, index);

    index = adc_average_skip_cnt.connect(ap_data, index);
    index = adc_experiment_gain.connect(ap_data, index);
    index = adc_experiment_filter.connect(ap_data, index);

    index = old_adc_additional_gain.connect(ap_data, index);
    index = old_adc_ref.connect(ap_data, index);
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
    index = elab_iso_k.connect(ap_data, index);
    index = elab_iso_t.connect(ap_data, index);
    index = elab_iso_out.connect(ap_data, index);
    index = elab_pid_fade_out.connect(ap_data, index);
    index = elab_pid_fade_t.connect(ap_data, index);
    index = elab_pid_avg.connect(ap_data, index);
    index = elab_pid_sko.connect(ap_data, index);
    index = elab_pid_avg_norm.connect(ap_data, index);
    index = elab_pid_sko_norm.connect(ap_data, index);
    index = elab_pid_avg_cnt.connect(ap_data, index);
    index = elab_pid_target_sko.connect(ap_data, index);
    index = elab_pid_target_sko_norm.connect(ap_data, index);
    index = elab_pid_ref.connect(ap_data, index);
    index = adc_meas_freq.connect(ap_data, index);
    index = adc_point_time.connect(ap_data, index);
    index = adc_sko.connect(ap_data, index);
    index = adc_sko_impf.connect(ap_data, index);
    index = adc_max_value_prot.connect(ap_data, index);
    index = adc_max_value_no_prot.connect(ap_data, index);
    index = adc_impf_alternate.connect(ap_data, index);
    index = adc_impf_alternate_sko.connect(ap_data, index);
    index = adc_raw.connect(ap_data, index);

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
    index++;
    //  ADC actual params
    index = adc_gain.connect(ap_data, index);
    index = adc_filter.connect(ap_data, index);
    index = adc_channel.connect(ap_data, index);
    index = adc_cnv_cnt.connect(ap_data, index);
    index = adc_additional_gain.connect(ap_data, index);
    index = adc_ref.connect(ap_data, index);
    index = adc_cont_cnv_cnt.connect(ap_data, index);
    index = adc_impf_iterations_cnt.connect(ap_data, index);
    index = adc_impf_type.connect(ap_data, index);
    index = adc_cont_mode.connect(ap_data, index);
    index = adc_cont_sko.connect(ap_data, index);
    //  ADC free / voltage
    index = adc_free_vx_gain.connect(ap_data, index);
    index = adc_free_vx_filter.connect(ap_data, index);
    index = adc_free_vx_channel.connect(ap_data, index);
    index = adc_free_vx_cnv_cnt.connect(ap_data, index);
    index = adc_free_vx_additional_gain.connect(ap_data, index);
    index = adc_free_vx_ref.connect(ap_data, index);
    index = adc_free_vx_cont_cnv_cnt.connect(ap_data, index);
    index = adc_free_vx_impf_iterations_cnt.connect(ap_data, index);
    index = adc_free_vx_impf_type.connect(ap_data, index);
    index = adc_free_vx_cont_mode.connect(ap_data, index);
    index = adc_free_vx_cont_sko.connect(ap_data, index);
    //  ADC free / temperature
    index = adc_free_th_gain.connect(ap_data, index);
    index = adc_free_th_filter.connect(ap_data, index);
    index = adc_free_th_channel.connect(ap_data, index);
    index = adc_free_th_cnv_cnt.connect(ap_data, index);
    index = adc_free_th_additional_gain.connect(ap_data, index);
    index = adc_free_th_ref.connect(ap_data, index);
    index = adc_free_th_cont_cnv_cnt.connect(ap_data, index);
    index = adc_free_th_impf_iterations_cnt.connect(ap_data, index);
    index = adc_free_th_impf_type.connect(ap_data, index);
    index = adc_free_th_cont_mode.connect(ap_data, index);
    index = adc_free_th_cont_sko.connect(ap_data, index);
    //  ADC manual
    index = adc_manual_gain.connect(ap_data, index);
    index = adc_manual_filter.connect(ap_data, index);
    index = adc_manual_channel.connect(ap_data, index);
    index = adc_manual_cnv_cnt.connect(ap_data, index);
    index = adc_manual_additional_gain.connect(ap_data, index);
    index = adc_manual_ref.connect(ap_data, index);
    index = adc_manual_cont_cnv_cnt.connect(ap_data, index);
    index = adc_manual_impf_iterations_cnt.connect(ap_data, index);
    index = adc_manual_impf_type.connect(ap_data, index);
    index = adc_manual_cont_mode.connect(ap_data, index);
    index = adc_manual_cont_sko.connect(ap_data, index);
    //  ADC balance
    index = adc_balance_gain.connect(ap_data, index);
    index = adc_balance_filter.connect(ap_data, index);
    index = adc_balance_channel.connect(ap_data, index);
    index = adc_balance_cnv_cnt.connect(ap_data, index);
    index = adc_balance_additional_gain.connect(ap_data, index);
    index = adc_balance_ref.connect(ap_data, index);
    index = adc_balance_cont_cnv_cnt.connect(ap_data, index);
    index = adc_balance_impf_iterations_cnt.connect(ap_data, index);
    index = adc_balance_impf_type.connect(ap_data, index);
    index = adc_balance_cont_mode.connect(ap_data, index);
    index = adc_balance_cont_sko.connect(ap_data, index);
    //  ADC elab
    index = adc_elab_gain.connect(ap_data, index);
    index = adc_elab_filter.connect(ap_data, index);
    index = adc_elab_channel.connect(ap_data, index);
    index = adc_elab_cnv_cnt.connect(ap_data, index);
    index = adc_elab_additional_gain.connect(ap_data, index);
    index = adc_elab_ref.connect(ap_data, index);
    index = adc_elab_cont_cnv_cnt.connect(ap_data, index);
    index = adc_elab_impf_iterations_cnt.connect(ap_data, index);
    index = adc_elab_impf_type.connect(ap_data, index);
    index = adc_elab_cont_mode.connect(ap_data, index);
    index = adc_elab_cont_sko.connect(ap_data, index);
    //  Bridge Assimetry Correction
    index = bac_old_coefficient.connect(ap_data, index);
    index = bac_new_coefficient.connect(ap_data, index);
    index = bac_new_int_coefficient.connect(ap_data, index);
    index = bac_new_int_multiplier.connect(ap_data, index);
    //  OnBoard temperature sensors, voltages and fans
    index = th_dac.connect(ap_data, index);
    index = th_box_ldo.connect(ap_data, index);
    index = th_box_adc.connect(ap_data, index);
    index = th_mcu.connect(ap_data, index);
    index = th_ext_1.connect(ap_data, index);
    index = th_ext_2.connect(ap_data, index);
    index = volt_box_neg.connect(ap_data, index);
    index = volt_box_pos.connect(ap_data, index);
    index = fan_mode.connect(ap_data, index);
    index = fan_status.connect(ap_data, index);
    index = fan_ac_speed.connect(ap_data, index);
    index = fan_dc_speed.connect(ap_data, index);
    index = fan_dc_speed_sense.connect(ap_data, index);
    //  Termoregulator
    index = treg_ref.connect(ap_data, index);
    index = treg_result.connect(ap_data, index);
    index = treg_k.connect(ap_data, index);
    index = treg_ki.connect(ap_data, index);
    index = treg_kd.connect(ap_data, index);
    index = treg_iso_k.connect(ap_data, index);
    index = treg_iso_t.connect(ap_data, index);
    index = treg_pwm_rate_slope.connect(ap_data, index);
    index = treg_pid_out.connect(ap_data, index);
    index = treg_amplitude_code_float.connect(ap_data, index);
    treg_enabled.connect(ap_data, index, 0);
    treg_pid_reg_enabled.connect(ap_data, index, 1);
    treg_polarity_pin_bit_data.connect(ap_data, index, 2);
    index = treg_options.connect(ap_data, index);
    
    return index;
  }
};  //  eth_data_t

struct eeprom_data_t {
  irs::conn_data_t<irs_u32> exp_cnt;                  //  4
  irs::conn_data_t<double> etalon;                    //  8
  irs::conn_data_t<double> checked;                   //  8
  irs::conn_data_t<irs_u32> dac_lin;                  //  4 24
  irs::conn_data_t<irs_u32> elab_cnt;                 //  4
  irs::conn_data_t<irs_u32> dac_pause_ms;             //  4
  irs::conn_data_t<irs_u32> relay_pause_ms;           //  4
  irs::conn_data_t<dac_value_t> dac_center_scan;      //  8 20
  irs::conn_data_t<irs_i32> int_dac_center_scan;      //  4
  irs::conn_data_t<irs_u32> prepare_pause;            //  4
  irs::conn_data_t<irs_u32> adc_average_cnt;          //  4
  irs::conn_data_t<adc_value_t> adc_filter_constant;  //  8 20
  irs::conn_data_t<irs_u32> adc_average_skip_cnt;     //  4
  irs::conn_data_t<irs_u32> adc_experiment_gain;      //  4
  irs::conn_data_t<irs_u32> adc_experiment_filter;    //  4
  irs::conn_data_t<adc_value_t> adc_additional_gain;  //  8 20
  irs::conn_data_t<adc_value_t> adc_ref;              //  8
  irs::conn_data_t<dac_value_t> dac_voltage_pos;      //  8
  irs::conn_data_t<dac_value_t> dac_voltage_neg;      //  8
  irs::conn_data_t<irs_u32> options;                  //  4 28  112
  irs::bit_data_t no_prot;
  irs::bit_data_t optimize_balance;
  irs::bit_data_t wild_relays;
  irs::bit_data_t auto_elab_step;
  irs::bit_data_t adc_simply_show;
  irs::bit_data_t use_impf;
  irs::bit_data_t adc_show_points;
  irs::bit_data_t adc_continious;
  irs::conn_data_t<irs_u8> elab_mode;
  irs::conn_data_t<irs_i32> elab_step;                //  4
  irs::conn_data_t<irs_i32> min_elab_cnt;             //  4
  irs::conn_data_t<irs_i32> max_elab_cnt;             //  4
  irs::conn_data_t<irs_u32> dac_elab_pause_ms;        //  4
  irs::conn_data_t<dac_value_t> elab_step_multiplier; //  8
  irs::conn_data_t<dac_value_t> elab_max_delta;       //  8
  irs::conn_data_t<double> ratio;                     //  8
  irs::conn_data_t<double> checked_prev;          //  8 64
  irs::conn_data_t<irs_u8> ip_0;          //  1 byte
  irs::conn_data_t<irs_u8> ip_1;          //  1 byte
  irs::conn_data_t<irs_u8> ip_2;          //  1 byte
  irs::conn_data_t<irs_u8> ip_3;          //  1 byte  //  4
  irs::conn_data_t<irs_u8> mask_0;        //  1 byte
  irs::conn_data_t<irs_u8> mask_1;        //  1 byte
  irs::conn_data_t<irs_u8> mask_2;        //  1 byte
  irs::conn_data_t<irs_u8> mask_3;        //  1 byte  //  4
  irs::conn_data_t<irs_u8> gateway_0;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_1;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_2;     //  1 byte
  irs::conn_data_t<irs_u8> gateway_3;     //  1 byte  //  4
  irs::bit_data_t dhcp_on;                //  1 bit   //  4 16
  irs::conn_data_t<double> termostat_off_pause;       //  8
  irs::conn_data_t<double> elab_pid_kp;               //  8
  irs::conn_data_t<double> elab_pid_ki;               //  8
  irs::conn_data_t<double> elab_pid_kd;               //  8
  irs::conn_data_t<double> elab_iso_k;                //  8
  irs::conn_data_t<double> elab_iso_t;                //  8
  irs::conn_data_t<double> elab_pid_fade_t;           //  8
  irs::conn_data_t<irs_u32> elab_pid_avg_cnt;         //  4
  irs::conn_data_t<double> elab_pid_target_sko_norm;  //  8
  irs::conn_data_t<double> elab_pid_ref;              //  8
  irs::conn_data_t<irs_u32> impf_iterations_cnt;      //  4 80  272
  //  ADC free / voltage
  irs::conn_data_t<irs_u8> adc_free_vx_gain;          //  1
  irs::conn_data_t<irs_u8> adc_free_vx_filter;        //  1
  irs::conn_data_t<irs_u8> adc_free_vx_channel;       //  1
  irs::conn_data_t<irs_u32> adc_free_vx_cnv_cnt;      //  4
  irs::conn_data_t<double> adc_free_vx_additional_gain;       //  8
  irs::conn_data_t<double> adc_free_vx_ref;           //  8
  irs::conn_data_t<irs_u32> adc_free_vx_cont_cnv_cnt; //  4
  irs::conn_data_t<irs_u32> adc_free_vx_impf_iterations_cnt;  //  4
  irs::conn_data_t<irs_u8> adc_free_vx_impf_type;     //  1
  irs::conn_data_t<irs_u8> adc_free_vx_cont_mode;     //  1
  irs::conn_data_t<double> adc_free_vx_cont_sko;      //  8
  //  ADC free / temperature
  irs::conn_data_t<irs_u8> adc_free_th_gain;          //  1
  irs::conn_data_t<irs_u8> adc_free_th_filter;        //  1
  irs::conn_data_t<irs_u8> adc_free_th_channel;       //  1
  irs::conn_data_t<irs_u32> adc_free_th_cnv_cnt;      //  4
  irs::conn_data_t<double> adc_free_th_additional_gain;       //  8
  irs::conn_data_t<double> adc_free_th_ref;           //  8
  irs::conn_data_t<irs_u32> adc_free_th_cont_cnv_cnt; //  4
  irs::conn_data_t<irs_u32> adc_free_th_impf_iterations_cnt;  //  4
  irs::conn_data_t<irs_u8> adc_free_th_impf_type;     //  1
  irs::conn_data_t<irs_u8> adc_free_th_cont_mode;     //  1
  irs::conn_data_t<double> adc_free_th_cont_sko;      //  8
  //  ADC manual
  irs::conn_data_t<irs_u8> adc_manual_gain;           //  1
  irs::conn_data_t<irs_u8> adc_manual_filter;         //  1
  irs::conn_data_t<irs_u8> adc_manual_channel;        //  1
  irs::conn_data_t<irs_u32> adc_manual_cnv_cnt;       //  4
  irs::conn_data_t<double> adc_manual_additional_gain;        //  8
  irs::conn_data_t<double> adc_manual_ref;            //  8
  irs::conn_data_t<irs_u32> adc_manual_cont_cnv_cnt;  //  4
  irs::conn_data_t<irs_u32> adc_manual_impf_iterations_cnt;   //  4
  irs::conn_data_t<irs_u8> adc_manual_impf_type;      //  1
  irs::conn_data_t<irs_u8> adc_manual_cont_mode;      //  1
  irs::conn_data_t<double> adc_manual_cont_sko;       //  8
  //  ADC balance
  irs::conn_data_t<irs_u8> adc_balance_gain;          //  1
  irs::conn_data_t<irs_u8> adc_balance_filter;        //  1
  irs::conn_data_t<irs_u8> adc_balance_channel;       //  1
  irs::conn_data_t<irs_u32> adc_balance_cnv_cnt;      //  4
  irs::conn_data_t<double> adc_balance_additional_gain;       //  8
  irs::conn_data_t<double> adc_balance_ref;           //  8
  irs::conn_data_t<irs_u32> adc_balance_cont_cnv_cnt; //  4
  irs::conn_data_t<irs_u32> adc_balance_impf_iterations_cnt;  //  4
  irs::conn_data_t<irs_u8> adc_balance_impf_type;     //  1
  irs::conn_data_t<irs_u8> adc_balance_cont_mode;     //  1
  irs::conn_data_t<double> adc_balance_cont_sko;      //  8
  //  ADC elab
  irs::conn_data_t<irs_u8> adc_elab_gain;             //  1
  irs::conn_data_t<irs_u8> adc_elab_filter;           //  1
  irs::conn_data_t<irs_u8> adc_elab_channel;          //  1
  irs::conn_data_t<irs_u32> adc_elab_cnv_cnt;         //  4
  irs::conn_data_t<double> adc_elab_additional_gain;          //  8
  irs::conn_data_t<double> adc_elab_ref;              //  8
  irs::conn_data_t<irs_u32> adc_elab_cont_cnv_cnt;    //  4
  irs::conn_data_t<irs_u32> adc_elab_impf_iterations_cnt;     //  4
  irs::conn_data_t<irs_u8> adc_elab_impf_type;        //  1
  irs::conn_data_t<irs_u8> adc_elab_cont_mode;        //  1
  irs::conn_data_t<double> adc_elab_cont_sko;         //  8
  //
  irs::conn_data_t<double> adc_max_value_prot;        //  8
  irs::conn_data_t<double> adc_max_value_no_prot;     //  8
  
  irs::conn_data_t<double> bac_old_coefficient;       //  8
  irs::conn_data_t<double> bac_new_coefficient;       //  8
  irs::conn_data_t<irs_i32> bac_new_int_coefficient;  //  4
  irs::conn_data_t<double> bac_new_int_multiplier;    //  8
  //
  irs::conn_data_t<irs_u8> fan_mode;                  //  1
  irs::conn_data_t<irs_u8> fan_ac_speed;              //  1
  irs::conn_data_t<irs_u8> fan_dc_speed;              //  1
  //
  irs::conn_data_t<th_value_t> treg_ref;              //  8
  irs::conn_data_t<th_value_t> treg_k;                //  8
  irs::conn_data_t<th_value_t> treg_ki;               //  8
  irs::conn_data_t<th_value_t> treg_kd;               //  8
  irs::conn_data_t<th_value_t> treg_iso_k;            //  8
  irs::conn_data_t<th_value_t> treg_iso_t;            //  8
  irs::conn_data_t<th_value_t> treg_pwm_rate_slope;   //  8
  irs::conn_data_t<irs_u32> treg_options;             //  4
  irs::bit_data_as_bool_t treg_enabled;
  irs::bit_data_as_bool_t treg_pid_reg_enabled;
  irs::bit_data_as_bool_t treg_polarity_pin_bit_data;
  
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
    use_impf.connect(ap_data, index, 5);
    adc_show_points.connect(ap_data, index, 6);
    adc_continious.connect(ap_data, index, 7);
    elab_mode.connect(ap_data, index + 1);
    index = options.connect(ap_data, index);
    index = elab_step.connect(ap_data, index);
    index = min_elab_cnt.connect(ap_data, index);
    index = max_elab_cnt.connect(ap_data, index);
    index = dac_elab_pause_ms.connect(ap_data, index);
    index = elab_step_multiplier.connect(ap_data, index);
    index = elab_max_delta.connect(ap_data, index);
    index = ratio.connect(ap_data, index);
    index = checked_prev.connect(ap_data, index);
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
    index = elab_iso_k.connect(ap_data, index);
    index = elab_iso_t.connect(ap_data, index);
    index = elab_pid_fade_t.connect(ap_data, index);
    index = elab_pid_avg_cnt.connect(ap_data, index);
    index = elab_pid_target_sko_norm.connect(ap_data, index);
    index = elab_pid_ref.connect(ap_data, index);
    index = impf_iterations_cnt.connect(ap_data, index);
    //  ADC free / voltage
    index = adc_free_vx_gain.connect(ap_data, index);
    index = adc_free_vx_filter.connect(ap_data, index);
    index = adc_free_vx_channel.connect(ap_data, index);
    index = adc_free_vx_cnv_cnt.connect(ap_data, index);
    index = adc_free_vx_additional_gain.connect(ap_data, index);
    index = adc_free_vx_ref.connect(ap_data, index);
    index = adc_free_vx_cont_cnv_cnt.connect(ap_data, index);
    index = adc_free_vx_impf_iterations_cnt.connect(ap_data, index);
    index = adc_free_vx_impf_type.connect(ap_data, index);
    index = adc_free_vx_cont_mode.connect(ap_data, index);
    index = adc_free_vx_cont_sko.connect(ap_data, index);
    //  ADC free / temperature
    index = adc_free_th_gain.connect(ap_data, index);
    index = adc_free_th_filter.connect(ap_data, index);
    index = adc_free_th_channel.connect(ap_data, index);
    index = adc_free_th_cnv_cnt.connect(ap_data, index);
    index = adc_free_th_additional_gain.connect(ap_data, index);
    index = adc_free_th_ref.connect(ap_data, index);
    index = adc_free_th_cont_cnv_cnt.connect(ap_data, index);
    index = adc_free_th_impf_iterations_cnt.connect(ap_data, index);
    index = adc_free_th_impf_type.connect(ap_data, index);
    index = adc_free_th_cont_mode.connect(ap_data, index);
    index = adc_free_th_cont_sko.connect(ap_data, index);
    //  ADC manual
    index = adc_manual_gain.connect(ap_data, index);
    index = adc_manual_filter.connect(ap_data, index);
    index = adc_manual_channel.connect(ap_data, index);
    index = adc_manual_cnv_cnt.connect(ap_data, index);
    index = adc_manual_additional_gain.connect(ap_data, index);
    index = adc_manual_ref.connect(ap_data, index);
    index = adc_manual_cont_cnv_cnt.connect(ap_data, index);
    index = adc_manual_impf_iterations_cnt.connect(ap_data, index);
    index = adc_manual_impf_type.connect(ap_data, index);
    index = adc_manual_cont_mode.connect(ap_data, index);
    index = adc_manual_cont_sko.connect(ap_data, index);
    //  ADC balance
    index = adc_balance_gain.connect(ap_data, index);
    index = adc_balance_filter.connect(ap_data, index);
    index = adc_balance_channel.connect(ap_data, index);
    index = adc_balance_cnv_cnt.connect(ap_data, index);
    index = adc_balance_additional_gain.connect(ap_data, index);
    index = adc_balance_ref.connect(ap_data, index);
    index = adc_balance_cont_cnv_cnt.connect(ap_data, index);
    index = adc_balance_impf_iterations_cnt.connect(ap_data, index);
    index = adc_balance_impf_type.connect(ap_data, index);
    index = adc_balance_cont_mode.connect(ap_data, index);
    index = adc_balance_cont_sko.connect(ap_data, index);
    //  ADC elab
    index = adc_elab_gain.connect(ap_data, index);
    index = adc_elab_filter.connect(ap_data, index);
    index = adc_elab_channel.connect(ap_data, index);
    index = adc_elab_cnv_cnt.connect(ap_data, index);
    index = adc_elab_additional_gain.connect(ap_data, index);
    index = adc_elab_ref.connect(ap_data, index);
    index = adc_elab_cont_cnv_cnt.connect(ap_data, index);
    index = adc_elab_impf_iterations_cnt.connect(ap_data, index);
    index = adc_elab_impf_type.connect(ap_data, index);
    index = adc_elab_cont_mode.connect(ap_data, index);
    index = adc_elab_cont_sko.connect(ap_data, index);
    //
    index = adc_max_value_prot.connect(ap_data, index);
    index = adc_max_value_no_prot.connect(ap_data, index);
    //
    index = bac_old_coefficient.connect(ap_data, index);
    index = bac_new_coefficient.connect(ap_data, index);
    index = bac_new_int_coefficient.connect(ap_data, index);
    index = bac_new_int_multiplier.connect(ap_data, index);
    
    index = fan_mode.connect(ap_data, index);
    index = fan_ac_speed.connect(ap_data, index);
    index = fan_dc_speed.connect(ap_data, index);
    index++;
    //  Termoregulator
    index = treg_ref.connect(ap_data, index);
    index = treg_k.connect(ap_data, index);
    index = treg_ki.connect(ap_data, index);
    index = treg_kd.connect(ap_data, index);
    index = treg_iso_k.connect(ap_data, index);
    index = treg_iso_t.connect(ap_data, index);
    index = treg_pwm_rate_slope.connect(ap_data, index);
    treg_enabled.connect(ap_data, index, 0);
    treg_pid_reg_enabled.connect(ap_data, index, 1);
    treg_polarity_pin_bit_data.connect(ap_data, index, 2);
    index = treg_options.connect(ap_data, index);
    return index;
  }
  inline void reset_to_default()
  {
    irs::mlog() << irsm("EEPROM reset to default") << endl;
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
    adc_additional_gain = 1.0;
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
    adc_show_points = 0;
    adc_continious = 0;
    elab_max_delta = 1.0;
    ratio = 1.0;
    checked_prev = 1.0;
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
    elab_iso_k = 1.0;
    elab_iso_t = 0.0;
    elab_mode = 0;
    elab_pid_fade_t = 0.0;
    elab_pid_avg_cnt = 10;
    elab_pid_target_sko_norm = 1.0 / pow(2.0, 20);
    elab_pid_ref = 0.0;
    impf_iterations_cnt = 10;
    //  ADC free / voltage
    adc_free_vx_gain = adc_default_gain;
    adc_free_vx_filter = adc_default_filter;
    adc_free_vx_channel = adc_default_channel_voltage;
    adc_free_vx_cnv_cnt = adc_default_cnv_cnt_voltage;
    adc_free_vx_additional_gain = adc_additional_gain;
    adc_free_vx_ref = adc_ref;
    adc_free_vx_cont_cnv_cnt = adc_default_cont_cnv_cnt;
    adc_free_vx_impf_iterations_cnt = adc_default_impf_iterations_cnt;
    adc_free_vx_impf_type = adc_default_impf_type;
    adc_free_vx_cont_mode = adc_default_cont_mode;
    adc_free_vx_cont_sko = adc_default_cont_sko;
    //  ADC free / temperature
    adc_free_th_gain = adc_default_gain;
    adc_free_th_filter = adc_default_filter;
    adc_free_th_channel = adc_default_channel_temperature;
    adc_free_th_cnv_cnt = adc_default_cnv_cnt_temperature;
    adc_free_th_additional_gain = adc_additional_gain;
    adc_free_th_ref = adc_ref;
    adc_free_th_cont_cnv_cnt = adc_default_cont_cnv_cnt;
    adc_free_th_impf_iterations_cnt = adc_default_impf_iterations_cnt;
    adc_free_th_impf_type = adc_default_impf_type;
    adc_free_th_cont_mode = adc_default_cont_mode;
    adc_free_th_cont_sko = adc_default_cont_sko;
    //  ADC manual
    adc_manual_gain = adc_default_gain;
    adc_manual_filter = adc_default_filter;
    adc_manual_channel = adc_default_channel_voltage;
    adc_manual_cnv_cnt = adc_default_cnv_cnt_voltage;
    adc_manual_additional_gain = adc_additional_gain;
    adc_manual_ref = adc_ref;
    adc_manual_cont_cnv_cnt = adc_default_cont_cnv_cnt;
    adc_manual_impf_iterations_cnt = adc_default_impf_iterations_cnt;
    adc_manual_impf_type = adc_default_impf_type;
    adc_manual_cont_mode = adc_default_cont_mode;
    adc_manual_cont_sko = adc_default_cont_sko;
    //  ADC balance
    adc_balance_gain = adc_default_gain;
    adc_balance_filter = adc_default_filter;
    adc_balance_channel = adc_default_channel_voltage;
    adc_balance_cnv_cnt = adc_default_cnv_cnt_voltage;
    adc_balance_additional_gain = adc_additional_gain;
    adc_balance_ref = adc_ref;
    adc_balance_cont_cnv_cnt = adc_default_cont_cnv_cnt;
    adc_balance_impf_iterations_cnt = adc_default_impf_iterations_cnt;
    adc_balance_impf_type = adc_default_impf_type;
    adc_balance_cont_mode = adc_default_cont_mode;
    adc_balance_cont_sko = adc_default_cont_sko;
    //  ADC elab
    adc_elab_gain = adc_default_gain;
    adc_elab_filter = adc_default_filter;
    adc_elab_channel = adc_default_channel_voltage;
    adc_elab_cnv_cnt = adc_default_cnv_cnt_voltage;
    adc_elab_additional_gain = adc_additional_gain;
    adc_elab_ref = adc_ref;
    adc_elab_cont_cnv_cnt = adc_default_cont_cnv_cnt;
    adc_elab_impf_iterations_cnt = adc_default_impf_iterations_cnt;
    adc_elab_impf_type = adc_default_impf_type;
    adc_elab_cont_mode = adc_default_cont_mode;
    adc_elab_cont_sko = adc_default_cont_sko;
    //
    adc_max_value_prot = 0.4;
    adc_max_value_no_prot = 2.0;
    //
    bac_old_coefficient = 0.0;
    bac_new_coefficient = 1.0;
    bac_new_int_coefficient = 1;
    bac_new_int_multiplier = 1000.0;
    //
    fan_mode = fan_already_off;
    //
    treg_ref = 20.0;
    treg_k = 0.1;
    treg_ki = 0.1;
    treg_kd = 0.0;
    treg_iso_k = 1.0;
    treg_iso_t = 0.0;
    treg_pwm_rate_slope = 0.4;
    treg_enabled = 1;
    treg_pid_reg_enabled = 1;
    treg_polarity_pin_bit_data = 1;
  }
};

} // namespace hrm

#endif // datah
