#ifndef datah
#define datah

#include <irsdefs.h>

#include <mxdata.h>

#include <irsfinal.h>

namespace hrm {

enum mode_t {
  md_free = 0,
  md_manual = 1,
  md_balance = 2
};
  
struct eth_data_t {
  irs::conn_data_t<irs_u32> counter;      //  4 byte
  //  command
  irs::conn_data_t<irs_u8> mode;
  irs::conn_data_t<irs_u8> range;
  irs::bit_data_t etpol;
  irs::bit_data_t apply;
  //  status
  irs::conn_data_t<irs_u8> current_mode;  //  1 byte
  irs::conn_data_t<irs_u8> current_range; //  1 byte
  irs::bit_data_t current_etpol;
  //  values
  irs::conn_data_t<double> etalon;        //  8 byte
  irs::conn_data_t<double> checked;       //  8 byte
  irs::conn_data_t<double> result;        //  8 byte
  irs::conn_data_t<double> result_error;  //  8 byte
  irs::conn_data_t<irs_u32> adc;          //  4 byte
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
    index++;
    etpol.connect(ap_data, index, 0);
    apply.connect(ap_data, index, 7);
    index++;
    
    index = current_mode.connect(ap_data, index);
    index = current_range.connect(ap_data, index);
    index++;
    current_etpol.connect(ap_data, index, 0);
    index++;
    
    index = etalon.connect(ap_data, index);
    index = checked.connect(ap_data, index);
    index = result.connect(ap_data, index);
    index = result_error.connect(ap_data, index);
    index = adc.connect(ap_data, index);

    return index;
  }
};  //  eth_data_t

} // namespace hrm

#endif // datah
