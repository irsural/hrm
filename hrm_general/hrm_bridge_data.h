#ifndef datah
#define datah

#include <irsdefs.h>

#include <mxdata.h>

#include <irsfinal.h>

namespace hrm {

struct eth_data_t {
  irs::conn_data_t<irs_u32> counter;      //  4 byte
  irs::conn_data_t<irs_u32> command;      //  4 byte
  irs::conn_data_t<irs_u32> status;       //  4 byte
  irs::conn_data_t<float> etalon;         //  4 byte
  irs::conn_data_t<float> checked;        //  4 byte
  irs::conn_data_t<float> result;         //  4 byte
  //------------------------------------------
  //  = 6 * 4 = 24 bytes

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
    index = command.connect(ap_data, index);
    index = status.connect(ap_data, index);
    index = etalon.connect(ap_data, index);
    index = checked.connect(ap_data, index);
    index = result.connect(ap_data, index);

    return index;
  }
};  //  eth_data_t

} // namespace hrm

#endif // datah
