#ifndef datah
#define datah

#include <irsdefs.h>

#include <mxdata.h>

#include <irsfinal.h>

namespace hrm {

enum {
  m_read_bits_byte_count = 0,
  m_rw_bits_byte_count = 0,
  m_rw_regs_count = 10,
  m_read_regs_count = 0
};

struct eth_data_t {
  irs::conn_data_t<float> internal_temp;  //  4 byte
  irs::conn_data_t<irs_u32> counter;      //  4 byte
  irs::conn_data_t<irs_u32> options;      //  4 byte
  irs::bit_data_t led_1;
  irs::bit_data_t led_2;
  irs::bit_data_t led_3;
  irs::bit_data_t led_4;
  irs::bit_data_t led_5;
  irs::bit_data_t led_6;
  irs::bit_data_t r2r_busy;
  irs::bit_data_t r2r_comparator;
  irs::bit_data_t zero_relay;
  irs::conn_data_t<irs_u32> tbi_0; //  4 byte
  irs::conn_data_t<irs_u32> tbi_1; //  4 byte
  //------------------------------------------
  //  = 5 * 4 = 20 bytes

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

    index = internal_temp.connect(ap_data, index);
    index = counter.connect(ap_data, index);

    led_1.connect(ap_data, index, 1);
    led_2.connect(ap_data, index, 2);
    led_3.connect(ap_data, index, 3);
    led_4.connect(ap_data, index, 4);
    led_5.connect(ap_data, index, 5);
    led_6.connect(ap_data, index, 6);

    r2r_busy.connect(ap_data, index + 1, 0);
    r2r_comparator.connect(ap_data, index + 1, 1);
    zero_relay.connect(ap_data, index + 1, 2);

    index = options.connect(ap_data, index);
    index = tbi_0.connect(ap_data, index);
    index = tbi_1.connect(ap_data, index);

    return index;
  }
};  //  eth_data_t

} // namespace hrm

#endif // datah
