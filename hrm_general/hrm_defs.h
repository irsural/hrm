#ifndef HRM_DEFS_H
#define HRM_DEFS_H

#include <irsdefs.h>
#include <irsstring.h>
#include <irsfinal.h>

namespace hrm {

#define ntostr(x) (irs::irsstr_from_number_russian(irs::char_t(), x))

typedef irs_u32 code_t;

enum {
  r2r_resolution = 24,
  r2r_max_code = 16777215
};

enum balance_mode_t {
  bm_multimeter,
  bm_comparator
};

enum direction_t {
  di_up,
  di_down
};

enum status_t {
  st_free,
  st_balance,
  st_calibre,
  st_idle,
  st_balance_emulate,
  st_buffer_test,
  st_r2r_scan,
  st_r2r_scan_emulate,
  st_one_meas
};

template <class T>
T convert_code(vector<T>* ap_weight_vector, code_t a_in_code);

} //  hrm

template <class T>
T hrm::convert_code(vector<T>* ap_weight_vector, code_t a_in_code)
{
  T out_code = 0;
  for (size_t i = 0; i < ap_weight_vector->size(); i++) {
    size_t inv_i = ap_weight_vector->size() - i - 1;
    T weidht = pow(2., i) / (*ap_weight_vector)[inv_i];
    if (a_in_code & (1 << i)) {
      out_code += weidht;
    }
  }
  return out_code;
}

#endif // HRM_DEFS_H
