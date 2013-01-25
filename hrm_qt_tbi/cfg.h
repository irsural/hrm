#ifndef CFG_H
#define CFG_H

#include <irsdefs.h>

#include <hardflowg.h>
#include <mxdata.h>
#include <niusbgpib_hardflow.h>

#include <irsfinal.h>

namespace hrm {

class cfg_t {
public:
  cfg_t();
  irs::hardflow_t* get_eth_hardflow();
  irs::hardflow_t* get_gpib_hardflow();
private:
  irs::handle_t<irs::hardflow_t> mp_eth_hardflow;
  irs::hardflow::ni_usb_gpib_flow_t m_gpib_hardflow;
};

} //  namespace hrm

#endif // CFG_H
