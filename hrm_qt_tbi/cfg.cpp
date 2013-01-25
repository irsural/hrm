#include "cfg.h"

hrm::cfg_t::cfg_t():
  mp_eth_hardflow(irs::hardflow::make_udp_flow_client(irst("192.168.0.212"),
    irst("5006"))),
  m_gpib_hardflow(0, 22)
{
}

irs::hardflow_t* hrm::cfg_t::get_eth_hardflow()
{
  return mp_eth_hardflow.get();
}

irs::hardflow_t* hrm::cfg_t::get_gpib_hardflow()
{
  return &m_gpib_hardflow;
}
