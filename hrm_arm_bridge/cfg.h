#ifndef cfgh
#define cfgh

#include <irsdefs.h>

#include <armcfg.h>
#include <armspi.h>
#include <armgpio.h>
#include <armeth.h>
#include <armadc.h>
#include <irskbd.h>
#include <irserror.h>
#include <irsdev.h>
#include <mxdata.h>

#include <irsconfig.h>
//#include <irstcpip.h>
#include <irslwip.h>
#include <hardflowg.h>

#include "privatecfg.h"

#include <irsfinal.h>

namespace hrm {

//#define HRM_DEBUG

#ifdef HRM_DEBUG
#define HRM_ASSERT(assert_expr) IRS_ASSERT(assert_expr)
#define HRM_ASSERT_MSG(msg) IRS_ASSERT_MSG(msg)
#define HRM_NEW_ASSERT(new_expr, file_idx)\
  IRS_NEW_ASSERT(new_expr, file_idx)
#define HRM_ERROR(error_code, msg) IRS_ERROR(error_code, msg)
#define HRM_ERROR_IF_NOT(assert_expr, error_code, msg)\
  IRS_ERROR_IF_NOT(assert_expr, error_code, msg)
#define HRM_DBG_RAW_MSG(msg) IRS_DBG_RAW_MSG(msg)
#define HRM_DBG_MSG(msg) IRS_LIB_DBG_MSG(msg)
#else //HRM_DEBUG
#define HRM_ASSERT(assert_expr)
#define HRM_ASSERT_MSG(msg)
#define HRM_NEW_ASSERT(new_expr, file_idx) (new new_expr)
#define HRM_ERROR(error_code, msg)
#define HRM_ERROR_IF_NOT(assert_expr, error_code, msg)
#define HRM_DBG_RAW_MSG(msg)
#define HRM_DBG_MSG(msg)
#endif //HRM_DEBUG

#define HRM_DELETE_ASSERT(expr) IRS_DELETE_ASSERT(expr)

class adc_exti_t
{
public:
//  inline adc_exti_t()
//  {
//    irs::clock_enable(IRS_PORTF_BASE);
//    irs::gpio_moder_input_enable(PF1);
//    SETENA0_bit.SETENA_EXTI1 = 1;     //  exti1 ï¿½ï¿½ï¿½ï¿½ï¿½ 1
//    SYSCFG_EXTICR1_bit.EXTI1 = 5;     //  PORT F
//    EXTI_IMR_bit.MR1 = 0;   // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ 1 ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//    EXTI_FTSR_bit.TR1 = 1;  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½
//  }
//  inline ~adc_exti_t()
//  {
//    SETENA0_bit.SETENA_EXTI1 = 0;
//  };
//  inline void add_event(mxfact_event_t *ap_event)
//  {
//    irs::interrupt_array()->int_event_gen(irs::arm::exti1_int)->add(ap_event);
//  }
//  inline void start()   { EXTI_IMR_bit.MR1 = 1; }
//  inline void stop()    { EXTI_IMR_bit.MR1 = 0; }
//  inline bool stopped() { return EXTI_IMR_bit.MR1; }
  inline adc_exti_t()
  {
    irs::clock_enable(IRS_PORTF_BASE);
    irs::gpio_moder_input_enable(PF0);
    SETENA0_bit.SETENA_EXTI0 = 1;     //  exti1 ï¿½ï¿½ï¿½ï¿½ï¿½ 0
    SYSCFG_EXTICR1_bit.EXTI0 = 5;     //  PORT F
    EXTI_IMR_bit.MR0 = 0;   // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ 0 ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
    EXTI_FTSR_bit.TR0 = 1;  // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½
  }
  inline ~adc_exti_t()
  {
    SETENA0_bit.SETENA_EXTI0 = 0;
  };
  inline void add_event(mxfact_event_t *ap_event)
  {
    irs::interrupt_array()->int_event_gen(irs::arm::exti0_int)->add(ap_event);
  }
  inline void start()   { EXTI_IMR_bit.MR0 = 1; }
  inline void stop()    { EXTI_IMR_bit.MR0 = 0; }
  inline bool stopped() { return EXTI_IMR_bit.MR0; }
};

class pulse_gen_t
{
public:
  inline pulse_gen_t()
  {
    //  TIM1 - master of master, make interval
    //  TIM2 - master timer, make N of pulses
    //  TIM3 - slave timer, make cnv pulses
    irs::clock_enable(IRS_PORTB_BASE);
    irs::gpio_moder_alternate_function_enable(PB0);
    irs::gpio_alternate_function_select(PB0, GPIO_AF_TIM3);
    irs::gpio_ospeedr_select(PB0, IRS_GPIO_SPEED_100MHZ);
    irs::clock_enable(IRS_TIM3_BASE);
    TIM3_CR1 = 0;
    TIM3_ARR = 37;  //  2 MHz = 2xFapb1 / (37+1) = 75 / (37+1) = 1,97 MHz
    TIM3_CCR3 = 37;  //  TIM3 Ch3 Output Compare - for one pulse
    TIM3_CCMR2_bit.OC3M = 7;  //  PWM2 Mode, ch 1 is active TIMx_CNT>=TIMx_CCR3
    TIM3_CR1_bit.DIR = 0; //  Upcounting
    TIM3_CCER_bit.CC3E = 1; //  Signal is output on the corresponding output pin
    TIM3_SMCR = 0;  //  Slave Mode Register -> Clk Src = CK_INT
    TIM3_SMCR_bit.TS = 1;   //  (TIM2_TRGO)
    TIM3_SMCR_bit.SMS = 5;  //  Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high
    TIM3_CR1_bit.CEN = 1;
    //  Test output for TIM2
    irs::clock_enable(IRS_PORTA_BASE);
    irs::gpio_moder_alternate_function_enable(PA15);
    irs::gpio_alternate_function_select(PA15, GPIO_AF_TIM2);
    irs::gpio_ospeedr_select(PA15, IRS_GPIO_SPEED_100MHZ);
    //
    irs::clock_enable(IRS_TIM2_BASE);
    TIM2_CCR1 = 1;
    TIM2_SMCR_bit.MSM = 1;
    TIM2_CCMR1_bit.OC1M = 7;//  PWM2 Mode, ch 1 is active TIMx_CNT>=TIMx_CCR1
    TIM2_CCER_bit.CC1E = 1; //  Signal is output on the corresponding output pin
    TIM2_CR1_bit.OPM = 1;   //  One Pulse Mode
    TIM2_CR2_bit.MMS = 1;   //  CNT_EN is used as trigger output (TRGO)
    TIM2_CR1_bit.URS = 1;   //  Only counter overflow/underflow generates an update interrupt
    TIM2_SMCR_bit.TS = 0;   //  TIM1_TRGO
    //
    irs::clock_enable(IRS_TIM1_PWM1_BASE);
    TIM1_CR1 = 0;
    TIM1_CR2 = 0;
    TIM1_CR2_bit.MMS = 2; //  Update event is selected as trigger output (TRGO)
    TIM1_PSC = 1500-1; //  Prescaler 150 MHz / 1500 = 100 kHz; Resolution = 10 us
    TIM1_ARR = 2500-1; //  25 ms
    //
    NVIC_EnableIRQ(TIM2_IRQn);
  }
  inline ~pulse_gen_t()
  {
    ;
  }
  void add_event(mxfact_event_t *ap_event)
  {
    irs::interrupt_array()->int_event_gen(irs::arm::tim2_int)->add(ap_event);
  }
  void set_n(irs_u32 a_n)
  {
    TIM2_ARR = (1 << a_n) * (TIM3_ARR + 1);
  }
  void set_t(irs_u16 a_t)
  {
    TIM1_ARR = a_t - 1; //  * 10us
  }
  void start_single()
  {
    TIM3_CNT = 0;
    TIM2_CNT = 0;
    TIM1_CNT = 0;
    TIM2_SMCR_bit.SMS = 0;  //  Slave mode disabled - if CEN = ‘1 then the prescaler is clocked directly by the internal clock
    TIM2_DIER_bit.UIE = 1;  //  Update interrupt enabled
    TIM2_CR1_bit.CEN = 1;
  }
  void start_continious(irs_u16 a_t)
  {
    TIM3_CNT = 0;
    TIM2_CNT = 0;
    TIM1_CNT = 0;
    TIM1_ARR = a_t - 1; //  * 10us
    TIM2_SMCR_bit.SMS = 6;  //  Trigger Mode - The counter starts at a rising edge of the trigger TRGI
    TIM2_DIER_bit.UIE = 1;  //  Update interrupt enabled
    TIM2_CR1_bit.CEN = 1;
    TIM1_CR1_bit.CEN = 1;
  }
  bool ready()
  {
    return TIM2_SR_bit.UIF;
  }
  void stop()
  {
    TIM1_CR1_bit.CEN = 0;
    TIM2_DIER_bit.UIE = 0;  //  Update interrupt disabled
    TIM2_SR_bit.UIF = 0;  //  Update interrupt flag
  }
  void clear_uif()
  {
    TIM3_CNT = 0;
    TIM2_CNT = 0;
    TIM2_SR_bit.UIF = 0;  //  Update interrupt flag
  }
};

class adc_ad4630_ready_t
{
public:
  inline adc_ad4630_ready_t()
  {
    //  Interrupt on negative edge PF4 (BUSY Signal of AD4630), PF4 = EXTI4
    irs::clock_enable(IRS_PORTF_BASE);
    irs::gpio_moder_input_enable(PF4);
    SYSCFG_EXTICR2_bit.EXTI4 = 5; //  PORT F
    EXTI_IMR_bit.MR4 = 0;         //  EXTI masked
    EXTI_FTSR_bit.TR4 = 1;        //  Falling Edge Enabled
    NVIC_EnableIRQ(EXTI4_IRQn);   //  Vector enabled in NVIC
  }
  inline ~adc_ad4630_ready_t()  
  { 
    EXTI_IMR_bit.MR4 = 0;         //  EXTI masked
  };
  inline void add_event(mxfact_event_t *ap_event)
  {
    irs::interrupt_array()->int_event_gen(irs::arm::exti4_int)->add(ap_event);
  }
  inline void start()   { EXTI_IMR_bit.MR4 = 1; }
  inline void stop()    { EXTI_IMR_bit.MR4 = 0; }
  inline bool stopped() { return EXTI_IMR_bit.MR4; }
};

class spi_dma_reader_t
{
public:
  spi_dma_reader_t()
  {
    //  DMA for SPI1, SPI1, we mean, is already used and configured
    irs::clock_enable(IRS_DMA2_BASE);
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);   //  Vector enabled in NVIC
    //  RX
    DMA2_S0CR = 0;  //  Stream0, RX
    DMA2_S0CR_bit.CHSEL = 3;  //  Channel 3
    DMA2_S0CR_bit.MBURST = 0; //  Memory burst transfer configuration: Single transfer
    DMA2_S0CR_bit.PBURST = 0; //  Peripheral burst transfer configuration: Single transfer
    DMA2_S0CR_bit.CT = 0;     //  Current target (only in double buffer mode)
    DMA2_S0CR_bit.DBM = 0;    //  Double buffer mode: No buffer switching at the end of transfer
    DMA2_S0CR_bit.PL = 0;     //  Priority level Low
    DMA2_S0CR_bit.PINCOS = 0; //  The offset size for the peripheral address calculation is linked to the PSIZE
    DMA2_S0CR_bit.MSIZE = 0;  //  Memory data size is 8 bit (byte)
    DMA2_S0CR_bit.PSIZE = 0;  //  Peripheral data size is 8 bit (size of SPI DR)
    DMA2_S0CR_bit.MINC = 1;   //  Memory address pointer is incremented after each data transfer
    DMA2_S0CR_bit.PINC = 0;   //  Peripheral address pointer is fixed
    DMA2_S0CR_bit.CIRC = 0;   //  Circular mode disabled
    DMA2_S0CR_bit.DIR = 0;    //  Data transfer direction Peripheral-to-memory
    DMA2_S0CR_bit.PFCTRL = 0; //  Peripheral flow controller: The DMA is the flow controller
    DMA2_S0PAR = reinterpret_cast<irs_u32>(&SPI1_DR);  //  Peripheral address
    DMA2_S0FCR_bit.DMDIS = 0; //  Direct mode, no FIFO
    //  TX
    DMA2_S3CR = 0;  //  Stream0, RX
    DMA2_S3CR_bit.CHSEL = 3;  //  Channel 3
    DMA2_S3CR_bit.MBURST = 0; //  Memory burst transfer configuration: Single transfer
    DMA2_S3CR_bit.PBURST = 0; //  Peripheral burst transfer configuration: Single transfer
    DMA2_S3CR_bit.CT = 0;     //  Current target (only in double buffer mode)
    DMA2_S3CR_bit.DBM = 0;    //  Double buffer mode: No buffer switching at the end of transfer
    DMA2_S3CR_bit.PL = 0;     //  Priority level Low
    DMA2_S3CR_bit.PINCOS = 0; //  The offset size for the peripheral address calculation is linked to the PSIZE
    DMA2_S3CR_bit.MSIZE = 0;  //  Memory data size is 8 bit (byte)
    DMA2_S3CR_bit.PSIZE = 0;  //  Peripheral data size is 8 bit (size of SPI DR)
    DMA2_S3CR_bit.MINC = 1;   //  Memory address pointer is incremented after each data transfer
    DMA2_S3CR_bit.PINC = 0;   //  Peripheral address pointer is fixed
    DMA2_S3CR_bit.CIRC = 0;   //  Circular mode disabled
    DMA2_S3CR_bit.DIR = 1;    //  Data transfer direction Memory-to-peripheral
    DMA2_S3CR_bit.PFCTRL = 0; //  Peripheral flow controller: The DMA is the flow controller
    DMA2_S3PAR = reinterpret_cast<irs_u32>(&SPI1_DR);  //  Peripheral address
    DMA2_S3FCR_bit.DMDIS = 0; //  Direct mode, no FIFO
  }
  ~spi_dma_reader_t() {};
  void add_event(mxfact_event_t *ap_event)
  {
    irs::interrupt_array()->int_event_gen(irs::arm::dma2_stream3_int)->add(ap_event);
  }
  void start(irs_u8* ap_write_buf, irs_u8* ap_read_buf, size_t a_size)
  {
    SPI1_CR2_bit.TXDMAEN = 1; //  SPI1 TX request for DMA enabled
    SPI1_CR2_bit.RXDMAEN = 1; //  SPI1 RX request for DMA enabled
    DMA2_S0NDTR = a_size;     //  number of data register
    DMA2_S0M0AR = reinterpret_cast<irs_u32>(ap_read_buf);  //  memory 0 address register
    DMA2_S3NDTR = a_size;     //  number of data register
    DMA2_S3M0AR = reinterpret_cast<irs_u32>(ap_write_buf); //  memory 0 address register
    DMA2_S0CR_bit.EN = 1;     //  RX Stream enable
    DMA2_S3CR_bit.TCIE = 1;   //  Transfer complete interrupt enable
    DMA2_S3CR_bit.EN = 1;     //  TX Stream enable
  }
  void stop()
  {
    DMA2_LIFCR_bit.CTCIF3 = 1;//  Clear Stream3 transfer complete interrupt flag
    DMA2_LIFCR_bit.CTCIF0 = 1;//  Clear Stream0 transfer complete interrupt flag
    DMA2_S3CR_bit.TCIE = 0;   //  Transfer complete interrupt enable
    DMA2_S0CR_bit.EN = 0;     //  RX Stream disable
    DMA2_S3CR_bit.EN = 0;     //  TX Stream disable
    SPI1_CR2_bit.TXDMAEN = 0; //  SPI1 TX request for DMA disabled
    SPI1_CR2_bit.RXDMAEN = 0; //  SPI1 RX request for DMA disabled
  }
  
  bool spi_busy()
  {
    return (SPI1_SR_bit.BSY == 1)/* && (SPI1_SR_bit.TXE == 0)*/;
  }
};

class network_config_t
{
public:
  network_config_t(
    irs::arm::st_ethernet_t* ap_arm_eth,
    irs::handle_t<irs::lwip::ethernet_t>* ap_ethernet,
    irs::handle_t<irs::hardflow::lwip::udp_t>* ap_udp_client,
    irs::hardflow::connector_t* ap_connector_hardflow);
  void set_ip(mxip_t a_ip);
  void set_dhcp(bool a_dhcp);
  void set_mask(mxip_t a_mask);
  void set_gateway(mxip_t a_gateway);
  void get(mxip_t* ap_ip, mxip_t* ap_mask, mxip_t* ap_gateway,
    bool* ap_dhcp_enabled);
  void set(mxip_t a_ip, mxip_t a_mask, mxip_t a_gateway, bool a_dhcp_enabled);
  void get_mac(mxmac_t* ap_mac);
private:
  void reset();
  network_config_t();
  //enum { channel_max_count = 10 };
  enum { channel_max_count = 3 };
  irs::arm::st_ethernet_t* mp_arm_eth;
  irs::handle_t<irs::lwip::ethernet_t>* mp_ethernet;
  irs::handle_t<irs::hardflow::lwip::udp_t>* mp_udp_client;
  irs::hardflow::connector_t* mp_connector_hardflow;
  irs::lwip::ethernet_t::configuration_t m_config;
};

class cfg_t
{
public:
  cfg_t();
  void tick();

private:
  mxmac_t m_local_mac;
  irs::arm::st_ethernet_t::config_t m_config;
  irs::arm::st_ethernet_t::config_t phy_config();
  irs::arm::st_ethernet_t m_arm_eth;
  irs::handle_t<irs::lwip::ethernet_t> lwip_ethernet;
  irs::handle_t<irs::hardflow::lwip::udp_t> udp_client;
public:
  //  Ethernet
  irs::hardflow::connector_t connector_hardflow;
  network_config_t network_config;
  //  AUX
  irs::arm::io_pin_t led_blink;
  irs::arm::io_pin_t ee_cs;
//  irs::arm::io_pin_t aux1;
//  irs::arm::io_pin_t aux2;
//  irs::arm::io_pin_t aux3;
//  irs::arm::io_pin_t mezzo1;
//  irs::arm::io_pin_t mezzo2;
  irs::arm::io_pin_t mezzo3;
  irs::arm::io_pin_t mezzo4;
  //  UI
  irs::arm::io_port_t lcd_port;
  irs::arm::io_pin_t lcd_rs_pin;
  irs::arm::io_pin_t lcd_e_pin;
  vector<irs::handle_t<irs::gpio_pin_t> > key_drv_horizontal_pins;
  vector<irs::handle_t<irs::gpio_pin_t> > key_drv_vertical_pins;
  irs::pwm_pin_t buzzer;
  //  AD4630
  irs::arm::io_pin_t ad4630_rst;
  irs::arm::io_pin_t ad4630_cs;
  irs::arm::io_pin_t ad4630_busy;
  irs::arm::io_pin_t ad4630_pwr;
  irs::arm::io_pin_t ad4630_point_control;
  pulse_gen_t pulse_gen;
  adc_ad4630_ready_t adc_ready;
  spi_dma_reader_t spi_dma_reader;
  //  Bridge voltage DAC
  irs::arm::io_pin_t bridge_voltage_dac_cs;
  //  Relays
  irs::arm::io_pin_t relay_bridge_pos_on;
  irs::arm::io_pin_t relay_bridge_pos_off;
  irs::arm::io_pin_t relay_bridge_neg_on;
  irs::arm::io_pin_t relay_bridge_neg_off;
  irs::arm::io_pin_t relay_prot;
  //  Relays AD4630
  irs::arm::io_pin_t relay_bridge_divp_on;
  irs::arm::io_pin_t relay_bridge_divp_off;
  irs::arm::io_pin_t relay_bridge_divn_on;
  irs::arm::io_pin_t relay_bridge_divn_off;
  //  Relay DIVSW
  irs::arm::io_pin_t relay_divsw_on;  //  ON = REF
  irs::arm::io_pin_t relay_divsw_off; //  OFF = SENSE
  irs::arm::io_pin_t led_divsw;       //  Debug led, yellow, on DIVSW board
  //  SPI
  irs_u32 m_spi_bitrate;
  irs::arm::arm_spi_t spi_dac;
  irs::arm::arm_spi_t spi_aux;
  //  Climate
  static const gpio_channel_t peltier_pwm1_channel = PE6;
  irs::arm::st_pwm_gen_t peltier_pwm1_gen;
  static const gpio_channel_t peltier_pwm2_channel = PE5;
  irs::arm::st_pwm_gen_t peltier_pwm2_gen;
  irs::arm::io_pin_t peltier_pol1;
  irs::arm::io_pin_t peltier_pol2;
  irs::arm::io_pin_t fan_ac_on;
  irs::arm::io_pin_t fan_dc_ls;
  irs::arm::io_pin_t fan_dc_hs;
  irs::arm::io_pin_t fan_dc_sen;
};

} //  hrm

#endif // cfgh
