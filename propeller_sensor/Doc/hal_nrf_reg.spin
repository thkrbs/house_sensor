{{ Register definitions for the nRF HAL module
 * @defgroup nordic_hal_nrf_reg nRF24L01 Register definitions
 * @{
 * @ingroup nordic_hal_nrf
 * Header file defining register mapping with bit definitions.\ This file is radio-chip dependent, and are included with the hal_nrf.h
}}

CON
 
'' - Instruction Set - */
'' nRF24L01 Instruction Definitions */
  CMD_WRITE_REG     = $20  '''' Register write command */
  CMD_RD_RX_PLOAD_W = $60  '''' Read RX payload command */
  CMD_RD_RX_PLOAD   = $61  '' Read RX payload command */
  CMD_WR_TX_PLOAD   = $A0  '' Write TX payload command */
  CMD_WR_ACK_PLOAD  = $A8  '' Write ACK payload command */
  CMD_WR_NAC_TX_PLOAD = $B0  '' Write ACK payload command */
  CMD_FLUSH_TX      = $E1  '' Flush TX register command */
  CMD_FLUSH_RX      = $E2  '' Flush RX register command */
  CMD_REUSE_TX_PL   = $E3  '' Reuse TX payload command */
  CMD_LOCK_UNLOCK   = $50  '' Lock/unlcok exclusive features */

  CMD_NOP           = $FF  '' No Operation command, used for reading status register */

''/** @name  - Register Memory Map - */
'' nRF24L01 * Register Definitions * */
  CONFIG        = $00  ' nRF24L01 config register */
  EN_AA         = $01  ' nRF24L01 enable Auto-Acknowledge register */
  EN_RXADDR     = $02  ' nRF24L01 enable RX addresses register */
  SETUP_AW      = $03  ' nRF24L01 setup of address width register */
  SETUP_RETR    = $04  ' nRF24L01 setup of automatic retransmission register */
  RF_CH         = $05  ' nRF24L01 RF channel register */
  RF_SETUP      = $06  ' nRF24L01 RF setup register */
  STATUS        = $07  ' nRF24L01 status register - also recieved parralel to each command send
  OBSERVE_TX    = $08  ' nRF24L01 transmit observe register */
  'CD            = $09  ' nRF24L01 carrier detect register */
  RPD           = $09  ' nRF24L01+ carrier detect register - register named differend here:  IF recvlevel > -64dbm THEN bit0 = 1 
  RX_ADDR_P0    = $0A  ' nRF24L01 receive address data pipe0 */
  RX_ADDR_P1    = $0B  ' nRF24L01 receive address data pipe1 */
  RX_ADDR_P2    = $0C  ' nRF24L01 receive address data pipe2 */
  RX_ADDR_P3    = $0D  ' nRF24L01 receive address data pipe3 */
  RX_ADDR_P4    = $0E  ' nRF24L01 receive address data pipe4 */
  RX_ADDR_P5    = $0F  ' nRF24L01 receive address data pipe5 */
  TX_ADDR       = $10  ' nRF24L01 transmit address */
  RX_PW_P0      = $11  ' nRF24L01 \# of bytes in rx payload for pipe0 */
  RX_PW_P1      = $12  ' nRF24L01 \# of bytes in rx payload for pipe1 */
  RX_PW_P2      = $13  ' nRF24L01 \# of bytes in rx payload for pipe2 */
  RX_PW_P3      = $14  ' nRF24L01 \# of bytes in rx payload for pipe3 */
  RX_PW_P4      = $15  ' nRF24L01 \# of bytes in rx payload for pipe4 */
  RX_PW_P5      = $16  ' nRF24L01 \# of bytes in rx payload for pipe5 */
  FIFO_STATUS   = $17  ' nRF24L01 FIFO status register */
  DYNPD         = $1C  ' nRF24L01 Dynamic payload setup */
  FEATURE       = $1D  ' nRF24L01 Exclusive feature setup */

'' nRF24L01 related definitions */
''/* Interrupt definitions */
''/* Operation mode definitions */

{{/** An enum describing the radio's irq sources.
 *
 */
typedef enum {
    HAL_NRF_MAX_RT = 4,     '' Max retries interrupt */
    HAL_NRF_TX_DS,          '' TX data sent interrupt */
    HAL_NRF_RX_DR           '' RX data received interrupt */
} hal_nrf_irq_source_t;

/* Operation mode definitions */
/** An enum describing the radio's power mode.
 *
 */
typedef enum {
    HAL_NRF_PTX,            '' Primary TX operation */
    HAL_NRF_PRX             '' Primary RX operation */
} hal_nrf_operation_mode_t;

/** An enum describing the radio's power mode.
 *
 */
typedef enum {
    HAL_NRF_PWR_DOWN,       '' Device power-down */
    HAL_NRF_PWR_UP          '' Device power-up */
} hal_nrf_pwr_mode_t;

/** An enum describing the radio's output power mode's.
 *
 */
typedef enum {
    HAL_NRF_18DBM,          '' Output power set to -18dBm */
    HAL_NRF_12DBM,          '' Output power set to -12dBm */
    HAL_NRF_6DBM,           '' Output power set to -6dBm  */
    HAL_NRF_0DBM            '' Output power set to 0dBm   */
} hal_nrf_output_power_t;

/** An enum describing the radio's on-air datarate.
 *
 */
typedef enum {
    HAL_NRF_1MBPS,          '' Datarate set to 1 Mbps  */
    HAL_NRF_2MBPS           '' Datarate set to 2 Mbps  */
} hal_nrf_datarate_t;

/** An enum describing the radio's PLL mode.
 *
 */
typedef enum {
    HAL_NRF_PLL_UNLOCK,     '' PLL unlocked, normal operation  */
    HAL_NRF_PLL_LOCK        '' PLL locked, test mode  */
} hal_nrf_pll_mode_t;

/** An enum describing the radio's LNA mode.
 *
 */
typedef enum {
    HAL_NRF_LNA_LCURR,      '' LNA set to low current mode */
    HAL_NRF_LNA_HCURR       '' LNA set to high current mode */
} hal_nrf_lna_mode_t;

/** An enum describing the radio's CRC mode.
 *
 */
typedef enum {
    HAL_NRF_CRC_OFF,        '' CRC check disabled */
    HAL_NRF_CRC_8BIT = 2,   '' CRC check set to 8-bit */
    HAL_NRF_CRC_16BIT       '' CRC check set to 16-bit */
} hal_nrf_crc_mode_t;

/** An enum describing the read/write payload command.
 *
 */
typedef enum {
    HAL_NRF_TX_PLOAD = 7,   '' TX payload definition */
    HAL_NRF_RX_PLOAD,        '' RX payload definition */
    HAL_NRF_ACK_PLOAD
} hal_nrf_pload_command_t;

/** Structure containing the radio's address map.
 * Pipe0 contains 5 unique address bytes,
 * while pipe[1..5] share the 4 MSB bytes, set in pipe1.
 * <p><b> - Remember that the LSB byte for all pipes have to be unique! -</b>
 */
// nRF24L01 Address struct

/*
//typedef struct {
//   uint8_t p0[5];            '' Pipe0 address, 5 bytes */
//    uint8_t p1[5];            '' Pipe1 address, 5 bytes, 4 MSB bytes shared for pipe1 to pipe5 */
//    uint8_t p2[1];            '' Pipe2 address, 1 byte */
//    uint8_t p3[1];            '' Pipe3 address, 1 byte */
 //   uint8_t p4[1];            '' Pipe3 address, 1 byte */
 //   uint8_t p5[1];            '' Pipe3 address, 1 byte */
 //   uint8_t tx[5];            '' TX address, 5 byte */
//} hal_nrf_l01_addr_map;
}}

'' An enum describing the nRF24L01 pipe addresses and TX address.
''typedef enum {
'    HAL_NRF_PIPE0,              '' Select pipe0 */
'    HAL_NRF_PIPE1,              '' Select pipe1 */
'    HAL_NRF_PIPE2,              '' Select pipe2 */
'    HAL_NRF_PIPE3,              '' Select pipe3 */
'    HAL_NRF_PIPE4,              '' Select pipe4 */
'    HAL_NRF_PIPE5,              '' Select pipe5 */
'    HAL_NRF_TX,                 '' Refer to TX address*/
'    HAL_NRF_ALL = $FF          '' Close or open all pipes*/
                                '' @see hal_nrf_set_address @see hal_nrf_get_address
                                 '@see hal_nrf_open_pipe  @see hal_nrf_close_pipe */
''} hal_nrf_address_t;

'' An enum describing the radio's address width.
''typedef enum {
''    HAL_NRF_AW_3BYTES = 3,      '' Set address width to 3 bytes */
''    HAL_NRF_AW_4BYTES,          '' Set address width to 4 bytes */
''    HAL_NRF_AW_5BYTES           '' Set address width to 5 bytes */
''} hal_nrf_address_width_t;


'' CONFIG register bit definitions */
''
  MASK_RX_DR    = 6     '' CONFIG register bit 6 */
  MASK_TX_DS    = 5     '' CONFIG register bit 5 */
  MASK_MAX_RT   = 4     '' CONFIG register bit 4 */
  EN_CRC        = 3     '' CONFIG register bit 3 */
  CRCO          = 2     '' CONFIG register bit 2 */
  PWR_UP        = 1     '' CONFIG register bit 1 */
  PRIM_RX       = 0     '' CONFIG register bit 0 */

'' RF_SETUP register bit definitions */

  PLL_LOCK      = 4     '' RF_SETUP register bit 4 */
  RF_DR         = 3     '' RF_SETUP register bit 3 */
  RF_PWR1       = 2     '' RF_SETUP register bit 2 */
  RF_PWR0       = 1     '' RF_SETUP register bit 1 */
  LNA_HCURR     = 0     '' RF_SETUP register bit 0 */

''  STATUS $07 */
'' STATUS register bit definitions */

  RX_DR         = 6     '' STATUS register bit 6 */
  TX_DS         = 5     '' STATUS register bit 5 */
  MAX_RT        = 4     '' STATUS register bit 4 */
  TX_FULL       = 0     '' STATUS register bit 0 */


'' FIFO_STATUS $17 */
'' FIFO_STATUS register bit definitions */

  TX_REUSE      = 6     '' FIFO_STATUS register bit 6 */
  TX_FIFO_FULL  = 5     '' FIFO_STATUS register bit 5 */
  TX_EMPTY      = 4     '' FIFO_STATUS register bit 4 */
  RX_FULL       = 1     '' FIFO_STATUS register bit 1 */
  RX_EMPTY      = 0     '' FIFO_STATUS register bit 0 */

PUB halnrfreg