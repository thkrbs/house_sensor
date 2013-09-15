''ThK 2013-08-19
''

OBJ
  SPI : "SPI_4wire_Spin"      ''The Standalone  engine SPI   
  

CON


  RF_Channel = 99
  RF_POWER_UP_DELAY = 2
  RF_PAYLOAD_LENGTH = 10
  RF_RETRANSMITS = 15
  RF_TRANS_DELAY = 500 ' if length <=18 : 250  Defines the retransmit delay. Should be a multiple of 250.

{{ Register definitions for the nRF HAL module
 * Header file defining register mapping with bit definitions.\ This file is radio-chip dependent, and are included with the hal_nrf.h
}}

CON
 
'' - Instruction Set - */
'' nRF24L01 Instruction Definitions */
  CMD_R_REGISTER = %0000_0000 '& reg  ' Read command and status registers. AAAAA = 5 bit Register Map Address
  CMD_W_REGISTER = %0010_0000 '& reg  ' Write command and status registers. AAAAA = 5 bit Register Map Address Executable in power down or standby modes only.
  CMD_R_RX_PAYLOAD = %0110_0001       ' Read RX-payload: 1 - 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode.
  CMD_W_TX_PAYLOAD = %1010_0000       ' Write TX-payload: 1 - 32 bytes. A write operation always starts at byte 0 used in TX payload.
  CMD_FLUSH_TX = %1110_0001           ' Flush TX FIFO, used in TX mode
  CMD_FLUSH_RX = %1110_0010           ' Flush RX FIFO, used in RX mode Should not be executed during transmission of acknowledge, that is, acknowledge package will not be completed.
  CMD_REUSE_TX_PL = %1110_0011        ' Used for a PTX device Reuse last transmitted payload. TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or deactivated during package transmission.
  CMD_R_RX_PL_WID = %0110_0000        ' Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO. Note: Flush RX FIFO if the read value is larger than 32 bytes.
  CMD_W_ACK_PAYLOAD = %1010_1000 '+ pipe Used in RX mode. Write Payload to be transmitted together with ACK packet on PIPE PPP. (PPP valid in the range from 000 to 101). Maximum three ACK packet payloads can be pending. Payloads with same PPP are handled using first in - first out principle. Write payload: 1– 32 bytes. A write operation always starts at byte 0.
  CMD_W_TX_PAYLOAD_NOACK = %1011_0000 ' Used in TX mode. Disables AUTOACK on this specific packet.
  CMD_NOP = %1111_1111               ' No Operation. Might be used to read the STATUS register

  'LOCK_UNLOCK   = $50  '' Lock/unlcok exclusive features */

 
''/** @name  - Register Memory Map - */
'' nRF24L01 * Register Definitions * */
  reg_CONFIG        = $00  ' Configuration Register
  reg_EN_AA         = $01  ' Enable "Auto Acknowledgment" Function 
  reg_EN_RXADDR     = $02  ' Enabled RX Addresses
  reg_SETUP_AW      = $03  ' Setup of Address Widths (common for all data pipes)
  reg_SETUP_RETR    = $04  ' Setup of Automatic Retransmission
  reg_RF_CH         = $05  ' RF Channel
  reg_RF_SETUP      = $06  ' RF Setup Register
  reg_STATUS        = $07  ' Status Register - also recieved parralel to each command send
  reg_OBSERVE_TX    = $08  ' Transmit observe register
  reg_RPD           = $09  ' Received Power Detector This register is called CD (Carrier Detect) in the nRF24L01.  IF recvlevel > -64dbm THEN bit0 = 1 
  reg_RX_ADDR_P0    = $0A  ' Receive address data pipe 0. 5 Bytes maximum length. reset value: 0xE7E7E 7E7E7(LSByte is written first. Write the number of bytes defined by SETUP_AW)
  reg_RX_ADDR_P1    = $0B  ' Receive address data pipe 1. 5 Bytes maximum length. reset value: 0xC2C2C 2C2C2
  reg_RX_ADDR_P2    = $0C  ' Receive address data pipe 2. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8] reset value: 0c3 
  reg_RX_ADDR_P3    = $0D  ' Receive address data pipe 3. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8] reset value: 0c4
  reg_RX_ADDR_P4    = $0E  ' Receive address data pipe 4. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8] reset value: 0c5
  reg_RX_ADDR_P5    = $0F  ' Receive address data pipe 5. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8] reset value: 0c6/
  reg_TX_ADDR       = $10  ' Transmit address. Used for a PTX device only. reset value: 0xE7E7E 7E7E7 (LSByte is written first) 
  reg_RX_PW_P0      = $11  ' Number of bytes in RX payload in data pipe 0
  reg_RX_PW_P1      = $12  ' Number of bytes in RX payload in data pipe 1
  reg_RX_PW_P2      = $13  ' Number of bytes in RX payload in data pipe 2
  reg_RX_PW_P3      = $14  ' Number of bytes in RX payload in data pipe 3
  reg_RX_PW_P4      = $15  ' Number of bytes in RX payload in data pipe 4
  reg_RX_PW_P5      = $16  ' Number of bytes in RX payload in data pipe 5
  reg_FIFO_STATUS   = $17  ' FIFO Status Register
  reg_DYNPD         = $1C  ' Enable dynamic payload length
  reg_FEATURE       = $1D  ' Feature Register

'' nRF24L01 related definitions */
''/* Interrupt definitions */
''/* Operation mode definitions */

{/** An enum describing the radio's irq sources.
typedef enum {
    HAL_NRF_MAX_RT = 4,     '' Max retries interrupt */
    HAL_NRF_TX_DS,          '' TX data sent interrupt */
    HAL_NRF_RX_DR           '' RX data received interrupt */
} hal_nrf_irq_source_t;

/* Operation mode definitions */
/** An enum describing the radio's power mode.
typedef enum {
    HAL_NRF_PTX,            '' Primary TX operation */
    HAL_NRF_PRX             '' Primary RX operation */
} hal_nrf_operation_mode_t;

/** An enum describing the radio's power mode.
typedef enum {
    HAL_NRF_PWR_DOWN,       '' Device power-down */
    HAL_NRF_PWR_UP          '' Device power-up */
} hal_nrf_pwr_mode_t;

/** An enum describing the radio's output power mode's.
typedef enum {
    HAL_NRF_18DBM,          '' Output power set to -18dBm */
    HAL_NRF_12DBM,          '' Output power set to -12dBm */
    HAL_NRF_6DBM,           '' Output power set to -6dBm  */
    HAL_NRF_0DBM            '' Output power set to 0dBm   */
} hal_nrf_output_power_t;

/** An enum describing the radio's on-air datarate.
typedef enum {
    HAL_NRF_1MBPS,          '' Datarate set to 1 Mbps  */
    HAL_NRF_2MBPS           '' Datarate set to 2 Mbps  */
} hal_nrf_datarate_t;

/** An enum describing the radio's PLL mode.
typedef enum {
    HAL_NRF_PLL_UNLOCK,     '' PLL unlocked, normal operation  */
    HAL_NRF_PLL_LOCK        '' PLL locked, test mode  */
} hal_nrf_pll_mode_t;

/** An enum describing the radio's LNA mode.
typedef enum {
    HAL_NRF_LNA_LCURR,      '' LNA set to low current mode */
    HAL_NRF_LNA_HCURR       '' LNA set to high current mode */
} hal_nrf_lna_mode_t;

/** An enum describing the radio's CRC mode.
typedef enum {
    HAL_NRF_CRC_OFF,        '' CRC check disabled */
    HAL_NRF_CRC_8BIT = 2,   '' CRC check set to 8-bit */
    HAL_NRF_CRC_16BIT       '' CRC check set to 16-bit */
} hal_nrf_crc_mode_t;

/** An enum describing the read/write payload command.
typedef enum {
    HAL_NRF_TX_PLOAD = 7,   '' TX payload definition */
    HAL_NRF_RX_PLOAD,        '' RX payload definition */
    HAL_NRF_ACK_PLOAD
} hal_nrf_pload_command_t;


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

'' FIFO_STATUS $17 */
'' FIFO_STATUS register bit definitions */

  TX_REUSE      = 6     '' FIFO_STATUS register bit 6 */
  TX_FIFO_FULL  = 5     '' FIFO_STATUS register bit 5 */
  TX_EMPTY      = 4     '' FIFO_STATUS register bit 4 */
  RX_FULL       = 1     '' FIFO_STATUS register bit 1 */
  RX_EMPTY      = 0     '' FIFO_STATUS register bit 0 */
}
  
VAR

  'byte SPI_MOSI              '' Set Master Out, Data In Input Pin
  'byte SPI_MISO              '' Set Master In, Data Out Input Pin
  'byte SPI_SCK               '' Set  Clock Pin
  'byte SPI_CSN               '' Set  Chip Select Pin 

  byte nRF_CE_pin            '' remember Propeller pin connected to  CE pin 

  byte temp_val
  
PUB radio_init(MOSI, MISO, SCK, CSN, CE)
   nRF_CE_pin := CE
   SPI.SPI_config_pins(MOSI, MISO, SCK, CSN)

   ' default config:
   set_Other_reg($03, %0000_00_11)  'SETUP_AW, 5byte addr.

   enable_ack_pl 'etc.

PUB is_status_TX_FULL(status)
{TX FIFO full flag.
1: TX FIFO full.
0: Available locations in TX FIFO. }
  return (status & |<0)

PUB is_status_MAX_RT(status)
{ Maximum number of TX retransmits interrupt
Write 1 to clear bit. If MAX_RT is asserted it must
be cleared to enable further communication
}
  return (status & |<4)

PUB is_status_TX_DS(status)
{ Data Sent TX FIFO interrupt. Asserted when
packet transmitted on TX. If AUTO_ACK is activated,
this bit is set high only when ACK is
received.
Write 1 to clear bit.
}
  return (status & |<5)

PUB is_status_RX_DR(status)
{ Data Ready RX FIFO interrupt. Asserted when
new data arrives RX FIFOc.
Write 1 to clear bit.
}
  return (status & |<6)
  
'----------------------------------------------------------- one octet commands ----                                              
PUB do_NOP
{ No Operation command. Use this function to receive the radio's status register.}
  return get_rf_status

PUB get_rf_status : status
  return SPI.Read_Write_One_Octet(CMD_NOP)

PUB flush_rx(void)
{{ Flush RX FIFO. Use this function to flush the radio's RX FIFO }}  'write_reg(FLUSH_RX, 0)
  return SPI.Read_Write_One_Octet(CMD_FLUSH_RX)
  
PUB flush_tx(void)
{{ Flush TX FIFO.Use this function to flush the radio's  TX FIFO  }} 'write_reg(FLUSH_TX, 0)
  return SPI.Read_Write_One_Octet(CMD_FLUSH_TX)

'----------------------------------------------------------- two octet commands ---- whole octets                                             
  

PUB set_rf_channel(setchannel)
{{ Set radio's RF channel.  }}
  return SPI.Read_Write_Two_Octet(CMD_W_REGISTER | Reg_RF_CH , setchannel & $7F, 0) 

PUB get_rf_channel(read_channelPtr)
{{ Get radio's RF channel. }}
  return SPI.Read_Write_Two_Octet(CMD_R_REGISTER | Reg_RF_CH , 0, read_channelPtr) 

PUB set_Other_reg(reg, val)
  {{ eg for
    set_address_width(address_width_t address_width)  write_reg(SETUP_AW, (UINT8(address_width) - 2))
    set_auto_retr(uint8_t retr, uint16_t delay) write_reg(SETUP_RETR, (((delay/250)-1)<<4) | retr) 
  }}
  return SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg, val, 0)

PUB get_Other_reg(reg, valPtr)
  {{ eg. for
  get_address_width(void) Get address width for radio. read_reg(SETUP_AW) + 2
  }}
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg, 0, valPtr)
  return
  
PUB get_power_mode( pwr_modePtr)       'CMD_R_REGISTER | reg_CONFIG
  temp_val := SPI.Read_Write_Two_Octet(0, 0, pwr_modePtr)
  return temp_val


'----------------------------------------------------------- two octet commands ---- single bits                                             

PUB set_rf_receive(on)
{{ Set radio's RF channel.
 * Use this function to select which RF channel to use.
 * @param channel RF channel
}}

  SPI.Read_Two_Octet(CMD_R_REGISTER | reg_CONFIG, @temp_val)
  
  if on == 1
    SPI.Write_Two_Octet(CMD_W_REGISTER | reg_CONFIG, temp_val | %0000_0001) ' force low power bit 1+2
  else
    SPI.Write_Two_Octet(CMD_W_REGISTER | reg_CONFIG, temp_val & %1111_1110) ' force low power bit 1+2


PUB set_power_mode( pwr_mode)
{{ Set radio's power mode.
 * Use this function to power_up or power_down radio.
 *
 * @param pwr_mode POWER_UP or POWER_DOWN
}}
'  if(pwr_mode == PWR_UP)
'    write_reg(CONFIG, (read_reg(CONFIG) | (1<<PWR_UP)))
'  else
'    write_reg(CONFIG, (read_reg(CONFIG) & ~(1<<PWR_UP)))
  
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg_CONFIG, 0, @temp_val)
  if pwr_mode == 1
    SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg_CONFIG, temp_val | %0000_0010, 0) 
  else
    SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg_CONFIG, temp_val & %1111_1101, 0) 
  return temp_val


PUB set_output_power(power)
{{ Set radio's TX output power.
 * Use this function set the radio's TX output power.
 *
 * @param power Radio's TX output power
}}
'  write_reg(RF_SETUP, (read_reg(RF_SETUP) & ~((1<<RF_PWR1)|(1<<RF_PWR0))) | (UINT8(power)<<1))
  SPI.Read_Two_Octet(CMD_R_REGISTER | reg_RF_SETUP, @temp_val)
  
  SPI.Write_Two_Octet(CMD_W_REGISTER | reg_RF_SETUP, temp_val & %1111_1001) ' force low power bit 1+2
{RF_PWR 2:1 11 R/W Set RF output power in TX mode
'00' – -18dBm
'01' – -12dBm
'10' – -6dBm
'11' – 0dBm}

PUB get_receive_power_detected 
{{ used to detect stationary disturbance on selected RF channel.
}}
  
  High(nRF_CE_pin)
  'waitcnt(clkfreq/1000 + cnt)
  waitcnt(5000 + cnt)
  'temp_val :=0
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg_RPD,  0, @temp_val) 

  Low(nRF_CE_pin)
  return temp_val
  'return (temp_val & %0000_0001) 'bit 0 is receive power detected.

'----------------------------------------------------------- multi octet commands ----                                              

PUB set_RX_Addr(pipe, addrPtr)
{{ Set radio's RX address and TX address.
 * Use this function to set a RX address, or to set the TX address.
 * Beware of the difference for single and multibyte address registers.
 *
 * @param pipe Which pipe address to set
 * @param addrPtr Buffer from which the address is stored in
}}
  if pipe == 0
    SPI.Read_Write_Six_Octet_Address(CMD_W_REGISTER | Reg_RX_ADDR_P0, addrPtr, 0)
  elseif pipe == 1
    SPI.Read_Write_Six_Octet_Address(CMD_W_REGISTER | Reg_RX_ADDR_P1, addrPtr, 0)
  elseif pipe == 2
    SPI.Read_Write_Two_Octet(CMD_W_REGISTER | Reg_RX_ADDR_P2, byte[addrPtr], 0)
  elseif pipe == 3
    SPI.Read_Write_Two_Octet(CMD_W_REGISTER | Reg_RX_ADDR_P3, byte[addrPtr], 0)
  elseif pipe == 4
    SPI.Read_Write_Two_Octet(CMD_W_REGISTER | Reg_RX_ADDR_P4, byte[addrPtr], 0)
  elseif pipe == 5
    SPI.Read_Write_Two_Octet(CMD_W_REGISTER | Reg_RX_ADDR_P5, byte[addrPtr], 0)
  
PUB get_RX_Addr(pipe, addrPtr)
  {{ Get address for selected pipe. get_address(uint8_t address, uint8_t *addr) }}
  if pipe == 0
    SPI.Read_Write_Six_Octet_Address(CMD_R_REGISTER | Reg_RX_ADDR_P0, 0, addrPtr)
  elseif pipe == 1
    SPI.Read_Write_Six_Octet_Address(CMD_R_REGISTER | Reg_RX_ADDR_P1, 0, addrPtr)
  elseif pipe == 2
    SPI.Read_Write_Two_Octet(CMD_R_REGISTER | Reg_RX_ADDR_P2, 0, addrPtr)
  elseif pipe == 3
    SPI.Read_Write_Two_Octet(CMD_R_REGISTER | Reg_RX_ADDR_P3, 0, addrPtr)
  elseif pipe == 4
    SPI.Read_Write_Two_Octet(CMD_R_REGISTER | Reg_RX_ADDR_P4, 0, addrPtr)
  elseif pipe == 5
    SPI.Read_Write_Two_Octet(CMD_R_REGISTER | Reg_RX_ADDR_P5, 0, addrPtr)


PUB set_TX_Addr(addrPtr)
  SPI.Read_Write_Six_Octet_Address(CMD_W_REGISTER | Reg_TX_ADDR, addrPtr, 0)

PUB get_TX_Addr(addrPtr)
  SPI.Read_Write_Six_Octet_Address(CMD_R_REGISTER | Reg_TX_ADDR, 0, addrPtr)



PRI High(Pin)
    dira[Pin]~~
    outa[Pin]~~
         
PRI Low(Pin)
    dira[Pin]~~
    outa[Pin]~

PUB enable_dynamic_ack
  {{ Enables the no-ack feature }} 'write_reg(FEATURE, (read_reg(FEATURE) | 0x01))
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg_FEATURE, 0, @temp_val)
  SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg_FEATURE, temp_val | (|< 0), 0) 

  
PUB disable_dynamic_ack
{{ Disables the no-ack feature }} 'write_reg(FEATURE, (read_reg(FEATURE) & ~0x01))
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg_FEATURE, 0, @temp_val)
  SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg_FEATURE, temp_val & (!|< 0), 0) 
  
PUB enable_ack_pl
  '' Enables the ACK payload feature   'write_reg(FEATURE, (read_reg(FEATURE) | 0x02))
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg_FEATURE, 0, @temp_val)
  SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg_FEATURE, temp_val | (|< 1), 0) 

PUB disable_ack_pl
  ''Disables the ACK payload feature   ' write_reg(FEATURE, (read_reg(FEATURE) & ~0x02))
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg_FEATURE, 0, @temp_val)
  SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg_FEATURE, temp_val & (!|< 1), 0) 

PUB enable_dynamic_pl
  {{ Enables the dynamic payload feature }} 'write_reg(FEATURE, (read_reg(FEATURE) | 0x04))
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg_FEATURE, 0, @temp_val)
  SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg_FEATURE, temp_val | (|< 2), 0) 
   
PUB disable_dynamic_pl
  {{ Disables the dynamic payload feature }} 'write_reg(FEATURE, (read_reg(FEATURE) & ~0x04))
  SPI.Read_Write_Two_Octet(CMD_R_REGISTER | reg_FEATURE, 0, @temp_val)
  SPI.Read_Write_Two_Octet(CMD_W_REGISTER | reg_FEATURE, temp_val & (!|< 2), 0) 

PUB read_rx_pl_w
  {{ Reads the payload width of the received ack payload
  * @return Payload width of the received ack payload
  }}
  'uint8_t temp,   CSN_LOW(),   rw(RD_RX_PLOAD_W),  temp = rw(0),   CSN_HIGH() 

  SPI.Read_Write_Two_Octet(CMD_R_RX_PL_WID, 0, @temp_val)
  return temp_val
  