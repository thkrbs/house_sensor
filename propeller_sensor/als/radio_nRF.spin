''ThK 2013-08-19
''
ObJ
  HalRadio : "hal_nrf24L01p"   'radio implementation for nRF24L01
  SER      : "FullDuplexSerialPlus"  'Serial port

CON

' enum radio_staus_t
  radio_status_rf_idle = 0   'Radio is idle
  radio_status_rf_max_rt = 1 'Maximum number of retries have occured
  radio_status_rf_tx_ds = 2  'The data are sent
  radio_status_rf_rx_dr = 3  'Data is recieved
  radio_status_rf_tx_ap = 4  'Ack payload recieved
  radio_status_rf_busy = 5   'Radio is busy


{{
#include "hal_nrf.h"
#include "system.h"
#include "radio.h"
#include "target_includes.h"

/** The payload sent over the radio. Also contains the recieved data. 
 * Should be read with radio_get_pload_byte(). */
static xdata uint8_t pload[RF_PAYLOAD_LENGTH];
/** The current status of the radio. Should be set with radio_set_status(), 
 * and read with radio_get_status().
 */
}}

  _clkmode = xtal1 + pll16x                           
  _xinfreq = 5_000_000

  SPI_CE   = 3
  SPI_CSN  = 4     
  SPI_SCK  = 5 
  SPI_MOSI = 7 
  SPI_MISO = 6
  SPI_IRQ  = 9

VAR
  BYTE Radio_status
  byte read_channel
  byte addr[5]
  byte raddr[5]

PUB unit_test_HAL_LIB  | carrier, c, i , pipe,  read_val, channel,  val

  SER.start(31, 30, 0, 57600)
  'Set IRQ pin state 
  dira[SPI_IRQ] := 0

  Init( SPI_MISO, SPI_MOSI, SPI_SCK, SPI_CSN, SPI_CE)  

  repeat
    waitcnt(clkfreq*2 + cnt)
    'SER.tx(SER#CLS)
    SER.str(String(SER#CR)) 
    SER.str(String(SER#CR)) 
    SER.str(String(SER#CR)) 
    SER.str(String(SER#CR)) 
    SER.str(String(SER#CR)) 
    SER.str(String(SER#CR)) 
    SER.str(String(SER#CR)) 
    SER.str(String("hi... starting up radio_nRF unittest:"))
    SER.str(String(SER#CR)) 
    addr[1] := 7
    checkptrarry(@addr)
    if addr[1] <> 23
      SER.str(String(" Wert: '"))
      ser.dec(addr[1])
      SER.str(String("' "))
      SER.str(String(SER#CR)) 
    
    
    SER.str(String("Read status value STATUS ...'"))
    val := $EE
    val := get_RF_Status          ' uses SPI.Read_Write_One_Octet(CMD_NOP) 

    ser.hex(val,2) 
    SER.str(String(" "))
    ser.bin(val,8) 

    if val <> 255 and val <> 0
      SER.str(String("' ok."))
    else
      SER.str(String("' NOK. Value should not be zero"))
    SER.str(String(SER#CR)) 


    SER.str(String("Dump all registers ...'"))
    SER.str(String(SER#CR)) 
    read_channel := $AA
    repeat i from 0 to $17
      ser.hex(i,2) 
      SER.str(String(" - ")) 

      HalRadio.get_Other_reg(i, @read_channel)
      
      ser.hex(read_channel,2) 
      SER.str(String(" : ")) 
      ser.bin(read_channel,8) 
      SER.str(String(SER#CR)) 
    
    SER.str(String("Read Register value CONFIG ...'"))
    read_val := $EE
    HalRadio.get_Other_reg(0, @read_val)
    'HalRadio.get_power_mode(@read_val)

    ser.hex(read_val,2) 
    SER.str(String(" "))
    ser.bin(read_val,8) 

    if read_val <> $EE and read_val <> 0
      SER.str(String("' ok."))
    else
      SER.str(String("' NOK. Config_Reg should not be zero "))
      ser.hex(read_val,2) 
    SER.str(String(SER#CR)) 


    SER.str(String("Write Register value CHANNEL ...'"))
    channel := 25
    HalRadio.set_rf_channel(channel)           ' uses SPI.Read_Write_Two_Octet(Register, CnfgValue, RcvValuePtr)
    'read_channel := $AAAAAAAA
    HalRadio.get_rf_channel(@read_channel)

    ser.hex(read_channel,2)
    
    if read_channel == channel
      SER.str(String("' ok."))
    else
      SER.str(String("' NOK. Channel is not "))
      ser.hex(channel,2) 
      SER.str(String("' but "))
      ser.hex(read_channel,2) 
    SER.str(String(SER#CR)) 
    
    SER.str(String("Write Register SETUP_AW ...'"))
    HalRadio.set_Other_reg($03, %0000_00_11)  'SETUP_AW, 5byte addr.

    HalRadio.get_Other_reg($03, @val)
    ser.hex(val,2)
    
    if val <> $03     
      SER.str(String("' NOK. Should: "))
      ser.hex($03,2)
    SER.str(String(SER#CR)) 

    SER.str(String("Write Register Adress value RX_ADDR ...'"))
    SER.str(String(SER#CR)) 
    addr[0] := $A1
    addr[1] := $A2
    addr[2] := $A3
    addr[3] := $A4
    addr[4] := $A5
    raddr[0] := $15
    raddr[1] := $25
    raddr[2] := $35
    raddr[3] := $45
    raddr[4] := $55
    
    'pipe 0
    HalRadio.set_RX_Addr(0, @addr)       'uses SPI.Read_Write_Five_Octet_Data(Register, WriteDataPtr, ReadDataPtr)
    
    HalRadio.get_RX_Addr(0, @raddr)
    
    repeat i from 0 to 4
      ser.hex(raddr[i],2)
      if raddr[i] <> addr[i]
        SER.str(String("- NOK. rx pipe0 Should: "))
        ser.hex(addr[i],2)
        SER.str(String(" "))
        SER.str(String(SER#CR)) 
      else
        SER.str(String("-ok "))
        
    'pipe 1
    raddr[0] := $15
    raddr[1] := $25
    raddr[2] := $35
    raddr[3] := $45
    raddr[4] := $55

    HalRadio.set_RX_Addr(1, @addr)       'uses SPI.Read_Write_Five_Octet_Data(Register, WriteDataPtr, ReadDataPtr)

    HalRadio.get_RX_Addr(1, @raddr)

    repeat i from 0 to 4
      ser.hex(addr[i],2)
      if raddr[i] <> addr[i]
        SER.str(String("- NOK. rx pipe1 Should: "))
        ser.hex(raddr[i],2)
        SER.str(String(" "))
        SER.str(String(SER#CR)) 
      else
        SER.str(String(" ok. "))
    SER.str(String(SER#CR)) 

    'pipe 2 - 5
    repeat pipe from 2 to 5
      c := pipe
      HalRadio.set_RX_Addr(pipe, @c)
      
    repeat pipe from 2 to 5
      c := $be
      HalRadio.get_RX_Addr(pipe, @c)
      ser.hex(c,2)
      if c <> pipe
        SER.str(String("' NOK. Should: '"))
        ser.hex(pipe,2)
        SER.str(String(SER#CR)) 
      else
        SER.str(String(" ok. "))
   
    SER.str(String(SER#CR)) 

    SER.str(String("Write Register Adress value TX_ADDR ...'"))
    addr[0] := $1A
    addr[1] := $2A
    addr[2] := $3A
    addr[3] := $4A
    addr[4] := $5A
    raddr[0] := $51
    raddr[1] := $52
    raddr[2] := $53
    raddr[3] := $54
    raddr[4] := $55
    'pipe 0
    HalRadio.set_TX_Addr(@addr)       'uses SPI.Read_Write_Five_Octet_Data(Register, WriteDataPtr, ReadDataPtr)

    HalRadio.get_TX_Addr(@raddr)

    repeat i from 0 to 4
      ser.hex(raddr[i],2)
      if raddr[i] <> addr[i]
        SER.str(String("' NOK. Should: '"))
        ser.hex(addr[i],2)
    SER.str(String(SER#CR)) 


    SER.str(String("Set / clear Register bit FEATURE / EN_ACK_PAY ... '"))
    HalRadio.enable_ack_pl

    val := $EE
    HalRadio.get_Other_reg($1d, @val)
    ser.hex(val,2)
    
    if val == $EE or val == 0 or val == 255    
      SER.str(String("' NOK."))
      'ser.dec($EE)
      SER.str(String(SER#CR)) 
    else
      SER.str(String("' ok. '")) 

    SER.str(String(SER#CR)) 

    HalRadio.disable_ack_pl


    val := $EE
    HalRadio.get_Other_reg($1d, @val)
    ser.hex(val,2)
    
    if val == $EE or val == $02 or val == 255    
      SER.str(String("' NOK."))
    else
      SER.str(String("' ok.")) 

    SER.str(String(SER#CR)) 

    'Initialize Nordic nRF24L01
    'Init( SPI_MISO, SPI_MOSI, SPI_SCK, SPI_CSN, SPI_CE)
    waitcnt(clkfreq/20 + cnt)
    Carrier := 0
    carrier := get_RF_Status
    SER.bin(carrier,32)
    SER.str(String(SER#CR)) 
     
    SER.str(String("set_power_mode(0)"))
    c := HalRadio.set_power_mode(0)
    SER.bin(c,32)
    SER.str(String(SER#CR))
     
    SER.str(String("set_power_mode(1)"))
    c := HalRadio.set_power_mode(1)
    SER.bin(c,32)
    SER.str(String(SER#CR)) 
     
    SER.str(String("set_power_mode(0)"))
    c := HalRadio.set_power_mode(0)
    SER.bin(c,32)
    SER.str(String(SER#CR)) 
     
    repeat 3
      SER.str(String("... done.")) 
      SER.str(String(SER#CR)) 


pub checkPtrArry(valPtr)

  byte[valPtr][1] := 32 ' schreib einen wert
  checkval(@byte[valptr][1])   ' und schau ob wieder ein andere geschrieben wird.

pub checkPtrval(valPtr)

  long[valPtr] := 23 ' schreib einen wert
  checkval(valPtr)   ' und schau ob wieder ein andere geschrieben wird.

pub checkval(valPtr)

  byte[valPtr] := byte[valPtr] + 2
      
' **************************************************** lib ********************************************************+++  
PUB init(MOSI, MISO, SCK, CSN, CE)
  
  HalRadio.radio_init(MOSI, MISO, SCK, CSN, CE)
  

PUB get_status | status  'radio_status
'Get the current status of the radio.

  return status
  
PUB set_status(new_status)
'Sets the status of the radio. Input parameter is checked to see if it is allowed.
  Radio_status := new_status


PUB get_RF_Status
return HalRadio.get_rf_status

PUB get_channel(ChannelPtr)
  HalRadio.get_rf_channel(ChannelPtr)

PUB set_channel(Channel)
  HalRadio.set_rf_channel(Channel)


PUB is_channel_busy(channel) : pd
' set channel and wait min. 40us

  HalRadio.set_rf_channel(channel)              ' // Frequenzy =  2400 + CHANNEL
  
  HalRadio.set_rf_receive(1)   ' enable receiver
  HalRadio.set_power_mode(1)'HAL_NRF_PWR_UP)       ' // Power up device

  pd := HalRadio.get_receive_power_detected
  
  HalRadio.set_power_mode(0)
  'radio_set_status (RF_IDLE);                    // Radio now ready
  HalRadio.set_rf_receive(0)   ' disable receiver
  
  return pd 
 
PUB radio_irq
'This function reads the interrupts. It does the work
' * of a interrupt handler by manually reading the interrupt
' * flags and act on them. Sets the status with \
{{  if (RADIO_ACTIVITY())                         // Check if an interupt is
                                               // triggered
    switch(hal_nrf_get_clear_irq_flags ())
    {
      case (1<<HAL_NRF_MAX_RT):                 // Max retries reached
        hal_nrf_flush_tx();                     // flush tx fifo, avoid fifo jam
        radio_set_status (RF_MAX_RT);
        break;
      
      case (1<<HAL_NRF_TX_DS):                  // Packet sent
        radio_set_status (RF_TX_DS);
        break;
      
      case (1<<HAL_NRF_RX_DR):                  // Packet received
        while (!hal_nrf_rx_fifo_empty ())
        {
          hal_nrf_read_rx_pload(pload);
        }
        radio_set_status (RF_RX_DR);
        break;
  
      case ((1<<HAL_NRF_RX_DR)|(1<<HAL_NRF_TX_DS)): // Ack payload recieved
        while (!hal_nrf_rx_fifo_empty ())
        {
          hal_nrf_read_rx_pload(pload);
        }
        radio_set_status (RF_TX_AP);
        break;
  
      default:
        break;    
    }

    RESET_RADIO_ACTIVITY()
}}    
PUB radio_get_pload_byte (byte_index)'uinit8
'Gets the bit at position @a byte_index in @b pload.
'  return pload[byte_index]
  
PUB radio_send_packet(packet, length)
'This function load the data to be sent into the radio, sends it, and waits for the response.
'  hal_nrf_write_tx_pload(packet, lenght)
'  CE_PULSE()
'  radio_set_status(RF_BUSY)

PUB radio_init_sb(address, hal_nrf_operation_mode, operational_mode)

PUB radio_init_esb(address, hal_nrf_operation_mode, operational_mode)

{{/** @ingroup PL 
 * @file
 * Initialise the radio in Enhanced ShockBurst mode with Bidirectional data. 
 * This is done by opening @b pipe0 with auto ACK and with auto retransmits. 
 * It also opens for the use of ACK payload (@b hal_nrf_enable_ack_pl()) and 
 * dynamic payload width (@b hal_nrf_enable_dynamic_pl() for general enabeling 
 * and @b hal_nrf_setup_dyn_pl() to enable on specific pipes).
 *
 * @author Per Kristian Schanke
 */ }}
PUB radio_pl_init(address, hal_nrf_operation_mode, operational_mode)
{{
#include "hal_nrf.h"
#include "radio_pl.h"
#include "system.h"
#include "radio.h"

/** For turning on dynamic payload on all pipes. Sets bits 0-6 */
#define ALL_PIPES (0x3F)

/** Initializes the radio in Enhanced ShockBurst mode with ACK payload. This mean that we 
 * enable auto-retransmit and auto-acknowledgment as in Enhanced ShockBurst, and the 
 * features auto-ack payload and dynamic payload width.
 *
 * @param address The radios working address
 * @param operational_mode The operational mode, either @c HAL_NRF_PRX or @c HAL_NRF_PTX
 */
 
void radio_pl_init (const uint8_t *address, hal_nrf_operation_mode_t operational_mode)
{
  hal_nrf_close_pipe(HAL_NRF_ALL);               // First close all radio pipes
                                                 // Pipe 0 and 1 open by default
  hal_nrf_open_pipe(HAL_NRF_PIPE0, true);        // Then open pipe0, w/autoack 

  hal_nrf_set_crc_mode(HAL_NRF_CRC_16BIT);       // Operates in 16bits CRC mode
  hal_nrf_set_auto_retr(RF_RETRANSMITS, RF_RETRANS_DELAY);
                                                 // Enables auto retransmit.
                                                 // 3 retrans with 250ms delay

  hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);  // 5 bytes address width
  hal_nrf_set_address(HAL_NRF_TX, address);      // Set device's addresses
  hal_nrf_set_address(HAL_NRF_PIPE0, address);   // Sets recieving address on 
                                                 // pipe0

/*****************************************************************************
 * Changed from esb/radio_esb.c                                              *
 * Enables:                                                                  *
 *  - ACK payload                                                            *
 *  - Dynamic payload width                                                  *
 *  - Dynamic ACK                                                            *
 *****************************************************************************/
  hal_nrf_enable_ack_pl();                       // Try to enable ack payload

  // When the features are locked, the FEATURE and DYNPD are read out 0x00
  // even after we have tried to enable ack payload. This mean that we need to
  // activate the features.
  if(hal_nrf_read_reg(FEATURE) == 0x00 && (hal_nrf_read_reg(DYNPD) == 0x00))
  {
    hal_nrf_lock_unlock ();                      // Activate features
    hal_nrf_enable_ack_pl();                     // Enables payload in ack
  }

  hal_nrf_enable_dynamic_pl();                   // Enables dynamic payload
  hal_nrf_setup_dyn_pl(ALL_PIPES);               // Sets up dynamic payload on
                                                 // all data pipes.
/*****************************************************************************
 * End changes from esb/radio_esb.c                                          *
 *****************************************************************************/
   
  if(operational_mode == HAL_NRF_PTX)            // Mode depentant settings
  {
    hal_nrf_set_operation_mode(HAL_NRF_PTX);     // Enter TX mode
  }
  else
  {
    hal_nrf_set_operation_mode(HAL_NRF_PRX);     // Enter RX mode
    hal_nrf_set_rx_pload_width((uint8_t)HAL_NRF_PIPE0, RF_PAYLOAD_LENGTH);
                                                 // Pipe0 expect 
                                                 // PAYLOAD_LENGTH byte payload
                                                 // PAYLOAD_LENGTH in radio.h
  }

  hal_nrf_set_rf_channel(RF_CHANNEL);            // Operating on static channel
                                                 // Defined in radio.h. 
                                                 // Frequenzy = 
                                                 //        2400 + RF_CHANNEL
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);        // Power up device
  
  start_timer(RF_POWER_UP_DELAY);                // Wait for the radio to 
  wait_for_timer();                              // power up
  
  radio_set_status (RF_IDLE);                    // Radio now ready
}

)
}}

 