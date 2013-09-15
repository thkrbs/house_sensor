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

'Get the current status of the radio.
PUB radio_get_status()  'radio_status
  return status
  
'Sets the status of the radio. Input parameter is checked to see if it is allowed.
PUB radio_set_status(new_status)
  status = new_status
  
'This function reads the interrupts. It does the work
' * of a interrupt handler by manually reading the interrupt
' * flags and act on them. Sets the status with \
PUB radio_irq(void)
  if (RADIO_ACTIVITY())                         // Check if an interupt is
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
    
'Gets the bit at position @a byte_index in @b pload.
PUB radio_get_pload_byte (byte_index)'uinit8
  return pload[byte_index]
  
'This function load the data to be sent into the radio, sends it, and waits for the response.
PUB radio_send_packet(packet, length)
  hal_nrf_write_tx_pload(packet, lenght)
  CE_PULSE()
  radio_set_status(RF_BUSY)

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