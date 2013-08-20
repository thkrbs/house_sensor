PUB set_irq_mode( int_source,  irq_state)
{{Enable or disable interrupt for radio.
 * Use this function to enable or disable
 * one of the interrupt sources for the radio.
 * This function only changes state for selected
 * int_type, the rest of the interrupt sources
 * are left unchanged.
 *
 * @param int_source Radio interrupt Source.
 * @param irq_state Enable or Disable.
 }}
  if(irq_state)
    write_reg(NRF.CONFIG, read_reg(CONFIG) & ~SET_BIT(int_source))
  else
    write_reg(CONFIG, read_reg(CONFIG) | SET_BIT(int_source))

PUB lock_unlock(void)
  {{  Activate features
  * Sends the ACTIVATE command to the RF tranceiver. By calling this function
  * once it is possible to use the functions related to the FEATURE register.
  }}
  CSN_LOW()

  rw(LOCK_UNLOCK)             
  rw(0x73) 

  CSN_HIGH()
PUB enable_ack_pl(void)
  '' Enables the ACK payload feature
  write_reg(FEATURE, (read_reg(FEATURE) | 0x02))

PUB disable_ack_pl(void)
  ''Disables the ACK payload featur
   write_reg(FEATURE, (read_reg(FEATURE) & ~0x02))

PUB enable_dynamic_pl(void)
  {{ Enables the dynamic payload feature }}
  write_reg(FEATURE, (read_reg(FEATURE) | 0x04))
   
PUB disable_dynamic_pl(void)
  {{ Disables the dynamic payload feature }}
  write_reg(FEATURE, (read_reg(FEATURE) & ~0x04))
  
PUB setup_dyn_pl( setup)
{{ Sets the dynamic payload features for the RX pipes
* The input parameter contains is a byte where the bit values tells weather the
*  pipe uses the ack payload feature or not. For example if bit 0 is set then 
* Pipe 0 uses ack payload,
 * @param setup Byte value with bit set for pipes that uses the dynamic payload feature
}}
  write_reg(DYNPD, setup & ~0xC0)
  
PUB read_rx_pl_w(void) : uint8
  {{ Reads the payload width of the received ack payload
  * @return Payload width of the received ack payload
  }}
  uint8_t temp 
  
  CSN_LOW()

  rw(RD_RX_PLOAD_W)
  temp = rw(0)
  CSN_HIGH() 

  return temp
  
PUB write_ack_pload(uint8_t pipe, uint8_t *tx_pload, uint8_t length)
  {{ Writes the payload that will be transmitted with the ack on the given pipe.
  * @param pipe Pipe that transmits the payload
  * @param tx_pload Pointer to the payload data
  * @param length Size of the data to transmit }}
  CSN_LOW()

  rw(WR_ACK_PLOAD | pipe)
  while(length--)
  {
    rw(*tx_pload++);
  }

  CSN_HIGH()
  
PUB enable_dynamic_ack(void)
  {{ Enables the no-ack feature }}
  write_reg(FEATURE, (read_reg(FEATURE) | 0x01))

  
PUB disable_dynamic_ack(void)
{{ Disables the no-ack feature
}}
  write_reg(FEATURE, (read_reg(FEATURE) & ~0x01))
  
PUB get_clear_irq_flags(void)  : uint8
{{ Read then clears all interrupt flags.
 * Use this function to get the interrupt flags and clear them in the same operation.
 * Reduced radio interface activity and speed optimized.
 *
 * @return  Interrupt_flags
 * @retval 0x10 Max Retransmit interrupt
 * @retval 0x20 TX Data sent interrupt
 * @retval 0x40 RX Data received interrupt
}}
  write_reg(STATUS, (BIT_6|BIT_5|BIT_4)) & (BIT_6|BIT_5|BIT_4)

  
PUB clear_irq_flag(irq_source_t int_source)
{{ Clear one selected interrupt flag. Use this function to clear one @a spesific interrupt flag. Other interrupt flags are left unchanged.
 *
 * @param int_source Interrupt source of which flag to clear
}}
  write_reg(STATUS, SET_BIT(int_source))

PUB set_crc_mode(crc_mode_t crc_mode)
{{ Set the CRC mode used by the radio.
 * Use this function to set the CRC mode; CRC disabled, 1 or 2 bytes.
 *
 * @param crc_mode CRC mode to use
}}
  write_reg(CONFIG, (read_reg(CONFIG) & ~(BIT_3|BIT_2)) | (UINT8(crc_mode)<<2))

PUB open_pipe(address_t pipe_num, bool auto_ack)
{{ Open radio pipe(s) and enable/ disable auto acknowledge.
 * Use this function to open one or all pipes,
 * with or without auto acknowledge.
 *
 * @param pipe_num Radio pipe to open
 * @param auto_ack Auto_Ack ON/OFF
 * @see address
}}
  switch(pipe_num)
  {
    case PIPE0:
    case PIPE1:
    case PIPE2:
    case PIPE3:
    case PIPE4:
    case PIPE5:
      write_reg(EN_RXADDR, read_reg(EN_RXADDR) | SET_BIT(pipe_num));

      if(auto_ack)
        write_reg(EN_AA, read_reg(EN_AA) | SET_BIT(pipe_num));
      else
        write_reg(EN_AA, read_reg(EN_AA) & ~SET_BIT(pipe_num));
      break;

    case ALL:
      write_reg(EN_RXADDR, ~(BIT_7|BIT_6));

      if(auto_ack)
        write_reg(EN_AA, ~(BIT_7|BIT_6));
      else
        write_reg(EN_AA, 0);
      break;
      
    default:
      break;
  }
  
PUB close_pipe(address_t pipe_num)
{{ Close radio pipe(s).
 * Use this function to close one pipe or all pipes.
 *
 * @param pipe_num Pipe# number to close
}}
  switch(pipe_num)
  {
    case PIPE0:
    case PIPE1:
    case PIPE2:
    case PIPE3:
    case PIPE4:
    case PIPE5:
      write_reg(EN_RXADDR, read_reg(EN_RXADDR) & ~SET_BIT(pipe_num));
      write_reg(EN_AA, read_reg(EN_AA) & ~SET_BIT(pipe_num));
      break;
    
    case ALL:
      write_reg(EN_RXADDR, 0);
      write_reg(EN_AA, 0);
      break;
      
    default:
      break;
  }
  
PUB set_address(address_t address, uint8_t *addr)
{{ Set radio's RX address and TX address.
 * Use this function to set a RX address, or to set the TX address.
 * Beware of the difference for single and multibyte address registers.
 *
 * @param address Which address to set
 * @param *addr Buffer from which the address is stored in
}}
  switch(address)
  {
    case TX:
    case PIPE0:
    case PIPE1:
      write_multibyte_reg((uint8_t) address, addr, 0);
      break;

    case PIPE2:
    case PIPE3:
    case PIPE4:
    case PIPE5:
      write_reg(RX_ADDR_P0 + (uint8_t) address, *addr);
      break;

    default:
      break;
  }
  
PUB set_auto_retr(uint8_t retr, uint16_t delay)
{{ Set auto acknowledge parameters.
 * Use this function to set retransmit and retransmit delay
 * parameters.
 *
 * @param retr Number of retransmit, 0 equ retransmit OFF
 * @param delay Retransmit delay in µs
}}
  write_reg(SETUP_RETR, (((delay/250)-1)<<4) | retr)
  
PUB set_address_width(address_width_t address_width)
{{ Set radio's address width.
 * Use this function to define the radio's address width,
 * referes to both RX and TX.
 *
 * @param address_width Address with in bytes
}}
  write_reg(SETUP_AW, (UINT8(address_width) - 2))
  
PUB set_rx_pload_width(uint8_t pipe_num, uint8_t pload_width)
{{ Set payload width for selected pipe.
 * Use this function to set the number of bytes expected
 * on a selected pipe.
 *
 * @param pipe_num Pipe number to set payload width for
 * @param pload_width number of bytes expected
}}
   write_reg(RX_PW_P0 + pipe_num, pload_width)

   
PUB get_irq_mode(uint8_t int_type) : bool
{{ Read current interrupt mode for selected interrupt source (type).
 * Use this function to get the interrupt source's mode,
 * either enabled or disabled.
 *
 * @param int_source Interrupt source to get mode from
 *
 * @return Interrupt Mode
 * @retval FALSE Interrupt disabled
 * @retval TRUE Interrupt enabled
}}
  if(read_reg(CONFIG) & SET_BIT(int_type))
    return false
  else
    return true
    
PUB  get_irq_flags(void) : uint8
{{ Read all interrupt flags.
 * Use this function to get the interrupt flags. This function is similar
 * to get_clear_irq_flags with the exception that it does <I><B>NOT</B></I> clear
 * the irq_flags.
 *
 * @return Interrupt_flags
 * @retval 0x10 Max Retransmit interrupt
 * @retval 0x20 TX Data sent interrupt
 * @retval 0x40 RX Data received interrupt
}}
  return nop() & (BIT_6|BIT_5|BIT_4)
  
PUB get_crc_mode(void) : uint8
{{  Get CRC mode.
 * Use this function to check which CRC mode is used.
 *
 * @return CRC_mode
 * @retval 0x00 CRC_OFF
 * @retval 0x02 CRC_8BIT
 * @retval 0x03 CRC_16BIT
}}
  return (read_reg(CONFIG) & (BIT_3|BIT_2)) >> CRCO

  
PUB get_pipe_status(uint8_t pipe_num) : uint8
{{ Get pipe status.
 * Use this function to check status for a selected pipe.
 *
 * @param  pipe_num Pipe number to check status for
 *
 * @return Pipe_Status
 * @retval 0x00 Pipe is closed, autoack disabled
 * @retval 0x01 Pipe is open, autoack disabled
 * @retval 0x03 Pipe is open, autoack enabled
}}
  uint8_t en_rx, en_aa

  en_rx = read_reg(EN_RXADDR) & (1<<pipe_num)
  en_aa = read_reg(EN_AA) & (1<<pipe_num)

  en_rx >>= pipe_num
  en_aa >>= pipe_num

  return (en_aa << 1) + en_rx
  
PUB get_address(uint8_t address, uint8_t *addr) : uint8
{{ Get address for selected pipe.
 * Use this function to get address for selected pipe.
 *
 *
 * @param address Which address to get, Pipe- or TX-address
 * @param *addr buffer in which address bytes are written.
 * <BR><BR>For pipes containing only LSB byte of address, this byte is returned
 * in the<BR> *addr buffer.
 *
 * @return Address_Width in bytes
}}
  switch(address)
  {
    case PIPE0:
    case PIPE1:
    case TX:
      return read_multibyte_reg(address, addr);

    default:
      *addr = read_reg(RX_ADDR_P0 + address);
      return get_address_width();
  }
  
PUB get_auto_retr_status(void) : uint8 
{{ Get auto retransmit parameters.
 * Use this function to get the auto retransmit parameters,
 * retrans count and retrans delay.
 *
 * @return AutoRetrans Parameters
 *
 * @retval UpperNibble Retransmit Delay
 * @retval LowerNibble Retransmit Count
}}
  return read_reg(OBSERVE_TX)
  
PUB get_packet_lost_ctr(void) : uint8
{{ Get packet lost counter
 * Use this function to get the packet(s) counter.
 *
 * @return packet lost counter
}}
  return (read_reg(OBSERVE_TX) & (BIT_7|BIT_6|BIT_5|BIT_4)) >> 4
  
PUB get_address_width(void) : uint8 
{{ Get address width for radio.
 * Use this function to get the address width used for
 * the radio, both RX and TX.
 *
 * @return Address_Width in bytes
}}
  return (read_reg(SETUP_AW) + 2)

  
PUB get_rx_pload_width(uint8_t pipe_num) : uint8 
{{ Get RX payload width for selected pipe.
 * Use this function to get the expected payload
 * width for selected ppe number.
 *
 * @param pipe_num Pipe number to get payload width for
 *
 * @return Payload_Width in bytes
}}
  return read_reg(RX_PW_P0 + pipe_num)

  
PUB set_operation_mode(operation_mode_t op_mode)
{{ Set radio's operation mode.
 * Use this function to enter PTX (primary TX)
 * or PRX (primary RX).
 *
 * @param op_mode Operation mode
}}
  if(op_mode == PRX)
    write_reg(CONFIG, (read_reg(CONFIG) | (1<<PRIM_RX)))
  else
    write_reg(CONFIG, (read_reg(CONFIG) & ~(1<<PRIM_RX)))
  

  

PUB set_datarate(datarate_t datarate)
{{ Set radio's on-air datarate.
 * Use this function to select radio's on-air
 * datarate.
 *
 * @param datarate On-air datarat
}}
  if(datarate == 1MBPS)
    write_reg(RF_SETUP, (read_reg(RF_SETUP) & ~(1<<RF_DR)))
  else
    write_reg(RF_SETUP, (read_reg(RF_SETUP) | (1<<RF_DR)))
  
PUB get_operation_mode(void) : uint8
{{ Get radio's current operation mode.
 * Use this function to get the radio's current
 * operation mode, PTX or PRX.
 *
 * @return Operation_Mode
 * @retval 0x00 Primary RX (PRX)
 * @retval 0x01 Primary TX (PTX)
}}
  return (read_reg(CONFIG) & (1<<PRIM_RX)) >> PRIM_RX
  
PUB get_power_mode(void) : uint8
{{ Get radio's current power mode.
 * Use this function to get the radio's currnet
 * power mode, POWER_UP or POWER_DOWN.
 *
 * @return Power_Mode
 * @retval 0x00 POWER_DOWN
 * @retval 0x01 POWER_UP
}}
  return (read_reg(CONFIG) & (1<<PWR_UP)) >> PWR_UP
  
PUB get_rf_channel(void) : uint8 
{{ Get radio's current RF channel.
 * Use this function to get the radio's current
 * selected RF channel
 *
 * @return RF channel
}}
  return read_reg(RF_CH)
  
PUB get_output_power(void) : uint8
{{ Get radio's current TX output power.
 * Use this function to get the radio's current
 * TX output power setting.
 *
 * @return TX_power_output
 * @retval 0x00 -18dBm
 * @retval 0x01 -12dBm
 * @retval 0x02 -6dBm
 * @retval 0x03 0dBm
}}
  return (read_reg(RF_SETUP) & ((1<<RF_PWR1)|(1<<RF_PWR0))) >> RF_PWR0
  
PUB get_datarate(void) : uint8
{{ Get radio's current on-air datarate.
 * Use this function to get the radio's current
 * on-air datarate setting.
 *
 * @return On-air datarate
 * @retval 0x00 1Mbps selected
 * @retval 0x01 2Mbps selected
}}
  return (read_reg(RF_SETUP) & (1<<RF_DR)) >> RF_DR
  
PUB get_tx_fifo_status(void)  : uint8 
{{ Get radio's TX FIFO status.
 * Use this function to get the radio's TX
 * FIFO status.
 *
 * @return TX FIFO status
 * @retval 0x00 TX FIFO NOT empty, but NOT full
 * @retval 0x01 FIFO empty
 * @retval 0x02 FIFO full
}}
  return ((read_reg(FIFO_STATUS) & ((1<<TX_FIFO_FULL)|(1<<TX_EMPTY))) >> 4)
  
PUB tx_fifo_empty(void) : bool
{{ Check for TX FIFO empty.
 * Use this function to check if TX FIFO
 * is empty.
 *
 * @return TX FIFO empty bit
 * @retval FALSE TX FIFO NOT empty
 * @retval TRUE TX FIFO empty
}}
  return (bool)((read_reg(FIFO_STATUS) >> TX_EMPTY) & 1)
  
PUB tx_fifo_full(void)  : bool
{{ Check for TX FIFO full.
 * Use this function to check if TX FIFO
 * is full.
 *
 * @return TX FIFO full bit
 * @retval FALSE TX FIFO NOT full
 * @retval TRUE TX FIFO full
}}
  return (bool)((read_reg(FIFO_STATUS) >> TX_FIFO_FULL) & 1)
  
PUB get_rx_fifo_status(void) : uint8
{{ Get radio's RX FIFO status.
 * Use this function to get the radio's TX
 * FIFO status.
 *
 * @return RX FIFO status
 * @retval 0x00 RX FIFO NOT empty, but NOT full
 * @retval 0x01 RX FIFO empty
 * @retval 0x02 RX FIFO full
}}
  return (read_reg(FIFO_STATUS) & ((1<<RX_FULL)|(1<<RX_EMPTY)))
  
PUB rx_fifo_empty(void) : bool
{{ Check for RX FIFO empty.
 * Use this function to check if RX FIFO
 * is empty.
 *
 * Reads STATUS register to check this, not FIFO_STATUS  
 *
 * @return RX FIFO empty bit
 * @retval FALSE RX FIFO NOT empty
 * @retval TRUE RX FIFO empty
}}
  if(get_rx_data_source()==7)
    return true
  else
    return false
  
PUB rx_fifo_full(void) : bool
{{ Check for RX FIFO full.
 * Use this function to check if RX FIFO
 * is full.
 *
 * @return RX FIFO full bit
 * @retval FALSE RX FIFO NOT full
 * @retval TRUE RX FIFO full
}}
  return (bool)((read_reg(FIFO_STATUS) >> RX_EMPTY) & 1)

  
PUB get_transmit_attempts(void) : uint8
{{ Get radio's transmit attempts status.
 * Use this function to get number of retransmit
 * attempts and number of packet lost.
 *
 * @return Retransmit attempts counters
}}
  return read_reg(OBSERVE_TX) & (BIT_3|BIT_2|BIT_1|BIT_0)
  
  
PUB get_rx_data_source(void) : uint8 
{{ Get RX data source.
 * Use this function to read which RX pipe data
 * was received on for current top level FIFO data packet.
 *
 * @return pipe number of current packet present
}}
  return ((nop() & (BIT_3|BIT_2|BIT_1)) >> 1)
  ''// Fixed: returns length==0 and pipe==7 means FIFO empty

  
PUB read_rx_pload(uint8_t *rx_pload) : uint16
{{ Read RX payload.
 * Use this function to read top level payload
 * available in the RX FIFO.
 *
 * @param  *rx_pload pointer to buffer in which RX payload are stored
 * @return pipe number (MSB byte) and packet length (LSB byte)
}}
   return read_multibyte_reg(UINT8(RX_PLOAD), rx_pload)
   
PUB write_tx_pload(uint8_t *tx_pload, uint8_t length)
{{  Write TX payload to radio.
 * Use this function to write a packet of
 * TX payload into the radio.
 * <I>length</I> number of bytes, which are stored in <I>*tx_pload</I>.
 *
 * @param *tx_pload pointer to buffer in which TX payload are present
 * @param length number of bytes to write
}}
  write_multibyte_reg(UINT8(TX_PLOAD), tx_pload, length)
  
PUB reuse_tx(void)
{{ Reuse TX payload.
 * Use this function to set that the radio is using
 * the last transmitted payload for the next packet as well
}}
  write_reg(REUSE_TX_PL, 0)
  
PUB get_reuse_tx_status(void) : bool
{{ Get status of reuse TX function.
 * Use this function to check if reuse TX payload is
 * activated
 *
 * @return Reuse TX payload mode
 * @retval FALSE Not activated
 * @retval TRUE Activated
}}
  return (bool)((get_fifo_status() & (1<<TX_REUSE)) >> TX_REUSE)
  
PUB flush_rx(void)
{{ Flush RX FIFO. Use this function to flush the radio's RX FIFO }}
  write_reg(FLUSH_RX, 0)
  
PUB flush_tx(void)
{{ Flush TX FIFO.Use this function to flush the radio's  TX FIFO  }}
  write_reg(FLUSH_TX, 0)
  
PUB nop(void) : uint8
{{ No Operation command.
 * Use this function to receive the radio's
 * status register.
 *
 * @return Status register
}}
  return write_reg(NOP,0)
  
PUB set_pll_mode(pll_mode_t pll_mode)
{{ Set radio's PLL mode.
 * Use this function to either LOCK
 * or UNLOCK the radio's PLL.
 *
 * @param pll_mode PLL locked, TRUE or FALSE
}}
  if(pll_mode == PLL_LOCK)

    write_reg(RF_SETUP, (read_reg(RF_SETUP) | (1<<PLL_LOCK)))

  else

    write_reg(RF_SETUP, (read_reg(RF_SETUP) & ~(1<<PLL_LOCK)))

  
PUB get_pll_mode(void) : pll_mode
{{ Get PLL mode.
 * Use this function to get the radio's
 * current PLL mode.
 *
 * @return PLL_mode
}}
  return (pll_mode_t)((read_reg(RF_SETUP) & (1<<PLL_LOCK)) >> PLL_LOCK)
  
PUB set_lna_gain(lna_mode_t lna_gain)
{{ Set radio's LNA gain mode.
 * Use this function to either use HI
 * current or LOW current mode for the radio.
 *
 * @param lna_gain LNA gain mode
}}
  if(lna_gain == LNA_HCURR)
    write_reg(RF_SETUP, (read_reg(RF_SETUP) | (1<<LNA_HCURR)))
  else
    write_reg(RF_SETUP, (read_reg(RF_SETUP) & ~(1<<LNA_HCURR)))
  
PUB get_lna_gain(void) : lna_mode 
{{ Get LNA gain mode.
 * Use this function to get the radio's
 * current LNA gain mode.
 *
 * @return LNA gain mode
 * @retval 0 LNA LOW current
 * @retval 1 LNA HI current
}}
  return (lna_mode_t) ( (read_reg(RF_SETUP) & (1<<LNA_HCURR)) >> LNA_HCURR )
  
PUB read_reg(uint8_t reg) : uint8
{{ Basis function read_reg.
 * Use this function to read the contents
 * of one radios register.
 *
 * @param reg Register to read
 * @return Register contents
}}
  uint8_t temp
  CSN_LOW()
  rw(reg)
  temp = rw(0)
  CSN_HIGH()

  return temp


PUB write_reg(uint8_t reg, uint8_t value) : uint8
{{ Basis function write_reg.
 * Use this function to write a new value to
 * a radio register.
 *
 * @param reg Register to write
 * @param value New value to write
 * @return Status register
}}
  uint8_t retval
  CSN_LOW()
  if(reg < WRITE_REG)   // i.e. this is a register access
  {
    retval = rw(WRITE_REG + reg);
    rw(value);
  }
  else            // single byte cmd OR future command/register access
  {
    if(!(reg == FLUSH_TX) && !(reg == FLUSH_RX) && !(reg == REUSE_TX_PL) && !(reg == NOP))
    {
      retval = rw(reg);
      rw(value);
    }
    else          // single byte L01 command
    {
      retval = rw(reg);
    }
  }
  CSN_HIGH()

  return retval
  
PUB read_multibyte_reg(uint8_t reg, uint8_t *pbuf) : unit16
{{ Basis function, read_multibyte register .
 * Use this function to read multiple bytes from
 * a multibyte radio-register
 *
 * @param reg Multibyte register to read from
 * @param *pbuf Pointer to buffer in which to store read bytes to
 *
 * @return pipe# of received data (MSB), if operation used by a read_rx_pload
 * @return length of read data (LSB), either for read_rx_pload or
 * for get_address.
}}
  uint8_t ctr, length
  switch(reg)
  {
    case PIPE0:
    case PIPE1:
    case TX:
      length = ctr = get_address_width();
      CSN_LOW();
      rw(RX_ADDR_P0 + reg);
      break;
      
    case RX_PLOAD:
      if( (reg = get_rx_data_source()) < 7)
      {
        length = ctr = read_rx_pl_w();

        CSN_LOW();
        rw(RD_RX_PLOAD);
      }
      else
      {
       ctr = length = 0;
      }
      break;

    default:
      ctr = length = 0;
      break;
  }

  while(ctr--)
  {
    *pbuf++ = rw(0);
  }

  CSN_HIGH()

  return (((uint16_t) reg << 8) | length)
  
PUB write_multibyte_reg(uint8_t reg, uint8_t *pbuf, uint8_t length)
{{ Basis function, write_multibyte register.
 * Use this function to write multiple bytes to
 * a multiple radio register.
 *
 * @param reg Register to write
 * @param *pbuf pointer to buffer in which data to write is
 * @param length \# of bytes to write 
}}
  switch(reg)
  {
    case PIPE0:
    case PIPE1:
    case TX:
      length = get_address_width();
      CSN_LOW();
      rw(WRITE_REG + RX_ADDR_P0 + reg);
      break;
      
    case TX_PLOAD:
      CSN_LOW();
      rw(WR_TX_PLOAD);
      break;      
    default:
      break;
  }

  while(length--)
  {
    rw(*pbuf++);
  }

  CSN_HIGH()
 