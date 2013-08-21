''Inspired by Nikita Kareev (via OBEX) and Maniacbug (via  github.com/maniacbug/RF24Network.git)
'' Nordic nRF24L01 driver http://www.sparkfun.com/commerce/product_info.php?products_id=691
'' Using the prototypes like from the nordic application note
   
''Using version 6 SEP 2009 from Nikita Kareev and adding interface from maniacbug  
''
'' 

{{
Target ist to use something like this:

struct RF24NetworkHeader
{
  uint16_t from_node; /**< Logical address where the message was generated */
  uint16_t to_node; /**< Logical address where the message is going */
  uint16_t id; /**< Sequential message ID, incremented every message */
  unsigned char type; /**< Type of the packet.  0-127 are user-defined types, 128-255 are reserved for system */
  unsigned char reserved; /**< Reserved for future use */

  static uint16_t next_id; /**< The message ID of the next message to be sent */

  /**
   * Default constructor
   *
   * Simply constructs a blank header
   */
  RF24NetworkHeader() {}

  /**
   * Send constructor
   *
   * Use this constructor to create a header and then send a message
   *
   * @code
   *  RF24NetworkHeader header(recipient_address,'t');
   *  network.write(header,&message,sizeof(message));
   * @endcode
   *
   * @param _to The logical node address where the message is going
   * @param _type The type of message which follows.  Only 0-127 are allowed for
   * user messages.
   */
  RF24NetworkHeader(uint16_t _to, unsigned char _type = 0): to_node(_to), id(next_id++), type(_type&0x7f) {}

  /**
   * Create debugging string
   *
   * Useful for debugging.  Dumps all members into a single string, using
   * internal static memory.  This memory will get overridden next time
   * you call the method.
   *
   * @return String representation of this object
   */
  const char* toString(void) const;
};

/**
 * Network Layer for RF24 Radios
 *
 * This class implements an OSI Network Layer using nRF24L01(+) radios driven
 * by RF24 library.
 */

class RF24Network
{
public:
  /** Construct the network  
   * @param _radio The underlying radio driver instance
   */
  RF24Network( RF24& _radio );

  /** Bring up the network
   * @warning Be sure to 'begin' the radio first.
   * @param _channel The RF channel to operate on
   * @param _node_address The logical address of this node
   */
  void begin(uint8_t _channel, uint16_t _node_address );
  
  /**Main layer loop
   *This function must be called regularly to keep the layer going.  This is where all
   * the action happens!
   */
  void update(void);

  /**Test whether there is a message available for this node
   * @return Whether there is a message available for this node
   */
  bool available(void);
 
  /**Reads the next available header without advancing to the next
   * incoming message.  Useful for doing a switch on the message type
   *If there is no message available, the header is not touched
   * @param[out] header The header (envelope) of the next message
   */
  void peek(RF24NetworkHeader& header);

  /**Read a message
   * @param[out] header The header (envelope) of this message
   * @param[out] message Pointer to memory where the message should be placed
   * @param maxlen The largest message size which can be held in @p message
   * @return The total number of bytes copied into @p message
   */
  size_t read(RF24NetworkHeader& header, void* message, size_t maxlen);
  
  /**Send a message
   * @param[in,out] header The header (envelope) of this message.  The critical
   * thing to fill in is the @p to_node field so we know where to send the
   * message.  It is then updated with the details of the actual header sent.
   * @param message Pointer to memory where the message is located 
   * @param len The size of the message 
   * @return Whether the message was successfully received 
   */
  bool write(RF24NetworkHeader& header,const void* message, size_t len);

protected:
  void open_pipes(void);
  uint16_t find_node( uint16_t current_node, uint16_t target_node );
  bool write(uint16_t);
  bool write_to_pipe( uint16_t node, uint8_t pipe );
  bool enqueue(void);

  bool is_direct_child( uint16_t node );
  bool is_descendant( uint16_t node );
  uint16_t direct_child_route_to( uint16_t node );
  uint8_t pipe_to_descendant( uint16_t node );
  void setup_address(void);

private:
  RF24& radio; /**< Underlying radio driver, provides link/physical layers */ 
  uint16_t node_address; /**< Logical node address of this unit, 1 .. UINT_MAX */
  const static int frame_size = 32; /**< How large is each frame over the air */ 
  uint8_t frame_buffer[frame_size]; /**< Space to put the frame that will be sent/received over the air */
  uint8_t frame_queue[5*frame_size]; /**< Space for a small set of frames that need to be delivered to the app layer */
  uint8_t* next_frame; /**< Pointer into the @p frame_queue where we should place the next received frame */

  uint16_t parent_node; /**< Our parent's node address */
  uint8_t parent_pipe; /**< The pipe our parent uses to listen to us */
  uint16_t node_mask; /**< The bits which contain signfificant node address information */
};

or

class Nrf24l {
        public:
                Nrf24l();

                void init();
                void config();
                void send(uint8_t *value);
                void setRADDR(uint8_t * adr);
                void setTADDR(uint8_t * adr);
                bool dataReady();
                bool isSending();
                bool rxFifoEmpty();
                bool txFifoEmpty();
                void getData(uint8_t * data);
                uint8_t getStatus();
                
                void transmitSync(uint8_t *dataout,uint8_t len);
                void transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len);
                void configRegister(uint8_t reg, uint8_t value);
                void readRegister(uint8_t reg, uint8_t * value, uint8_t len);
                void writeRegister(uint8_t reg, uint8_t * value, uint8_t len);
                void powerUpRx();
                void powerUpTx();
                void powerDown();
                
                void csnHi();
                void csnLow();

                void ceHi();
                void ceLow();
                void flushRx();

                /*In sending mode. */
                uint8_t PTX;

                /* CE Pin controls RX / TX, default 8.*/
                uint8_t cePin;

                /* CSN Pin Chip Select Not, default 7.*/
                uint8_t csnPin;

                /*Channel 0 - 127 or 0 - 84 in the US.*/
                uint8_t channel;

                /*Payload width in bytes default 16 max 32.*/
                uint8_t payload;
                
                /* Rate is RF_SETUP, 0x06 is 1Mbps, Max power*/
                uint8_t rfsetup;
                
                /*Spi interface (must extend spi).*/
                MirfSpiDriver *spi;
};

}}
 
CON

  'nRF24L01 constants:
  
  'Commands
  R_REGISTER = %0000_0000 '+ reg
  W_REGISTER = %0010_0000 '+ reg
  R_RX_PAYLOAD = %0110_0001
  W_TX_PAYLOAD = %1010_0000
  FLUSH_TX = %1110_0001
  FLUSH_RX = %1110_0010
  REUSE_TX_PL = %1110_0011
  R_RX_PL_WID = %0110_0000
  W_ACK_PAYLOAD = %1010_1000 '+ pipe
  W_TX_PAYLOAD_NOACK = %1011_0000
  NOOP = %1111_1111

  'Registers
  CONFIG = $00   '%0000_1000 / $08
  EN_AA = $01  '%0011_1111  / $3F
  EN_RXADDR = $02  '%0000_0011
  SETUP_AW = $03   '%0000_0011
  SETUP_RETR = $04  '%0000_0011
  RF_CH = $05   '%0000_0010
  RF_SETUP = $06  '%0000_1110
  STATUS = $07     '%0000_1110 / $E 
  OBSERVE_TX = $08  '%0000_0000
  RPD = $09   '%0000_0000 
  RX_ADDR_P0 = $0A '$E7E7E7E7E7
  RX_ADDR_P1 = $0B '$C2C2C2C2C2
  RX_ADDR_P2 = $0C  '$C3
  RX_ADDR_P3 = $0D  '$C4 
  RX_ADDR_P4 = $0E  '$C5 
  RX_ADDR_P5 = $0F  '$C6 
  TX_ADDR = $10    '$E7E7E7E7E7
  RX_PW_P0 = $11   '%0000_0000
  RX_PW_P1 = $12   '%0000_0000 
  RX_PW_P2 = $13   '%0000_0000 
  RX_PW_P3 = $14  '%0000_0000 
  RX_PW_P4 = $15  '%0000_0000 
  RX_PW_P5 = $16  '%0000_0000 
  FIFO_STATUS = $17 '%0001_0001 
  DYNPD = $1C   '%0000_0000
  FEATURE = $1D  '%0000_0000

  

OBJ 

  TIME  : "Clock"                                       'Clock

VAR

  'Pins:
  
  byte SPI_Sck
  byte SPI_Miso
  byte SPI_Mosi 
  byte SPI_Csn    
  byte SPI_Ce
  byte SPI_Irq   

PUB Init(sck, miso, mosi, csn, ce)

  SPI_Sck := sck
  SPI_Miso := miso
  SPI_Mosi := mosi 
  SPI_Csn := csn     
  SPI_Ce := ce

  'Initialize clock object
  TIME.Init(5_000_000)
  
  'Configure for receive
  ConfigureRX






{{
PUB ConfigureRX
  Configure nRF24L01 registers for receive mode.
  1. Data pipe 0 used
  2. RX Address is E7E7E7E7E7 
  3. Data rate is 1Mb (compatability mode) - no ESB
  4. No Auto Ack (compatability mode) - no ESB 
  Tested with Nordic FOB from Sparkfun
}}
  
  Low(SPI_Ce)
    
  'Set PRX, CRC enabled
  Low(SPI_Csn)
  SpiReadWrite($20)
  SpiReadWrite($39) 
  High(SPI_Csn)   
    
  'Disable auto-ack for all channels
  Low(SPI_Csn)      
  SpiReadWrite($21)
  SpiReadWrite($00)     
  High(SPI_Csn)    
    
  'Set address width = 5 bytes
  Low(SPI_Csn)   
  SpiReadWrite($23)
  SpiReadWrite($03)    
  High(SPI_Csn)    
   
  'Data rate = 1Mb
  Low(SPI_Csn)   
  SpiReadWrite($26)
  SpiReadWrite($07)    
  High(SPI_Csn)

  'Set 4 byte payload
  Low(SPI_Csn)   
  SpiReadWrite($31)
  SpiReadWrite($04)    
  High(SPI_Csn)    

  'Set channel 2
  Low(SPI_Csn)
  SpiReadWrite($25)
  SpiReadWrite($02)    
  High(SPI_Csn)     

  'Set pipe 0 address E7E7E7E7E7
  Low(SPI_Csn)
  SpiReadWrite($30)
  repeat 5
    SpiReadWrite($E7) 
  High(SPI_Csn)  
    
  'PWR_UP = 1
  Low(SPI_Csn)   
  SpiReadWrite($20)
  SpiReadWrite($3B)   
  High(SPI_Csn)
    
  'Start receiving
  High(SPI_Ce)

PUB ReadPayload | idx, payload[4]
{{
  Reads payload from nRF24L01.
  Also flushes RX FIFO and resets IRQ state
}}    

  'Stop receiving
  Low(SPI_Ce)
    
  'Read RX payload   
  Low(SPI_Csn)    
  SpiReadWrite(R_RX_PAYLOAD) 
    
  repeat idx from 0 to 3  'Payload size = 4 byte
    payload[idx] := SpiReadWrite(NOOP)
  High(SPI_Csn)
    
  'Flush RX FIFO    
  Low(SPI_Csn)    
  SpiReadWrite($E2)    
  High(SPI_Csn)

  'Reset IRQ 
  Low(SPI_Csn)
  SpiReadWrite($27)
  SpiReadWrite($40)    
  High(SPI_Csn)

  'Start receiving
  High(SPI_Ce)
  return payload

PUB SpiReadWrite(byte_out) : byte_in | bit
{{
  SPI read-write procedure (8-bit SPI mode 0)
  Read and write are synced, i.e. for each byte in it is one byte out
  For deatails see: http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus
}}

  byte_in := byte_out

  repeat bit from 0 to 7
    'Write MOSI on trailing edge of previous clock
    if (byte_in & $80)
      High(SPI_Mosi)
    else
      Low(SPI_Mosi)
    byte_in <<= 1
 
    'Half a clock cycle before leading/rising edge
    TIME.PauseUSec(1)
    High(SPI_Sck)
 
    'Half a clock cycle before trailing/falling edge
    TIME.PauseUSec(1)
 
    'Read MISO on trailing edge
    byte_in |= Read(SPI_Miso)
    Low(SPI_Sck)

  return byte_in
  
PUB High(Pin)
    dira[Pin]~~
    outa[Pin]~~
         
PUB Low(Pin)
    dira[Pin]~~
    outa[Pin]~
    
PUB Read(Pin)
    dira[Pin]~
    return ina[Pin]
    
{{

/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

"or" is alternative interface: * <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
or * can do whatever you want with this stuff. If we meet some day, and you think
or * this stuff is worth it, you can buy me a coffee in return.
or * -----------------------------------------------------------------------------
or * This library is based on this library: 
or *   https://github.com/aaronds/arduino-nrf24l01
or * Which is based on this library: 
or *   http://www.tinkerer.eu/AVRLib/nRF24L01

#include <RF24_config.h>

/**
 * Power Amplifier level.
 *
 * For use with setPALevel()
 */
typedef enum { RF24_PA_MIN = 0,RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR } rf24_pa_dbm_e ;

/**
 * Data rate.  How fast data moves through the air.
 *
 * For use with setDataRate()
 */
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;

/**
 * CRC Length.  How big (if any) of a CRC is included.
 *
 * For use with setCRCLength()
 */
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

/**
 * Driver for nRF24L01(+) 2.4GHz Wireless Transceiver
 */

class RF24
{
private:
  uint8_t ce_pin; /**< "Chip Enable" pin, activates the RX or TX role */
  uint8_t csn_pin; /**< SPI Chip select */
  bool wide_band; /* 2Mbs data rate in use? */
  bool p_variant; /* False for RF24L01 and true for RF24L01P */
  uint8_t payload_size; /**< Fixed size of payloads */
  bool ack_payload_available; /**< Whether there is an ack payload waiting */
  bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */ 
  uint8_t ack_payload_length; /**< Dynamic size of pending ack payload. */
  uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */

protected:
  /**
   * @name Low-level internal interface.
   *
   *  Protected methods that address the chip directly.  Regular users cannot
   *  ever call these.  They are documented for completeness and for developers who
   *  may want to extend this class.
   */
  /**@{*/

  /**
   * Set chip select pin
   *
   * Running SPI bus at PI_CLOCK_DIV2 so we don't waste time transferring data
   * and best of all, we make use of the radio's FIFO buffers. A lower speed
   * means we're less likely to effectively leverage our FIFOs and pay a higher
   * AVR runtime cost as toll.
   *
   * @param mode HIGH to take this unit off the SPI bus, LOW to put it on
   */
  void csn(int mode);

  /**
   * Set chip enable
   *
   * @param level HIGH to actively begin transmission or LOW to put in standby.  Please see data sheet
   * for a much more detailed description of this pin.
   */
  void ce(int level);

  /**
   * Read a chunk of data in from a register
   *
   * @param reg Which register. Use constants from nRF24L01.h
   * @param buf Where to put the data
   * @param len How many bytes of data to transfer
   * @return Current value of status register
   */
  uint8_t read_register(uint8_t reg, uint8_t* buf, uint8_t len);

  /**
   * Read single byte from a register
   *
   * @param reg Which register. Use constants from nRF24L01.h
   * @return Current value of register @p reg
   */
  uint8_t read_register(uint8_t reg);

  /**
   * Write a chunk of data to a register
   *
   * @param reg Which register. Use constants from nRF24L01.h
   * @param buf Where to get the data
   * @param len How many bytes of data to transfer
   * @return Current value of status register
   */
  uint8_t write_register(uint8_t reg, const uint8_t* buf, uint8_t len);

  /**
   * Write a single byte to a register
   *
   * @param reg Which register. Use constants from nRF24L01.h
   * @param value The new value to write
   * @return Current value of status register
   */
  uint8_t write_register(uint8_t reg, uint8_t value);

  /**
   * Write the transmit payload
   *
   * The size of data written is the fixed payload size, see getPayloadSize()
   *
   * @param buf Where to get the data
   * @param len Number of bytes to be sent
   * @return Current value of status register
   */
  uint8_t write_payload(const void* buf, uint8_t len);

  /**
   * Read the receive payload
   *
   * The size of data read is the fixed payload size, see getPayloadSize()
   *
   * @param buf Where to put the data
   * @param len Maximum number of bytes to read
   * @return Current value of status register
   */
  uint8_t read_payload(void* buf, uint8_t len);

  /**
   * Empty the receive buffer
   *
   * @return Current value of status register
   */
  uint8_t flush_rx(void);

  /**
   * Empty the transmit buffer
   *
   * @return Current value of status register
   */
  uint8_t flush_tx(void);

  /**
   * Retrieve the current status of the chip
   *
   * @return Current value of status register
   */
  uint8_t get_status(void);

  /**
   * Decode and print the given status to stdout
   *
   * @param status Status value to print
   *
   * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
   */
  void print_status(uint8_t status);

  /**
   * Decode and print the given 'observe_tx' value to stdout
   *
   * @param value The observe_tx value to print
   *
   * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
   */
  void print_observe_tx(uint8_t value);

  /**
   * Print the name and value of an 8-bit register to stdout
   *
   * Optionally it can print some quantity of successive
   * registers on the same line.  This is useful for printing a group
   * of related registers on one line.
   *
   * @param name Name of the register
   * @param reg Which register. Use constants from nRF24L01.h
   * @param qty How many successive registers to print
   */
  void print_byte_register(const char* name, uint8_t reg, uint8_t qty = 1);

  /**
   * Print the name and value of a 40-bit address register to stdout
   *
   * Optionally it can print some quantity of successive
   * registers on the same line.  This is useful for printing a group
   * of related registers on one line.
   *
   * @param name Name of the register
   * @param reg Which register. Use constants from nRF24L01.h
   * @param qty How many successive registers to print
   */
  void print_address_register(const char* name, uint8_t reg, uint8_t qty = 1);

  /**
   * Turn on or off the special features of the chip
   *
   * The chip has certain 'features' which are only available when the 'features'
   * are enabled.  See the datasheet for details.
   */
  void toggle_features(void);
  /**@}*/

public:
  /**
   * @name Primary public interface
   *
   *  These are the main methods you need to operate the chip
   */
  /**@{*/

  /**
   * Constructor
   *
   * Creates a new instance of this driver.  Before using, you create an instance
   * and send in the unique pins that this chip is connected to.
   *
   * @param _cepin The pin attached to Chip Enable on the RF module
   * @param _cspin The pin attached to Chip Select
   */
  RF24(uint8_t _cepin, uint8_t _cspin);

  /**
   * Begin operation of the chip
   *
   * Call this in setup(), before calling any other methods.
   */
  void begin(void);
or void    nrf24_init()

  /**
   * Start listening on the pipes opened for reading.
   *
   * Be sure to call openReadingPipe() first.  Do not call write() while
   * in this mode, without first calling stopListening().  Call
   * isAvailable() to check for incoming traffic, and read() to get it.
   */
  void startListening(void);

  /**
   * Stop listening for incoming messages
   *
   * Do this before calling write().
   */
  void stopListening(void);

  /**
   * Write to the open writing pipe
   *
   * Be sure to call openWritingPipe() first to set the destination
   * of where to write to.
   *
   * This blocks until the message is successfully acknowledged by
   * the receiver or the timeout/retransmit maxima are reached.  In
   * the current configuration, the max delay here is 60ms.
   *
   * The maximum size of data written is the fixed payload size, see
   * getPayloadSize().  However, you can write less, and the remainder
   * will just be filled with zeroes.
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @return True if the payload was delivered successfully false if not
   */
  bool write( const void* buf, uint8_t len );

  /**
   * Test whether there are bytes available to be read
   *
   * @return True if there is a payload available, false if none is
   */
  bool available(void);

  /**
   * Read the payload
   *
   * Return the last payload received
   *
   * The size of data read is the fixed payload size, see getPayloadSize()
   *
   * @note I specifically chose 'void*' as a data type to make it easier
   * for beginners to use.  No casting needed.
   *
   * @param buf Pointer to a buffer where the data should be written
   * @param len Maximum number of bytes to read into the buffer
   * @return True if the payload was delivered successfully false if not
   */
  bool read( void* buf, uint8_t len );

  /**
   * Open a pipe for writing
   *
   * Only one pipe can be open at once, but you can change the pipe
   * you'll listen to.  Do not call this while actively listening.
   * Remember to stopListening() first.
   *
   * Addresses are 40-bit hex values, e.g.:
   *
   * @code
   *   openWritingPipe(0xF0F0F0F0F0);
   * @endcode
   *
   * @param address The 40-bit address of the pipe to open.  This can be
   * any value whatsoever, as long as you are the only one writing to it
   * and only one other radio is listening to it.  Coordinate these pipe
   * addresses amongst nodes on the network.
   */
  void openWritingPipe(uint64_t address);
or void    nrf24_tx_address(uint8_t* adr);

  /**
   * Open a pipe for reading
   *
   * Up to 6 pipes can be open for reading at once.  Open all the
   * reading pipes, and then call startListening().
   *
   * @see openWritingPipe
   *
   * @warning Pipes 1-5 should share the first 32 bits.
   * Only the least significant byte should be unique, e.g.
   * @code
   *   openReadingPipe(1,0xF0F0F0F0AA);
   *   openReadingPipe(2,0xF0F0F0F066);
   * @endcode
   *
   * @warning Pipe 0 is also used by the writing pipe.  So if you open
   * pipe 0 for reading, and then startListening(), it will overwrite the
   * writing pipe.  Ergo, do an openWritingPipe() again before write().
   *
   * @todo Enforce the restriction that pipes 1-5 must share the top 32 bits
   *
   * @param number Which pipe# to open, 0-5.
   * @param address The 40-bit address of the pipe to open.
   */
  void openReadingPipe(uint8_t number, uint64_t address);
or void    nrf24_rx_address(uint8_t* adr);

  /**@}*/
  /**
   * @name Optional Configurators 
   *
   *  Methods you can use to get or set the configuration of the chip.
   *  None are required.  Calling begin() sets up a reasonable set of
   *  defaults.
   */
  /**@{*/
  /**
   * Set the number and delay of retries upon failed submit
   *
   * @param delay How long to wait between each retry, in multiples of 250us,
   * max is 15.  0 means 250us, 15 means 4000us.
   * @param count How many retries before giving up, max 15
   */
  void setRetries(uint8_t delay, uint8_t count);

  /**
   * Set RF communication channel
   *
   * @param channel Which RF channel to communicate on, 0-127
   */
  void setChannel(uint8_t channel);

  /**
   * Set Static Payload Size
   *
   * This implementation uses a pre-stablished fixed payload size for all
   * transmissions.  If this method is never called, the driver will always
   * transmit the maximum payload size (32 bytes), no matter how much
   * was sent to write().
   *
   * @todo Implement variable-sized payloads feature
   *
   * @param size The number of bytes in the payload
   */
  void setPayloadSize(uint8_t size);

or void    nrf24_config(uint8_t channel, uint8_t pay_length);

  /**
   * Get Static Payload Size
   *
   * @see setPayloadSize()
   *
   * @return The number of bytes in the payload
   */
  uint8_t getPayloadSize(void);

  /**
   * Get Dynamic Payload Size
   *
   * For dynamic payloads, this pulls the size of the payload off
   * the chip
   *
   * @return Payload length of last-received dynamic payload
   */
  uint8_t getDynamicPayloadSize(void);
 
or  /* use in dynamic length mode */
or uint8_t nrf24_payloadLength();


or /* Returns the payload length */
or uint8_t nrf24_payload_length();
 
  /**
   * Enable custom payloads on the acknowledge packets
   *
   * Ack payloads are a handy way to return data back to senders without
   * manually changing the radio modes on both units.
   *
   * @see examples/pingpair_pl/pingpair_pl.pde
   */
  void enableAckPayload(void);

  /**
   * Enable dynamically-sized payloads
   *
   * This way you don't always have to send large packets just to send them
   * once in a while.  This enables dynamic payloads on ALL pipes.
   *
   * @see examples/pingpair_pl/pingpair_dyn.pde
   */
  void enableDynamicPayloads(void);

  /**
   * Determine whether the hardware is an nRF24L01+ or not.
   *
   * @return true if the hardware is nRF24L01+ (or compatible) and false
   * if its not.
   */
  bool isPVariant(void) ;

  /**
   * Enable or disable auto-acknowlede packets
   *
   * This is enabled by default, so it's only needed if you want to turn
   * it off for some reason.
   *
   * @param enable Whether to enable (true) or disable (false) auto-acks
   */
  void setAutoAck(bool enable);

  /**
   * Enable or disable auto-acknowlede packets on a per pipeline basis.
   *
   * AA is enabled by default, so it's only needed if you want to turn
   * it off/on for some reason on a per pipeline basis.
   *
   * @param pipe Which pipeline to modify
   * @param enable Whether to enable (true) or disable (false) auto-acks
   */
  void setAutoAck( uint8_t pipe, bool enable ) ;

  /**
   * Set Power Amplifier (PA) level to one of four levels.
   * Relative mnemonics have been used to allow for future PA level
   * changes. According to 6.5 of the nRF24L01+ specification sheet,
   * they translate to: RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm,
   * RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
   *
   * @param level Desired PA level.
   */
  void setPALevel( rf24_pa_dbm_e level ) ;

  /**
   * Fetches the current PA level.
   *
   * @return Returns a value from the rf24_pa_dbm_e enum describing
   * the current PA setting. Please remember, all values represented
   * by the enum mnemonics are negative dBm. See setPALevel for
   * return value descriptions.
   */
  rf24_pa_dbm_e getPALevel( void ) ;

  /**
   * Set the transmission data rate
   *
   * @warning setting RF24_250KBPS will fail for non-plus units
   *
   * @param speed RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
   * @return true if the change was successful
   */
  bool setDataRate(rf24_datarate_e speed);
  
  /**
   * Fetches the transmission data rate
   *
   * @return Returns the hardware's currently configured datarate. The value
   * is one of 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS, as defined in the
   * rf24_datarate_e enum.
   */
  rf24_datarate_e getDataRate( void ) ;

  /**
   * Set the CRC length
   *
   * @param length RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
   */
  void setCRCLength(rf24_crclength_e length);

  /**
   * Get the CRC length
   *
   * @return RF24_DISABLED if disabled or RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
   */
  rf24_crclength_e getCRCLength(void);

  /**
   * Disable CRC validation
   *
   */
  void disableCRC( void ) ;

  /**@}*/
  /**
   * @name Advanced Operation 
   *
   *  Methods you can use to drive the chip in more advanced ways 
   */
  /**@{*/

  /**
   * Print a giant block of debugging information to stdout
   *
   * @warning Does nothing if stdout is not defined.  See fdevopen in stdio.h
   */
  void printDetails(void);

  /**
   * Enter low-power mode
   *
   * To return to normal power mode, either write() some data or
   * startListening, or powerUp().
   */
  void powerDown(void);

  /**
   * Leave low-power mode - making radio more responsive
   *
   * To return to low power mode, call powerDown().
   */
  void powerUp(void) ;

or /* power management */
or void    nrf24_powerUpRx();
or void    nrf24_powerUpTx();
or void    nrf24_powerDown();

  /**
   * Test whether there are bytes available to be read
   *
   * Use this version to discover on which pipe the message
   * arrived.
   *
   * @param[out] pipe_num Which pipe has the payload available
   * @return True if there is a payload available, false if none is
   */
  bool available(uint8_t* pipe_num);

  /**
   * Non-blocking write to the open writing pipe
   *
   * Just like write(), but it returns immediately. To find out what happened
   * to the send, catch the IRQ and then call whatHappened().
   *
   * @see write()
   * @see whatHappened()
   *
   * @param buf Pointer to the data to be sent
   * @param len Number of bytes to be sent
   * @return True if the payload was delivered successfully false if not
   */
  void startWrite( const void* buf, uint8_t len );

  /**
   * Write an ack payload for the specified pipe
   *
   * The next time a message is received on @p pipe, the data in @p buf will
   * be sent back in the acknowledgement.
   *
   * @warning According to the data sheet, only three of these can be pending
   * at any time.  I have not tested this.
   *
   * @param pipe Which pipe# (typically 1-5) will get this response.
   * @param buf Pointer to data that is sent
   * @param len Length of the data to send, up to 32 bytes max.  Not affected
   * by the static payload set by setPayloadSize().
   */
  void writeAckPayload(uint8_t pipe, const void* buf, uint8_t len);

  /**
   * Determine if an ack payload was received in the most recent call to
   * write().
   *
   * Call read() to retrieve the ack payload.
   *
   * @warning Calling this function clears the internal flag which indicates
   * a payload is available.  If it returns true, you must read the packet
   * out as the very next interaction with the radio, or the results are
   * undefined.
   *
   * @return True if an ack payload is available.
   */
  bool isAckPayloadAvailable(void);

or/* core TX / RX functions */
or void    nrf24_send(uint8_t* value);
or void    nrf24_getData(uint8_t* data);

  /**
   * Call this when you get an interrupt to find out why
   *
   * Tells you what caused the interrupt, and clears the state of
   * interrupts.
   *
   * @param[out] tx_ok The send was successful (TX_DS)
   * @param[out] tx_fail The send failed, too many retries (MAX_RT)
   * @param[out] rx_ready There is a message waiting to be read (RX_DS)
   */
  void whatHappened(bool& tx_ok,bool& tx_fail,bool& rx_ready);

/* post transmission analysis */
or uint8_t nrf24_lastMessageStatus();
or uint8_t nrf24_retransmissionCount();

  /**
   * Test whether there was a carrier on the line for the
   * previous listening period.
   *
   * Useful to check for interference on the current channel.
   *
   * @return true if was carrier, false if not
   */
  bool testCarrier(void);

  /**
   * Test whether a signal (carrier or otherwise) greater than
   * or equal to -64dBm is present on the channel. Valid only
   * on nRF24L01P (+) hardware. On nRF24L01, use testCarrier().
   *
   * Useful to check for interference on the current channel and
   * channel hopping strategies.
   *
   * @return true if signal => -64dBm, false if not
   */
  bool testRPD(void) ;

or /* state check functions */
or uint8_t nrf24_dataReady();
or uint8_t nrf24_isSending();
or uint8_t nrf24_getStatus();
or uint8_t nrf24_rxFifoEmpty();


or >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 /* low level interface ... */
uint8_t spi_transfer(uint8_t tx);
void    nrf24_transmitSync(uint8_t* dataout,uint8_t len);
void    nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len);
void    nrf24_configRegister(uint8_t reg, uint8_t value);
void    nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len);
void    nrf24_writeRegister(uint8_t reg, uint8_t* value, uint8_t len);

/* You should implement the platform spesific functions in your code */

/* In this function you should do the following things:
 *    - Set MISO pin input
 *    - Set MOSI pin output
 *    - Set SCK pin output
 *    - Set CSN pin output
 *    - Set CE pin output     */
extern void nrf24_setupPins();

/* nrf24 CE pin control function
 *    - state:1 => Pin HIGH
 *    - state:0 => Pin LOW     */
extern void nrf24_ce_digitalWrite(uint8_t state);

/* nrf24 CE pin control function
 *    - state:1 => Pin HIGH
 *    - state:0 => Pin LOW     */
extern void nrf24_csn_digitalWrite(uint8_t state);

/* nrf24 SCK pin control function
 *    - state:1 => Pin HIGH
 *    - state:0 => Pin LOW     */
extern void nrf24_sck_digitalWrite(uint8_t state);

/* nrf24 MOSI pin control function
 *    - state:1 => Pin HIGH
 *    - state:0 => Pin LOW     */
extern void nrf24_mosi_digitalWrite(uint8_t state);

/* nrf24 MISO pin read function
/* - returns: Non-zero if the pin is high */
extern uint8_t nrf24_miso_digitalRead();
<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< or


};

 * @li <a href="http://maniacbug.github.com/RF24/">Documentation Main Page</a>
 * @li <a href="http://maniacbug.github.com/RF24/classRF24.html">RF24 Class Documentation</a>
 * @li <a href="https://github.com/maniacbug/RF24/">Source Code</a>
 * @li <a href="https://github.com/maniacbug/RF24/archives/master">Downloads Page</a>
 * @li <a href="http://www.nordicsemi.com/files/Product/data_sheet/nRF24L01_Product_Specification_v2_0.pdf">Chip Datasheet</a>
 *
 * This chip uses the SPI bus, plus two chip control pins.  Remember that pin 10 must still remain an output, or
 * the SPI hardware will go into 'slave' mode.
 *  
}}  

{{
/* msprf24.h
 * Copyright (c) 2012, Eric Brundick <spirilis@linux.com>
*/

/* RF speed settings -- nRF24L01+ compliant, older nRF24L01 does not have 2Mbps. */
#define RF24_SPEED_250KBPS  0x20
#define RF24_SPEED_1MBPS    0x00
#define RF24_SPEED_2MBPS    0x08
#define RF24_SPEED_MAX      RF24_SPEED_2MBPS
#define RF24_SPEED_MIN      RF24_SPEED_250KBPS
#define RF24_SPEED_MASK     0x28

/* RF transmit power settings */
#define RF24_POWER_0DBM        0x06
#define RF24_POWER_MINUS6DBM   0x04
#define RF24_POWER_MINUS12DBM  0x02
#define RF24_POWER_MINUS18DBM  0x00
#define RF24_POWER_MAX         RF24_POWER_0DBM
#define RF24_POWER_MIN         RF24_POWER_MINUS18DBM
#define RF24_POWER_MASK        0x06

/* Available states for the transceiver's state machine */
#define RF24_STATE_NOTPRESENT  0x00
#define RF24_STATE_POWERDOWN   0x01
#define RF24_STATE_STANDBY_I   0x02
#define RF24_STATE_STANDBY_II  0x03
#define RF24_STATE_PTX         0x04
#define RF24_STATE_PRX         0x05
#define RF24_STATE_TEST        0x06

/* IRQ "reasons" that can be tested. */
#define RF24_IRQ_TXFAILED      0x10
#define RF24_IRQ_TX            0x20
#define RF24_IRQ_RX            0x40
#define RF24_IRQ_MASK          0x70
// Bit 7 used to signify that the app should check IRQ status, without
// wasting time in the interrupt vector trying to do so itself.
#define RF24_IRQ_FLAGGED       0x80

/* Queue FIFO states that can be tested. */
#define RF24_QUEUE_TXFULL      RF24_FIFO_FULL
#define RF24_QUEUE_TXEMPTY     RF24_TX_EMPTY
#define RF24_QUEUE_RXFULL      RF24_RX_FULL
#define RF24_QUEUE_RXEMPTY     RF24_RX_EMPTY

/* FUNCTIONS! */

// SPI driver needs to provide these
void spi_init();
char spi_transfer(char);  // SPI xfer 1 byte
int spi_transfer16(int);  // SPI xfer 2 bytes
int spi_transfer9(int);   // SPI xfer 9 bits (courtesy for driving LCD screens)

// Register & FIFO I/O
unsigned char r_reg(unsigned char addr);
void w_reg(unsigned char addr, char data);
void w_tx_addr(char *addr);             // Configure TX address to send next packet
void w_rx_addr(unsigned char pipe, char *addr);  // Configure RX address of "rf_addr_width" size into the specified pipe
void w_tx_payload(unsigned char len, char *data);
void w_tx_payload_noack(unsigned char len, char *data);  /* Only used in auto-ack mode with RF24_EN_DYN_ACK enabled;
                                                 * send this packet with no auto-ack.
                                                 */
unsigned char r_rx_peek_payload_size();  // Peek size of incoming RX payload
unsigned char r_rx_payload(unsigned char len, char *data);
void flush_tx();
void flush_rx();
void tx_reuse_lastpayload();   /* Enable retransmitting contents of TX FIFO endlessly until flush_tx() or the FIFO contents are replaced.
                                * Actual retransmits don't occur until CE pin is strobed using pulse_ce();
                                */
void pulse_ce();  // Pulse CE pin to activate retransmission of TX FIFO contents after tx_reuse_lastpayload();
void w_ack_payload(unsigned char pipe, unsigned char len, char *data);  // Used when RF24_EN_ACK_PAY is enabled to manually ACK a received packet



// Initialization and configuration
void msprf24_init();  /* Set the various configuration variables before running this.
                       * It will populate the channel/speed/power/default features/etc. values
                       */
void msprf24_close_pipe(unsigned char pipeid);       // Disable specified RX pipe
void msprf24_close_pipe_all();                       // Disable all RX pipes (used during initialization)
void msprf24_open_pipe(unsigned char pipeid, unsigned char autoack); // Enable specified RX pipe, optionally turn auto-ack (Enhanced ShockBurst) on
unsigned char msprf24_pipe_isopen(unsigned char pipeid); // Check if specified RX pipe is active
void msprf24_set_pipe_packetsize(unsigned char pipe, unsigned char size);  // Set static length of pipe's RX payloads (1-32), size=0 enables DynPD.
void msprf24_set_retransmit_delay(int us);           // 500-4000uS range, clamped by RF speed
void msprf24_set_retransmit_count(unsigned char count);       // 0-15 retransmits before MAX_RT (RF24_IRQ_TXFAILED) IRQ raised
unsigned char msprf24_get_last_retransmits();        // # times a packet was retransmitted during last TX attempt
unsigned char msprf24_get_lostpackets();      /* # of packets lost since last time the Channel was set.
                                               * Running msprf24_set_channel() without modifying rf_channel will reset this counter.
                                               */
unsigned char msprf24_is_alive();                    // Hello World, test if chip is present and/or SPI is working.
unsigned char msprf24_set_config(unsigned char cfgval);
void msprf24_set_speed_power();                      // Commit RF speed & TX power from rf_speed_power variable.
void msprf24_set_channel();                          // Commit RF channel setting from rf_channel variable.
void msprf24_set_address_width();                    // Commit Enhanced ShockBurst Address Width from rf_addr_width variable.
void msprf24_enable_feature(unsigned char feature);    /* Enable specified feature (RF24_EN_* from nRF24L01.h, except RF24_EN_CRC) */
void msprf24_disable_feature(unsigned char feature);   /* Disable specified feature                                                */

// Change chip state and activate I/O
unsigned char msprf24_current_state();    // Get current state of the nRF24L01+ chip, test with RF24_STATE_* #define's
void msprf24_powerdown();                 // Enter Power-Down mode (0.9uA power draw)
void msprf24_standby();                   // Enter Standby-I mode (26uA power draw)
void msprf24_activate_rx();               // Enable PRX mode (~12-14mA power draw)
void msprf24_activate_tx();               // Enable Standby-II or PTX mode; TX FIFO contents will be sent over the air (~320uA STBY2, 7-11mA PTX)
unsigned char msprf24_queue_state();      // Read FIFO_STATUS register; user should compare return value with RF24_QUEUE_* #define's
unsigned char msprf24_scan();             // Scan current channel for RPD (looks for any signals > -64dBm)

// IRQ handling
unsigned char msprf24_rx_pending();                /* Query STATUS register to determine if RX FIFO data is available for reading. */
unsigned char msprf24_get_irq_reason();            /* Query STATUS register for the IRQ flags, test with RF24_IRQ_* #define's
                                                    * Result is stored in rf_irq (note- RF24_IRQ_FLAGGED is not automatically cleared by this
                                                    * function, that's the user's responsibility.)
                                                    */
void msprf24_irq_clear(unsigned char irqflag);     /* Clear specified Interrupt Flags (RF24_IRQ_* #define's) from the transceiver.
                                                    * Required to allow further transmissions to continue.
                                                    */

}}

{{
Copyright (c) 2007 Stefan Engelke <mbox@stefanengelke.de>
    Some parts copyright (c) 2012 Eric Brundick <spirilis [at] linux dot com>
    
/* Register Map */
#define RF24_CONFIG      0x00
#define RF24_EN_AA       0x01
#define RF24_EN_RXADDR   0x02
#define RF24_SETUP_AW    0x03
#define RF24_SETUP_RETR  0x04
#define RF24_RF_CH       0x05
#define RF24_RF_SETUP    0x06
#define RF24_STATUS      0x07
#define RF24_OBSERVE_TX  0x08
#define RF24_CD          0x09
#define RF24_RPD         0x09
#define RF24_RX_ADDR_P0  0x0A
#define RF24_RX_ADDR_P1  0x0B
#define RF24_RX_ADDR_P2  0x0C
#define RF24_RX_ADDR_P3  0x0D
#define RF24_RX_ADDR_P4  0x0E
#define RF24_RX_ADDR_P5  0x0F
#define RF24_TX_ADDR     0x10
#define RF24_RX_PW_P0    0x11
#define RF24_RX_PW_P1    0x12
#define RF24_RX_PW_P2    0x13
#define RF24_RX_PW_P3    0x14
#define RF24_RX_PW_P4    0x15
#define RF24_RX_PW_P5    0x16
#define RF24_FIFO_STATUS 0x17
#define RF24_DYNPD       0x1C
#define RF24_FEATURE     0x1D

/* Register Bits */
/* configuratio nregister */
#define RF24_MASK_RX_DR  BIT6
#define RF24_MASK_TX_DS  BIT5
#define RF24_MASK_MAX_RT BIT4
#define RF24_EN_CRC      BIT3
#define RF24_CRCO        BIT2
#define RF24_PWR_UP      BIT1
#define RF24_PRIM_RX     BIT0

/* enable auto acknowledgment */
#define RF24_ENAA_P5     BIT5
#define RF24_ENAA_P4     BIT4
#define RF24_ENAA_P3     BIT3
#define RF24_ENAA_P2     BIT2
#define RF24_ENAA_P1     BIT1
#define RF24_ENAA_P0     BIT0

/* enable rx addresses */
#define RF24_ERX_P5      BIT5
#define RF24_ERX_P4      BIT4
#define RF24_ERX_P3      BIT3
#define RF24_ERX_P2      BIT2
#define RF24_ERX_P1      BIT1
#define RF24_ERX_P0      BIT0

/* setup of address width */
#define RF24_AW          BIT0

/* setup of auto re-transmission */
#define RF24_ARD         BIT4
#define RF24_ARC         BIT0

/* RF setup register */
#define RF24_PLL_LOCK    BIT4
#define RF24_CONT_WAVE   BIT7
#define RF24_RF_DR       BIT3
#define RF24_RF_DR_LOW   BIT5
#define RF24_RF_DR_HIGH  BIT3
#define RF24_RF_PWR      BIT1
#define RF24_LNA_HCURR   BIT0

/* general status register */
#define RF24_RX_DR       BIT6
#define RF24_TX_DS       BIT5
#define RF24_MAX_RT      BIT4
#define RF24_RX_P_NO     BIT1
#define RF24_TX_FULL     BIT0

/* transmit observe register */
#define RF24_PLOS_CNT    BIT4
#define RF24_ARC_CNT     BIT0

/* fifo status */
#define RF24_TX_REUSE    BIT6
#define RF24_FIFO_FULL   BIT5
#define RF24_TX_EMPTY    BIT4
#define RF24_RX_FULL     BIT1
#define RF24_RX_EMPTY    BIT0

/* dynamic length */
#define RF24_EN_DPL      BIT2
#define RF24_EN_ACK_PAY  BIT1
#define RF24_EN_DYN_ACK  BIT0
#define DPL_P0      0
#define DPL_P1      1
#define DPL_P2      2
#define DPL_P3      3
#define DPL_P4      4
#define DPL_P5      5

/* Instructions */
#define RF24_R_REGISTER    0x00 /* last 4 bits will indicate reg. address */
#define RF24_W_REGISTER    0x20 /* last 4 bits will indicate reg. address */
#define RF24_REGISTER_MASK 0x1F
#define ACTIVATE           0x50 
#define RF24_R_RX_PL_WID   0x60
#define RF24_R_RX_PAYLOAD  0x61
#define RF24_W_ACK_PAYLOAD 0xA8
#define RF24_W_TX_PAYLOAD_NOACK 0xB0
#define RF24_W_TX_PAYLOAD  0xA0
#define RF24_FLUSH_TX      0xE1
#define RF24_FLUSH_RX      0xE2
#define RF24_REUSE_TX_PL   0xE3

#define RF24_NOP           0xFF

}}