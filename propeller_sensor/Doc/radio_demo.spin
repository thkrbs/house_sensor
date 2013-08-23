''ThK 2013-08-19
''

OBJ
  Radio : "Radio_rRF"         'radio routines (Hardware independed, use rfm70 eg later)
  HalRadio : "ha_nrf2401p"   'radio implementation for nRF24L01
  SER      : "FullDuplexSerialPlus"     'Serial port
  
CON

  'Pins
  SPI_SCK = 5 
  SPI_MISO = 7
  SPI_MOSI = 6 
  SPI_CSN = 4     
  SPI_CE = 3
  SPI_IRQ = 9


PUB Scanner | payload[4], idx , carrier
'' gona be a channel scanner ''

  'Set IRQ pin state 
  dira[SPI_IRQ] := 0

  'Initialize serial (use Parallax Debug Terminal for feedback)
  SER.start(31, 30, 0, 57600)
  waitcnt(clkfreq*2 + cnt)
  SER.tx(SER#CLS)
  SER.str(String("hi...")) 

  'Initialize Nordic nRF24L01
  Radio.Init(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CSN, SPI_CE)
  SER.str(String("Receiver configured. Waiting for packets..."))
  SER.str(String(SER#CR)) 

  repeat Channel from 0 to 99
    SER.str(String(Channel)

  repeat Channel from 0 to 99
      Radio.setChannel(Channel)
      carrier := Radio.get_carrier_detect()


repeat
    if ina[SPI_IRQ]  == 0 'Wait for incoming data
      'We have some data
      carrier := Radio.get_carrier_detect()
      SER.str(String("Data received: "))
      repeat idx from 0 to 3 
        SER.hex(payload[idx], 2)
      SER.str(String(SER#CR))
