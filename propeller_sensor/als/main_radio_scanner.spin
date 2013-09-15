''ThK 2013-08-20
''

OBJ
  Radio    : "radio_nRF"     'radio routines (for nRF24L01, planned to be Hardware independed, use rfm70 eg later)
  SER      : "FullDuplexSerialPlus"  'Serial port
  
CON

  _clkmode = xtal1 + pll16x                           
  _xinfreq = 5_000_000

  'Pins
  SPI_CE   = 3
  SPI_CSN  = 4     
  SPI_SCK  = 5 
  SPI_MOSI = 7 
  SPI_MISO = 6
  SPI_IRQ  = 9

VAR
   long i

PUB Main_Scanner | payload[4], idx , carrier, channel  , c
'' gona be a channel scanner ''

  'Set IRQ pin state 
  dira[SPI_IRQ] := 0

  'Initialize serial (use Parallax Debug Terminal for feedback)
  SER.start(31, 30, 0, 57600)
  waitcnt(clkfreq*2 + cnt)
  SER.tx(SER#CLS)
  SER.str(String("hi... starting up ...")) 
  SER.str(String(SER#CR)) 

  'Initialize Nordic nRF24L01
  Radio.Init( SPI_MISO, SPI_MOSI, SPI_SCK, SPI_CSN, SPI_CE)
  waitcnt(clkfreq/10 + cnt)
  carrier := Radio.get_RF_Status
  SER.bin(carrier,32)
  SER.str(String(SER#CR)) 
   
  
  SER.str(String("Receiver configured. Waiting for packets..."))
  SER.str(String(SER#CR)) 

  ' Channel Layout
  '       1 2  3 4  5  6 7 8  9 10 11 12 13  14                      
  ' 0         1         2         3         4         5         6
  ' 0123456789012345678901234567890123456789012345678901234567890123
   
  repeat
    'WLan channels  
    SER.str(String("      1 2  3 4  5  6 7 8  9 10 11 12 13  14                     "))
    SER.str(String(SER#CR)) 
 
    repeat Channel from 0 to 99
      'SER.dec(Channel//10)
      if (Channel//10) == 0
        SER.Dec(Channel / 10)
      else
        SER.Str(String(" "))
    SER.Str(String("|"))    
    SER.str(String(SER#CR)) 
    repeat Channel from 0 to 99
      SER.Dec(Channel//10)
    SER.Str(String("|"))
    SER.str(String(SER#CR)) 
    
    repeat i from 1 to 10 ' scan 10 time then repeat the channel layout
      'SER.str(String(">"))
      repeat Channel from 0 to 99   'scan channels 2400 - 2500 MHz ISM Band in Germany : 2400 - 2483,5 is datacom (WLAN) Band in Germany
          'SER.str(String("_")) 
          carrier := Radio.is_channel_busy(Channel)
          'SER.bin(carrier,32)
          'SER.str(String(SER#CR)) 
          if carrier == true
             SER.str(String("#"))
          else
             SER.str(String("."))
      SER.Str(String("|"))    
      SER.str(String(SER#CR)) 
      'waitcnt(clkfreq + cnt)
      'carrier := Radio.get_RF_Status
      'SER.bin(carrier,32)
      'SER.str(String(SER#CR)) 
  
SER.str(String(SER#CR)) 
SER.str(String(SER#CR)) 
SER.str(String(SER#CR)) 
     
'repeat
'    if ina[SPI_IRQ]  == 0 'Wait for incoming data
'      'We have some data
'      carrier := Radio.get_carrier_detect()
'      SER.str(String("Data received: "))
'      repeat idx from 0 to 3 
'        SER.hex(payload[idx], 2)
'      SER.str(String(SER#CR))