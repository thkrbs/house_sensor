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
  byte val
  byte addr[5]
  byte raddr[5]
 
PUB Main_Scanner | payload[4], idx , carrier, channel  , c , radio_ok
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

  repeat
    waitcnt(clkfreq/10 + cnt)
    radio_ok := true
     
    'able to read from radio?
    val := $EE
    val := Radio.get_RF_Status 
      if val == 255 or val == 0 or val == $EE
        SER.str(String("' not able to read valid data from radio. Found:"))
        SER.hex(val,2)
        SER.str(String(SER#CR))
        radio_ok := false
 
         
    'able to set channel on radio?
    Radio.set_Channel(25)
    Radio.get_Channel(@val)
    if val <> 25
      SER.str(String("'Failure setting channel. Set: "))
      ser.hex(25,2) 
      SER.str(String("' Readback: "))
      ser.hex(val,2) 
      radio_ok := false
    
    'able to set rx address on radio? (Default 5 byte addresses)
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
 
    Radio.set_RX_Address(0, @addr)
    
    Radio.get_RX_Address(0, @raddr)
    
    repeat i from 0 to 4
      'ser.hex(raddr[i],2)
      if raddr[i] <> addr[i]
        SER.str(String("Failure setting RX address. Set:"))
        repeat i from 0 to 4
          ser.hex(addr[i],2)
        SER.str(String("Readback: "))
        repeat i from 0 to 4
          ser.hex(raddr[i],2)
        SER.str(String(SER#CR)) 
        radio_ok := false

    if ! radio_ok
      SER.str(String("Please fix connection to the nRF24L01+ board. Repeating ..."))
      SER.str(String(SER#CR)) 
  while !radio_ok
       
  
  SER.str(String("Receiver configured. Locking for carrier ... (showing WLAN 20MHz channel position. WLAN channel are 22Mhz wide!)"))
  SER.str(String(SER#CR)) 

  ' Channel Layout (WLAN 20MHz width)
  ' WLAN        1    2    3    4    5    6    7    8    9    10   11   12     13        14                      
  ' 0         1         2         3         4         5         6         7         8         9
  ' 01234567890123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890
   
  repeat
    'WLan channels  
    SER.str(String("WLAN        1    2    3    4    5    6    7    8    9    10   11   12     13        14"))
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
      repeat Channel from 0 to 99   'scan channels 2400 - 2500 MHz ISM Band in Germany : 2400 - 2483,5 is for datacom (WLAN) in Germany
          'SER.str(String("_")) 
          carrier := Radio.is_channel_busy(Channel)
          'SER.bin(carrier,32)
          'SER.str(String(SER#CR)) 
          if carrier == 1
             SER.str(String("#"))
          else
             SER.str(String("."))
      SER.Str(String("|"))    
      SER.str(String(SER#CR)) 
      'waitcnt(clkfreq + cnt)
      'carrier := Radio.get_RF_Status
      'SER.bin(carrier,32)
      'SER.str(String(SER#CR))
  {
  'listen on channel 2, enhanced something, variable payload
  Radio.set_channel(2)
  
  'RX adress $deadbeaf01
  
  addr[0] := $DE
  addr[1] := $AD
  addr[2] := $BE
  addr[3] := $AF
  addr[4] := $01
  Radio.set_RX_Address(0,@addr)
  'SER.str(String("Wait for data on channel 2")
  waitcnt(clkfreq/10 + cnt)
  if isbuffer
  } 
  repeat 3
    SER.str(String(SER#CR)) 

     
'repeat
'    if ina[SPI_IRQ]  == 0 'Wait for incoming data
'      'We have some data
'      carrier := Radio.get_carrier_detect()
'      SER.str(String("Data received: "))
'      repeat idx from 0 to 3 
'        SER.hex(payload[idx], 2)
'      SER.str(String(SER#CR))