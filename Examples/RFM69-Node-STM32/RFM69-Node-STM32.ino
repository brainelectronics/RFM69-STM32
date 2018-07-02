/**
 * @file RFM69-Node-STM32.ino
 * 
 * @author     Jonas Scharpf <jonas@brainelectronics.de>
 * @date       July 2018
 * @version    1.0
 * 
 * @brief      Sample RFM69 sender/node sketch, with ACK and optional 
 *             encryption, and Automatic Transmission Control driven by a 
 *             STM32F103C8 or similar
 * 
 * Description:
 *   This sketch has been successfully tested on a STM32F103C8 bluepill.
 *   
 *   It sends periodic messages of increasing length to gateway (id = 1)
 *   The code has a 10sec startup delay, due to the
 *   missing wait-until-USB-connected function of the STM32.
 *   The default LED is active during the setup function.
 *   The sketch prints out the file name as well as compilation date and time
 *  
 *  
 * Limits (At the date of creation):
 *    None
 *
 *
 * Note:
 *  for further understandings of the code, please read the doxygen docs
 *  also consider taking a look at the following code snippets/examples
 *  - None
 * 
 * 
 * TBD:
 *  see issues at the git
 * 
 * 
 * Improvements:
 *  None
 *  
 * 
 * Versions:
 *    1.0:
 *      set and get CS pin, IRQ pin, get power level
 *      
 *    Original RFM69 library and sample code by Felix Rusu
 *    http://LowPowerLab.com/contact
 *    Copyright Felix Rusu (2015)
 *  
 *  
 * The circuit:
 *  STM32F103C Bluepill Board
 *  RFM69 attached to SPI_1 
 *  CS    <-->  PA4 <-->  BOARD_SPI1_NSS_PIN
 *  SCK   <-->  PA5 <-->  BOARD_SPI1_SCK_PIN
 *  MISO  <-->  PA6 <-->  BOARD_SPI1_MISO_PIN
 *  MOSI  <-->  PA7 <-->  BOARD_SPI1_MOSI_PIN
 *  DIO0  <-->  PA8 <-->  Interrupt on receiving message
 * 
 * 
 * RFM69 library and sample code by Felix Rusu - http://LowPowerLab.com/contact
 * Copyright Felix Rusu (2015)
 * 
 * 
 ***************************************************************************/

// include all libs
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>
//#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
//
#define NODEID        3    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     100  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID     1
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY   RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************

// define LED pins for ATmegas
#if defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define atmega         true
  #define active         HIGH
  #define inactive       LOW
  #define onboardLED     13
  #include <avr/wdt.h>                  // AVR Watchdog lib
#else
  #define active         LOW
  #define inactive       HIGH
  #define onboardLED     PC13
  #include <libmaple/iwdg.h>            // STM Marple Watchdog lib
  #define iwdg_init_ms(N) iwdg_init(IWDG_PRE_256,((N)/5))
#endif

int TRANSMITPERIOD = 150; //transmit a packet to gateway so often (in ms)
char payload[] = "123 ABCDEFGHIJKLMNOPQRSTUVWXYZ";
char buff[20];
byte sendSize = 0;
boolean requestACK = true;
//SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

/*
// you can define the RFM69 pin connections for SPI CS and IRQ Pins like this
// RFM69(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM)
// or you can set it at the setup loop BEFORE the "radio.initialize()" function
// NOTE: on the STM32 the interrupt pin and interrupt number must be the same
#ifdef ENABLE_ATC
  RFM69_ATC radio = RFM69_ATC(PA4, PA8, true, PA8);
#else
  RFM69 radio = RFM69(PA4, PA8, true, PA8);
#endif
  */
#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif

// the setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(9600);     // USB:           PA11 (DM), PA12 (DP)
  // Serial1.begin(9600);    // PA9 (STM-TX), PA10 (STM-RX)
  // Serial2.begin(9600);    // PA2 (STM-TX), PA3 (STM-RX)
  // Serial3.begin(9600);    // PB10 (STM-TX), PB11 (STM-RX)
  
#ifdef atmega
  Serial.println("I'm a ATmega");

  // initWatchdog(uint16_t interruptTimeMillis)
  initWatchdog(8000);
#else
  delay(10000);
  Serial.println("I'm a STM32 or non ATmega");

  initWatchdog(8000);
#endif

  // define onboard led pin as output and activate it
  pinMode(onboardLED, OUTPUT);
  digitalWrite(onboardLED, active);

  // set the slave select (CS) pin (PA4 is default for SPI_1)
  radio.setSlaveSelectPin(PA4);
  // get the IRQ pin (which is connected to RFM69's DIO0)
  radio.setInterruptPin(PA8);
  // set the IRQ pin number (at the STM32 must be same as interrupt pin)
  radio.setInterruptNumber(radio.getInterruptPin());

  radio.initialize(FREQUENCY, NODEID, NETWORKID);

#ifdef IS_RFM69HW
  radio.setHighPower(); // uncomment only for RFM69HW!
#endif

  radio.encrypt(ENCRYPTKEY);
  //radio.setFrequency(919000000); // set frequency to some custom frequency

//Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
//For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
//For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
//Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
#ifdef ENABLE_ATC
  radio.enableAutoPower(-70);
#endif

  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

  /*
  if (flash.initialize())
  {
    Serial.print("SPI Flash Init OK ... UniqueID (MAC): ");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
  else
    Serial.println("SPI Flash MEM not found (is chip soldered?)...");
  */
  
#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  // report successfull performed setup
  Serial.println("Setup done, starting soon...");

  // turn onboard led off
  digitalWrite(onboardLED, inactive);
}

long lastPeriod = 0;
// the loop function runs over and over again forever
void loop()
{
  feedTheDog();

  // process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input >= 48 && input <= 57) //[0,9]
    {
      TRANSMITPERIOD = 100 * (input-48);
      if (TRANSMITPERIOD == 0) TRANSMITPERIOD = 1000;
      Serial.print("\nChanging delay to ");
      Serial.print(TRANSMITPERIOD);
      Serial.println("ms\n");
    }
    if (input == 'i') // print all available setup infos
    {
      Serial.println();
      Serial.println("***************************************************************************");
      //return name of file and compile date/time
      Serial.println(__FILE__ " " __DATE__ " " __TIME__);
      
      Serial.print("Slave Select Pin: ");
      Serial.println(radio.getSlaveSelectPin());
      Serial.print("Interrupt Pin: ");
      Serial.println(radio.getInterruptPin());
      Serial.print("Interrupt Number: ");
      Serial.println(radio.getInterruptNumber());
      Serial.print("Is RFM69HW: ");
      Serial.println(radio.getHighPower());
      Serial.print("Power Level: ");
      Serial.println(radio.getPowerLevel());
      Serial.print("Frequency: ");
      Serial.println(radio.getFrequency());
      Serial.print("NODEID: ");
      Serial.println(radio.getAdress());
      Serial.print("NETWORKID: ");
      Serial.println(radio.getNetwork());

      Serial.print("Test Pin (LED on STM32F103C Bluepill): ");
      Serial.println(radio.getTestPin());
      Serial.println("***************************************************************************");
      Serial.println();
    }
    if (input == 'r') //d=dump register values
    {
      radio.readAllRegs();
    }
    if (input == 'E') //E=enable encryption
    {
     radio.encrypt(ENCRYPTKEY);
    }
    if (input == 'e') //e=disable encryption
    {
     radio.encrypt(null);
    }
    if (input == 't')
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      byte fTemp = 1.8 * temperature + 32; // 9/5=1.8
      Serial.print( "Radio Temp is ");
      Serial.print(temperature);
      Serial.print("C, ");
      Serial.print(fTemp); //converting to F loses some resolution, obvious when C is on edge between 2 values (ie 26C=78F, 27C=80F)
      Serial.println('F');
    }
    /*
    if (input == 'd') //d=dump flash area
    {
      Serial.println("Flash content:");
      uint16_t counter = 0;

      Serial.print("0-256: ");
      while(counter<=256){
        Serial.print(flash.readByte(counter++), HEX);
        Serial.print('.');
      }
      while(flash.busy());
      Serial.println();
    }
    if (input == 'e')
    {
      Serial.print("Erasing Flash chip ... ");
      flash.chipErase();
      while(flash.busy());
      Serial.println("DONE");
    }
    if (input == 'i')
    {
      Serial.print("DeviceID: ");
      word jedecid = flash.readDeviceId();
      Serial.println(jedecid, HEX);
    }
    */
  }

  feedTheDog();

  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
    Blink(onboardLED, 3);
    Serial.println();
  }

  feedTheDog();

  int currPeriod = millis()/TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    lastPeriod = currPeriod;

    Serial.print("Sending[");
    Serial.print(sendSize);
    Serial.print("]: ");
    for(byte i = 0; i < sendSize; i++)
      Serial.print((char)payload[i]);

    if (radio.sendWithRetry(GATEWAYID, payload, sendSize))
    {
     Serial.print(" ok!");
    }
    else
    {
     Serial.print(" nothing...");
    }
    sendSize = (sendSize + 1) % 31;
    Serial.println();
    Blink(onboardLED, 3);
  }
}

/**
 * @brief      blink function
 *
 * @param[in]  PIN       The pin
 * @param[in]  DELAY_MS  The delay between on and off in milliseconds
 */
void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, active);
  delay(DELAY_MS);
  digitalWrite(PIN, inactive);
}

/**
 * @brief      init watchdog function
 *
 * @param[in]  interruptTime  The interrupt time
 */
void initWatchdog(uint16_t interruptTimeMillis)
{
#if defined atmega
  wdt_disable();                      // disable any ongoing watchdog
  
  switch(interruptTimeMillis) 
  {
    case 15: wdt_enable(WDTO_15MS); break;  // 15 ms
    case 30: wdt_enable(WDTO_30MS); break;  // 30 ms
    case 60: wdt_enable(WDTO_60MS); break;  // 60 ms
    case 120: wdt_enable(WDTO_120MS); break;// 120 ms
    case 250: wdt_enable(WDTO_250MS); break;// 250 ms
    case 500: wdt_enable(WDTO_500MS); break;// 500 ms
    case 1000: wdt_enable(WDTO_1S); break;  // 1 sec
    case 2000: wdt_enable(WDTO_2S); break;  // 2 sec
    case 4000: wdt_enable(WDTO_4S); break;  // 4 sec, not working at ATMega8
    case 8000: wdt_enable(WDTO_8S); break;  // 8 sec, not working at ATMega8
    default: 
      Serial.println("No valid watchdog time, watchdog disabled"); 
      break;
  }
  wdt_reset();                        // feed the dog to keep him calm
#else
  // init an 8 second wd timer by prescaling the ~50KHz clock and then calculating resulting ticks in 8 secs...
  //iwdg_init(IWDG_PRE_256, 1600);

  // or init an 8s wd timer a bit more obviously using a wrapper
  // iwdg_init_ms(8000);

  iwdg_init_ms(interruptTimeMillis);
#endif
}

/**
 * @brief      Reset the watchdog
 */
void feedTheDog(void)
{
#if defined atmega
  wdt_reset();                        // feed the dog to keep him calm
#else
  iwdg_feed();                        // same as wdt_reset() at AVR chips
#endif
}

