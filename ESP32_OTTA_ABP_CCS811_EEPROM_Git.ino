//check each comment with "//maybe modify" before uploading!!!!

/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI for OTAA Example part
 * Copyright (c) 2020 Ulrich Spizig with a lot of inspiration and modifications from Ed Smallenburg ESP8266 https://github.com/Edzelf/LoRa/blob/master/ESP_lora_tracker/ESP_LoRa_tracker.ino
 * 
 * This sketch is optimized for ESP32 on Arduino
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 *
 * Version for ESP32.                                                                             *
* The first time, the device will connect over OTAA.  TTN will return a fresh network session key  *
* and an application session key that should be used for the next connection (after reset).        *
* Once OTAA Join was sucessful and EEPROm Data was stored the next power cycle the Join Data is read from RTC Memory and/or EEPROM.
* Aftre every message, the keys (and the sequence number) are updated in RTC memory.               *
* after 100 transmittes packets, the keys are saved in EEPROM.                                             *
* On start-up/wakeup from sleep, the keys and sequence number are retrieved from RTC memory or EEPROM.
 *
 
 * 
 * After sucessful joining data is stored to RTC Memory and EEPROM
 * 
 * This sketch works best with MCCI Stack : https://github.com/mcci-catena/arduino-lmicDisplays

 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *Maybe you need some changes in the Stack:
 *#define LMICbandplan_getInitialDrJoin() (EU868_DR_SF8) in lmic_bandplan_eu868.h
 *in oslmic.h change #define OSTICKS_PER_SEC (original) 32768 to 50000 //Uspizig does not really help
 *If you have a different SPI Pinning then replace in hal.cpp spi-begin() to SPI.begin(lmic_pins.sck, lmic_pins.miso, lmic_pins.mosi, lmic_pins.nss); e.g. spi.begin(14, 2, 15, 26); to match your pinning
   static void hal_spi_init () {
    //SPI.begin();//Original
  //SPI.begin(lmic_pins.sck, lmic_pins.miso, lmic_pins.mosi, lmic_pins.nss);
  SPI.begin(14, 2, 15, 26);
  }

 *******************************************************************************/







//for Debugging
//#define SingleChannelMode 1 //to check on own gateway or single channel forwareder the Join Behaviour
#define LMIC_DEBUG_LEVEL 1
#define geraet3 //ESP32 TTGO Mini V2 COM9


// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc  
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).

#ifdef geraet1
  const unsigned TX_INTERVAL = 60; 
  #define BMP_ADRESS 0x76
  #define BMP280_CONNECTED //geraet1
#endif
#ifdef geraet2 //limit uplinks on testboard
  const unsigned TX_INTERVAL = 180; 
  #define BMP_ADRESS 0x76
  #define BMP280_CONNECTED //geraet1
#endif
#ifdef geraet3 //limit uplinks on testboard
  const unsigned TX_INTERVAL = 45; 
  #define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A
#endif

//const unsigned long UpdateInterval = (60L * 20L - 03) * 1000000L; // Update delay in microseconds
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

//WetterSymbols Size
int wettercount = 1;
#define LEFT 1
#define CENTER 2
#define RIGHT 3
boolean LargeIcon = true, SmallIcon = false;
//250x122 Pixel
int wetter_symbol_x = 210; 
int wetter_symbol_y = 30; 
int status_symbol_x = 210;
int status_symbol_y = 122;
boolean wetter_symbol_size = true;
#define Large  5           // For icon drawing, needs to be odd number for best effect
#define Small  3            // For icon drawing, needs to be odd number for best effect

#ifdef geraet3 //ESP32 TTGO MINI V2
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO ESP32 Mini V2 mit EINK+Lora Addon Board
  #define LORA_PIN_SPI_MOSI 5 //maybe modify
  #define LORA_PIN_SPI_MISO 19 //maybe modify
  #define LORA_PIN_SPI_SCK  18//14 = 1 Jumperstellung //18 = 3 Jumperstellung //maybe modify
  #define LORA_PIN_SPI_NSS  26  //maybe modify
  #define LORA_PIN_SPI_RST  33  //maybe modify
  #define LORA_PIN_SPI_DIO1 27//maybe modify
  #define LORA_PIN_SPI_DIO0 27//maybe modify
#endif


//Battery Pin
  #define BATTERY_PIN 35
  
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>

//#include "LoraEncoder.h"
#include <LoraMessage.h> //https://github.com/thesolarnomad/lora-serialization/blob/master/src/LoraMessage.h

//Temperatur:
#include <Wire.h>
#ifdef BMP280_CONNECTED
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp; // I2C
#endif

#ifdef BME280_CONNECTED
  #define SEALEVELPRESSURE_HPA (1013.25)
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme;
#endif

#ifdef CCS811_CONNECTED 
  #include <SparkFunCCS811.h> //Click here to get the library: http://librarymanager/All#SparkFun_CCS811
  CCS811 myCCS811(CCS811_ADRESS);
#endif


#ifdef geraet3
  #define sda 21 ///* I2C Pin Definition */
  #define scl 22 ///* I2C Pin Definition */
#else
  #define sda 21 ///* I2C Pin Definition */
  #define scl 22 ///* I2C Pin Definition */
#endif

float temp = 23;
float pressure = 980;
float humidity = 50;
float ccs811_CO2 = 450;
float ccs811_TVOC = 2;


//Wifi
#include "WiFi.h"


uint32_t start_time;
uint32_t next_time;
uint32_t previous_time;
uint32_t previous_full_update;

uint32_t total_seconds = 0;
uint32_t startup_seconds = 0;
uint32_t seconds, minutes, hours, days;

uint32_t join_total_seconds = 0;
uint32_t join_seconds, join_minutes, join_hours, join_days;

//Partial Update:
const uint32_t partial_update_period_s = 30;
//const uint32_t full_update_period_s = 60 * 60 * 60;//alle 2.5 Tage
const uint32_t full_update_period_s = 60 * 60;//(jede Stunde)
//const uint32_t full_update_period_s = 60;(jede Minute)
/*EINK Teile ENDE*/

//Variables for ABP after OTAA Join
u1_t NWKSKEY[16] ;                               // LoRaWAN NwkSKey, network session key.
u1_t APPSKEY[16] ;                               // LoRaWAN AppSKey, application session key.
u4_t DEVADDR ;

#ifdef geraet1
  #define DATAVALID 0xACF2AFC2//1                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet2
  #define DATAVALID 0xACF2AFC2//3                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet3
  #define DATAVALID 0xACF2AFCA                     // Pattern for data valid in EEPROM/RTC memory
#endif

//RealTimeVariables which don't get erased during Sleep
  
  RTC_DATA_ATTR uint32_t SAVED_dataValid ;                           // DATAVALID if valid data (joined)
  RTC_DATA_ATTR uint8_t  SAVED_devaddr[4] ;                          // Device address after join
  RTC_DATA_ATTR uint8_t  SAVED_nwkKey[16] ;                          // Network session key after join
  RTC_DATA_ATTR uint8_t  SAVED_artKey[16] ;                          // Aplication session key after join
  RTC_DATA_ATTR uint32_t SAVED_seqnoUp ;                             // Sequence number       
  bool OTAA = true ;   //startup with OTAA if No EEPROM was Saved before
  

#ifdef geraet3
  //01 bei ttgo lora eink
  /*static const u1_t PROGMEM APPEUI[8]={ 0x48, 0x7F, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ 0x01, 0x00, 0x01, 0x06, 0x05, 0x02, 0x03, 0x07 };
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  
  static const u1_t PROGMEM APPKEY[16] = { 0xC9, 0x62, 0xB9, 0x87, 0xC1, 0x14, 0xA7, 0x13, 0xA3, 0xDF, 0xAB, 0x4B, 0x4E, 0x15, 0xC6, 0xB6 };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}*/

  //sensornet_ccs811_bme280_ App 03
  static const u1_t PROGMEM APPEUI[8]={ 0x5B, ...//lsb }; //maybe modify
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ 0x08, ...//lsb};//maybe modify
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  
  static const u1_t PROGMEM APPKEY[16] = { 0x61, ...//msb};//maybe modify
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif


int verbunden_indicator = 0;
int Packet_Transmission_ongoing = 0; // Display wird nur aktualisiert wenn kein Paket gesendet wird
int packet_counter =0;
int packet_counter_rx =0;
static osjob_t sendjob;
devaddr_t DeviceName = 0;

// Pin mapping

const lmic_pinmap lmic_pins = {
    .nss = LORA_PIN_SPI_NSS, 
    .rxtx = LMIC_UNUSED_PIN, 
    .rst = LMIC_UNUSED_PIN,
    #ifdef geraet1
      .dio = {LORA_PIN_SPI_DIO0, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN}, //maybe modify
    #else
      .dio = {LORA_PIN_SPI_DIO1, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN}, //maybe modify
    #endif
    /*
    //workaround to use 1 pin for all 3 radio dio pins
    // optional: set polarity of rxtx pin.
    .rxtx_rx_active = 0,
    // optional: set RSSI cal for listen-before-talk
    // this value is in dB, and is added to RSSI
    // measured prior to decision.
    // Must include noise guardband! Ignored in US,
    // EU, IN, other markets where LBT is not required.
    .rssi_cal = 0,
    // optional: override LMIC_SPI_FREQ if non-zero
    .spi_freq = 0,*/
    .rxtx_rx_active = 0,//kopiert von TTGO Board
    .rssi_cal = 10,//kopiert von TTGO Board
    //.spi_freq = 8000000, /* 8MHz */ //kopiert von TTGO Board
    .spi_freq = 4000000, //4 MHZ from GxEPD2 https://github.com/ZinggJM/GxEPD2/tree/master/extras/sw_spi //maybe modify
};



void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            verbunden_indicator = 1;
            break;
        case EV_JOINED:
            verbunden_indicator = 2;
            total_seconds =0;
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              DeviceName = devaddr;
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
              saveToRTC(); // Speichere Werte
              //SAVED_dataValid = 0;// Setze zu Testzwecken Datavalid = 0 damit Daten aus dem EEPROM Gelesen werden
              //retrieveKeys(); /Rücklesen der Werte aus EEPROM
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            verbunden_indicator = 4;
            packet_counter++;
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              for (int i = 0; i < LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                  Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                packet_counter_rx++;
              }
              Serial.println(F(" das war der Inhalt"));
            }
            Packet_Transmission_ongoing = 0;
            saveToRTC();// Versuch
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            verbunden_indicator = 3;
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: No JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        //byte buffer[2];
        //LoraEncoder encoder(buffer);
        //encoder.writeTemperature(temp);
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
        
        LoraMessage message;
        //message.addUnixtime(1468075322);
        //message.addLatLng(-33.905024, 151.26648);
        message.addTemperature(temp);//1 //maybe modify
        message.addHumidity(humidity);//2 //maybe modify
        message.addUint16(int(pressure));//4 //maybe modify
        message.addUint16(int(ccs811_CO2));//5 //maybe modify
        message.addUint16(int(ccs811_TVOC));//6 //maybe modify
        
        //message.addHumidity(
        LMIC_setTxData2(1, message.getBytes(), message.getLength(), 0);
        
        //Serial.print("Temp:");Serial.print(temp); 
        Serial.println(F("Packet queued"));
        Packet_Transmission_ongoing = 1;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}





//Init Lora Stack, sets ADR Mode, 
void setup_lora(void){
  pinMode(LORA_PIN_SPI_DIO1, INPUT_PULLDOWN);//to enable PullDown but update your ESP32 Lib to avoid https://esp32.com/viewtopic.php?t=439
  pinMode(LORA_PIN_SPI_DIO0, INPUT_PULLDOWN);
  // LMIC init
    os_init();
    
    LMIC_reset(); // Reset the MAC state. Session and pending data transfers will be discarded.
    
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band //maybe modify
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band //maybe modify
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band //maybe modify
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band //maybe modify
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band //maybe modify
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band //maybe modify
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band //maybe modify
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band //maybe modify
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band //maybe modify
    
    //Debugging Purpose for Single Channel
    #ifdef SingleChannelMode
      #define CHANNEL  1
      for (uint8_t i = 0; i < 9; i++) {
        if (i != CHANNEL) {
          LMIC_disableChannel(i);
        }
      }
    #endif
    
    //Adaptive Data Rate Mode https://www.thethingsnetwork.org/docs/lorawan/adaptive-data-rate.html 
    //LMIC_setAdrMode(1); //Adapts Datenrate nach 64 Paketen
    LMIC_setLinkCheckMode(0);

    
    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
    
    // Set data rate and transmit power for uplink moved depending if OTAA or ABP
    
    //LMIC_setDrTxpow(DR_SF12,14);//Langsamster Modus: elendslange AirTime ~820ms aber sichere Übertragung
    //LMIC_setDrTxpow(DR_SF8,14); //Kurz schnell unzuverlässig 
    
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);//https://www.thethingsnetwork.org/forum/t/need-help-with-mcci-lmic-and-ttn-join-wait-issue/30846

    if ( OTAA )
    {
      // start joining
      Serial.println ( "Start joining" ) ;
      LMIC_setDrTxpow(DR_SF9,14); //with SF9 initial Join is faster... yes I know for this Reset Cycle it will stay on SF9 and consume more airtime
      do_send (&sendjob) ;
    }
    else
    {
      Serial.printf ( "starte mit SF8: gespeichertes SAVED_seqnoUp: %d\n", SAVED_seqnoUp ) ;
      LMIC_setDrTxpow(DR_SF8,14); //otherwise stupid ABP will start with SF12!!!
      memdmp ( "No OTAA needed - Set Session, DEVADDR:", (uint8_t*)&DEVADDR, 4 ) ;
      memdmp ( "NWKSKEY:", NWKSKEY, 16 ) ;
      memdmp ( "APPSKEY:", APPSKEY, 16 ) ;
      LMIC_setSession ( 0x13, DEVADDR, NWKSKEY, APPSKEY ) ;
      //Serial.printf ( "Seqnr set to %d\n", SAVED_seqnoUp ) ;
      LMIC.seqnoUp = SAVED_seqnoUp ;  
      do_send (&sendjob) ;
    }
    
    
    //Eingebaut in /src/lmic.c geht nicht
    //LMIC_dn2dr = EU868_DR_SF9;//https://github.com/mcci-catena/arduino-lmic/issues/455
    //LMIC_selectSubBand(0); //https://github.com/mcci-catena/arduino-lorawan/issues/74
    // Start job (sending automatically starts OTAA too)
}


//***************************************************************************************************
//                                      M E M D M P                                                 *
//***************************************************************************************************
// Dump memory for debug
//***************************************************************************************************
void memdmp ( const char* header, uint8_t* p, uint16_t len )
{
  uint16_t i ;                                                        // Loop control

  Serial.print ( header ) ;                                           // Show header
  for ( i = 0 ; i < len ; i++ )
  {
    if ( ( i & 0x0F ) == 0 )                                          // Continue opn next line?
    {
      if ( i > 0 )                                                    // Yes, continuation line?
      {
        Serial.printf ( "\n" ) ;                                      // Yes, print it
      }
      Serial.printf ( "%04X: ", i ) ;                                 // Print index
    }
    Serial.printf ( "%02X ", *p++ ) ;                                 // Print one data byte
  }
  Serial.println() ;
}


//***************************************************************************************************
//                                    S A V E T O R T C                                             *
//***************************************************************************************************
// Save data in RTC memory.  Every 100th call the data will also be saved in EEPROM memory.         *
// The EEPROM is also updates if OTAA was used.                                                     *
// The space in RTC memory is limited to 512 bytes.                                                 *
//***************************************************************************************************
void saveToRTC()
{
  uint16_t        eaddr ;                                  // Address in EEPROM
  uint8_t*        p ;                                      // Points into savdata

  Serial.printf ( "\n Save data to RTC memory:\n" ) ;
  memcpy ( SAVED_devaddr, &LMIC.devaddr, 4 ) ;           // Fill struct to save
  memcpy ( SAVED_nwkKey,  LMIC.nwkKey, 16 ) ;
  memcpy ( SAVED_artKey,  LMIC.artKey, 16 ) ;
  SAVED_seqnoUp = LMIC.seqnoUp ;
  SAVED_dataValid = DATAVALID ;
  memdmp ( "devaddr:", SAVED_devaddr, 4 ) ;  
  memdmp ( "artKey:",  SAVED_artKey, 16 ) ;
  memdmp ( "nwkKey:",  SAVED_nwkKey, 16 ) ;
  Serial.printf ( "SeqnoUp is %d\n", SAVED_seqnoUp ) ;
  Serial.printf ( "SeqnoDown is %d\n", LMIC.seqnoDn ) ;
  if ( ( ( LMIC.seqnoUp % 50 ) == 0 ) || OTAA )           // Need to save data in EEPROM?
  {
    int EEPROM_Data_Counter =0;
    
    Serial.println ( "Saving to EEPROM" ) ;
    p = (uint8_t*)&SAVED_dataValid ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < sizeof(SAVED_dataValid) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
      
    }
    Serial.printf ( "\n Saved %d Bytes of datavalid to EEPROM", EEPROM_Data_Counter);
    p = (uint8_t*)&SAVED_devaddr ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of devadr to EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)& SAVED_nwkKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
      Serial.printf ( "\n Saved %d Bytes of nwkey to EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)& SAVED_artKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of artkey to EEPROM", EEPROM_Data_Counter);   
    
    p = (uint8_t*)& SAVED_seqnoUp ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)+sizeof(SAVED_seqnoUp)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of seqnr to EEPROM", EEPROM_Data_Counter);   
    
    EEPROM.commit() ;                                      // Commit data to EEPROM
    Serial.printf ( "\n EEPROM operation finished Number of bytes Written: %d", EEPROM_Data_Counter);   
  }
}

//***************************************************************************************************
//                                R E T R I E V E K E Y S                                           *
//***************************************************************************************************
// Try to retrieve the keys en seqnr from non-volitile memory.                                      *
//***************************************************************************************************
void retrieveKeys()
{
  uint16_t eaddr ;                                          // Address in EEPROM
  uint8_t* p ;                                              // Pointer into savdata
  
  // return ;                                               // Return if OTAA is required
  
  //Hier Entscheidung ob RTC Gültig ist oder Defaultwerte drin und aus EEPROM Gelesen werden muss
  //z.B bei Reboot, battery getauscht usw.
  if ( SAVED_dataValid == DATAVALID )                     // DATA in RTC memory valid?
  {
    Serial.println ( "Keys retrieved from RTC memory\n" ) ; // Show retrieve result 
  }
  else
  {
    Serial.println ( "\n Reading Keys from EEPROM :\n" ) ; 
    // No data vailable in RTC memory.  Use EEPROM data. Hole alles aus EEPROM
    int EEPROM_Data_Counter =0;
    p = (uint8_t*)&SAVED_dataValid ;
    for ( eaddr = EEPROM_Data_Counter ; eaddr < sizeof(SAVED_dataValid) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of datavalid from EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)&SAVED_devaddr ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of devadr from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_nwkKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
      Serial.printf ( "\n Recovered %d Bytes of nwkKey from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_artKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of artKey from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_seqnoUp ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)+sizeof(SAVED_seqnoUp)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of SeqnoUp from EEPROM", EEPROM_Data_Counter);    
    SAVED_seqnoUp += 50 ;                                // Counter may be not up-to-date
    Serial.println ( "Recovered Keys from EEPROM are:\n" ) ; 
    /*
    memdmp ( "devaddr is:",
             SAVED_devaddr, 4 ) ;
    memdmp ( "appsKey is:",
             SAVED_artKey, 16 ) ;
    memdmp ( "nwksKey is:",
             SAVED_nwkKey, 16 ) ;
    */
  }

  //check if Data is valid after it has been readout from EEPROM
  if ( SAVED_dataValid == DATAVALID )                     // DATA in RTC or EEPROM memory valid?
  {
    Serial.printf ( "Valid data in NVS\n" ) ;               // Yes, show
    memdmp ( "devaddr is:",
             SAVED_devaddr, 4 ) ;
    memdmp ( "nwksKey is:",
             SAVED_nwkKey, 16 ) ;
    memdmp ( "appsKey is:",
             SAVED_artKey, 16 ) ;
    Serial.printf ( "seqnr is %d\n", SAVED_seqnoUp ) ;
    memcpy ( (uint8_t*)&DEVADDR,
             SAVED_devaddr, sizeof(DEVADDR) ) ;          // LoraWAN DEVADDR, end node device address
    memcpy ( NWKSKEY,
             SAVED_nwkKey,  sizeof(NWKSKEY) ) ;          // LoRaWAN NwkSKey, network session key.
    memcpy ( APPSKEY,
             SAVED_artKey,  sizeof(APPSKEY) ) ;          // LoRaWAN AppSKey, application session key.
    OTAA = false ;                                         // Do not use OTAA
  }
  else
  {
    Serial.printf ( "No saved data, using OTAA\n" ) ;
  }
}

void begin_sleep(){
  //esp_sleep_enable_timer_wakeup(UpdateInterval);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println(F("Starting deep-sleep period..."));
  esp_deep_sleep_start();         // Sleep for e.g. 30 minutes
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    EEPROM.begin ( 512 ) ; 
    Serial.println(F("Starting"));
    
    retrieveKeys(); 
    
    Serial.println("** Stopping WiFi+BT");
    WiFi.mode(WIFI_OFF);
    btStop();

    setup_lora();    
}


void loop() {
    os_runloop_once();
    if (verbunden_indicator == 4){
      verbunden_indicator = 2;
       #ifdef geraet3
         //do not use display
       #else
          //showPartialUpdate(); 
          //showPartialUpdateWetter(wetter_symbol_x, wetter_symbol_y, wetter_symbol_size);
       #endif
       //begin_sleep();
    }
}
