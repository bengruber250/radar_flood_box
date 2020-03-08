//from ext
#include <SPI.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include "IWR1443.h"
//EDIT BELOW FOR SENSOR NODE ID
#define NODE_ID 3
#define SLEEP_TRIGGER 3
#define PULL_UP 8
#define TEMP_PIN A0
// iot example
#define turnOffShield // Turn off shield after posting data


//volatile int f_timer = 0;
//volatile int cycle_count = 0;
//volatile int pull_up_count = 0;
volatile int packet_id = 0;

/*  This is an example sketch to test the core functionalities of SIMCom-based cellular modules.
    This code supports the SIM7000-series modules (LTE/NB-IoT shields) for low-power IoT devices!

    Note: this code is specifically meant for AVR microcontrollers (Arduino Uno, Mega, Leonardo, etc)
    However, if you are using an ESP8266 please make the minor modifications mentioned below in the
    comments for the pin definitions and software serial initialization.

    For ESP32 please use the ESP32_LTE_Demo instead: https://github.com/botletics/SIM7000-LTE-Shield/blob/master/Code/examples/ESP32_LTE_Demo/ESP32_LTE_Demo.ino

    Author: Timothy Woo (www.botletics.com)
    Github: https://github.com/botletics/SIM7000-LTE-Shield
    Last Updated: 1/15/2018
    License: GNU GPL v3.0
*/

/******* ORIGINAL ADAFRUIT FONA LIBRARY TEXT *******/
/***************************************************
  This is an example for our Adafruit FONA Cellular Module

  Designed specifically to work with the Adafruit FONA
  ----> http://www.adafruit.com/products/1946
  ----> http://www.adafruit.com/products/1963
  ----> http://www.adafruit.com/products/2468
  ----> http://www.adafruit.com/products/2542

  These cellular modules use TTL Serial to communicate, 2 pins are
  required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_FONA.h"

#define SIMCOM_7000 // SIM7000A/C/E/G

// For SIM7000 shield
#define FONA_PWRKEY 6
#define FONA_RST 7
#define FONA_TX 10 // Microcontroller RX
#define FONA_RX 11 // Microcontroller TX
//#define T_ALERT 12 // Connect with solder jumper


// We default to using software serial. If you want to use hardware serial
// (because softserial isnt supported) comment out the following three lines 
// and uncomment the HardwareSerial line
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial debug = SoftwareSerial(2, 12);
unsigned int tempValue = 0;

  
// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
// Notice how we don't include the reset pin because it's reserved for emergencies on the LTE module!
#if defined(SIMCOM_7000) || defined(SIMCOM_7500)
  Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();
#endif

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!

void wdt_setup();
void powerOn();
void flushSerial();

void setup() {
  wdt_setup();
  
  debug.begin(9600);
  pinMode(SLEEP_TRIGGER, OUTPUT);
  
  debug.println("init iwr...");
  IWR1443 iwr = IWR1443(debug);
  iwr.setupComms();
  debug.println("init iwr...OK");
  
  debug.println("reading from maxbotix..."); 
  double results[5];
  iwr.getSamples(results, 5);
  debug.println("reading from maxbotix...OK"); 
  debug.print("Maxbotix printing...");
  for (int j = 0; j<5; j++) {
    wdt_reset();
    debug.println(results[j]);
  }
  //TODO turn off the radar pls
  iwr.stopComms();
  
  debug.println(F("FONA basic test"));
  debug.println(F("Initializing....(May take several seconds)"));

  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH); // Default state

  pinMode(FONA_PWRKEY, OUTPUT);

  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  powerOn(); // See function definition at the very end of the sketch

  //delay(5000);
  // Note: The SIM7000A baud rate seems to reset after being power cycled (SIMCom firmware thing)
  // SIM7000 takes about 3s to turn on but SIM7500 takes about 15s
  // Press reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset
  
  fonaSS.begin(115200); // Default SIM7000 shield baud rate
  
  
  fonaSS.listen();
  
  
  debug.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    debug.println(F("Couldn't find FONA"));
    while (1); // Don't proceed if it couldn't find the device
  }
  //delay(5000); <--- breaks the system
  type = fona.type();
  debug.println(F("FONA is OK"));
  debug.print(F("Found "));
  switch (type) {
    case SIM800L:
      debug.println(F("SIM800L")); break;
    case SIM800H:
      debug.println(F("SIM800H")); break;
    case SIM808_V1:
      debug.println(F("SIM808 (v1)")); break;
    case SIM808_V2:
      debug.println(F("SIM808 (v2)")); break;
    case SIM5320A:
      debug.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      debug.println(F("SIM5320E (European)")); break;
    case SIM7000A:
      debug.println(F("SIM7000A (American)")); break;
    case SIM7000C:
      debug.println(F("SIM7000C (Chinese)")); break;
    case SIM7000E:
      debug.println(F("SIM7000E (European)")); break;
    case SIM7000G:
      debug.println(F("SIM7000G (Global)")); break;
    case SIM7500A:
      debug.println(F("SIM7500A (American)")); break;
    case SIM7500E:
      debug.println(F("SIM7500E (European)")); break;
    default:
      debug.println(F("???")); break;

  }

  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    debug.print("Module IMEI: "); debug.println(imei);
  }

  // Set modem to full functionality
  fona.setFunctionality(1); // AT+CFUN=1
  wdt_reset();

  // Configure a GPRS APN, username, and password.
  // You might need to do this to access your network's GPRS/data
  // network.  Contact your provider for the exact APN, username,
  // and password values.  Username and password are optional and
  // can be removed, but APN is required.
  fona.setNetworkSettings(F("hologram")); // For Hologram SIM card

  // Optionally configure HTTP gets to follow redirects over SSL.
  // Default is not to follow SSL redirects, however if you uncomment
  // the following line then redirects over SSL will be followed.
  //fona.setHTTPSRedirect(true);

  /*
  // Other examples of some things you can set:
  fona.setPreferredMode(38); // Use LTE only, not 2G
  fona.setPreferredLTEMode(1); // Use LTE CAT-M only, not NB-IoT
  fona.setOperatingBand("CAT-M", 12); // AT&T uses band 12
//  fona.setOperatingBand("CAT-M", 13); // Verizon uses band 13
  fona.enableRTC(true);
  
  fona.enableSleepMode(true);
  fona.set_eDRX(1, 4, "0010");
  fona.enablePSM(true);

  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  fona.setNetLED(true, 2, 64, 3000); // on/off, mode, timer_on, timer_off
  fona.setNetLED(false); // Disable network status LED
  */

  //printMenu();

  debug.print("Turning GPRS OFF... JK ");
  // turn GPRS off(g)
//  if (!fona.enableGPRS(false))
//    debug.println(F("Failed to turn off"));
  
  debug.println("Done.");
  
  // turn GPRS off first for SIM7500(G)
  #ifdef SIMCOM_7500
    fona.enableGPRS(false);
    debug.print("through");
  #endif
        
  debug.print("Turning GPRS ON... ");
  // turn GPRS on(G)
  wdt_disable();
  int connection_attempts = 0;
  while(!fona.enableGPRS(true)) {
    wdt_setup();
    connection_attempts++;
    if (connection_attempts >= 3) {
      debug.println("Failed to talk to tower 3 times.");
      debug.println("Triggering System Power Off");
      digitalWrite(SLEEP_TRIGGER, HIGH);
      break;
    }
    wdt_disable();
  }
  wdt_setup();
  debug.println("Done.");
  //from
  
  pinMode(PULL_UP, OUTPUT);

  digitalWrite(PULL_UP, LOW);
  packet_id = EEPROM.read(0);
  //packet tracking
  packet_id = packet_id + 1;
  if (packet_id == 256) {
    packet_id = 0;
  }


  char data[50] = {};
  char sensor_arr[5][5] = {};
  char temp_arr[5] = {};
  char node_id_arr[5] = {};
  char packet_id_arr[5] = {};

  memset(data, 0, sizeof(data));

  itoa(NODE_ID, node_id_arr, 10);
  itoa(packet_id, packet_id_arr, 10);
  
  for (int j = 0; j < 5; j++) {
    char heightVal[6];
    dtostrf(results[j], 5, 0, heightVal);
    debug.println(results[j]);
    //snprintf(data + strlen(data), 5, "%f", results[j]);
    strcat(data, heightVal);
    strcat(data, ",");
  }
  
  itoa(tempValue, temp_arr, 10);
  strcat(data, temp_arr);
  strcat(data, ",");
  
  strcat(data, packet_id_arr);
  strcat(data, ",");
  strcat(data, node_id_arr);
  
  char data_message[150] = {};
  strcat(data_message, "{\"k\":\"SZxBwu9,\",\"d\":\"");
  strcat(data_message, data);
  strcat(data_message, "\",\"t\":[\"sensor");
  strcat(data_message, node_id_arr);
  strcat(data_message, "\",\"data\",\"senior_design\"]}");
  
  debug.print("Message to send: ");
  debug.println(data_message);
  
  // strcat("{\"k\":\"8bav[_7+\",\"d\":\"send this 4\",\"t\":[\"topic1\",\"topic2\"]}");
  int len_data = strlen(data_message);
  char len_arr[2] ={};
  itoa(len_data,len_arr,10);
  
  char send_command[13] = {};
  debug.println("Starting Send");
  strcat (send_command, "AT+CIPSEND=");
  strcat (send_command, len_arr);
   
  wdt_reset();
  delay(1000);
  fonaSS.println("AT+CIPSHUT");
  delay(500);
  fonaSS.println("AT+CIPSTART= \"TCP\", \"cloudsocket.hologram.io\",9999");
  delay(2000);
  wdt_reset();
  fonaSS.println(send_command);
  delay(500);
  fonaSS.println(data_message);
  delay(2000);

  wdt_reset();
  debug.println("Done sending");
  EEPROM.write(0,packet_id);


  // turn off shield
  #ifdef turnOffShield
  // Disable GPRS
  // Note that you might not want to check if this was successful, but just run it
  // since the next command is to turn off the module anyway
  if (!fona.enableGPRS(false)) debug.println(F("Failed to disable GPRS!"));

  // Turn off GPS
  //if (!fona.enableGPS(false)) debug.println(F("Failed to turn off GPS!"));
  
  // Power off the module. Note that you could instead put it in minimum functionality mode
  // instead of completely turning it off. Experiment different ways depending on your application!
  // You should see the "PWR" LED turn off after this command
//  if (!fona.powerDown()) debug.println(F("Failed to power down FONA!")); // No retries
  int  counter = 0;
  while (counter < 3 && !fona.powerDown()) { // Try shutting down 
    debug.println(F("Failed to power down FONA!"));
    counter++; // Increment counter
    }
   #endif

   delay(2000);
   //turn off board
   debug.println("Triggering System Shutdown.");
   digitalWrite(SLEEP_TRIGGER, HIGH);

}


void loop() {
  /*
    case 'g': {
        // turn GPRS off
        if (!fona.enableGPRS(false))
          debug.println(F("Failed to turn off"));
        break;
      }
    case 'G': {
        // turn GPRS off first for SIM7500
        #ifdef SIMCOM_7500
          fona.enableGPRS(false);
        #endif
        
        // turn GPRS on
        if (!fona.enableGPRS(true))
          debug.println(F("Failed to turn on"));
        break;
      }
      */
    
  // flush input
  flushSerial();
  while (fona.available()) {
    debug.write(fona.read());
  }

}

void flushSerial() {
  while (debug.available())
    debug.read();
}

char readBlocking() {
  while (!debug.available());
  return debug.read();
}
uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //debug.print(c);
  }
  debug.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    debug.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
  uint16_t buffidx = 0;
  boolean timeoutvalid = true;
  if (timeout == 0) timeoutvalid = false;

  while (true) {
    if (buffidx > maxbuff) {
      //debug.println(F("SPACE"));
      break;
    }

    while (debug.available()) {
      char c =  debug.read();

      //debug.print(c, HEX); debug.print("#"); debug.println(c);

      if (c == '\r') continue;
      if (c == 0xA) {
        if (buffidx == 0)   // the first 0x0A is ignored
          continue;

        timeout = 0;         // the second 0x0A is the end of the line
        timeoutvalid = true;
        break;
      }
      buff[buffidx] = c;
      buffidx++;
    }

    if (timeoutvalid && timeout == 0) {
      //debug.println(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  buff[buffidx] = 0;  // null term
  return buffidx;
}

// Power on the module
void powerOn() {
  digitalWrite(FONA_PWRKEY, LOW);
  // See spec sheets for your particular module
  #if defined(SIMCOM_2G)
    delay(1050);
  #elif defined(SIMCOM_3G)
    delay(180); // For SIM5320
  #elif defined(SIMCOM_7000)
    delay(100); // For SIM7000
  #elif defined(SIMCOM_7500)
    delay(500); // For SIM7500
  #endif
  
 digitalWrite(FONA_PWRKEY, HIGH);
}

// Watchdog timer setup code
void wdt_setup(){
  wdt_reset();
  cli();
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) |(1<<WDP0);
  sei();
}

// ISR(WDT_vect)
// {
//   digitalWrite(SLEEP_TRIGGER, HIGH);
// }
