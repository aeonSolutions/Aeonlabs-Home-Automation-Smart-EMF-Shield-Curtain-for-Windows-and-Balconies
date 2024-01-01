/*
  Copyright (c) 2023 Miguel Tomas, http://www.aeonlabs.science

  License Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)
  You are free to:
   Share — copy and redistribute the material in any medium or format
   Adapt — remix, transform, and build upon the material

  The licensor cannot revoke these freedoms as long as you follow the license terms. Under the following terms:
  Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.

  NonCommercial — You may not use the material for commercial purposes.

  ShareAlike — If you remix, transform, or build upon the material, you must distribute your contributions under
  the same license as the original.

  No additional restrictions — You may not apply legal terms or technological measures that legally restrict others
  from doing anything the license permits.

  Notices:
  You do not have to comply with the license for elements of the material in the public domain or where your use
  is permitted by an applicable exception or limitation.
  No warranties are given. The license may not give you all of the permissions necessary for your intended use.
  For example, other rights such as publicity, privacy, or moral rights may limit how you use the material.


  Before proceeding to download any of AeonLabs software solutions for open-source development
  and/or PCB hardware electronics development make sure you are choosing the right license for your project. See
  https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/wiki/AeonLabs-Solutions-for-Open-Hardware-&-Source-Development
  for Open Hardware & Source Development for more information.

*/

#define uS_TO_S_FACTOR 1000000

/************************* Adafruit.io Setup *********************************/
//required message queue telemetry transport protocol to subscribe and publish feeds
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// the adafruit server to connect
#define AIO_SERVER    "io.adafruit.com"

// the port to connect
#define AIO_SERVERPORT 1883

// get this key from your own Adafruit Account
#define AIO_USERNAME "XXXXXXXX"
#define AIO_KEY "XXXXXXXXXXXXXXXXXXX"
#include <WiFi.h>
WiFiClient wclient;

Adafruit_MQTT_Client mqtt(&wclient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//Feeds Settings

//temperature value that will be read through BMP180 sensor
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temp");

//pressure value that will be read through BMP180 sensor
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidity");

//altitude value that will be read through BMP180 sensor
Adafruit_MQTT_Publish emp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/emp");


//----------------------------------------------------------------------------------------
// Components Testing  **************************************
bool SCAN_I2C_BUS = false;
bool TEST_FINGERPRINT_ID_IC = false;

//----------------------------------------------------------------------------------
#include <math.h>
#include <cmath>
#include "SPI.h"
#include <semphr.h>

#include "esp32-hal-psram.h"
// #include "rom/cache.h"
extern "C"
{
#include <esp_himem.h>
#include <esp_spiram.h>
}

#include <Wire.h>
// custom includes **********************************
#include "nvs_flash.h"  //preferences lib

// External sensor moeasurements
#include "telegram_emp.h"
TELEGRAM_EMP_CLASS* telegram = new TELEGRAM_EMP_CLASS();

// custom functions
#include "m_file_functions.h"

// Interface class ******************************
#include "interface_class.h"
INTERFACE_CLASS* interface = new INTERFACE_CLASS();
#define DEVICE_NAME "EMP Monitor"

// GBRL commands  ***************************
#include "gbrl.h"
GBRL gbrl = GBRL();

// Onboard sensors  *******************************
#include "onboard_sensors.h"
ONBOARD_SENSORS* onBoardSensors = new ONBOARD_SENSORS();

//
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// Pin connected to the ALERT/RDY signal for new sample notification.
constexpr int READY_PIN = 10;


// unique figerprint data ID
#include "m_atsha204.h"

// serial comm
#include <HardwareSerial.h>
HardwareSerial UARTserial(0);

#include "mserial.h"
mSerial* mserial = new mSerial(true, nullptr);

// File class
#include <esp_partition.h>
#include "FS.h"
#include <LittleFS.h>
#include "m_file_class.h"

FILE_CLASS* drive = new FILE_CLASS(mserial);

// WIFI Class
#include <ESP32Ping.h>
#include "m_wifi.h"
#include "esp_wifi.h"
M_WIFI_CLASS* mWifi = new M_WIFI_CLASS();

// Certificates
#include "cert/github_cert.h"

/********************************************************************/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pCharacteristicTX, *pCharacteristicRX;
BLEServer *pServer;
BLEService *pService;


bool BLE_advertise_Started = false;

bool newValueToSend = false;
String $BLE_CMD = "";
bool newBLESerialCommandArrived = false;
SemaphoreHandle_t MemLockSemaphoreBLE_RX = xSemaphoreCreateMutex();

float txValue = 0;
String valueReceived = "";


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      mWifi->setBLEconnectivityStatus (true);
      mserial->printStr("BLE connection init ", mserial->DEBUG_BOTH_USB_UART);

      interface->onBoardLED->led[0] = interface->onBoardLED->LED_BLUE;
      interface->onBoardLED->statusLED(100, 1);

      String dataStr = "Connected to the Smart Concrete Maturity device (" + String(interface->firmware_version) + ")" + String(char(10)) + String(char(13)) + "Type $? or $help to see a list of available commands" + String(char(10));
      dataStr += String(interface->rtc.getDateTime(true)) + String(char(10)) + String(char(10));

      if (mWifi->getNumberWIFIconfigured() == 0 ) {
        dataStr += "no WiFi Networks Configured" + String(char(10)) + String(char(10));
      }
      //interface->sendBLEstring(dataStr, mserial->DEBUG_TO_BLE);
    }

    void onDisconnect(BLEServer* pServer) {
      mWifi->setBLEconnectivityStatus (false);

      interface->onBoardLED->led[0] = interface->onBoardLED->LED_BLUE;
      interface->onBoardLED->statusLED(100, 0.5);
      interface->onBoardLED->led[0] = interface->onBoardLED->LED_RED;
      interface->onBoardLED->statusLED(100, 0.5);
      interface->onBoardLED->led[0] = interface->onBoardLED->LED_BLUE;
      interface->onBoardLED->statusLED(100, 0.5);

      pServer->getAdvertising()->start();
    }
};

class pCharacteristicTX_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String txValue = String(pCharacteristic->getValue().c_str());
      txValue.trim();
      mserial->printStrln("Transmitted TX Value: " + String(txValue.c_str()) , mserial->DEBUG_BOTH_USB_UART);

      if (txValue.length() == 0) {
        mserial->printStr("Transmitted TX Value: empty ", mserial->DEBUG_BOTH_USB_UART);
      }
    }

    void onRead(BLECharacteristic *pCharacteristic) {
      mserial->printStr("TX onRead...", mserial->DEBUG_BOTH_USB_UART);
      //pCharacteristic->setValue("OK");
    }
};

class pCharacteristicRX_Callbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      delay(10);

      String rxValue = String(pCharacteristic->getValue().c_str());
      rxValue.trim();
      mserial->printStrln("Received RX Value: " + String(rxValue.c_str()), mserial->DEBUG_BOTH_USB_UART );

      if (rxValue.length() == 0) {
        mserial->printStr("Received RX Value: empty " , mserial->DEBUG_BOTH_USB_UART);
      }

      $BLE_CMD = rxValue;
      mWifi->setBLEconnectivityStatus(true);

      xSemaphoreTake(MemLockSemaphoreBLE_RX, portMAX_DELAY);
      newBLESerialCommandArrived = true; // this needs to be the last line
      xSemaphoreGive(MemLockSemaphoreBLE_RX);

      delay(50);
    }

    void onRead(BLECharacteristic *pCharacteristic) {
      mserial->printStr("RX onRead..." , mserial->DEBUG_BOTH_USB_UART);
      //pCharacteristic->setValue("OK");
    }

};

// ********************************************************
// *************************  == SETUP == *****************
// ********************************************************
//variaveis que indicam o núcleo
static uint8_t taskCoreZero = 0;
static uint8_t taskCoreOne  = 1;

long int prevMeasurementMillis;


void setup() {
  ESP_ERROR_CHECK(nvs_flash_erase());
  nvs_flash_init();

  interface->settings_defaults();
  // Firmware Build Version / revision ______________________________
  interface->firmware_version = "1.0.0";

  // Serial Communication Init ______________________________
  interface->UARTserial = nullptr; // &UARTserial;
  mserial->DEBUG_TO = mserial->DEBUG_TO_UART;
  mserial->DEBUG_EN = true;
  mserial->DEBUG_TYPE = mserial->DEBUG_TYPE_VERBOSE; // DEBUG_TYPE_INFO;

  mserial->start(115200);

  // ......................................................................................................
  // .......................... START OF IO & PIN CONFIGURATION..............................................
  // ......................................................................................................

  // I2C IOs  __________________________
  interface->I2C_SDA_IO_PIN = 8;
  interface->I2C_SCL_IO_PIN = 9;
  int I2C_PWR_PIN=6;
  
  pinMode(I2C_PWR_PIN, OUTPUT);
  digitalWrite(I2C_PWR_PIN, HIGH);
  
  // WIRE.begin: SDA, SCL, FREQ
  bool result = Wire.begin( interface->I2C_SDA_IO_PIN , interface->I2C_SCL_IO_PIN ); //, 100000 );
  
  // Power Saving ____________________________________
  interface->LIGHT_SLEEP_EN = true;


  // ________________ Onboard LED  _____________
  interface->onBoardLED = new ONBOARD_LED_CLASS();
  interface->onBoardLED->LED_RED = 5;
  interface->onBoardLED->LED_BLUE = 2;
  interface->onBoardLED->LED_GREEN = 4;

  interface->onBoardLED->LED_RED_CH = 4;
  interface->onBoardLED->LED_BLUE_CH = 1;
  interface->onBoardLED->LED_GREEN_CH = 3;

  // ___________ MCU freq ____________________
  interface-> SAMPLING_FREQUENCY = 80;
  interface-> WIFI_FREQUENCY = 80; // min WIFI MCU Freq is 80-240
  interface->MIN_MCU_FREQUENCY = 80;
  interface-> SERIAL_DEFAULT_SPEED = 115200;

// _____________________ TELEGRAM _____________________________
  telegram->OWNER_CHAT_ID = "xxxxxxxxxxxx";
  // Initialize Telegram BOT
  telegram->BOTtoken = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";  // your Bot Token (Get from Botfather)

  // ......................................................................................................
  // .......................... END OF IO & PIN CONFIGURATION..............................................
  // ......................................................................................................

  interface->onBoardLED->init();

  interface->onBoardLED->led[0] = interface->onBoardLED->LED_RED;
  interface->onBoardLED->statusLED(100, 0);

  //init storage drive ___________________________
  drive->partition_info();
  if (drive->init(LittleFS, "storage", 2, mserial,   interface->onBoardLED ) == false)
    while (1);

  //init interface ___________________________
  interface->init(mserial, true); // debug EN ON

  if ( !interface->loadSettings() ) {
    interface->onBoardLED->led[0] = interface->onBoardLED->LED_RED;
    interface->onBoardLED->led[0] = interface->onBoardLED->LED_GREEN;
    interface->onBoardLED->statusLED(100, 2);
  }

  // init onboard sensors ___________________________
  onBoardSensors->init(interface, mserial);

  if (SCAN_I2C_BUS) {
    onBoardSensors->I2Cscanner();
  }
  onBoardSensors->initOnboardSensors();

  mserial->printStrln("Getting differential reading from AIN0 (P) and AIN1 (N)");
  mserial->printStrln("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    mserial->printStrln("Failed to initialize ADS.");
  }
  pinMode(READY_PIN, INPUT);
  
 
  // Data Authenticity IC
  if (TEST_FINGERPRINT_ID_IC) {
    mserial->printStrln("Testing the Unique FingerPrind ID for Sensor Data Measurements");

    mserial->printStr("This Smart Device  Serial Number is : ");
    mserial->printStrln(CryptoICserialNumber(interface));

    mserial->printStrln("Testing Random Genenator: " + CryptoGetRandom(interface));
    mserial->printStrln("");

    mserial->printStrln("Testing Sensor Data Validation hashing");
    mserial->printStrln( macChallengeDataAuthenticity(interface, "TEST IC"));
    mserial->printStrln("");
  }

  mserial->printStrln("\nMicrocontroller specifications:");
  interface->CURRENT_CLOCK_FREQUENCY = getCpuFrequencyMhz();
  mserial->printStr("Internal Clock Freq = ");
  mserial->printStr(String(interface->CURRENT_CLOCK_FREQUENCY));
  mserial->printStrln(" MHz");

  interface->Freq = getXtalFrequencyMhz();
  mserial->printStr("XTAL Freq = ");
  mserial->printStr(String(interface->Freq));
  mserial->printStrln(" MHz");

  interface->Freq = getApbFrequency();
  mserial->printStr("APB Freq = ");
  mserial->printStr(String(interface->Freq / 1000000));
  mserial->printStrln(" MHz");

  interface->setMCUclockFrequency( interface->WIFI_FREQUENCY);
  mserial->printStrln("setting Boot MCU Freq to " + String(getCpuFrequencyMhz()) +"MHz");
  mserial->printStrln("");

  // init BLE
  BLE_init();

    // init BLE
  BLE_init();

  //init wifi
  mWifi->init(interface, drive, interface->onBoardLED);
  mWifi->setTxPower = WIFI_POWER_8_5dBm;
  mWifi->OTA_FIRMWARE_SERVER_URL = "https://github.com/aeonSolutions/openScience-Smart-DAQ-to-Upload-Live-Experimental-Data-to-a-Data-Repository/releases/download/openFirmware/firmware.bin";
  
  mWifi->add_wifi_network("XXXXXXXXXXXXXXX","XXXXXXXXXXXXXXXX");
  mWifi->ALWAYS_ON_WIFI = false;

  mWifi->WIFIscanNetworks(true);

  // check for firmwate update
  mWifi->startFirmwareUpdate();
  
  mserial->printStrln( "\nRequesting Geo Location FingerPrint  =====================================" );
  mWifi->get_ip_geo_location_data("", true);
  
  ESP32Time rtc(0);
  rtc.setTime( mWifi->requestGeoLocationDateTime );

  mserial->printStrln("Internet IP       : " + mWifi->InternetIPaddress );
  mserial->printStrln("Geo Location Time : " + String( rtc.getDateTime(true) ) );
  if ( mWifi->geoLocationInfoJson.isNull() == false ){
    float lat =0.0f;
    float lon = 0.0f;

    if( mWifi->geoLocationInfoJson.containsKey("lat")){
      lat = mWifi->geoLocationInfoJson["lat"];
      mserial->printStr( "Latitude : " + String(lat,4) );
    }else{
      mserial->printStr( "Latitude : - -" );
    }

    if( mWifi->geoLocationInfoJson.containsKey("lon")){
      lon = mWifi->geoLocationInfoJson["lon"];
      mserial->printStr( "  Longitude: " + String(lon,4) );
    }else{
      mserial->printStr( "  Longitude: - -" );
    }
    
    if( mWifi->geoLocationInfoJson.containsKey("regionName")){
      String regionName = mWifi->geoLocationInfoJson["regionName"];
      mserial->printStr( "\nLocation: " + regionName );
    }else{
      mserial->printStr( "\nLocation: - -" );
    }

    if( mWifi->geoLocationInfoJson.containsKey("country")){
      String country = mWifi->geoLocationInfoJson["country"];
      mserial->printStrln( ", " + country );
    }else{
      mserial->printStrln( " , - -" );
    }

    mserial->printStrln( "\nGEO Location FingerPrint is : " );
    String geoFingerprint = String( interface->rtc.getEpoch()  ) + "-" + String(mWifi->requestGeoLocationDateTime) + "-" + mWifi->InternetIPaddress + "-" + String(lat) + "-" + String(lon);
    mserial->printStrln( geoFingerprint + "\nGEO Location FingerPrint is ID: " );
    geoFingerprint = macChallengeDataAuthenticity( interface, geoFingerprint );
    mserial->printStrln( geoFingerprint );
    mserial->printStrln("Offline response is:\n" + macChallengeDataAuthenticityOffLine(interface, (char*) geoFingerprint.c_str() ) );

  }
  mserial->printStrln( "\n ====================== done =======================" );
  
  interface->onBoardLED->led[0] = interface->onBoardLED->LED_RED;
  interface->onBoardLED->statusLED(100, 0);

    
  // initialize Telegram
  telegram->init(interface, mWifi);
  
  //Init GBRL
  gbrl.init(interface, mWifi);

  interface->$espunixtimePrev = millis();
  interface->$espunixtimeStartMeasure = millis();

  mWifi->$espunixtimeDeviceDisconnected = millis();

  prevMeasurementMillis = millis();
  
  mserial->printStr("\nStarting MCU cores... ");
/*
  xTaskCreatePinnedToCore (
    loop2,     // Function to implement the task
    "loop2",   // Name of the task
    1000,      // Stack size in bytes
    NULL,      // Task input parameter
    0,         // Priority of the task
    NULL,      // Task handle.
    0          // Core where the task should run
  );
*/
  MemLockSemaphoreBLE_RX = xSemaphoreCreateMutex();
  mserial->printStrln("done. ");

  mserial->printStrln("Free memory: " + addThousandSeparators( std::string( String(esp_get_free_heap_size() ).c_str() ) ) + " bytes");
  
  mserial->printStrln("============================================================================");
  mserial->printStrln("Setup is completed. You may start using the " + String(DEVICE_NAME) );
  mserial->printStrln("Type $? for a List of commands.");
  mserial->printStrln("============================================================================\n");

  interface->onBoardLED->led[0] = interface->onBoardLED->LED_GREEN;
  interface->onBoardLED->statusLED(100, 1);

}

// *********************** MQTT *****************************
bool MQTT_connect() {
   int8_t retvar;
   if (mqtt.connected()) {
      return true;
   }
   mserial->printStrln("Connecting to MQTT... ");
   uint8_t trytimes = 3;
   while ((retvar = mqtt.connect()) != 0) {
      mserial->printStrln(String(mqtt.connectErrorString(retvar)));
      mserial->printStrln("retrying MQTT connection in 5 seconds...");
      mqtt.disconnect();
      delay(5500); // wait 5 seconds
      trytimes--;
      if (trytimes == 0) {
          mserial->printStrln("Error!");
         return false;
      }
   }
   mserial->printStrln("connection sucessful.");
   return true;
}
// ....................................
bool publishData(Adafruit_MQTT_Publish *publishDataMQTT, double dataVal){
   if (! publishDataMQTT->publish(dataVal,4)) {
     mserial->printStrln("Data publish Failed: " + String(dataVal,4) );
     return false;
   } else {
     mserial->printStrln("Data publish OK. value: " + String(dataVal,4) );
     return true;
   }
}
// ********END SETUP *********************************************************

void GBRLcommands(String command, uint8_t sendTo) {
  if (gbrl.commands(command, sendTo) == false) {
    if ( onBoardSensors->gbrl_commands(command, sendTo ) == false) {
      if (mWifi->gbrl_commands(command, sendTo ) == false) {
            if ( command.indexOf("$") > -1) {
              interface->sendBLEstring("$ CMD ERROR \r\n", sendTo);
            } else {
              // interface->sendBLEstring("$ CMD UNK \r\n", sendTo);
            }
          }
        }
      }
    }

// ************************************************************

void BLE_init() {
  // Create the BLE Device
  BLEDevice::init(String("LDAD " + interface->config.DEVICE_BLE_NAME).c_str());  // max 29 chars

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic

  pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );


  pCharacteristicTX->addDescriptor(new BLE2902());

  pCharacteristicRX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                      );

  pCharacteristicTX->setCallbacks(new pCharacteristicTX_Callbacks());
  pCharacteristicRX->setCallbacks(new pCharacteristicRX_Callbacks());

  interface->init_BLE(pCharacteristicTX);

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
}
// *******************************************************************************************
// ******************************* ==  LOOP == ***********************************************

unsigned long lastMillisWIFI = 0;
unsigned long MEASUREMENT_INTERVAL = 5*60*1000;
int waitTimeWIFI = 0;

String dataStr = "";
long int eTime;
long int statusTime = millis();
long int beacon = millis();
long int MQTT_beacon = millis();

// adc_power_release()

unsigned int cycle = 0;
uint8_t updateCycle = 0;

double EMP_voltage[100]; 
double EMP_temp[100];
double EMP_humi[100];  
int EMP_voltage_idx = 0;
double tension_dif=0.0;

// ********************* == Core 1 : Data Measurements Acquisition == ******************************
void loop2 (void* pvParameters) {
  //measurements->runExternalMeasurements();
  delay(2000);
}


//************************** == Core 2: Connectivity WIFI & BLE == ***********************************************************
void loop(){  
  if (millis() - beacon > 60000) {    
    beacon = millis();
    mserial->printStrln("(" + String(beacon) + ") Free memory: " + addThousandSeparators( std::string( String(esp_get_free_heap_size() ).c_str() ) ) + " bytes\n", mSerial::DEBUG_TYPE_VERBOSE, mSerial::DEBUG_ALL_USB_UART_BLE);
  }

  if ( (WiFi.status() != WL_CONNECTED) && ( millis() - statusTime > 30000 ) ) { //10 sec
    statusTime = millis();
    interface->onBoardLED->led[1] = interface->onBoardLED->LED_GREEN;
    interface->onBoardLED->statusLED(100, 0.04);
  }

  // .............................................................................
  // disconnected for at least 3min
  // change MCU freq to min
  if ( mWifi->ALWAYS_ON_WIFI == false && mWifi->getBLEconnectivityStatus() == false && ( millis() - mWifi->$espunixtimeDeviceDisconnected > 180000) && interface->CURRENT_CLOCK_FREQUENCY >= interface->WIFI_FREQUENCY) {
    mserial->printStrln("setting min MCU freq.");
    
    btStop();
    mWifi->setBLEconnectivityStatus(false);
    mWifi->stop();
    
    interface->onBoardLED->led[0] = interface->onBoardLED->LED_RED;
    interface->onBoardLED->led[1] = interface->onBoardLED->LED_GREEN;
    interface->onBoardLED->statusLED(100, 2);
    
    interface->setMCUclockFrequency(interface->MIN_MCU_FREQUENCY);
    interface->onBoardLED->init();
  }

  // ................................................................................
  // Ligth Sleeep
  eTime = millis() - prevMeasurementMillis;
  if ( mWifi->getBLEconnectivityStatus() == false && interface->LIGHT_SLEEP_EN) {
    interface->onBoardLED->turnOffAllStatusLED();
    delay(100);    
    mWifi->stop();
    mserial->printStr("Entering light sleep for "+String( (MEASUREMENT_INTERVAL - eTime) / 1000)+" sec ....");
    esp_sleep_enable_timer_wakeup( ( ( MEASUREMENT_INTERVAL - eTime) / 1000)  * uS_TO_S_FACTOR);
    delay(100);
    esp_light_sleep_start();
    mserial->printStrln("wake up done.");
    mWifi->start();
  }
  prevMeasurementMillis = millis();

  
  //onBoardSensors->request_onBoard_Sensor_Measurements();
  
  // .....................................................................
  // Telegram
  telegram->runTelegramBot(EMP_voltage[EMP_voltage_idx], onBoardSensors->ONBOARD_TEMP, onBoardSensors->ONBOARD_HUMIDITY);

  // .............................................................
  // do measurements

  if (millis() - MQTT_beacon > MEASUREMENT_INTERVAL) {
    int16_t res1= ads.readADC_SingleEnded(1);
    int16_t res2= ads.readADC_SingleEnded(2);
    double tension_A1  = ads.computeVolts(res1);
    double tension_A2  = ads.computeVolts(res2);
    tension_dif = tension_A1 - tension_A2;
 
    EMP_voltage[EMP_voltage_idx] = tension_dif;
    onBoardSensors->request_onBoard_Sensor_Measurements();
    EMP_temp[EMP_voltage_idx] = onBoardSensors->ONBOARD_TEMP;
    EMP_humi[EMP_voltage_idx] = onBoardSensors->ONBOARD_HUMIDITY;
    
    EMP_voltage_idx ++;
    if ( EMP_voltage_idx > 99 )
      EMP_voltage_idx = 0;
 
    mserial->printStrln("Voltage A1: " +String(tension_A1)+ "mV" ); 
    mserial->printStrln("Voltage A2: " +String(tension_A2)+ "mV" ); 
    mserial->printStrln("Differential 1-2: " + String(tension_dif) + "mV");
        
    bool pub_ok = true;     
    MQTT_beacon = millis();
    
    if (WiFi.status() != WL_CONNECTED)
      mWifi->connect(10000,5);
    if (WiFi.status() == WL_CONNECTED && EMP_voltage_idx != 0){
      MQTT_connect();

        for (int i=EMP_voltage_idx; i>=0; i--){
          mserial->printStrln("EMP("+String(i)+"): " + String(EMP_voltage[i],4) +" mV");    
          if ( true != publishData( &emp, EMP_voltage[i] ) ){
            publishData( &temp,  EMP_temp[i] );
            publishData( &humidity, EMP_humi[i] );
            pub_ok = false;
            break;
          }
          if ( EMP_voltage_idx > 1){
            mserial->printStrln("61 sec delay to avoid Adafruit temporary ban");
            delay(61000);
          }
        }      
      if (pub_ok)
        EMP_voltage_idx = 0;
    }
  }

  // ................................................................................    

  if (mserial->readSerialData()){
    GBRLcommands(mserial->serialDataReceived, mserial->DEBUG_TO_USB);
  }
  // ................................................................................    

  if (mserial->readUARTserialData()){
    GBRLcommands(mserial->serialUartDataReceived, mserial->DEBUG_TO_UART);
  }
  // ................................................................................


  if (newBLESerialCommandArrived){
    xSemaphoreTake(MemLockSemaphoreBLE_RX, portMAX_DELAY); 
      newBLESerialCommandArrived=false; // this needs to be the last line       
    xSemaphoreGive(MemLockSemaphoreBLE_RX);

    GBRLcommands($BLE_CMD, mserial->DEBUG_TO_BLE);
  }


  // ....................................................................................
  // OTA Firmware
  if ( mWifi->forceFirmwareUpdate == true )
    mWifi->startFirmwareUpdate();



// ---------------------------------------------------------------------------
  if (millis() - lastMillisWIFI > 60000) {
    xSemaphoreTake(interface->MemLockSemaphoreCore2, portMAX_DELAY);
    waitTimeWIFI++;
    lastMillisWIFI = millis();
    xSemaphoreGive(interface->MemLockSemaphoreCore2);
  }

}
// -----------------------------------------------------------------
