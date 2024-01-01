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

#include "telegram_emp.h"
#include "Arduino.h"


TELEGRAM_EMP_CLASS::TELEGRAM_EMP_CLASS() {
    this->lastTimeBotRan = 0;
    this->openOrderRequest = false;

    this->CupOrderSize ="normal";
    this->orderRequestUsername = "xxxxxx";

    this->orderRequestChatID = "xxxxxxxxxx";
    this->orderRequestType = "xxxxxxxx";
    this->orderRequestTime = 0;
    this->EMP_voltage = 0.0;
    this->ONBOARD_TEMP = 0.0;
    this->ONBOARD_HUMIDITY = 0.0;
        
    this->OWNER_CHAT_ID = "xxxxxxxxxxx";
    // Initialize Telegram BOT
    this->BOTtoken = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";  // your Bot Token (Get from Botfather)
}

void TELEGRAM_EMP_CLASS::init(INTERFACE_CLASS* interface, M_WIFI_CLASS* mWifi ){
  this->interface=interface;
  this->interface->mserial->printStr("\ninit Telegram ...");
  this->mWifi= mWifi;
  
  this->botRequestDelay = 15000;

  this->client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  this->bot = new UniversalTelegramBot(this->BOTtoken, this->client);

  this->interface->mserial->printStrln("done.");
}

// ************************************************************
void TELEGRAM_EMP_CLASS::runTelegramBot( float EMP_voltage, float ONBOARD_TEMP, float ONBOARD_HUMIDITY ){
    this->EMP_voltage = EMP_voltage;
    this->ONBOARD_TEMP = ONBOARD_TEMP;
    this->ONBOARD_HUMIDITY = ONBOARD_HUMIDITY;
    
  if (millis() > this->lastTimeBotRan + this->botRequestDelay)  {
    if (WiFi.status() != WL_CONNECTED)
      mWifi->connect(10000,5);
    if (WiFi.status() == WL_CONNECTED){
      this->interface->mserial->printStr("telegram get updates....");
      
      int numNewMessages = this->bot->getUpdates(this->bot->last_message_received + 1); // this is slow!!
      
      this->interface->mserial->printStrln("done.");

      while(numNewMessages) {
        this->handleNewMessages(numNewMessages);
        numNewMessages = this->bot->getUpdates(this->bot->last_message_received + 1);
      }
      this->lastTimeBotRan = millis();
    }
  }
}

// ***********************************************************************************************
// Handle what happens when you receive new messages
void TELEGRAM_EMP_CLASS::handleNewMessages(int numNewMessages) {
  //Serial.println("handleNewMessages");
  //Serial.println(String(numNewMessages));

  for (int i=0; i<numNewMessages; i++) {
    // Chat id of the requester
    String chat_id = String(this->bot->messages[i].chat_id);
    
    // Print the received message
    String text = this->bot->messages[i].text;
    this->interface->mserial->printStrln("Request received ("+chat_id+"): " + text);

    String from_name = this->bot->messages[i].from_name;

    if (text == "/start") {
      String welcome = "Welcome, " + from_name + ".\n";
      welcome += "The following commands are available.\n\n";
      welcome += "/emp to view current EM measurment\n\n";
      welcome += "/history to view historical EM measurements on Adafruit.IO";
      welcome += "/donation to support this open project\n";
      welcome += "/projectPage to view this open project on Github";

      this->bot->sendMessage(chat_id, welcome, "");
    }
        
    if (text == "/history") {
      String msg ="The history records for the smart EMP monitor open project is available on adafruit.io here:\n";
      msg += "https://io.adafruit.com/aeonlabs/dashboards/em-sensor-data\n";
      this->bot->sendMessage(chat_id, msg, "");
    }
    
    if (text == "/projectPage") {
      String msg ="This smart EMP monitor open project is available on my github here:\n";
      msg += "https://github.com/aeonSolutions/PCB-Prototyping-Catalogue/tree/main/Home%20Automation%20Safety%20%26%20Health\n";
      this->bot->sendMessage(chat_id, msg, "");
    }
    
    if (text == "/donation") {
      String msg ="To support this open hardware & open source project you can make a donation using:\n";
      msg += "- Paypal: mtpsilva@gmail.com\n";
      msg += "- or at https://www.buymeacoffee.com/migueltomas";
      this->bot->sendMessage(chat_id, msg, "");
    }
    

    // _________________________________________________________________
    String notifyRequestStr =""; 

    if (text == "/emp") {
      this->bot->sendMessage(chat_id, "The current EMP value is " + String(this->EMP_voltage) + " mV", "");
      
      this->orderRequestUsername = from_name;
      this->openOrderRequest = true;
      this->orderRequestChatID = chat_id;
      this->orderRequestType = text.substring(1,text.length());
      this->orderRequestTime=millis();
    }
  }
}



// ************************************************************
