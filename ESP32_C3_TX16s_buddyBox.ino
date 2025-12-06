/*Version 1.0

Copyright JP Gleyzes 2025
- full project description : https://hackaday.io/project/204529-tx16s-buddy-box-wireless-mastertrainer
- source code : https://github.com/f2knpw/Radiomaster_TX16s_buddy_box
-licence GPL V3 : https://www.gnu.org/licenses/gpl-3.0.fr.html

This firmware will add "buddy box" to Radiomaster TX16s radios to allow to wireless link two TX16s in Master/Trainer mode.

it has the following specifications:

- cheap (two fully working modules for less than 20$)
- auto configuration as master or slave or vice versa
- auto binding between two modules
- fully hidden into the "external module" compartment (no dangling wire outside the radio))
- no modification into the TX16s
- no firmware modification of your TX16s (stock EdgeTx firmware)
- high speed wireless communication with about 20m range
- automatic On/Off of the buddy boxes via EdgeTx
- compatible 16 channels SBUS and PPM signals
- extensible to further options (teasing: more to come !)


*/

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#define HAS_JOYSTICK

#ifdef HAS_JOYSTICK
//Bluetooth gamepad
#include <BleGamepad.h>  //https://github.com/lemmingDev/ESP32-BLE-Gamepad

BleGamepad bleGamepad("Radiomaster TX16s", "JPG", 100);
BleGamepadConfiguration bleGamepadConfig;  // Create a BleGamepadConfiguration object to store all of the options

int joystickValues[8];
#endif

//Preferences
#include <Preferences.h>
Preferences preferences;
uint8_t mac0, mac1, mac2, mac3, mac4, mac5;

#define BIND_PIN 0


//  ESPNow stuff: inpired by Rui Santos & Sara Santos - Random Nerd Tutorials https://RandomNerdTutorials.com/esp-now-auto-pairing-esp32-esp8266/
uint8_t peerAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t myMacAddress[6];

typedef struct struct_message {  // Structure to send/receive data
  uint8_t msgType;
  uint8_t macAddr[6];
  short channels[16];
} struct_message;

typedef struct struct_pairing {  // structure for pairing
  uint8_t msgType;
  uint8_t macAddr[6];
  uint8_t channel;
} struct_pairing;

esp_now_peer_info_t peer;

// Create 2 struct_message and one for binding
struct_message myData;  // data to send
struct_message inData;  // data received
struct_pairing pairingData;

enum PairingStatus { NOT_PAIRED,
                     PAIR_REQUEST,
                     PAIR_REQUESTED,
                     PAIR_PAIRED
};
PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType { PAIRING,
                   PAIRED,
                   DATA
};
MessageType messageType;

int channel = 1;  //wifi channel (must be the same on both buddy boxes)

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 10;          // Interval at which to publish sensor readings
unsigned long start;               // used to measure Pairing time
bool dataToSend = false;
long dataInterval;

#define DEBUG_ESPNOW
#define DEBUG_PPM
#define DEBUG_SBUS
//#define DEBUG_DATA  //will display data received and sent (comment  for higher speed)

#define RX_PIN 20  //CPMM PIN connected to Rx pin of ESP32-C3
#define TX_PIN 21  //Tx pin of ESP32-C3 connected to S_PORT pin

//SBUS library
#include "sbus.h"  //https://github.com/bolderflight/sbus/tree/main

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial0, RX_PIN, TX_PIN, true);  //EPS32 C3 mini (5V level true = inverted from CPPM pin on external RF/SBUS and 3.3V from FSIA6B SBUS pin)
//SbusRx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin, const bool inv)
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial0, RX_PIN, TX_PIN, true);  //ESP32 C3 mini (3.3V level true = inverted to FSIA6B SPORT pin on Trainer/Master/SBUSmodule)

bfs::SbusData data;  // SBUS data
long sbusStarted;
bool sbusActive = true;

int value = 1000;

//PPM
#include <ESP32_ppm.h>  //https://registry.platformio.org/libraries/fanfanlatulipe26/ESP32_ppm

int *ppmArray;
ppmReader myPPM_RX;
bool ppmTested = false;
bool ppmActive = false;

#define spyDuration 1000  // duration of test in ms
result_ppmSpy_t *ppmResultSpy;
ppmSpy myPPM_Spy;
bool spyFinished = false;
long ppmSpyStarted;

int expectedVal = 0;
int nbrReceived = 0;
void checkFrame();

//ESPNow
void readGetMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    printMAC(baseMac);
  } else {
    Serial.println("Failed to read MAC address");
  }
  myMacAddress[0] = baseMac[0];
  myMacAddress[1] = baseMac[1];
  myMacAddress[2] = baseMac[2];
  myMacAddress[3] = baseMac[3];
  myMacAddress[4] = baseMac[4];
  myMacAddress[5] = baseMac[5];
}

void addPeer(const uint8_t *mac_addr, uint8_t chan) {  //add peer to ESPNow peers list
#ifdef DEBUG_ESPNOW
  Serial.print("addpeer ");
  printMAC(mac_addr);
#endif
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);                     //delete previous peer with same MAC address
  memset(&peer, 0, sizeof(esp_now_peer_info_t));  // erase peer (store 0 in each byte)
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(peerAddress, mac_addr, sizeof(uint8_t[6]));  //copy macaddr to sender address
}

void printMAC(const uint8_t *mac_addr) {  // print MAC address in hexa
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  /* Serial.print("\r\nsend Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.print(" to ");
  printMAC(mac_addr); */
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {  // triggered when receiving data on ESPNow link
  /*Serial.print("Packet received with ");
  Serial.print("data size = ");
  Serial.print(sizeof(incomingData));*/
  uint8_t type = incomingData[0];
  switch (type) {
    case DATA:                           // we received data from sender
      if (pairingStatus == PAIR_PAIRED)  //only accept these data when paired
      {
        blinkLed();
        memcpy(&inData, incomingData, sizeof(inData));
        if ((inData.macAddr[0] + inData.macAddr[1] + inData.macAddr[2] + inData.macAddr[3] + inData.macAddr[4] + inData.macAddr[5]) != (myMacAddress[0] + myMacAddress[1] + myMacAddress[2] + myMacAddress[3] + myMacAddress[4] + myMacAddress[5])) {
          {
#ifdef DEBUG_DATA
            Serial.print("ESPNow data received,\t");

            for (int8_t i = 0; i < 16; i++) {  // do something with the SBUS values for each channel
              Serial.print(inData.channels[i]);
              Serial.print("\t");
            }
            Serial.println("");
#endif

            for (int8_t i = 0; i < 16; i++) {  // resend to the SBUS S_PORT pin for each channel
              data.ch[i] = inData.channels[i];
            }
            data.lost_frame = false;  //initialize SBUS lost
            data.failsafe = false;
            dataToSend = true;
          }
        }
      }
      break;

    case PAIRING:  // we received pairing data from server
      //if (pairingStatus != PAIR_PAIRED)  //only accept these data when not paired
      {
        memcpy(&pairingData, incomingData, sizeof(pairingData));
        if ((pairingData.macAddr[0] + pairingData.macAddr[1] + pairingData.macAddr[2] + pairingData.macAddr[3] + pairingData.macAddr[4] + pairingData.macAddr[5]) != (myMacAddress[0] + myMacAddress[1] + myMacAddress[2] + myMacAddress[3] + myMacAddress[4] + myMacAddress[5])) {
#ifdef DEBUG_ESPNOW
          Serial.print("received PAIRING : Pairing done for sender MAC Address: ");
          printMAC(pairingData.macAddr);
          Serial.print(" in ");
          Serial.print(millis() - start);
          Serial.println("ms");
#endif
          start = millis();
          addPeer(pairingData.macAddr, pairingData.channel);  // add the sender  to the peer list

          preferences.putInt("mac0", pairingData.macAddr[0]);  //keep track of the peer address
          preferences.putInt("mac1", pairingData.macAddr[1]);
          preferences.putInt("mac2", pairingData.macAddr[2]);
          preferences.putInt("mac3", pairingData.macAddr[3]);
          preferences.putInt("mac4", pairingData.macAddr[4]);
          preferences.putInt("mac5", pairingData.macAddr[5]);

          pairingData.msgType = PAIRED;                                   //and send back a "PAIRED" message to sender
          memcpy(pairingData.macAddr, myMacAddress, sizeof(uint8_t[6]));  //with my MACaddress
          esp_now_send(peerAddress, (uint8_t *)&pairingData, sizeof(pairingData));
          previousMillis = millis();
          pairingStatus = PAIR_PAIRED;  // set the pairing status
        }
      }
      break;


    case PAIRED:  // we received pairing data from peer
      memcpy(&pairingData, incomingData, sizeof(pairingData));
      //if ((pairingData.macAddr[0] + pairingData.macAddr[1] + pairingData.macAddr[2] + pairingData.macAddr[3] + pairingData.macAddr[4] + pairingData.macAddr[5]) != (myMacAddress[0] + myMacAddress[1] + myMacAddress[2] + myMacAddress[3] + myMacAddress[4] + myMacAddress[5]))
      {
#ifdef DEBUG_ESPNOW
        Serial.println("Paired message received from ");
        printMAC(pairingData.macAddr);
#endif
        addPeer(pairingData.macAddr, pairingData.channel);  // add the sender  to the peer list

        preferences.putInt("mac0", pairingData.macAddr[0]);  //store again the Peer MacAddress
        preferences.putInt("mac1", pairingData.macAddr[1]);
        preferences.putInt("mac2", pairingData.macAddr[2]);
        preferences.putInt("mac3", pairingData.macAddr[3]);
        preferences.putInt("mac4", pairingData.macAddr[4]);
        preferences.putInt("mac5", pairingData.macAddr[5]);
        pairingStatus = PAIR_PAIRED;  // set the pairing status to "PAIRED" and now we can send/receive DATA
      }
      break;
  }
}



void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BIND_PIN, INPUT_PULLUP);
  //Preferences
  preferences.begin("Trainer", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");    // remove the counter key only

  //ESPNow
  if (digitalRead(BIND_PIN) == false) {  //during boot if "BIND" button is pressed
    preferences.clear();                 // Remove all preferences under the opened namespace
    Serial.println("binding cleared");
  }
  mac0 = preferences.getInt("mac0", 0xFF);  // retreive peer address (or FF:FF:FF:FF:FF:FF if "bind" has been pressed)
  mac1 = preferences.getInt("mac1", 0xFF);
  mac2 = preferences.getInt("mac2", 0xFF);
  mac3 = preferences.getInt("mac3", 0xFF);
  mac4 = preferences.getInt("mac4", 0xFF);
  mac5 = preferences.getInt("mac5", 0xFF);

  Serial.println("read preferences :");
  Serial.print("\tmacPeer ");
  peerAddress[0] = mac0;
  peerAddress[1] = mac1;
  peerAddress[2] = mac2;
  peerAddress[3] = mac3;
  peerAddress[4] = mac4;
  peerAddress[5] = mac5;
  printMAC(peerAddress);
  //preferences.end();  // Close the Preferences

  WiFi.mode(WIFI_STA);  //launch Wifi to get my Mac Address
  WiFi.STA.begin();
  Serial.print("my MAC Address:  ");
  readGetMacAddress();
  WiFi.disconnect();
  delay(1000);
  start = millis();

  // set WiFi channel
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  // set callback routines for ESPNow
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  // add peer and send request
  addPeer(peerAddress, channel);
  if (mac0 & mac1 & mac2 & mac3 & mac4 & mac5 == 0xFF) pairingStatus = PAIR_REQUEST;  //launch auto pairing if peerAddress is FF:FF:FF:FF:FF:FF
  else pairingStatus = PAIR_PAIRED;                                                   //launch auto pairing done when already paired

  //SBUS
  /* Begin the SBUS communication */
  sbus_rx.Begin();
  sbus_tx.Begin();
#ifdef DEBUG_SBUS
  Serial.println("SBUS started");
#endif
  data.lost_frame = true;  //initialize SBUS lost
  data.failsafe = true;
  sbusStarted = millis();


#ifdef HAS_JOYSTICK
  //gamepad
  // bleGamepad.begin();
  // Changing bleGamepadConfig after the begin function has no effect, unless you call the begin function again bleGamepad.begin();
  // The default bleGamepad.begin() above enables 16 buttons, all axes, one hat, and no simulation controls or special buttons

  // bleGamepadConfig.setAutoReport(false);
  // bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);  // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
  bleGamepadConfig.setVid(0x068F);
  bleGamepadConfig.setPid(0x0041);
  bleGamepadConfig.setAxesMin(0x0000);  // 0 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  bleGamepadConfig.setAxesMax(2048);    // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal

  bleGamepad.begin(&bleGamepadConfig);  // Begin gamepad with configuration options
#endif
}

void blinkLed(void) {
  if ((millis() - dataInterval) > 50) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    dataInterval = millis();
  }
}
void loop() {
  //ESPNow
  switch (pairingStatus) {
    case PAIR_REQUEST:
#ifdef DEBUG_ESPNOW
      Serial.print("I request pairing on channel ");
      Serial.println(channel);
#endif

      pairingData.msgType = PAIRING;  // set pairing data to send to the server
      pairingData.channel = channel;

      pairingData.macAddr[0] = myMacAddress[0];
      pairingData.macAddr[1] = myMacAddress[1];
      pairingData.macAddr[2] = myMacAddress[2];
      pairingData.macAddr[3] = myMacAddress[3];
      pairingData.macAddr[4] = myMacAddress[4];
      pairingData.macAddr[5] = myMacAddress[5];

      // add peer and send request
      addPeer(peerAddress, channel);
      esp_now_send(peerAddress, (uint8_t *)&pairingData, sizeof(pairingData));  // send the PAIRING message
      previousMillis = millis();
      pairingStatus = PAIR_REQUESTED;
      break;

    case PAIR_REQUESTED:
      currentMillis = millis();
      if (currentMillis - previousMillis > 4000) {  // time out to allow receiving response from peer
        previousMillis = currentMillis;
        Serial.println(" timeout pairing ");
        pairingStatus = PAIR_REQUEST;  //retry sending PAIRING message
      }
      break;



    case PAIR_PAIRED:  //send data
      if (dataToSend) {
        blinkLed();
        myData.msgType = DATA;
        for (int8_t i = 0; i < data.NUM_CH; i++) {  // do something with the SBUS values for each channel
          myData.channels[i] = constrain(data.ch[i], 192, 1792);
        }
        dataToSend = false;
        memcpy(pairingData.macAddr, myMacAddress, sizeof(uint8_t[6]));                     //with my MACaddress
        esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&myData, sizeof(myData));  //send data to the peer
#ifdef HAS_JOYSTICK
        if (bleGamepad.isConnected()) {
          for (int i = 0; i < 8; i++) {
            joystickValues[i] = map(myData.channels[i], 192, 1792, 0, 2048);
          }
          bleGamepad.setAxes(joystickValues[0], joystickValues[1], joystickValues[2], joystickValues[3], joystickValues[4], joystickValues[5], joystickValues[6], joystickValues[7]);  //setAxes in the order (x, y, z, rx, ry, rz, slider1, slider2) setHIDAxes in the order (x, y, z, rz, rx, ry)
        }
#endif
      }
      break;
  }
#ifdef HAS_JOYSTICK
  if ((dataToSend) && (bleGamepad.isConnected())) {  //if data has not been yet sent (then PAIRING is not done) simply send to the BLE Joystick
    blinkLed();
    for (int8_t i = 0; i < data.NUM_CH; i++) {  // do something with the SBUS values for each channel
      myData.channels[i] = constrain(data.ch[i], 192, 1792);
    }
    dataToSend = false;
    for (int i = 0; i < 8; i++) {
      joystickValues[i] = map(myData.channels[i], 192, 1792, 0, 2048);
    }
    bleGamepad.setAxes(joystickValues[0], joystickValues[1], joystickValues[2], joystickValues[3], joystickValues[4], joystickValues[5], joystickValues[6], joystickValues[7]);  //setAxes in the order (x, y, z, rx, ry, rz, slider1, slider2) setHIDAxes in the order (x, y, z, rz, rx, ry)
  }
#endif


  //SBUS on CPMM pin (input)
  if (sbus_rx.Read()) {
    data = sbus_rx.data();  //Grab the received data

    if ((data.failsafe == false) && (data.lost_frame == false)) {
      sbusActive = true;
      sbusStarted = millis();
    }
    // Display the received data

#ifdef DEBUG_DATA
    Serial.print("SBUS data received,\t");
    for (int8_t i = 0; i < data.NUM_CH; i++) {  // do something with the SBUS values for each channel
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    Serial.println(" ");
#endif

    dataToSend = true;
  }
  if ((millis() - sbusStarted) > 3000) sbusActive = false;  //SBUS is not active try PPM


  //PPM
  if (!sbusActive) {   //if no SBUS input
    if (!ppmTested) {  //check PPM input if not already done
      ppmArray = myPPM_RX.begin(RX_PIN);
      delay(1000);
      ppmResultSpy = myPPM_Spy.begin(RX_PIN);
      if (ppmResultSpy == NULL) {
        Serial.println("Error. ppmSpy not available with this configuration? (not supported by ESP32 ?)");
        while (true) { delay(500); }
      }
      myPPM_Spy.start(spyDuration);  // Default is 1000 (1s)
#ifdef DEBUG_PPM
      Serial.println("PPM Spy Started ...");
#endif
      ppmTested = true;
      ppmSpyStarted = millis();
    }
    if (!spyFinished) {  //what's the result of PPMSpy ?
      if (myPPM_Spy.doneSpy()) {
#ifdef DEBUG_PPM
        Serial.println("PPM found ...");
        Serial.printf("polarity:%s\nmaxFrame:%d\tminFrame:%d\n", (ppmResultSpy->polarity == RISING ? "RISING" : "FALLING"),
                      ppmResultSpy->maxFrame, ppmResultSpy->minFrame);
        Serial.printf("maxLow:%d\tminLow:%d\nmaxHigh:%d\tminHigh:%d\n",
                      ppmResultSpy->maxLow, ppmResultSpy->minLow,
                      ppmResultSpy->maxHigh, ppmResultSpy->minHigh);
        Serial.printf("minChan:%d\tmaxChan:%d\n", ppmResultSpy->minChan, ppmResultSpy->maxChan);
#endif
        if ((ppmResultSpy->minChan > 7) && (ppmResultSpy->maxChan > 7)) {
          ppmActive = true;  // if we have a strong 16 channels PPM
          spyFinished = true;
          myPPM_RX.start();  //start receiving
#ifdef DEBUG_PPM
          Serial.println("PPM is detected on CPMM pin");
#endif
        }
        myPPM_Spy.end();  //stop spying
      }
      if ((millis() - ppmSpyStarted) > 3000) ppmActive = false;  //PPM is not active .. it's a master buddy box
    }

    if (myPPM_RX.newFrame() && ppmActive) {
      for (int i = 1; i < (ppmArray[0] + 1); i++) {  //do something with the PPM values for each channel
        // Serial.printf("%d", ppmArray[i]);
        // Serial.print("\t");
        constrain(ppmArray[i], 1000, 2000);
        data.ch[i - 1] = map(ppmArray[i], 1000, 2000, 192, 1792);  //map(value, fromLow, fromHigh, toLow, toHigh). Convert PPM to SBUS
      }
#ifdef DEBUG_DATA
      Serial.print("PPM data received and converted to SBUS,\t");
      for (int8_t i = 0; i < data.NUM_CH; i++) {  // do something with the SBUS values for each channel
        Serial.print(data.ch[i]);
        Serial.print("\t");
      }
      Serial.println(" ");
#endif
      dataToSend = true;
    }
  }

  //SBUS output on S_Port pin
  if ((!sbusActive) && (!ppmActive)) {  //we are with a "Master" buddy box (no input from CPMM pin, neither PPM nor SBUS)
    //channel numbering starts at 0. values rang between 192 and 1792
    if (dataToSend) {
      sbus_tx.data(data);  // Set the SBUS TX data to the received data
      sbus_tx.Write();     // Write the data to the S_PORT pin
      dataToSend = false;
      //Serial.println("SPORT sending");
    }
  } else {
    // we are on a slave buddy box (ESPNow will take care to send data)
  }
}