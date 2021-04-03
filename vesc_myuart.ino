/*
    VESC UART BLE LORA extender
*/
#include "Arduino.h"
#include <TFT_eSPI.h>
#include <Button2.h>
#include <CircularBuffer.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define ADC_EN 14
#define ADC_PIN 34
#define BUTTON_1 35
#define BUTTON_2 0

// EByte module connection
#define EB_M0 21
#define EB_M1 22
#define EB_RX 17
#define EB_TX 32
#define EB_AUX 33

// Vesc serial connection
#define VESC_RX 26
#define VESC_TX 25

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

// VESC produces about 800 bytes of configs data on vesc_tool connection
// We need to store it, while sending over slow LoRa chanel
static CircularBuffer<uint8_t, 2048> loraToSend;
const size_t max_buf = 2048;
uint8_t buf[max_buf];
uint8_t bufB[max_buf];
uint8_t ISRbuf[max_buf];

void initTDisplay()
{
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setFreeFont(&FreeMono9pt7b);
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


// callback function for bluetooth receive packets
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {

        //A) send to lora
        Serial.print(rxValue.length());
        Serial.print("B");
        for (int i = 0; i < rxValue.length(); i++)
          loraToSend.push((uint8_t) rxValue[i]);
          
        //B) send to serial / VESC 
        for (int i = 0; i < rxValue.length(); i++)
          ISRbuf[i] = rxValue[i];
        Serial.print(rxValue.length());
        Serial.print('B');
        size_t written = Serial1.write(ISRbuf, rxValue.length()); // Send to Vesc
        Serial.print(written);
        Serial.print("V ");
        Serial.println();
       
        
      }
    }
};


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  initTDisplay();

  // Vesc serial
  Serial1.begin(115200, SERIAL_8N1, VESC_TX, VESC_RX); // Start Vesc serial

  //EBYE serial
  Serial2.begin(115200, SERIAL_8N1, EB_TX, EB_RX);
  pinMode(EB_AUX, INPUT); //ebyte is ready if high

  // Create the BLE Device
  BLEDevice::init("Vesc Extender");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  
  Serial.println("Waiting a client connection to notify...");
}

void appendToLora(uint8_t *buf, size_t len)
{
  if (loraToSend.available() >= len) // Otherwise append to large enough LoRa buf
  {
    for (int i = 0; i < len; i++)
      loraToSend.push(buf[i]);
  }
  else // Drop packet, not enough buffer space
    Serial.printf("loraToSend BUFFER OVERFLOW! %d bytes available in buffer, %d received\n", loraToSend.available(), len);
}

static int loopStep = 0;

void debugPacket(uint8_t *buf, int len)
{
  Serial.print('(');
  for (int i = 0; i < len; i++)
    i == len - 1 ? Serial.printf("%02x", buf[i]) : Serial.printf("%02x:", buf[i]);
  Serial.println(")");
}

//send lora buffer
//at longest for 300ms
void sendLoraBuffer()
{
 size_t len, avail;
 long myTimeout = 300;
 long myTimer = millis();
 while (millis() < (myTimeout + myTimer)) {
  int loraAvailableForWrite = Serial2.availableForWrite();
  len = loraToSend.size() < loraAvailableForWrite ? loraToSend.size() : loraAvailableForWrite;
  if (!len){
      myTimeout = 0;  //send --> exit loop
    }
  if (len && loraAvailableForWrite >= 58)
  {
    //Serial.printf("len: %d, ", len);
    delay(5);
    loraAvailableForWrite = Serial2.availableForWrite();
    len = loraToSend.size() < loraAvailableForWrite ? loraToSend.size() : loraAvailableForWrite;
    //Serial.printf("len: %d, ", len);
     
    //Serial.printf("loraToSend.size() %d, loraAvailableForWrite %d\n", loraToSend.size(), loraAvailableForWrite);
    //Serial.println();
    //len = loraToSend.size() < loraAvailableForWrite ? loraToSend.size() : loraAvailableForWrite;
    bool isLoraModuleReady = digitalRead(EB_AUX) == HIGH;
    //Serial.printf("isLoraModuleReady: %d, len: %d\n", isLoraModuleReady, len);

    if (isLoraModuleReady)
    {
      //Serial.printf("loraToSend.size() %d, loraAvailableForWrite %d\n", loraToSend.size(), loraAvailableForWrite);
      for (int i = 0; i < len; i++)
        buf[i] = loraToSend.shift();

      size_t written = Serial2.write(buf, len); // Send to Lora
      Serial.print(written);
      Serial.print("L");
      Serial.println();
      //debugPacket(buf, len);
    }
  }
 }
}

void loop() {

    loopStep++;
    size_t len, avail;

   // Vesc data available?
   //send to bluetooth and/or lora
   len = 0;
   avail = Serial1.available();
   if (avail)
        {
          Serial.printf("avail: %d, ", avail);
          delay(10);
          avail = Serial1.available();
          Serial.printf("avail: %d, ", avail);
          
          len = Serial1.readBytes(buf, avail < max_buf ? avail : max_buf);
          //A) send to Lora
          Serial.print(len);
          Serial.print("V");
          appendToLora(buf, len);
          sendLoraBuffer();
          //B) send to bluetooth if connected
          if (deviceConnected) {           
            Serial.print(len);
            Serial.print("V");
            pTxCharacteristic->setValue(buf, len);
            pTxCharacteristic->notify();
            Serial.print(len);
            Serial.print("B ");
            Serial.println();
            delay(10); // bluetooth stack will go into congestion, if too many packets are sent
          }
          //debugPacket(buf, len);
        }

  delay(10);
    
  // LoRa data available?
  len = 0;
  avail = Serial2.available();
  if (avail)
  {
    Serial.printf("avail: %d, ", avail);
    delay(5);
    avail = Serial2.available();
    Serial.printf("avail: %d, ", avail);
    len = Serial2.readBytes(buf, avail < max_buf ? avail : max_buf);
    
    //A) send to VESC
    Serial.print(len);
    Serial.print('L');
    size_t written = Serial1.write(buf, len); // Send to Vesc
    Serial.print(written);
    Serial.print("V ");
    Serial.println();

    //B) send to bluetooth if connected
    if (deviceConnected) {
            Serial.print(len);
            Serial.print("L");
            pTxCharacteristic->setValue(buf, len);
            pTxCharacteristic->notify();
            Serial.print(len);
            Serial.print("B ");
            Serial.println();
            delay(10); // bluetooth stack will go into congestion, if too many packets are sent
          }
          
    //debugPacket(buf, len);
  }

  //delay(10);


    // Bluetooth disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
 //delay(5);
}
