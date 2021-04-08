/*
VESC extender
*/
#include "Arduino.h"
#include <TFT_eSPI.h>
#include <Button2.h>
//#include <EBYTE.h>    //does not support e22
#include "LoRa_E22.h"
#include <CircularBuffer.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


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
#define HARDWARE_SERIAL_SELECTABLE_PIN

// Vesc serial connection
#define VESC_RX 26
#define VESC_TX 25
//#define VESC_RX 17
//#define VESC_TX 32

//TODO adapt time, 1000000 = 1 time per second,
//e.g. 1000000 => 1 time per second
//e.g. 100000 => 10 times per second
#define LORA_SYNC_TIMER 100000
long txPeriodTimeMs = LORA_SYNC_TIMER / 1000;
int airDataRate = 19200; //change it based on used air data baudrate, TODO get from ebyte config

// setting baudrate on hw serial does not work here, use workaround in setup()
//LoRa_E22 e22ttl100(&Serial2, EB_TX, EB_RX, EB_AUX, EB_M0, EB_M1, UART_BPS_RATE_9600); //  esp32 RX <-- e22 TX, esp32 TX --> e22 RX AUX M0 M1
LoRa_E22 e22ttl100(EB_TX, EB_RX, &Serial2, EB_AUX, EB_M0, EB_M1, UART_BPS_RATE_9600, SERIAL_8N1); //  esp32 RX <-- e22 TX, esp32 TX --> e22 RX AUX M0 M1

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

// VESC produces about 800 bytes of configs data on vesc_tool connection
// We need to store it, while sending over slow LoRa chanel
static CircularBuffer<uint8_t, 2048> loraToSend;
const size_t max_buf = 2048;
uint8_t buf[max_buf];
uint8_t bufB[max_buf];    //used in ISR

//communication synchronisation to avoid two send pakets at the same time
volatile bool inLoraSendMode = true;  //toogled in each synchronisation timer interrupt
volatile long lastModeToggleMillis = millis();    //store millis of last timer interrupt
volatile bool firstPaketInPeriod = true;    //reset timer on each first paket in period
volatile int bytesSend = 0;    //store bytes allready send in period
hw_timer_t * timer = NULL;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;

// called at each timer innterrupt for communication syncghronisation
void IRAM_ATTR onTime() {
   //portENTER_CRITICAL_ISR(&timerMux);
   inLoraSendMode = !inLoraSendMode;
   firstPaketInPeriod = true; //reset
   lastModeToggleMillis = millis();
   bytesSend = 0;
   //Serial.printf("Send Mode switched: %d \n", inLoraSendMode);
   //portEXIT_CRITICAL_ISR(&timerMux);
}

void printParameters(struct Configuration configuration);
void printModuleInformation(struct ModuleInformation moduleInformation);

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
        Serial.println();
        //A) send to lora
        Serial.print(rxValue.length());
        Serial.print("B");
        for (int i = 0; i < rxValue.length(); i++){
          loraToSend.push((uint8_t) rxValue[i]);
          bufB[i] = (uint8_t) rxValue[i];
        }
        
        //B) send to serial / VESC 
        //TODO remove this from ISR
        //Serial.print(rxValue.length());
        //Serial.print('B');
        size_t written = Serial1.write(bufB, rxValue.length()); // Send to Vesc
        Serial.print(written);
        Serial.print("V ");
        
      }
    }
};


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  initTDisplay();

  // Vesc serial
  Serial1.begin(115200, SERIAL_8N1, VESC_TX, VESC_RX); // Start Vesc serial

  //EBYTE serial
  //Serial2.begin(115200, SERIAL_8N1, EB_TX, EB_RX);
  //pinMode(EB_AUX, INPUT); //ebyte is ready if high
  // Startup all pins and UART
  e22ttl100.begin();
  ResponseStructContainer c;
  c = e22ttl100.getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration*) c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);
  printParameters(configuration);
  configuration.ADDL = 0x03;
  configuration.ADDH = 0x00;
  configuration.NETID = 0x00;
  configuration.CHAN = 23;
  //configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartBaudRate = UART_BPS_115200;    //workaround set hw serial baudrate manual afterwards!!
  //configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
  //configuration.SPED.airDataRate = AIR_DATA_RATE_100_96;
  configuration.SPED.airDataRate = AIR_DATA_RATE_101_192;
  //configuration.SPED.airDataRate = AIR_DATA_RATE_111_625;
  configuration.SPED.uartParity = MODE_00_8N1;
  configuration.OPTION.subPacketSetting = SPS_240_00;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.OPTION.transmissionPower = POWER_22;
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableRepeater = REPEATER_DISABLED;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  configuration.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;
  // Set configuration changed and set to not hold the configuration
  ResponseStatus rs = e22ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);

  c = e22ttl100.getConfiguration();
  // It's important get configuration pointer before all other operation
  configuration = *(Configuration*) c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);
  printParameters(configuration);
  /*
  ResponseStructContainer cMi;
  cMi = e22ttl100.getModuleInformation();
  // It's important get information pointer before all other operation
  ModuleInformation mi = *(ModuleInformation*)cMi.data;
  Serial.println(cMi.status.getResponseDescription());
  Serial.println(cMi.status.code);
  cMi.close(); 
  c.close(); */

  //workaround to change baudrate on hw serial
  delay(1000);
  Serial2.flush();
  delay(1000);
  Serial2.begin(115200, SERIAL_8N1, EB_TX, EB_RX);
  //Serial2.begin(115200);

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


   // Configure the Prescaler at 80 the quarter of the ESP32 is cadence at 80Mhz
   // 80000000 / 80 = 1000000 tics / seconde
   timer = timerBegin(0, 80, true);                
   timerAttachInterrupt(timer, &onTime, true);

   // Sets an alarm to toogle inLoraSendMode
   //LORA_SYNC_TIMER: 1000000 = 1 time per second, e.g. 1000000/10 => 10 times per second
   timerAlarmWrite(timer, LORA_SYNC_TIMER, true);           
   timerAlarmEnable(timer);
  
  Serial.printf("tx period in ms: %d\n", txPeriodTimeMs);
  //calculate possible bytes per send period
  int bytesPerPeriod = (airDataRate / 8) / 2.0 * txPeriodTimeMs / 1000.0 ;
  Serial.printf("Air data rate: %d --> calculated bytes per tx period: %d\n", airDataRate, bytesPerPeriod);
  
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

void loop() {

    loopStep++;
    size_t len, avail;

   // Vesc data available?
   // send to bluetooth and/or lora
   len = 0;
   avail = Serial1.available();
   if (avail)
        {
          len = Serial1.readBytes(buf, avail < max_buf ? avail : max_buf);
          //A) send to Lora
          Serial.print(len);
          Serial.print("V");
          appendToLora(buf, len);
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
 
    
  // LoRa data available?
  len = 0;
  if (e22ttl100.available())
  {
    //sync timer on each first paket in sending period
    if (firstPaketInPeriod){
      timerWrite(timer, 0);   //reset timer
      firstPaketInPeriod = false;   //is set true in ISR
      lastModeToggleMillis = millis();
      inLoraSendMode = false;
      //Serial.println("first paket received -> timer reset");
    }
     
    ResponseContainer rsc = e22ttl100.receiveMessage();
    if (rsc.status.code!=1){
      Serial.println(rsc.status.getResponseDescription());
     }else{
      len = rsc.data.length();
      //copy message
      for (int i = 0; i < rsc.data.length(); i++)
        buf[i] = rsc.data[i];
      
      //debugPacket(buf, len);            
      // Print the data received
      //Serial.println(rsc.status.getResponseDescription());
    }
    
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

   
  // send lora buffer. send only in defined intervals within a specific time window
  // split up large data in multiple messages/pakets
  // calculate possible bytes for remaining send window
  long possibleBytes = ( lastModeToggleMillis + txPeriodTimeMs - millis() ) * (airDataRate / 8.0 * 0.001) ;   // remaining send windows in ms * bytes per ms --> remaining bytes
  
  possibleBytes = possibleBytes - bytesSend - 40;  //safty margin
  if (possibleBytes < 0 )
    possibleBytes = 0;

  len = loraToSend.size();
  if (len > possibleBytes)
    len = possibleBytes;  
  if (inLoraSendMode && len)
  {   
    if (firstPaketInPeriod){
      timerWrite(timer, 0);   //reset timer
      firstPaketInPeriod = false;   //is set true in ISR
      lastModeToggleMillis = millis();
      inLoraSendMode = true;
      //Serial.println("first paket send -> timer reset");
    }
    firstPaketInPeriod = false;
    
     for (int i = 0; i < len; i++)
        buf[i] = loraToSend.shift();

    // Send message
    ResponseStatus rs = e22ttl100.sendMessage(&buf, len);
    // Check If there is some problem of succesfully send
    if (rs.code!= 1){
      Serial.println(rs.getResponseDescription());
      //TODO resend?
     }else{
      bytesSend = bytesSend + len;
      Serial.print(possibleBytes);
      Serial.print("P");
      Serial.print(len);
      Serial.print("L");
      Serial.println();
      //debugPacket(buf, len);
    }
  }
 
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

void printParameters(struct Configuration configuration) {
  DEBUG_PRINTLN("----------------------------------------");

  DEBUG_PRINT(F("HEAD : "));  DEBUG_PRINT(configuration.COMMAND, HEX);DEBUG_PRINT(" ");DEBUG_PRINT(configuration.STARTING_ADDRESS, HEX);DEBUG_PRINT(" ");DEBUG_PRINTLN(configuration.LENGHT, HEX);
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("AddH : "));  DEBUG_PRINTLN(configuration.ADDH, HEX);
  DEBUG_PRINT(F("AddL : "));  DEBUG_PRINTLN(configuration.ADDL, HEX);
  DEBUG_PRINT(F("NetID : "));  DEBUG_PRINTLN(configuration.NETID, HEX);
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("Chan : "));  DEBUG_PRINT(configuration.CHAN, DEC); DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.getChannelDescription());
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("SpeedParityBit     : "));  DEBUG_PRINT(configuration.SPED.uartParity, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getUARTParityDescription());
  DEBUG_PRINT(F("SpeedUARTDatte     : "));  DEBUG_PRINT(configuration.SPED.uartBaudRate, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getUARTBaudRateDescription());
  DEBUG_PRINT(F("SpeedAirDataRate   : "));  DEBUG_PRINT(configuration.SPED.airDataRate, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.SPED.getAirDataRateDescription());
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("OptionSubPacketSett: "));  DEBUG_PRINT(configuration.OPTION.subPacketSetting, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getSubPacketSetting());
  DEBUG_PRINT(F("OptionTranPower    : "));  DEBUG_PRINT(configuration.OPTION.transmissionPower, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getTransmissionPowerDescription());
  DEBUG_PRINT(F("OptionRSSIAmbientNo: "));  DEBUG_PRINT(configuration.OPTION.RSSIAmbientNoise, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.OPTION.getRSSIAmbientNoiseEnable());
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("TransModeWORPeriod : "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.WORPeriod, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  DEBUG_PRINT(F("TransModeTransContr: "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.WORTransceiverControl, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getWORTransceiverControlDescription());
  DEBUG_PRINT(F("TransModeEnableLBT : "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.enableLBT, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  DEBUG_PRINT(F("TransModeEnableRSSI: "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.enableRSSI, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  DEBUG_PRINT(F("TransModeEnabRepeat: "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.enableRepeater, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getRepeaterModeEnableByteDescription());
  DEBUG_PRINT(F("TransModeFixedTrans: "));  DEBUG_PRINT(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);DEBUG_PRINT(" -> "); DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());


  DEBUG_PRINTLN("----------------------------------------");
}
void printModuleInformation(struct ModuleInformation moduleInformation) {
  Serial.println("----------------------------------------");
  DEBUG_PRINT(F("HEAD: "));  DEBUG_PRINT(moduleInformation.COMMAND, HEX);DEBUG_PRINT(" ");DEBUG_PRINT(moduleInformation.STARTING_ADDRESS, HEX);DEBUG_PRINT(" ");DEBUG_PRINTLN(moduleInformation.LENGHT, DEC);

  Serial.print(F("Model no.: "));  Serial.println(moduleInformation.model, DEC);
  Serial.print(F("Version  : "));  Serial.println(moduleInformation.version, DEC);
  Serial.print(F("Features : "));  Serial.println(moduleInformation.features, BIN);
  Serial.println("----------------------------------------");

}
