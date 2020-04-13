#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLEDescriptor* d = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint32_t features = 0;
uint8_t location = 0;
uint16_t value = 150;


const uint16_t UUID_CYCLING_POWER_SERVICE = 0x1818;
const uint16_t UUID_SENSOR_LOCATION_CHAR = 0x2A5D;
const uint16_t UUID_CYCLING_POWER_MEASUREMENT_CHAR = 0x2A63;
const uint16_t UUID_CYCLING_POWER_FEATURE_CHAR = 0x2A65;
const uint16_t UUID_CYCLING_POWER_VECTOR_CHAR = 0x2A64;
const uint16_t UUID_CYCLING_POWER_CONTROL_POINT_CHAR = 0x2A66;

const uint16_t UUID_SERVER_CHAR = 0x2903;


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("onConnect1");
      deviceConnected = true;
    };

    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param)
    {
      Serial.println("onConnect2");
      deviceConnected = true; 
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("onDisconnect");
      deviceConnected = false;
    }
};

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
public:
  virtual void onRead(BLECharacteristic* pCharacteristic)
  {
    Serial.println("onRead");
  }
  virtual void onWrite(BLECharacteristic* pCharacteristic)
  {
    Serial.println("onWrite");
  }
  virtual void onNotify(BLECharacteristic* pCharacteristic)
  {
    Serial.println("onNotify");
  }
  virtual void onStatus(BLECharacteristic* pCharacteristic, Status s, uint32_t code)
  {
    Serial.println("onStatus");
  }
};

int buttonPin = 15;

//unsigned long mmpr = 2105;//wheel
unsigned long mmpr = 141;//roller

double getW(double kmph)
{
  double res = -1.705489 + 1.994343 * kmph + 0.2327045 * kmph * kmph + 0.001377346 * kmph * kmph * kmph;
  return res > 0 ? res : 0;
}

double getKmph(double rpm, unsigned long mmpr)
{
  return rpm * mmpr * 60.0 / 1000000.0;
}

double getRpm(unsigned long interval)
{
  unsigned long  startMillis = 0;
  unsigned long  currMillis = 0;
  unsigned long  firstMillis = 0;
  unsigned long  lastMillis = 0;
  int count = 0;
  bool firstOn = true;
  bool wasOff = false;

  startMillis = currMillis = millis();
  while (currMillis - startMillis  < interval)
  {
    int buttonValue = digitalRead(buttonPin);
    if (buttonValue == LOW && firstOn)
    {
      firstOn = false;
      lastMillis = firstMillis = millis();
      delay(5);
    }
    else if (buttonValue == LOW && wasOff)
    {
      count++;
      lastMillis = millis();
      wasOff = false;
      delay(5);
    }
    else if (buttonValue == HIGH && !firstOn && !wasOff)
    {
      wasOff = true;
      delay(5);
    }
    currMillis = millis();
  }
  unsigned long time = lastMillis - firstMillis;
  if (time == 0)
  {
    return 0;
  }
  return  60000.0 * count / time;
}


void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("FilipPower");
  BLEDevice::setPower(ESP_PWR_LVL_N0);
  BLEDevice::setMTU(8);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(UUID_CYCLING_POWER_SERVICE);

  pCharacteristic = pService->createCharacteristic(
                      UUID_CYCLING_POWER_FEATURE_CHAR,
                      BLECharacteristic::PROPERTY_READ
                    );

  pCharacteristic->setValue((uint8_t*)&features, 4);                 

  pCharacteristic = pService->createCharacteristic(
                      UUID_SENSOR_LOCATION_CHAR,
                      BLECharacteristic::PROPERTY_READ
                    );

  pCharacteristic->setValue((uint8_t*)&location, 1);

  pCharacteristic = pService->createCharacteristic(
                      UUID_CYCLING_POWER_CONTROL_POINT_CHAR,
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  d = new BLE2902();      
  pCharacteristic->addDescriptor(d);    
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());                

  pCharacteristic->setValue((uint8_t*)&features, 4);

                    
  pCharacteristic = pService->createCharacteristic(
                      UUID_CYCLING_POWER_MEASUREMENT_CHAR,
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_BROADCAST
                    );

  d = new BLE2902();
  ((BLE2902*)d)->setNotifications(true);          
  pCharacteristic->addDescriptor(d);

  d = new BLEDescriptor(UUID_SERVER_CHAR);
  uint16_t v = 0;
  d->setValue((uint8_t*)&v, 2);
  pCharacteristic->addDescriptor(d);

  pCharacteristic->setValue((uint8_t*)&features, 4);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
 /* BLEAdvertisementData advertisementData;
  advertisementData.setAppearance(1156);
  advertisementData.setCompleteServices(UUID_CYCLING_POWER_SERVICE);
  advertisementData.setManufacturerData("Filip");
  advertisementData.setName("SuperPower");
  pAdvertising->setScanResponseData(advertisementData); */
  pAdvertising->addServiceUUID(UUID_CYCLING_POWER_SERVICE);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0006);  // set value to 0x00 to not advertise this parameter
  pAdvertising->setMaxPreferred(0x0C80);  // set value to 0x00 to not advertise this parameter
  //pAdvertising->setAdvertisementType(ADV_TYPE_IND);

  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() {
    // notify changed value
    double kmph = getKmph(getRpm(3000), mmpr);
    Serial.println("********************kmph********************");
    Serial.println("*                                          *");
    Serial.print("*                   ");Serial.print(kmph);Serial.println("                   *");
    Serial.println("*                                          *");
    Serial.println("********************************************");
    value = getW(kmph);  
    if (deviceConnected) {
        uint8_t temp[4];
        temp[0] = 0;
        temp[1] = 0;
        temp[2] = value;
        temp[3] = value >> 8;
  
        pCharacteristic->setValue(temp, 4);
        pCharacteristic->notify();
        //value = value + random(-2, 3);
      //  delay(1000); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
    // disconnecting
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
}
