
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "1819"  // "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "2A67" // "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class RTKBleServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};


void setup_ble()
{

    Serial.println("Starting BLE");
    BLEDevice::init("Elyir GPS-RTK Rover");
    Serial.println("BLE started!");
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    Serial.println("BLE server created!");
    pServer->setCallbacks(new RTKBleServerCallbacks());
    Serial.println("BLE server callback set!");

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ 
            // | BLECharacteristic::PROPERTY_WRITE 
            // | BLECharacteristic::PROPERTY_NOTIFY 
            // | BLECharacteristic::PROPERTY_INDICATE
            );

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
    // Create a BLE Descriptor
    pCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection to notify...");
}

void loop_ble()
{
    
    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
    //     delay(500);                  // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
void send_ble_data(char *data)
{
    // notify changed value
    if (deviceConnected)
    {
        pCharacteristic->setValue(data);
        pCharacteristic->notify();
        // value++;
        // delay(10); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
    }
}