Code for Arduino in Arduino IDE 2.3.2

#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include <Adafruit_BNO055.h>

// Define the Bluetooth device name
const char *bleName = "IMU1";

// Define the UUIDs of the service and characteristics
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_UUID_TX "e3223119-9445-4e96-a4a1-85358c4046a2"

// Bluetooth characteristic
BLECharacteristic *pCharacteristic;
BLEServer *pServer;

// BNO055 Sensor setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Variables
int receivedCommand = 0;
int8_t lastTemp = 0;  // To store the last temperature
unsigned long lastOrientationTime = 0;  // To handle orientation sending interval
const unsigned long orientationInterval = 500;  // 500 ms

void setup() {
  Serial.begin(115200);  // Initialize the serial port

  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  setupBLE();  // Initialize the Bluetooth BLE
}

void loop() {
  // Only process data if a valid command (1, 2, or 3) has been received
  if (receivedCommand > 0) {
    if (receivedCommand == 1) {
      // Check if 500 ms have passed to send orientation data
      if (millis() - lastOrientationTime >= orientationInterval) {
        sendOrientationData();  // Send orientation (Euler angles)
        lastOrientationTime = millis();  // Update the last time orientation was sent
      }
    } else if (receivedCommand == 2) {
      // Check if the temperature has changed before sending
      int8_t currentTemp = bno.getTemp();
      if (currentTemp != lastTemp) {
        sendTemperatureData();  // Send temperature
        lastTemp = currentTemp;  // Update the last known temperature
      }
    } else if (receivedCommand == 3) {
      // Check if 500 ms have passed to send Euler angles
      if (millis() - lastOrientationTime >= orientationInterval) {
        sendEulerAngles();  // Send Euler angles
        lastOrientationTime = millis();  // Update the last time Euler angles were sent
      }
    }
  }

  // Read data from the serial port and send it to BLE characteristic
  if (Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    const char *newValue = str.c_str();
    pCharacteristic->setValue(newValue);
    pCharacteristic->notify();
  }
}

// BLE server callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Connected");
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println("Disconnected");
    pCharacteristic->setValue("Connection lost, reconnecting...");
    pCharacteristic->notify();
    pServer->getAdvertising()->start();  // Restart advertising after disconnection
    Serial.println("Waiting for a new client connection...");
  }
};

// BLE characteristic callbacks
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received: ");
    Serial.println(value.c_str());

    // Parse the received value as an integer
    int command = atoi(value.c_str());

    // Set the command (1 for orientation, 2 for temperature, 3 for Euler angles, 0 to stop communication, 4 to disconnect)
    if (command == 1 || command == 2 || command == 3) {
      receivedCommand = command;
      Serial.println("Command received, starting communication");
    } else if (command == 0) {
      receivedCommand = 0;  // Stop sending data
      Serial.println("Communication stopped");
      pCharacteristic->setValue("Communication stop");
      pCharacteristic->notify();  // Notify the client that communication stopped
    } else if (command == 4) {
      receivedCommand = 0;  // Stop any ongoing communication
      Serial.println("Disconnecting...");
      pCharacteristic->setValue("Disconnecting from server...");
      pCharacteristic->notify();  // Notify the client that the device is disconnecting
      pServer->disconnect(0);  // Disconnect the client
    } else {
      Serial.println("Invalid command received");
      pCharacteristic->setValue("Invalid command received");
      pCharacteristic->notify();  // Notify the client of the invalid command
    }
  }
};

// Initialize the Bluetooth BLE
void setupBLE() {
  BLEDevice::init(bleName);  // Initialize the BLE device
  pServer = BLEDevice::createServer();  // Create the BLE server
  pServer->setCallbacks(new MyServerCallbacks());  // Set the BLE server callbacks

  // Create the BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create the BLE characteristic for sending notifications and reading/writing data (TX)
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY | 
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->addDescriptor(new BLE2902());  // Add the descriptor

  // Create the BLE characteristic for receiving and reading/writing data (RX)
  BLECharacteristic *pCharacteristicRX = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE | 
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristicRX->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();  // Start the BLE service
  pServer->getAdvertising()->start();  // Start advertising
  Serial.println("Waiting for a client connection...");
}

// Function to send orientation data via BLE
void sendOrientationData() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Prepare the data to be sent
  String eulerData = "X:" + String(euler.x(),1) + " Y:" + String(euler.y(),1) + " Z:" + String(euler.z(),1);

  // Send the data as a BLE notification
  pCharacteristic->setValue(eulerData.c_str());
  pCharacteristic->notify();  // Notify the connected client with the data
  Serial.println(eulerData);  // Print the data for debugging
}

// Function to send Euler angles data via BLE
void sendEulerAngles() {

  
  imu::Quaternion quat = bno.getQuat();

  // Prepare the data to be sent
  String eulerAnglesData = "X:" + String(quat.x(),1) + " Y:" + String(quat.y(),1) + " Z:" + String(quat.z(),1);

  // Send the data as a BLE notification
  pCharacteristic->setValue(eulerAnglesData.c_str());
  pCharacteristic->notify();  // Notify the connected client with the data
  Serial.println(eulerAnglesData);  // Print the data for debugging
}

// Function to send temperature data via BLE
void sendTemperatureData() {
  int8_t temp = bno.getTemp();

  // Prepare the data to be sent
  String tempData = "Temperature: " + String(temp) + " C";

  // Send the data as a BLE notification
  pCharacteristic->setValue(tempData.c_str());
  pCharacteristic->notify();  // Notify the connected client with the data
  Serial.println(tempData);  // Print the data for debugging
}
