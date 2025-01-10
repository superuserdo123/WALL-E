#include <ArduinoBLE.h>
#include <Wire.h>

#define I2C_ADDRESS 0x08  // I2C address of the master or sensor sending data
#define WAKEUP_PIN 2      // Pin D2 as wake-up source
#define RED_LED 12        // GPIO pin 11 for the blue LED
#define GREEN_LED 13      // GPIO pin 13 for the green LED


BLEService dataService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic dataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);

unsigned long connectionTimeout = 8000; // 8 seconds
bool dataReceived = false;

// Buffer for up to 10 packets of 20 bytes each
#define MAX_PACKETS 10
#define PACKET_SIZE 20
uint8_t dataBuffer[MAX_PACKETS][PACKET_SIZE];
int bufferCount = 0; // Number of packets in the buffer


void setup() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);  // Configure the LED for visual feedback

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);

  //Serial.begin(115200);
  //while (!Serial);

  Wire.begin();
  BLE.begin();

  BLE.setLocalName("BLE_A");
  BLE.setAdvertisedService(dataService);
  dataService.addCharacteristic(dataCharacteristic);
  BLE.addService(dataService);
  BLE.advertise();

  //Serial.println("BLE Communication Peripheral");
}

void loop() {
  unsigned long startTime = millis();

  while (!BLE.central() && millis() - startTime < connectionTimeout) {
    // Wait until a central connects or the timeout expires
    delay(100);
  }

  if (!BLE.central()) {
    //Serial.println("No connection within timeout. Entering deep sleep.");
    enterSystemOff();
  }

  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("Connected to central: ");
    //Serial.println(central.address());

    while (central.connected()) {
        // Check for incoming data from the central
        if (dataCharacteristic.written()) {
          const uint8_t* receivedData = dataCharacteristic.value();
          size_t length = dataCharacteristic.valueLength();  // Get the length of the data

          //Serial.print("Received BLE packet: ");
          for (size_t i = 0; i < length; i++) {
            //Serial.print((char)receivedData[i]);
          }
          //Serial.println();

          if (bufferCount < MAX_PACKETS) {
          // Save to the buffer
          memcpy(dataBuffer[bufferCount], receivedData, length);
          bufferCount++;
          } else {
          //Serial.println("Buffer full! Dropping packet.");
          }

          dataReceived = true; // Mark data as received
        }
    }

    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());


    if (dataReceived) {
      //Serial.println("Data received successfully. I2C connection to STM32.");
      sendBufferedData();
      //Serial.println("All data sent. Entering deep sleep.");
      enterSystemOff();
    } else {
      //Serial.println("No data received. Re-advertising.");
      BLE.advertise();
    }

  }
}


void sendBufferedData() {
  for (int i = 0; i < bufferCount; i++) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(dataBuffer[i], PACKET_SIZE); // Send one packet
    Wire.endTransmission();

    //Serial.print("Sent packet ");
    //Serial.println(i + 1);
    delay(10); // Small pause between transmissions (optional)
  }

  bufferCount = 0; // Clear the buffer
}


void enterSystemOff() 
{
  digitalWrite(GREEN_LED, LOW);  // Turn LED OFF (active-low)
  delay(1000);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  Wire.end();
  NRF_TWIM0->ENABLE = 0; // Turn off TWIM0 (I2C module)

  BLE.stopAdvertise();
  BLE.end();

  // Configure pin D2 as wake-up source
  nrf_gpio_cfg_sense_input(digitalPinToPinName(WAKEUP_PIN), NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

  // Enter SYSTEM OFF mode
  NRF_POWER->SYSTEMOFF = 1;

  while(1);
}