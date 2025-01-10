#include <ArduinoBLE.h>
#include <Wire.h>

#define I2C_ADDRESS 0x08 // I2C address of the master or sensor sending data
#define WAKEUP_PIN 2  // Pin D2 as wake-up source
#define RED_LED 12  // 
#define GREEN_LED 13  // 


BLEService dataService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic dataCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 20);

#define MAX_PACKETS 10     // Maximum number of data packets in the buffer
#define PACKET_SIZE 20     // Maximum size per packet (20 bytes)

char buffer[MAX_PACKETS][PACKET_SIZE]; // Buffer for up to 10 packets of 20 bytes each
int bufferIndex = 0;                   // Current buffer index
bool dataReady = false;                // Flag to check if data is ready to send
int packetsInBuffer = 0; // Keep track of the number of packets in the buffer


void setup() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);  // Configure the LED for visual feedback

  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);

  //Serial.begin(115200);
  //while (!Serial);

  Wire.begin(I2C_ADDRESS); // Start I2C as slave
  Wire.onReceive(receiveData);

  // Wait a maximum of 3 seconds for I2C packets
  unsigned long startTime = millis(); // Record the start time

  while (packetsInBuffer < 2 && millis() - startTime < 3000) {
    // Wait in the loop and check the time and packet count
  }

  // Check if enough packets have been received
  if (packetsInBuffer < 2) {
    //Serial.println("Not enough packets received. Returning to deep sleep.");
    enterSystemOff();
  }

  // Continue if at least 2 packets have been received
  //Serial.println("Sufficient packets received. Proceeding.");
  
  BLE.begin();
  BLE.setLocalName("BLE_Central");
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");

  //Serial.println("BLE Central with I2C buffer ready");
}

void loop() {
  //Serial.println("Searching for Peripheral...");
  
  unsigned long startTime = millis(); // Record the start time
  BLEDevice peripheral;

  // Search for a Peripheral for a maximum of 5 seconds
  while ((millis() - startTime < 5000) && !(peripheral = BLE.available())) {
    // Keep searching for a Peripheral
  }

  // Check if a Peripheral was detected
  if (!peripheral) {
    //Serial.println("No Peripheral found within 10 seconds. Entering deep sleep.");
    enterSystemOff(); // Put the Central into deep sleep
    return;
  }

  //Serial.println("Peripheral found. Checking name...");

  // Check if it is the correct Peripheral
  if (peripheral.localName() != "BLE_A") {
    //Serial.println("Unknown Peripheral found. Entering deep sleep.");
    enterSystemOff(); // Put the Central into deep sleep
    return;
  }

  //Serial.println("Correct Peripheral found. Stopping scan.");
  BLE.stopScan(); // Stop scanning once the correct Peripheral is found

  // Check if data is ready to send
  if (dataReady) {
    sendBufferToPeripheral(peripheral); // Send the data
  }

  // Restart scanning for new Peripherals
  BLE.scanForUuid("19b10000-e8f2-537e-4f6c-d104768a1214");

}

void receiveData(int numBytes) {
  static char tempBuffer[PACKET_SIZE]; // Temporary buffer for incoming data
  static int tempIndex = 0;            // Index for the temporary buffer

  while (Wire.available()) {
    char receivedChar = Wire.read();
    
    buffer[bufferIndex][strlen(buffer[bufferIndex])] = receivedChar;

    // Check if the packet is complete (ends with ';')
    if (receivedChar == ';') {
      buffer[bufferIndex][strlen(buffer[bufferIndex])] = '\0'; // Make the string null-terminated

      // Print the received packet
      //Serial.print("Received packet: ");
      //Serial.println(buffer[bufferIndex]);

      // Move to the next buffer index and increment the packet count
      bufferIndex++;
      packetsInBuffer++;

      // Check if bufferIndex has reached the maximum
      if (bufferIndex >= MAX_PACKETS) {
        bufferIndex = 0; // Start over and overwrite old data
      }

      // Set dataReady to true if two or more packets have been received
      if (packetsInBuffer >= 2) {
        dataReady = true;
      }

      // Start a new packet
      memset(buffer[bufferIndex], 0, PACKET_SIZE); // Clear the buffer for the next packet
    }
  }
}


void sendBufferToPeripheral(BLEDevice peripheral) {
  //Serial.println("Connecting ...");

  unsigned long startTime = millis(); // Record the start time
  bool connected = false;

  // Try to connect with a 5-second timeout
  while (!connected && (millis() - startTime < 5000)) {
    if (peripheral.connect()) {
      connected = true;
    }
  }

  if (!connected) {
    //Serial.println("Could not connect within 10 seconds. Entering deep sleep.");
    enterSystemOff(); // Put the system into deep sleep
    return;
  }

  if (peripheral.connect()) {
    //Serial.println("Connected");
  } else {
    //Serial.println("Failed to connect!");
    return;
  }

  //Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    //Serial.println("Attributes discovered");
  } else {
    //Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  BLECharacteristic dataCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!dataCharacteristic) {
    //Serial.println("Peripheral does not have data characteristic!");
    peripheral.disconnect();
    return;
  }

  while (dataReady) {
    for (int i = 0; i < bufferIndex; i++) {
      dataCharacteristic.writeValue((uint8_t *)buffer[i], strlen(buffer[i]));
      //Serial.print("Sent: ");
      //Serial.println(buffer[i]);
      packetsInBuffer--; // Decrease the number of packets in the buffer
      delay(200); // Small delay between packets
    }
    dataReady = false; // Reset the flag
    bufferIndex = 0;   // Clear the buffer
  }

  //Serial.println("Sent all the data");
  peripheral.disconnect();

  //Serial.println("BLE goes to deep sleep");
  enterSystemOff(); // Put the system into SYSTEM OFF
}

void enterSystemOff() 
{
  digitalWrite(GREEN_LED, LOW);  // Turn LED OFF (active-low)
  delay(1000);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, HIGH);

  Wire.end();
  NRF_TWIM0->ENABLE = 0; // Turn off TWIM0 (I2C module)

  // Configure pin D2 as wake-up source
  nrf_gpio_cfg_sense_input(digitalPinToPinName(WAKEUP_PIN), NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);

  // Enter SYSTEM OFF mode
  NRF_POWER->SYSTEMOFF = 1;

  while(1);
}