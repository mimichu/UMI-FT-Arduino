#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // Include TinyUSB library for USB serial communication

// Constants
const byte START_BYTE = 2;
const byte END_BYTE = 3;
const char CMD_STREAM = 's';
const char CMD_IDLE = 'i';
const int PACKET_SIZE = 26; // Fixed packet size
const int MAX_CHANNELS = 12;    // Maximum number of channels

// Variables
byte data[PACKET_SIZE - 1]; // Exclude the start byte (already read)
int sensor_data[MAX_CHANNELS];
bool streaming = false;
int num_Channels;

void setup() {
  // Initialize USB serial communication (Serial Monitor)
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  // Initialize UART communication with PSoC 4100S
  Serial1.begin(115200);

  Serial.println("XIAO nRF52840 UART Communication Started");

  // Flush input buffer
  while (Serial1.available()) {
    Serial1.read();
  }

  // Since the packet size is fixed at 26 bytes, calculate the number of channels
  // Exclude start byte and end byte, then divide by 2 bytes per channel
  num_Channels = (PACKET_SIZE - 2) / 2; // (26 - 2) / 2 = 12 channels

  // Check if calculated number of channels is valid
  if (num_Channels > MAX_CHANNELS) {
    Serial.println("Number of channels exceeds maximum");
    while (1);
  }

  Serial.print("Fixed Packet Size: ");
  Serial.println(PACKET_SIZE);
  Serial.print("Number of Channels: ");
  Serial.println(num_Channels);

  // Inform the user about streaming control
  Serial.println("Type 's' to start streaming, 'i' to stop streaming.");
}

void loop() {
  // Check for user input from Serial Monitor to control streaming
  if (Serial.available()) {
    char userInput = Serial.read();

    if (userInput == CMD_STREAM) {
      // Send stream command ('s') to start data streaming
      Serial1.write(CMD_STREAM);
      streaming = true;
      Serial.println("Streaming started");
    } else if (userInput == CMD_IDLE) {
      // Send idle command ('i') to stop data streaming
      Serial1.write(CMD_IDLE);
      streaming = false;
      Serial.println("Streaming stopped");
    }
    // Clear any extra characters in the input buffer
    while (Serial.available()) {
      Serial.read();
    }
  }

  // If streaming, read data packets from PSoC 4100S
  if (streaming) {
    if (Serial1.available()) {
      int startByte = Serial1.read();
      if (startByte == START_BYTE) {
        // Read the rest of the packet
        int bytesToRead = PACKET_SIZE - 1; // Exclude the start byte (already read)
        int bytesRead = 0;
        unsigned long startReadTime = millis();

        // Read data bytes with a timeout
        while (bytesRead < bytesToRead && millis() - startReadTime < 1000) { // 1-second timeout
          if (Serial1.available()) {
            data[bytesRead++] = Serial1.read();
          }
        }

        if (bytesRead == bytesToRead) {
          // Check end framing byte
          if (data[bytesToRead - 1] == END_BYTE) {
            // Parse sensor data
            int sensor_number = 0;

            for (int byte_num = 0; byte_num < bytesToRead - 1; byte_num += 2) {
              int lowByte = data[byte_num];
              int highByte = data[byte_num + 1];
              sensor_data[sensor_number++] = lowByte + (highByte << 8);
            }

            // Print sensor data to Serial Monitor
            Serial.print("Sensor Data: ");
            for (int i = 0; i < num_Channels; i++) {
              Serial.print(sensor_data[i]);
              if (i < num_Channels - 1) {
                Serial.print(", ");
              }
            }
            Serial.println();

          } else {
            Serial.println("Bad end framing byte");
          }
        } else {
          Serial.println("Timeout reading data bytes");
        }
      } else {
        // Optionally handle unexpected bytes here
      }
    }
  }

  // Optional: Add a small delay to prevent overwhelming the Serial Monitor
  // delay(10);
}
