/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <mohana-pamidi-project-1_inferencing.h>
#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>
#include <Arduino_LSM9DS1.h> // Add IMU sensor for motion detection

// Buffer to store sensor data
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
int feature_idx = 0;

const char* deviceServiceUuid = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char* deviceServiceCharacteristicUuid = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

int gesture = -1;
int oldGestureValue = -1;  

/**
 * @brief      Copy raw feature data in out_ptr
 *             Function called by inference library
 *
 * @param[in]  offset   The offset
 * @param[in]  length   The length
 * @param      out_ptr  The out pointer
 *
 * @return     0
 */
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

// BLE Service and Characteristic
BLEService predictionService("4fafc201-1fb5-459e-8fcc-c5c9c331914b"); // Custom service UUID
BLEStringCharacteristic predictionCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8",  // Custom characteristic UUID
                                                 BLERead | BLENotify, 
                                                 20);     // Max length 20 chars

// The peripheral device we want to connect to
BLEDevice peripheral;

// Sample rate in Hz
const float sample_rate_hz = 50.0f;

// Function to collect sensor data
bool collectSensorData() {
    if (feature_idx >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        feature_idx = 0;
        return true; // Buffer is full, ready for inference
    }

    // Read accelerometer data
    float ax, ay, az;
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(ax, ay, az);
        
        // Store accelerometer data (in m/s^2)
        features[feature_idx++] = ax * 9.81f;
        features[feature_idx++] = ay * 9.81f;
        features[feature_idx++] = az * 9.81f;
    }
    
    // Read gyroscope data
    float gx, gy, gz;
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(gx, gy, gz);
        
        // Store gyroscope data (in deg/s)
        features[feature_idx++] = gx;
        features[feature_idx++] = gy;
        features[feature_idx++] = gz;
    }

    return false; // Buffer is not yet full
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  // Initialize sensors
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  
  Serial.println("Arduino BLE Central - Edge Impulse Prediction Sender");
  Serial.println("Scanning for peripherals...");
  Serial.println("IMU initialized successfully");
  
  BLE.scan();
}

void loop() {
  // Check if a peripheral has been discovered
  peripheral = BLE.available();
  
  if (peripheral) {
    // Print peripheral info
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.println("'");
    
    // Check if this is the peripheral we want to connect to
    if (peripheral.localName() == "buzzerESP32") {
      // Stop scanning before attempting to connect
      BLE.stopScan();
      Serial.println("Found target peripheral. Stopping scan.");
      
      // Attempt connection and processing
      if (peripheral.connect()) {
        Serial.println("Connected!");
        delay(1000); // Give some time for the connection to stabilize
        
        processPeripheralConnection(peripheral);
        
        // Disconnect after we're done
        Serial.println("Disconnecting...");
        peripheral.disconnect();
      } else {
        Serial.println("Failed to connect!");
      }
      
      // Wait a moment before starting to scan again
      delay(2000);
      
      // Restart scanning
      Serial.println("Restarting scan...");
      BLE.scan();
    }
  }

  // Collect sensor data even when not connected
  collectSensorData();
}

void processPeripheralConnection(BLEDevice peripheral) {
  // Discover peripheral attributes
  Serial.println("Discovering attributes...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
    
    // Find the specific service
    BLEService service = peripheral.service(deviceServiceUuid);
    
    if (service) {
      Serial.println("Service found");
      // Find the characteristic
      BLECharacteristic characteristic = service.characteristic(deviceServiceCharacteristicUuid);
      
      if (characteristic && characteristic.canWrite()) {
        Serial.println("Found writable characteristic");
        
        // Process inference and send data
        unsigned long lastSampleTime = 0;
        bool dataReady = false;
        
        // Reset feature index to ensure we collect fresh data
        feature_idx = 0;
        
        while (peripheral.connected()) {
          // Collect sensor data at the specified sample rate
          unsigned long currentTime = millis();
          if (currentTime - lastSampleTime >= (1000 / sample_rate_hz)) {
            lastSampleTime = currentTime;
            dataReady = collectSensorData();
          }
          
          // Run inference when we have enough data
          if (dataReady) {
            ei_impulse_result_t result;
            signal_t signal;
            
            int err = numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
            if (err != 0) {
              Serial.println("Signal error!");
              break;
            }
            
            err = run_classifier(&signal, &result, false);
            if (err != EI_IMPULSE_OK) {
              Serial.print("Classifier error: ");
              Serial.println(err);
              break;
            }
            
            // Display inference results
            Serial.println("Inference results:");
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
              Serial.print("    ");
              Serial.print(result.classification[ix].label);
              Serial.print(": ");
              Serial.println(result.classification[ix].value);
            }
            
            // Find highest confidence prediction
            float maxValue = 0;
            size_t maxIndex = 0;
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
              if (result.classification[ix].value > maxValue) {
                maxValue = result.classification[ix].value;
                maxIndex = ix;
              }
            }
            
            // Convert to plain string (simpler for BLE transmission)
            char predictionBuffer[20]; // Match the size of your characteristic
            snprintf(predictionBuffer, sizeof(predictionBuffer), "%s", result.classification[maxIndex].label);
            
            // Debug output
            Serial.print("Sending: ");
            Serial.println(predictionBuffer);
            
            // Send data
            if (characteristic.writeValue(predictionBuffer)) {
              Serial.println("Data sent successfully");
            } else {
              Serial.println("Failed to send data");
            }
            
            // Reset dataReady flag
            dataReady = false;
          }
        }
      } else {
        Serial.println("Characteristic not found or not writable");
      }
    } else {
      Serial.println("Service not found");
    }
  } else {
    Serial.println("Failed to discover attributes");
  }
}