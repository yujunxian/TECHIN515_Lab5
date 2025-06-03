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
#include <Arduino.h>
#include <TECHIN515_Lab4_inferencing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


#include "driver/gpio.h"
#include <HttpClient.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_system.h>

// WiFi credentials 
const char* ssid = "Xiaomi_112F";
const char* password = "tp45tnm9";

WiFiClient wifi;
// Server details 

const char* serverAddr = "192.168.31.64";
const char* serverUrl = "http://192.168.31.64:8080/predict";
const int port = 8000;

// Student identifier - set this to the student's UWNetID
const char* studentId = "123456";


#define  keyPin GPIO_NUM_5   //to key
#define CONFIDENCE_THRESHOLD 60.0

const int ledR =  D0;  //to led ==> Z
const int ledB =  D1;  //to led ==> O
const int ledG =  D2;  //to led ==> V
int current_led =  -1;

void sendRawDataToServer();
void run_inference2();
void set_current_led(int led)
{
    current_led=led;
}
void light_current_led()
{
    if (current_led>=0){
        digitalWrite(current_led, HIGH);
        delay(500);
        digitalWrite(current_led, LOW);
        delay(500);        
        digitalWrite(current_led, HIGH);
        delay(500);
        digitalWrite(current_led, LOW);
    }    
}

// MPU6050 sensor
Adafruit_MPU6050 mpu;

// Sampling and capture variables
#define SAMPLE_RATE_MS 10  // 100Hz sampling rate (10ms between samples)
#define CAPTURE_DURATION_MS 1000  // 1 second capture
#define FEATURE_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE  // Size of the feature array = 300

// Capture state variables
bool capturing = false;
unsigned long last_sample_time = 0;
unsigned long capture_start_time = 0;
int sample_count = 0;

// Feature array to store accelerometer data
float features[FEATURE_SIZE];

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

void print_inference_result(ei_impulse_result_t result);


void setupWiFi() {
    Serial.println("Connecting to WiFi...");

        
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

/**
 * @brief      Arduino setup function
 */
void setup()
{

    //pinMode(keyPin, INPUT);    
    pinMode(keyPin, INPUT_PULLUP);
    pinMode(ledR, OUTPUT);
    pinMode(ledB, OUTPUT);
    pinMode(ledG, OUTPUT);
    srand(time(NULL));
    // Initialize serial
    Serial.begin(115200);
    Serial.println("Initializing");

    setupWiFi();
    
    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
   
    // Configure MPU6050 - match settings with gesture_capture.ino
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    //test wand light
    {
        set_current_led(ledR);
        light_current_led();
        set_current_led(ledB);
        light_current_led();
        set_current_led(ledG);
        light_current_led();        

    }
    //digitalWrite(ledR, HIGH);
    
    set_current_led(-1);


    Serial.println("MPU6050 initialized successfully");
    Serial.println("Send 'o' to start gesture capture");

}

/**
 * @brief      Capture accelerometer data for inference
 */
void capture_accelerometer_data() {
    if (millis() - last_sample_time >= SAMPLE_RATE_MS) {
        last_sample_time = millis();
        
        // Get accelerometer data
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        // Store data in features array (x, y, z, x, y, z, ...)
        if (sample_count < FEATURE_SIZE / 3) {
            int idx = sample_count * 3;
            features[idx] = a.acceleration.x;
            features[idx + 1] = a.acceleration.y;
            features[idx + 2] = a.acceleration.z;
            sample_count++;
        }
        
        // Check if capture duration has elapsed
        if (millis() - capture_start_time >= CAPTURE_DURATION_MS && sample_count >= FEATURE_SIZE / 3) {
            capturing = false;
            Serial.println("Capture complete");
            
            // Run inference on captured data
            run_inference2();
        }
    }
}

/**
 * @brief      Run inference on the captured data
 */
void run_inference2() {
    // Check if we have enough data
    if (sample_count * 3 < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        Serial.println("ERROR: Not enough data for inference\n");
        Serial.println("the count of sample is ");
        Serial.println(sample_count);
        Serial.println("\n");
        
        return;
    }
    
    ei_impulse_result_t result = { 0 };

    // Create signal from features array
    signal_t features_signal;
    features_signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;  //=300
    features_signal.get_data = &raw_feature_get_data;       // this function directly use features[],to get data into model

    // Run the classifier
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
    if (res != EI_IMPULSE_OK) {
        Serial.print("ERR: Failed to run classifier (");
        Serial.print(res);
        Serial.println(")");
        return;
    }

    // Print inference result
    print_inference_result(result);
}

/**
 * @brief      Arduino main function
 */
void loop()
{
    // Check for serial commands
    if (!capturing){
        /*
        if (Serial.available() > 0) {
            char cmd = Serial.read();
            if (cmd == 'o') {
                // Start capturing data
                Serial.println("Starting gesture capture  by get O from RXT...");
                sample_count = 0;
                capturing = true;
                capture_start_time = millis();
                last_sample_time = millis();
            }
        }
        */
        // Check for button pressed 10 times
        int currentSteps=0;
        int button_state =-1;
        while (currentSteps < 10) {
            button_state=gpio_get_level(keyPin);            
            if (button_state == 0) {
                delay(10);  //in case of shaking
                button_state=gpio_get_level(keyPin);
                if (button_state == 0){
                    // Start capturing data
                    Serial.println("Starting gesture capture by press a key ...");
                    sample_count = 0;
                    capturing = true;
                    capture_start_time = millis();
                    last_sample_time = millis();
                    break;
                }
            } 
            currentSteps++;
            delay(10);  //in case of shaking
        }
    }
    // Capture data     if in capturing mode
    if (capturing) {
        capture_accelerometer_data();
    }
}

void print_inference_result(ei_impulse_result_t result) {
    // Find the prediction with highest confidence
    float max_value = 0;
    int max_index = -1;
    
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_value) {
            max_value = result.classification[i].value;
            max_index = i;
        }
    }
    
    // Only print the prediction with highest confidence
    if (max_index != -1) {
        if(max_value>=CONFIDENCE_THRESHOLD) {
            Serial.print("Prediction: ");
            Serial.print(ei_classifier_inferencing_categories[max_index]);
            Serial.print(" (");
            Serial.print(max_value * 100);
            Serial.println("%)");
            if (max_index==0){
                set_current_led(ledR);
            }
            if (max_index==1){
                set_current_led(ledB);
            }
            if (max_index==2){
                set_current_led(ledG);
            }
            light_current_led();
            set_current_led(-1);
        }
        else {
            Serial.println("Low confidence - sending raw data to server...");
			sendRawDataToServer();
        }

    }
}

void sendRawDataToServer() {
    
   IPAddress address=IPAddress(serverAddr);
   HttpClient http = HttpClient(wifi, address, port);

   //HttpClient http = HttpClient(wifi, serverUrl, port);
   
   //HttpClient http(WiFi, serverUrl);
   //HttpClient http;
   //http.begin(serverUrl);
   //http.addHeader("Content-Type", "application/json");//,false,false);
   
   http.sendHeader("Content-Type", "application/json");

   // Build JSON array from features[]
   // Your code here

    String jsonPayload= "{";
/*
    jsonPayload += "\"student_id\":";
    jsonPayload += "\"";
    jsonPayload += studentId;
    jsonPayload += "\",";
    jsonPayload += "\"gesture\":";
    jsonPayload += "\"";
    jsonPayload += "A";
    jsonPayload += "\",";
    jsonPayload += "\"confidence\":";
    jsonPayload += "80";    
    jsonPayload += "\",";
    jsonPayload += "\"data\":";
    jsonPayload += "80";
    jsonPayload += "}";
    
// set rand data
    jsonPayload= "{";    
    jsonPayload += "\"data\":[";
    for(int i=0;i<300;i++){
        uint32_t random_num = esp_random();
        float  f_rnd=1.0f*random_num/RAND_MAX;
        String str1 = String(f_rnd);
        jsonPayload += str1;
        if (i<299){
            jsonPayload += ",";
        }
    }
    jsonPayload += "]}";

*/ 
// Build JSON array from features[]
    jsonPayload= "{";    
    jsonPayload += "\"data\":[";
    for(int i=0;i<FEATURE_SIZE;i++){
        for(int k=0;k<3;k++){
            float  f_rnd=features[3*i+k];
            jsonPayload += String(f_rnd);
            if (k<3){
                jsonPayload += ",";
            }
        }
        if (i<FEATURE_SIZE-1){
            jsonPayload += ",";
        }
    }
    jsonPayload += "]}";


    Serial.println("\n--- Sending Prediction to Server ---");
    Serial.println("URL: " + String(serverUrl));
    Serial.println("Payload: " + jsonPayload);
    
    //post(const char* aURLPath, const char* aContentType, const char* aBody)
   int httpResponseCode = http.post(serverUrl,"application/json",jsonPayload);
   Serial.print("HTTP Response code: ");
   Serial.println(httpResponseCode);

   if (httpResponseCode > 0) {
      String response = http.readString();
      Serial.println("Server response: " + response);

      // Parse the JSON response`
      //DynamicJsonDocument doc(256);doc(256);
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, response);
      if (!error) {
            const char* gesture = doc["gesture"];
            float confidence = doc["confidence"];

            Serial.println("Server Inference Result:");
            Serial.print("Gesture: ");
            Serial.println(gesture);
            Serial.print("Confidence: ");
            Serial.print(confidence);
            Serial.println("%");
            // Your code to acutate LED
      } else {
            Serial.print("Failed to parse server response: ");
            Serial.println(error.c_str());
      }

   } else {
     //Serial.printf("Error sending POST: %s\n", http.errorToString(httpResponseCode).c_str());
     Serial.printf("Error sending POST: %s\n",httpResponseCode);
   }

   http.flush(); //end();
}