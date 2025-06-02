#include <dummy.h>
#include <lab_inferencing.h>  // Update to match your actual Edge Impulse header
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <flatbuffers/flatbuffers.h>

Adafruit_MPU6050 mpu;

#define SAMPLE_RATE_MS 10
#define CAPTURE_DURATION_MS 1000
#define FEATURE_SIZE EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE
#define LED_PIN 2
#define CONFIDENCE_THRESHOLD 80.0

// === WiFi Configuration ===
const char* ssid = "UW MPSK";
const char* password = "/}pYj&L7@i";
const char* serverUrl = "http://10.19.68.101:5001/";
const char* studentId = "qh2130";

unsigned long last_sample_time = 0;
unsigned long capture_start_time = 0;
int sample_count = 0;
bool capturing = false;

float features[FEATURE_SIZE];

// Forward declaration to resolve scope issues
void flashLED(int times, int delay_ms = 200);

void setupWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to " + String(ssid));
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    setupWiFi();

    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) delay(10);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 initialized successfully");

    capturing = true;
    sample_count = 0;
    capture_start_time = millis();
    last_sample_time = millis();
}

void loop() {
    if (capturing) {
        capture_accelerometer_data();
    } else {
        capturing = true;
        sample_count = 0;
        capture_start_time = millis();
        last_sample_time = millis();
    }
}

void capture_accelerometer_data() {
    if (millis() - last_sample_time >= SAMPLE_RATE_MS) {
        last_sample_time = millis();

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        if (sample_count < FEATURE_SIZE / 3) {
            int idx = sample_count * 3;
            features[idx] = a.acceleration.x;
            features[idx + 1] = a.acceleration.y;
            features[idx + 2] = a.acceleration.z;
            sample_count++;
        }

        if (millis() - capture_start_time >= CAPTURE_DURATION_MS) {
            capturing = false;
            Serial.println("Capture complete");
            run_inference();
        }
    }
}

void run_inference() {
    if (sample_count * 3 < FEATURE_SIZE) {
        Serial.println("ERROR: Not enough data for inference");
        return;
    }

    ei_impulse_result_t result = { 0 };
    signal_t features_signal;
    features_signal.total_length = FEATURE_SIZE;
    features_signal.get_data = &raw_feature_get_data;

    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
    if (res != EI_IMPULSE_OK) {
        Serial.print("ERR: Failed to run classifier (");
        Serial.print(res);
        Serial.println(")");
        return;
    }

    print_inference_result(result);
}

void sendGestureToServer(const char* gesture, float confidence) {
    String jsonPayload = "{";
    jsonPayload += "\"student_id\":\"" + String(studentId) + "\",";
    jsonPayload += "\"gesture\":\"" + String(gesture) + "\",";
    jsonPayload += "\"confidence\":" + String(confidence, 2);
    jsonPayload += "}";

    Serial.println("\n--- Sending Prediction to Server ---");
    Serial.println("URL: " + String(serverUrl));
    Serial.println("Payload: " + jsonPayload);

    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonPayload);
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("Server response: " + response);
    } else {
        Serial.printf("Error sending POST: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    http.end();
    Serial.println("--- End of Request ---\n");
}

void sendRawDataToServer() {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    String jsonPayload = "{";
    jsonPayload += "\"student_id\":\"" + String(studentId) + "\",";
    jsonPayload += "\"features\":[";
    for (int i = 0; i < FEATURE_SIZE; i++) {
        jsonPayload += String(features[i], 6);
        if (i < FEATURE_SIZE - 1) jsonPayload += ",";
    }
    jsonPayload += "]}";

    Serial.println("\n--- Sending Raw Features to Server ---");
    Serial.println("Payload: " + jsonPayload);

    int httpResponseCode = http.POST(jsonPayload);
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    if (httpResponseCode > 0) {
        String response = http.getString();
        Serial.println("Server response: " + response);

        DynamicJsonDocument doc(256);
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

            if (strcmp(gesture, "O") == 0) flashLED(1, 150);
            else if (strcmp(gesture, "V") == 0) flashLED(2, 200);
            else if (strcmp(gesture, "Z") == 0) flashLED(3, 250);
        } else {
            Serial.print("Failed to parse server response: ");
            Serial.println(error.c_str());
        }

    } else {
        Serial.printf("Error sending POST: %s\n", http.errorToString(httpResponseCode).c_str());
    }

    http.end();
    Serial.println("--- End of Raw Data Request ---\n");
}

void flashLED(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(delay_ms);
        digitalWrite(LED_PIN, LOW);
        delay(delay_ms);
    }
}

void print_inference_result(ei_impulse_result_t result) {
    float max_value = 0;
    int max_index = -1;

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_value) {
            max_value = result.classification[i].value;
            max_index = i;
        }
    }

    if (max_index != -1) {
        String label = ei_classifier_inferencing_categories[max_index];
        float confidence = max_value * 100.0;

        Serial.print("Prediction: ");
        Serial.print(label);
        Serial.print(" (");
        Serial.print(confidence, 2);
        Serial.println("%)");

        if (confidence < CONFIDENCE_THRESHOLD) {
            Serial.println("Low confidence - sending raw data to server...");
            sendRawDataToServer();
        } else {
            if (label == "O") flashLED(1, 150);
            else if (label == "V") flashLED(2, 200);
            else if (label == "Z") flashLED(3, 250);

            sendGestureToServer(label.c_str(), max_value);
        }
    }
}void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
