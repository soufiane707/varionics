#include <Wire.h>
#include <ESP32Servo.h>          // ESP32Servo library
#include <WiFi.h>
#include <Firebase_ESP_Client.h> // Firebase ESP Client library
#include "addons/TokenHelper.h"  // Token generation helper
#include "addons/RTDBHelper.h"   // Realtime database helper

// Wi-Fi credentials
#define WIFI_SSID "ARDUINO"
#define WIFI_PASSWORD "12345678"

// Firebase credentials
#define API_KEY "AIzaSyCE8-k_d3QSoryJ-9Cug_HyQpflaaZjXok"
#define DATABASE_URL "https://flight-control-28ed7-default-rtdb.firebaseio.com/"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOK = false;

// Gyroscope I2C pins
#define SDA 26
#define SCL 27

// Servo pins
#define SERVO1_PIN 21
#define SERVO2_PIN 19
#define SERVO3_PIN 13
#define SERVO4_PIN 12

Servo servo1, servo2, servo3, servo4; // Servo objects

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

// Function to read gyroscope signals
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6, 0);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

// Function to send data to Firebase
void sendToFirebase(float pitch, float roll, float yaw) {
  if (Firebase.ready() && signupOK) {
    // Send Pitch
    if (Firebase.RTDB.setFloat(&fbdo, "gyro/pitch", pitch)) {
      Serial.println("Pitch sent successfully");
    } else {
      Serial.printf("Failed to send pitch: %s\n", fbdo.errorReason().c_str());
    }

    // Send Roll
    if (Firebase.RTDB.setFloat(&fbdo, "gyro/roll", roll)) {
      Serial.println("Roll sent successfully");
    } else {
      Serial.printf("Failed to send roll: %s\n", fbdo.errorReason().c_str());
    }

    // Send Yaw
    if (Firebase.RTDB.setFloat(&fbdo, "gyro/yaw", yaw)) {
      Serial.println("Yaw sent successfully");
    } else {
      Serial.printf("Failed to send yaw: %s\n", fbdo.errorReason().c_str());
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(SDA, SCL, 400000);
  delay(250);

  // MPU6050 setup
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Calibration
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);

  // Set servos to neutral position
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);

  // Wi-Fi connection
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());

  // Firebase configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback;

  // Anonymous sign-up for Firebase
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase sign-up successful");
    signupOK = true;
  } else {
    Serial.printf("Sign-up error: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Calculate pitch and roll control (simplified example)
  float pitch_control = RatePitch * 2; // Adjust multiplier as needed
  float roll_control = RateRoll * 2;   // Adjust multiplier as needed

  // Map control values to servo angles
  int servo1_angle = constrain(90 + pitch_control + roll_control, 0, 180);
  int servo2_angle = constrain(90 + pitch_control - roll_control, 0, 180);
  int servo3_angle = constrain(90 - pitch_control + roll_control, 0, 180);
  int servo4_angle = constrain(90 - pitch_control - roll_control, 0, 180);

  // Set servo angles
  servo1.write(servo1_angle);
  servo2.write(servo2_angle);
  servo3.write(servo3_angle);
  servo4.write(servo4_angle);

  // Send data to Firebase
  sendToFirebase(RatePitch, RateRoll, RateYaw);

  // Debug output
  Serial.print("Roll Rate [/s]= ");
  Serial.print(RateRoll);
  Serial.print(" Pitch Rate [/s]= ");
  Serial.print(RatePitch);
  Serial.print(" Yaw Rate [/s]= ");
  Serial.println(RateYaw);

  delay(50);
}
