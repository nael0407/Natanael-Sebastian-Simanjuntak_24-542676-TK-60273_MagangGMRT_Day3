#include <Wire.h>
#include <MPU6050_tockn.h> 
#include <ESP32Servo.h>
#include <Arduino.h>

const int MPU_SDA = 21; 
const int MPU_SCL = 22; 
const int PIR_PIN = 14; 

const int SERVO1_PIN = 18; 
const int SERVO2_PIN = 19; 
const int SERVO3_PIN = 23; 
const int SERVO4_PIN = 25; 
const int SERVO5_PIN = 32; 

MPU6050 mpu6050(Wire);
Servo servo1, servo2, servo3, servo4, servo5;

const int INITIAL_POS = 0; 
const int MAX_ANGLE = 90;  
const int MIN_ANGLE = -90; 

unsigned long yaw_stop_time = 0;
const long YAW_RESET_DELAY = 1000; 
const float YAW_CHANGE_THRESHOLD = 0.3;
bool yaw_is_resetting = false;
bool yaw_reset_done = false;
float last_yaw_position = 0;
float previous_yaw = 0;
unsigned long last_movement_time = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(MPU_SDA, MPU_SCL); 

  mpu6050.begin();
  mpu6050.calcGyroOffsets(); 

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  servo1.write(INITIAL_POS + 90); 
  servo2.write(INITIAL_POS + 90);
  servo3.write(INITIAL_POS + 90);
  servo4.write(INITIAL_POS + 90);
  servo5.write(INITIAL_POS + 90);
  delay(1000);

  pinMode(PIR_PIN, INPUT);
}

void writeServo(Servo& servo, float angle) {

    angle = constrain(angle, MIN_ANGLE, MAX_ANGLE); 
    int servo_angle = (int)(angle + 90);
    servo.write(servo_angle);
}

void resetAllServos() {
    writeServo(servo1, INITIAL_POS);
    writeServo(servo2, INITIAL_POS);
    writeServo(servo3, INITIAL_POS);
    writeServo(servo4, INITIAL_POS);
    writeServo(servo5, INITIAL_POS);
}

void loop() {
  mpu6050.update();
  
  float roll = mpu6050.getAngleX();
  float pitch = mpu6050.getAngleY();
  float yaw = mpu6050.getAngleZ();

  roll = constrain(roll, MIN_ANGLE, MAX_ANGLE);
  pitch = constrain(pitch, MIN_ANGLE, MAX_ANGLE);
  yaw = constrain(yaw, MIN_ANGLE, MAX_ANGLE); 
  float yaw_change = abs(yaw - previous_yaw);

  Serial.print("Yaw: "); Serial.print(yaw);
  Serial.print(" | abs(Yaw) > 5: "); Serial.print(abs(yaw) > 5);
  Serial.print(" | Timer: "); Serial.println(yaw_stop_time);

  if (digitalRead(PIR_PIN) == HIGH) {
    Serial.println("Gerakan Eksternal Terdeteksi!");
    
    int custom_pos = 45; 
    writeServo(servo1, custom_pos);
    writeServo(servo2, custom_pos);
    writeServo(servo3, custom_pos);
    writeServo(servo4, custom_pos);
    writeServo(servo5, custom_pos);
    
    yaw_stop_time = 0;
    yaw_is_resetting = false;
    yaw_reset_done = false;
    last_yaw_position = 0;

    delay(500); 
    resetAllServos(); 
  } 

  else {
    float target_roll = -roll; 
    writeServo(servo1, target_roll);
    writeServo(servo2, target_roll);

    float target_pitch = pitch; 
    writeServo(servo3, target_pitch);
    writeServo(servo4, target_pitch);

    if (yaw_change > YAW_CHANGE_THRESHOLD) { 
        writeServo(servo5, yaw);
        last_yaw_position = yaw;
        last_movement_time = millis();
        // Reset semua flag dan timer
        yaw_stop_time = 0; 
        yaw_is_resetting = false;
        yaw_reset_done = false;
        
        Serial.println(">>> YAW: Mengikuti rotasi <<<");
    } 
    // Jika yaw berhenti berubah (rotasi berhenti)
    else { 
        // Cek apakah baru saja berhenti (dalam 100ms terakhir masih ada gerakan)
        if (last_movement_time > 0 && (millis() - last_movement_time < 100)) {
            // Masih dalam fase transisi berhenti, update servo ke posisi terakhir
            writeServo(servo5, yaw);
            last_yaw_position = yaw;
        }
        else if (!yaw_is_resetting && !yaw_reset_done) {
            // Pertahankan posisi terakhir
            writeServo(servo5, last_yaw_position);
            
            if (yaw_stop_time == 0) {
                yaw_stop_time = millis(); 
                Serial.println(">>> YAW: Rotasi berhenti. Timer 1 detik dimulai. <<<");
            }

            if (millis() - yaw_stop_time >= YAW_RESET_DELAY) {
                yaw_is_resetting = true;
                Serial.println("--- YAW: 1 detik selesai. Mulai reset... ---");
            }
        }
        else if (yaw_is_resetting && !yaw_reset_done) {
            writeServo(servo5, INITIAL_POS);
            yaw_reset_done = true;
            yaw_is_resetting = false;
            last_yaw_position = 0;
            Serial.println("*** YAW: Kembali ke posisi awal ***");
        }
        else if (yaw_reset_done) {
            writeServo(servo5, INITIAL_POS);
        }
    }
  }

  // Update previous_yaw untuk loop berikutnya
  previous_yaw = yaw;
  
  delay(10); 
}