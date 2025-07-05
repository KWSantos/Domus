#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "wifi_keys.h"
#include "motor_control.h" 
#include "distance_sensor.h"
#include "servo_control.h" 

#define WEBSOCKET_PORT 81

WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);
uint8_t client_num = 255; 

Adafruit_MPU6050 mpu;
float gyro_z_offset = 0.0;
unsigned long prev_time_micros = 0;
unsigned long actionStartTime = 0; 

// IPAddress staticIP(10, 42, 0, 3);
// IPAddress gateway(10, 42, 0, 1);
IPAddress staticIP(192, 168, 0, 107);   
IPAddress gateway(192, 168, 0, 1); 
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

enum RobotState {
  IDLE,
  MOVING_FORWARD,
  GRAB_SEQUENCE_START,
  GRAB_SEQUENCE_OPEN_CLAW,
  GRAB_SEQUENCE_ADVANCE,
  GRAB_SEQUENCE_CLOSE_CLAW,
  GRAB_SEQUENCE_TURN,
  GRAB_SEQUENCE_COMPLETE
};
RobotState currentState = IDLE; 

void calibrateGyro() {
  long total_gyro_z = 0;
  for (int i = 0; i < 1000; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    total_gyro_z += g.gyro.z;
    delay(2);
  }
  gyro_z_offset = (float)total_gyro_z / 1000.0;
}

void turnByDegrees(float targetAngle) {
  float yawTurned = 0;
  (targetAngle > 0) ? turnRight() : turnLeft();
  prev_time_micros = micros();

  while (abs(yawTurned) < abs(targetAngle)) {
    webSocket.loop(); 
    unsigned long current_time_micros = micros();
    if (current_time_micros > prev_time_micros) {
      float deltaTime = (current_time_micros - prev_time_micros) / 1000000.0;
      prev_time_micros = current_time_micros;
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float gyro_z_calibrated = g.gyro.z - gyro_z_offset;
      yawTurned += gyro_z_calibrated * (180.0 / PI) * deltaTime;
    }
    delay(1);
  }
  stopMotors();
  if (client_num != 255) {
    webSocket.sendTXT(client_num, "TURN_COMPLETE");
  }
}

void handleCommand(const String& command) {
  if (currentState != IDLE) {
    return;
  }

  if (command == "MOVE_FORWARD") {
    currentState = MOVING_FORWARD;
  } else if (command.startsWith("TURN:")) {
    float angle = command.substring(5).toFloat();
    turnByDegrees(angle); 
    currentState = IDLE;
  } else if (command == "STOP") {
    currentState = IDLE;
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar o chip MPU6050. Travando.");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  calibrateGyro();
  
  if (!WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Falha ao configurar o IP Estático (STA)");
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  setupServos();
  setupMotors();
  setupUltrasonic();

  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_CONNECTED) {
      client_num = num;
    } else if (type == WStype_DISCONNECTED) {
      client_num = 255;
      currentState = IDLE;
      stopMotors();
    } else if (type == WStype_TEXT) {
      String command = String((char*)payload);
      Serial.printf("Comando recebido: %s\n", command.c_str());
      handleCommand(command);
    }
  });

  Serial.println("Setup completo. Robô pronto.");
}

void loop() {
  webSocket.loop();

  switch (currentState) {
    case IDLE:
      stopMotors();
      break;

    case MOVING_FORWARD: {
      long distance = readDistanceCM();
      if (distance <= 15 && distance > 0) {
        stopMotors();
        currentState = GRAB_SEQUENCE_START;
      } else {
        moveForward();
      }
      break;
    }


    case GRAB_SEQUENCE_START:
      openClaw();
      actionStartTime = millis();
      currentState = GRAB_SEQUENCE_OPEN_CLAW;
      break;

    case GRAB_SEQUENCE_OPEN_CLAW:
      if (millis() - actionStartTime > 500) {
        moveForward();
        actionStartTime = millis();
        currentState = GRAB_SEQUENCE_ADVANCE;
      }
      break;

    case GRAB_SEQUENCE_ADVANCE:
      if (millis() - actionStartTime > 300) {
        stopMotors();
        closeClaw();
        actionStartTime = millis();
        currentState = GRAB_SEQUENCE_CLOSE_CLAW;
      }
      break;

    case GRAB_SEQUENCE_CLOSE_CLAW:
      if (millis() - actionStartTime > 500) {
        currentState = GRAB_SEQUENCE_TURN;
      }
      break;

    case GRAB_SEQUENCE_TURN:
      turnByDegrees(180.0);
      currentState = GRAB_SEQUENCE_COMPLETE;
      break;

    case GRAB_SEQUENCE_COMPLETE:
      webSocket.sendTXT(client_num, "GRAB_SEQUENCE_COMPLETE");
      currentState = IDLE;
      break;

    default:
      stopMotors();
      currentState = IDLE;
      break;
  }

  delay(10); 
}