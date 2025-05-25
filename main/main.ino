#include <WiFi.h>
#include <WebSocketsServer.h>
#include "wifi_keys.h"
#include "cam.h"
#include "motor_control.h"
#include "distance_sensor.h"

#define WEBSOCKET_PORT 81

WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);

enum RobotState {
  IDLE,
  MOVING_FORWARD,
  TURNING_LEFT,
  TURNING_RIGHT
};

RobotState currentState = IDLE;

void handleCommand(const String& command) {
  if (command == "MOVE_FORWARD") {
    currentState = MOVING_FORWARD;
  } else if (command == "TURN_LEFT") {
    currentState = TURNING_LEFT;
  } else if (command == "TURN_RIGHT") {
    currentState = TURNING_RIGHT;
  } else if (command == "STOP") {
    stopMotors();
    currentState = IDLE;
  } else {
    Serial.println("Comando desconhecido.");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando ao WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());

  startCamera();
  setupMotors(); 
  setupUltrasonic(); 

  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_CONNECTED) {
      Serial.printf("Cliente %u conectado\n", num);
    } else if (type == WStype_DISCONNECTED) {
      Serial.printf("Cliente %u desconectado\n", num);
    } else if (type == WStype_TEXT) {
      String command = String((char*)payload).substring(0, length);
      Serial.print("Comando recebido: ");
      Serial.println(command);
      handleCommand(command);
    }
  });

  Serial.println("WebSocket iniciado");
}

void loop() {
  webSocket.loop();

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Falha ao capturar imagem");
    return;
  }

  webSocket.broadcastBIN(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  switch (currentState) {
    case MOVING_FORWARD: {
      long distance = readDistanceCM();
      if (distance <= 10) {
        stopMotors();
        currentState = IDLE;
        Serial.println("Objeto alcançado.");
      } else {
        moveForward();
      }
      break;
    }

    case TURNING_LEFT:
      turnLeft();
      delay(100); 
      stopMotors();
      currentState = IDLE;
      break;

    case TURNING_RIGHT:
      turnRight();
      delay(100);
      stopMotors();
      currentState = IDLE;
      break;

    case IDLE:
    default:
      stopMotors();
      break;
  }

  delay(10);
}
