#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"

#include <IRremote.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "audio_data.h"
#include "wifi_keys.h"
#include "motor_control.h"
#include "distance_sensor.h"
#include "servo_control.h"

#define WEBSOCKET_PORT 81
const int IR_RECEIVE_PIN = 15;

#define I2S_BCLK      32
#define I2S_LRC       34
#define I2S_DOUT      35

AudioFileSourcePROGMEM *file = NULL;
AudioOutputI2S *out = NULL;
AudioGeneratorMP3 *mp3 = NULL;
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);
Adafruit_MPU6050 mpu;

uint8_t client_num = 255;
float gyro_z_offset = 0.0;
unsigned long prev_time_micros = 0;
unsigned long actionStartTime = 0;
bool hasObject = false;
bool bootSoundPlayed = false;

IPAddress staticIP(192, 168, 0, 107);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

enum RobotState {
  IDLE,
  MOVING_FORWARD,
  GRAB_SEQUENCE_START, GRAB_SEQUENCE_OPEN_CLAW, GRAB_SEQUENCE_ADVANCE, GRAB_SEQUENCE_CLOSE_CLAW, GRAB_SEQUENCE_TURN, GRAB_SEQUENCE_COMPLETE,
  RELEASE_SEQUENCE_START, RELEASE_SEQUENCE_BACKUP, RELEASE_SEQUENCE_COMPLETE
};
RobotState currentState = IDLE;


void playMP3(const char *audioData) {
  if (mp3 && mp3->isRunning()) {
    mp3->stop();
  }
  file = new AudioFileSourcePROGMEM(audioData, strlen(audioData));
  mp3->begin(file, out);
  Serial.println("Tocando áudio da memória de programa (PROGMEM)...");
}

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
  if (currentState != IDLE) { return; }
  if (command == "MOVE_FORWARD") { currentState = MOVING_FORWARD; }
  else if (command.startsWith("TURN:")) {
    float angle = command.substring(5).toFloat();
    turnByDegrees(angle);
    currentState = IDLE;
  } else if (command == "STOP") {
    currentState = IDLE;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin();
  if (!mpu.begin()) { Serial.println("MPU6050 não encontrado!"); while (1); }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  calibrateGyro();
  
  if (!WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS)) { Serial.println("Falha STA"); }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando ao WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi conectado!"); Serial.print("IP: "); Serial.println(WiFi.localIP());

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  Serial.println("Receptor Infravermelho iniciado.");
  
  out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  mp3 = new AudioGeneratorMP3();

  setupServos();
  setupMotors();
  setupUltrasonic();

  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_CONNECTED) { client_num = num; }
    else if (type == WStype_DISCONNECTED) {
      client_num = 255;
      currentState = IDLE;
      hasObject = false;
      stopMotors();
    } else if (type == WStype_TEXT) {
      String command = String((char*)payload);
      handleCommand(command);
    }
  });

  Serial.println("Setup completo. Robô pronto para iniciar o loop.");
}

void checkForIRCommands() {
  if (IrReceiver.decode()) {
    const char* audioToPlay = nullptr;
    String modeName = "";
    switch (IrReceiver.decodedIRData.command) {
      case 0xC:
        modeName = "FINDER";
        audioToPlay = finder_mode_mp3_b64;
        break;
      case 0x18:
        modeName = "BRING";
        audioToPlay = bring_mode_mp3_b64;
        break;
      case 0x5E:
        modeName = "FOLLOW";
        audioToPlay = follow_mode_mp3_b64;
        break;
    }
    if (modeName != "") {
      playMP3(audioToPlay);
      if (client_num != 255) {
        webSocket.sendTXT(client_num, "IR_COMMAND:" + modeName);
      }
    }
    IrReceiver.resume();
  }
}

void handleAudioLoop() {
  if (mp3 && mp3->isRunning()) {
    if (!mp3->loop()) {
      mp3->stop();
    }
  }
}

void loop() {
  if (!bootSoundPlayed) {
    playMP3(wake_up_mp3_b64);
    bootSoundPlayed = true;
  }

  webSocket.loop();
  checkForIRCommands();
  handleAudioLoop();

  switch (currentState) {
    case IDLE:
      stopMotors();
      break;
    case MOVING_FORWARD: {
      long distance = readDistanceCM();
      if (distance <= 12 && distance > 0) {
        stopMotors();
        if (hasObject == false) { currentState = GRAB_SEQUENCE_START; }
        else { currentState = RELEASE_SEQUENCE_START; }
      } else { moveForward(); }
      break;
    }
    case GRAB_SEQUENCE_START: openClaw(); actionStartTime = millis(); currentState = GRAB_SEQUENCE_OPEN_CLAW; break;
    case GRAB_SEQUENCE_OPEN_CLAW: if (millis() - actionStartTime > 500) { moveForward(); actionStartTime = millis(); currentState = GRAB_SEQUENCE_ADVANCE; } break;
    case GRAB_SEQUENCE_ADVANCE: if (millis() - actionStartTime > 300) { stopMotors(); closeClaw(); actionStartTime = millis(); currentState = GRAB_SEQUENCE_CLOSE_CLAW; } break;
    case GRAB_SEQUENCE_CLOSE_CLAW: if (millis() - actionStartTime > 500) { currentState = GRAB_SEQUENCE_TURN; } break;
    case GRAB_SEQUENCE_TURN: turnByDegrees(180.0); currentState = GRAB_SEQUENCE_COMPLETE; break;
    case GRAB_SEQUENCE_COMPLETE: webSocket.sendTXT(client_num, "GRAB_SEQUENCE_COMPLETE"); hasObject = true; currentState = IDLE; break;
    case RELEASE_SEQUENCE_START: openClaw(); actionStartTime = millis(); currentState = RELEASE_SEQUENCE_BACKUP; break;
    case RELEASE_SEQUENCE_BACKUP: if (millis() - actionStartTime > 500) { moveBackward(); actionStartTime = millis(); currentState = RELEASE_SEQUENCE_COMPLETE; } break;
    case RELEASE_SEQUENCE_COMPLETE:
      if (millis() - actionStartTime > 400) {
        stopMotors(); closeClaw(); webSocket.sendTXT(client_num, "DELIVERY_COMPLETE"); hasObject = false; currentState = IDLE;
      }
      break;
    default:
      stopMotors();
      currentState = IDLE;
      break;
  }
  delay(10);
}