#include <WiFi.h>
#include <esp_camera.h>
#include <WebSocketsServer.h>

// Config Wi-Fi
#define WIFI_SSID "I9FIBRA_JORGE"
#define WIFI_PASSWORD "2020.2020."
#define WEBSOCKET_PORT 81

WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);

// Pinos do módulo AI-Thinker
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     21
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       19
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Erro ao iniciar a câmera: 0x%x\n", err);
    while (true) delay(1000);
  }

  Serial.println("Câmera iniciada com sucesso");
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

  webSocket.begin();
  webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_CONNECTED) {
      Serial.printf("Cliente %u conectado\n", num);
    } else if (type == WStype_DISCONNECTED) {
      Serial.printf("Cliente %u desconectado\n", num);
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

  delay(33);  // Aproximadamente 30 FPS
}
