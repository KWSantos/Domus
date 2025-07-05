#include <WiFi.h>
#include <esp_camera.h>
#include <WebSocketsServer.h>
#include <wifi_keys.h>
#include <cam.h>

#define WEBSOCKET_PORT 81
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);

// IPAddress staticIP(10, 42, 0, 2);
// IPAddress gateway(10, 42, 0, 1);
IPAddress staticIP(192, 168, 0, 106);   
IPAddress gateway(192, 168, 0, 1);     
IPAddress subnet(255, 255, 255, 0); 
IPAddress primaryDNS(8, 8, 8, 8); 
IPAddress secondaryDNS(8, 8, 4, 4); 


void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);

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

  delay(150);
}