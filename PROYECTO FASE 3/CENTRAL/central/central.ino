#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>

// --- CONFIGURACIÓN DE RED Y UBIDOTS ---
const char* WIFI_SSID = "2S2026";
const char* WIFI_PASS = "1234567890";
const char* UBIDOTS_TOKEN = "BBUS-J2Z09hBPsRMbMna7uvXE2r050Ks4P7";

// --- PINES LORA ESP32 ---
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23
#define SS_PIN    5
#define RST_PIN   14
#define DIO0_PIN  2

// --- DECODIFICACIÓN HAMMING (7,4) ---
uint8_t decodificar_hamming74_nibble(uint8_t trama_7bits, uint8_t &sindrome_out) {
  uint8_t p1 = (trama_7bits >> 6) & 0x01;
  uint8_t p2 = (trama_7bits >> 5) & 0x01;
  uint8_t d1 = (trama_7bits >> 4) & 0x01;
  uint8_t p3 = (trama_7bits >> 3) & 0x01;
  uint8_t d2 = (trama_7bits >> 2) & 0x01;
  uint8_t d3 = (trama_7bits >> 1) & 0x01;
  uint8_t d4 = trama_7bits & 0x01;

  uint8_t s1 = p1 ^ d1 ^ d2 ^ d4;
  uint8_t s2 = p2 ^ d1 ^ d3 ^ d4;
  uint8_t s3 = p3 ^ d2 ^ d3 ^ d4;
  sindrome_out = (s3 << 2) | (s2 << 1) | s1;

  uint8_t trama_corr = trama_7bits;
  if (sindrome_out != 0) trama_corr ^= (1 << (7 - sindrome_out));

  uint8_t d1_c = (trama_corr >> 4) & 0x01;
  uint8_t d2_c = (trama_corr >> 2) & 0x01;
  uint8_t d3_c = (trama_corr >> 1) & 0x01;
  uint8_t d4_c = trama_corr & 0x01;

  return (d1_c << 3) | (d2_c << 2) | (d3_c << 1) | d4_c;
}

// --- FUNCIÓN PARA ENVIAR A UBIDOTS ---
void enviarAUbidots(uint8_t id_nodo, float temp, float hum, bool puerta, bool alarma, int rssi, float snr) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    // URL dinámica: crea/actualiza un dispositivo por cada nodo (nodo-1, nodo-2, nodo-3)
    String url = "http://industrial.api.ubidots.com/api/v1.6/devices/nodo-" + String(id_nodo);
    
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Auth-Token", UBIDOTS_TOKEN);

    // Armamos el JSON con los datos de telemetría y calidad de enlace LoRa
    String payload = "{\"temperatura\":" + String(temp) + 
                     ",\"humedad\":" + String(hum) + 
                     ",\"puerta_abierta\":" + String(puerta) + 
                     ",\"estado_alarma\":" + String(alarma) + 
                     ",\"rssi_lora\":" + String(rssi) + 
                     ",\"snr_lora\":" + String(snr) + "}";

    int httpResponseCode = http.POST(payload);
    
    if (httpResponseCode > 0) {
      Serial.printf("🌐 [Ubidots] Datos Nodo %d enviados correctamente (HTTP %d)\n", id_nodo, httpResponseCode);
    } else {
      Serial.printf("❌ [Ubidots] Error al enviar Nodo %d: %s\n", id_nodo, http.errorToString(httpResponseCode).c_str());
    }
    http.end();
  } else {
    Serial.println("⚠️ Wi-Fi desconectado, no se pudo enviar a la nube.");
  }
}

void setup() {
  Serial.begin(115200);
  
  // 1. Conexión Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Conectando a Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Wi-Fi Conectado!");

  // 2. Inicialización LoRa
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);
  if (!LoRa.begin(915E6)) {
    Serial.println("❌ Fallo módulo LoRa");
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  Serial.println("📡 ESTACIÓN CENTRAL SIMCOFI LISTA (Wi-Fi + LoRa)");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  
  if (packetSize >= 7) {
    uint8_t id_nodo = LoRa.read();
    uint8_t seq = LoRa.read();
    uint8_t temp = LoRa.read();
    uint8_t hum = LoRa.read();
    uint8_t flags_raw = LoRa.read();
    uint8_t ham_alto = LoRa.read();
    uint8_t ham_bajo = LoRa.read();

    if (id_nodo >= 1 && id_nodo <= 3) {
      // Corrección de Hamming
      uint8_t sindrome_h, sindrome_l;
      uint8_t nibble_h = decodificar_hamming74_nibble(ham_alto, sindrome_h);
      uint8_t nibble_l = decodificar_hamming74_nibble(ham_bajo, sindrome_l);
      uint8_t flags_corregidos = (nibble_h << 4) | nibble_l;

      bool puerta_abierta = (flags_corregidos & 0x01);
      bool alarma_activa = ((flags_corregidos >> 1) & 0x01);
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();

      Serial.printf("📥 Rx Nodo %d (Seq:%d) | Temp:%d°C Hum:%d%% | Puerta:%d Alarma:%d | RSSI:%d dBm\n", 
                    id_nodo, seq, temp, hum, puerta_abierta, alarma_activa, rssi);

      // Enviar a la nube
      enviarAUbidots(id_nodo, temp, hum, puerta_abierta, alarma_activa, rssi, snr);
    }
  }
}
