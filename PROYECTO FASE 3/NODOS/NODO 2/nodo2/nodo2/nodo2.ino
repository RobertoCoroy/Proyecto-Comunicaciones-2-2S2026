#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

// --- CONFIGURACIÓN DEL NODO ---
#define ID_NODO 2             // Cambiar a 1, 2 o 3 según corresponda
#define NUM_NODOS 3
#define SLOT_DURATION_MS 3000 // Cada nodo tiene 3 segundos de slot
#define FRAME_DURATION_MS (NUM_NODOS * SLOT_DURATION_MS) // Trama TDM de 9 segundos

// --- PINES ESP32 ---
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23
#define SS_PIN    5
#define RST_PIN   14
#define DIO0_PIN  2

#define DHTPIN    4
#define DHTTYPE   DHT11
#define DOOR_PIN  13
#define ALARM_LED 12

// --- UMBRALES DE CADENA DE FRÍO ---
#define TEMP_MAX_UMBRAL 8.0 // Umbral de temperatura en °C

DHT dht(DHTPIN, DHTTYPE);
uint8_t contador_paquetes = 0;

// --- FUNCIONES DE CÓDIGO DE HAMMING (7,4) ---
uint8_t codificar_hamming74_nibble(uint8_t datos4bits) {
  uint8_t d1 = (datos4bits >> 3) & 0x01;
  uint8_t d2 = (datos4bits >> 2) & 0x01;
  uint8_t d3 = (datos4bits >> 1) & 0x01;
  uint8_t d4 = datos4bits & 0x01;

  uint8_t p1 = d1 ^ d2 ^ d4;
  uint8_t p2 = d1 ^ d3 ^ d4;
  uint8_t p3 = d2 ^ d3 ^ d4;

  // Formato: [p1, p2, d1, p3, d2, d3, d4]
  return (p1 << 6) | (p2 << 5) | (d1 << 4) | (p3 << 3) | (d2 << 2) | (d3 << 1) | d4;
}

void setup() {
  Serial.begin(115200);
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(ALARM_LED, OUTPUT);
  dht.begin();

  // Inicialización de LoRa SPI
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DIO0_PIN);

  if (!LoRa.begin(915E6)) { // Banda 915 MHz
    Serial.println("❌ Fallo al iniciar módulo LoRa SX1276");
    while (1);
  }
  
  // Parámetros de comunicación LoRa
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setTxPower(14);

  Serial.printf("✅ Nodo %d iniciado correctamente.\n", ID_NODO);
}

void loop() {
  // Sincronización TDM básica basada en millis()
  unsigned long tiempo_actual = millis();
  unsigned long tiempo_en_trama = tiempo_actual % FRAME_DURATION_MS;
  unsigned long slot_esperado = (ID_NODO - 1) * SLOT_DURATION_MS;

  // Transmitir únicamente en el Slot asignado al nodo
  if (tiempo_en_trama >= slot_esperado && tiempo_en_trama < (slot_esperado + 500)) {
    
    // 1. Adquisición de Variables
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    bool puerta_abierta = (digitalRead(DOOR_PIN) == HIGH); // 1 = Abierta, 0 = Cerrada

    if (isnan(temp) || isnan(hum)) {
      temp = 0.0;
      hum = 0.0;
    }

    // 2. Evaluación de Alarma
    bool alarma_temp = (temp > TEMP_MAX_UMBRAL);
    if (alarma_temp || puerta_abierta) {
      digitalWrite(ALARM_LED, HIGH);
    } else {
      digitalWrite(ALARM_LED, LOW);
    }

    uint8_t temp_int = (uint8_t)temp;
    uint8_t hum_int = (uint8_t)hum;
    uint8_t estado_flags = (alarma_temp << 1) | (puerta_abierta ? 1 : 0);

    // 3. Codificación Hamming del byte de estado (dividido en 2 nibbles)
    uint8_t hamming_alto = codificar_hamming74_nibble((estado_flags >> 4) & 0x0F);
    uint8_t hamming_bajo = codificar_hamming74_nibble(estado_flags & 0x0F);

    // 4. Envío de Trama LoRa
    LoRa.beginPacket();
    LoRa.write(ID_NODO);             // Byte 0: ID Nodo
    LoRa.write(contador_paquetes);  // Byte 1: Secuencia Paquete
    LoRa.write(temp_int);           // Byte 2: Temperatura
    LoRa.write(hum_int);            // Byte 3: Humedad
    LoRa.write(estado_flags);       // Byte 4: Flags sin proteger
    LoRa.write(hamming_alto);       // Byte 5: Hamming Nibble Alto
    LoRa.write(hamming_bajo);       // Byte 6: Hamming Nibble Bajo
    LoRa.endPacket();

    Serial.printf("[TDM Slot %d] Paquete #%d enviado -> Temp: %.1f°C, Hum: %.1f%%, Puerta: %s, Alarma: %s\n",
                  ID_NODO, contador_paquetes, temp, hum, 
                  puerta_abierta ? "ABIERTA" : "CERRADA",
                  alarma_temp ? "ACTIVADA" : "OK");

    contador_paquetes++;
    delay(SLOT_DURATION_MS); // Evita retransmisiones múltiples dentro del mismo slot
  }
}
