/*
Estación Terrena (Esclavo) - Versión Mejorada con Multitarea
Recibe datos continuamente por LoRa y envía datos esporádicos por MQTT
Utiliza ambos núcleos del ESP32:
- Núcleo 0: Manejo de WiFi y MQTT
- Núcleo 1: Recepción y procesamiento de LoRa
*/

#define idManualLanzamiento "ETR_Movil"

// Librerías
#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Definición de tamaños de buffer
#define MQTT_MAX_PACKET_SIZE 128
#define MQTT_TOPIC_SIZE 20

// Variables de WiFi
#define ssid "UM4"
#define password "delunoalocho"
#define WIFI_RETRY_DELAY 5000 // 5 segundos entre intentos de reconexión WiFi

// Variables de MQTT
#define mqttServer "167.234.252.122"
#define mqttPort 1883
#define mqttID "E. Terrena - TTGO T-BEAM"
#define mqttUser "admin"
#define mqttKeepalive 15
#define mqttPassword "root1234"
#define MQTT_RETRY_DELAY 5000 // 5 segundos entre intentos

// Pines LoRa para ESP32
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 23   // 14 = ESP32 OLED - 23 = T-BEAM ESP32
#define DIO0 26

// Frecuencia de operación LoRa
#define BAND 915E6

// Definición de tareas
TaskHandle_t TaskLoRa;      // Tarea para LoRa (Núcleo 1)
TaskHandle_t TaskNetwork;   // Tarea para WiFi/MQTT (Núcleo 0)

// Variables para control de tiempo
unsigned long lastWifiRetry = 0;
unsigned long lastMqttRetry = 0;

// Cola para comunicación entre núcleos
QueueHandle_t dataQueue;

// Estructura para mensajes LoRa
struct Mensaje {
  String sensor;
  String datoLoRa;
};

// Estructura para la cola de mensajes entre núcleos
struct QueueMessage {
  char sensor[2];
  char data[MQTT_MAX_PACKET_SIZE];
};

// Variables globales
WiFiClient espClient;
PubSubClient client(espClient);
SemaphoreHandle_t mqttMutex; // Para proteger acceso a cliente MQTT

// Función para verificar y reconectar WiFi
boolean checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWifiRetry >= WIFI_RETRY_DELAY) {
      Serial.println("WiFi desconectado. Intentando reconexión...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      lastWifiRetry = currentMillis;
    }
    return false;
  }
  return true;
}

// Función para reconectar MQTT
boolean reconnectMQTT() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMqttRetry >= MQTT_RETRY_DELAY) {
    Serial.println("Intentando conexión MQTT...");
    if (client.connect(mqttID, mqttUser, mqttPassword)) {
      Serial.println("Conectado a MQTT");
      lastMqttRetry = 0; // Resetear el contador si la conexión es exitosa
      return true;
    } else {
      Serial.print("Falló conexión MQTT, rc=");
      Serial.print(client.state());
      Serial.println(" reintentando en 5 segundos");
      lastMqttRetry = currentMillis;
    }
  }
  return false;
}

// Tarea para manejar LoRa (Núcleo 1)
void TaskLoRaCode(void *parameter) {
  // Configurar LoRa
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  // Configurar el factor de propagación (SF) y ancho de banda
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);

  if (!LoRa.begin(BAND)) {
    Serial.println("Error iniciando LoRa");
    vTaskDelete(NULL); // Eliminar tarea si falla
  }
  Serial.println("Inicio exitoso de LoRa!");

  while (true) {
    // Recepción de paquetes LoRa
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String incoming = "";
      while (LoRa.available()) {
        incoming += (char)LoRa.read();
      }

      int separatorIndex = incoming.indexOf(':');
      
      if (separatorIndex != -1) {
        String sensor = incoming.substring(0, separatorIndex);
        String datoLoRa = incoming.substring(separatorIndex + 1);
        datoLoRa = datoLoRa + "," + idManualLanzamiento;
        
        // Preparar mensaje para la cola
        QueueMessage qMsg;
        memset(qMsg.sensor, 0, sizeof(qMsg.sensor));
        memset(qMsg.data, 0, sizeof(qMsg.data));
        
        sensor.toCharArray(qMsg.sensor, sizeof(qMsg.sensor));
        datoLoRa.toCharArray(qMsg.data, sizeof(qMsg.data));
        
        // Enviar a la cola con timeout de 10ms
        if (xQueueSend(dataQueue, &qMsg, pdMS_TO_TICKS(10)) != pdPASS) {
          Serial.println("Error: Cola llena, mensaje descartado");
        }
      }
    }
    // Pequeño delay para no saturar el CPU
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// Tarea para manejar red (Núcleo 0)
void TaskNetworkCode(void *parameter) {
  // Esta tarea ya se ejecuta en el núcleo 0 por defecto
  
  // Configurar WiFi
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado a WiFi");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFalló conexión WiFi inicial - continuando...");
  }

  // Configurar MQTT
  client.setKeepAlive(mqttKeepalive);
  client.setServer(mqttServer, mqttPort);
  
  if (WiFi.status() == WL_CONNECTED) {
    if (!reconnectMQTT()) {
      Serial.println("Fallo en conexión inicial MQTT - continuando...");
    }
  }

  QueueMessage receivedMsg;

  while (true) {
    // Verificar conexiones
    if (!checkWiFiConnection()) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue; // Si no hay WiFi, no intentamos MQTT
    }
    
    if (!client.connected()) {
      reconnectMQTT();
    }
    
    // Procesar cola de mensajes
    if (xQueueReceive(dataQueue, &receivedMsg, pdMS_TO_TICKS(10)) == pdPASS) {
      // Determinar el tópico basado en el sensor
      const char* topico;
      if (strcmp(receivedMsg.sensor, "G") == 0) {
        topico = "valuesGPS";
      } else if (strcmp(receivedMsg.sensor, "M") == 0) {
        topico = "valuesMPL";
      } else if (strcmp(receivedMsg.sensor, "A") == 0) {
        topico = "valuesBMM";
      } else if (strcmp(receivedMsg.sensor, "U") == 0) {
        topico = "valuesMPU";
      } else {
        topico = "unknownTopic";
      }

      // Publicar mensaje MQTT
      xSemaphoreTake(mqttMutex, portMAX_DELAY);
      
      Serial.print("Publicando en ");
      Serial.print(topico);
      Serial.print(" -> ");
      Serial.println(receivedMsg.data);
      
      if (client.publish(topico, receivedMsg.data)) {
        // Publicación exitosa
      } else {
        Serial.println("Error en la publicación");
      }
      
      client.loop();
      xSemaphoreGive(mqttMutex);
    }
    
    // Para el mantenimiento del cliente MQTT
    xSemaphoreTake(mqttMutex, portMAX_DELAY);
    client.loop();
    xSemaphoreGive(mqttMutex);
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Dar tiempo al monitor serie para iniciar
  
  Serial.println("Iniciando Estación Terrena con multitarea");
  
  // Crear semáforo para MQTT
  mqttMutex = xSemaphoreCreateMutex();
  
  // Crear cola para comunicación entre tareas
  dataQueue = xQueueCreate(10, sizeof(QueueMessage));
  
  if (dataQueue == NULL || mqttMutex == NULL) {
    Serial.println("Error creando recursos del sistema");
    while(1); // Error crítico
  }

  // Crear tareas
  xTaskCreatePinnedToCore(
    TaskLoRaCode,     // Función de la tarea
    "TaskLoRa",       // Nombre
    8192,             // Tamaño de stack (bytes)
    NULL,             // Parámetros
    1,                // Prioridad (0-24, mayor número = mayor prioridad)
    &TaskLoRa,        // Handle de la tarea
    1);               // Núcleo 1
    
  xTaskCreatePinnedToCore(
    TaskNetworkCode,  // Función de la tarea
    "TaskNetwork",    // Nombre
    8192,             // Tamaño de stack
    NULL,             // Parámetros
    1,                // Prioridad
    &TaskNetwork,     // Handle de la tarea
    0);               // Núcleo 0
}

void loop() {
  // El loop principal no se utiliza cuando se trabaja con tareas FreeRTOS
  vTaskDelay(pdMS_TO_TICKS(1000));
}