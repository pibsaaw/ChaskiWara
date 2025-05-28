/*
Funcionaria como esclavo (Estacion Terrena), ya que es el encargado de 
recibir datos de manera continua y realizar envios de datos esporadicos al
maestro (Estacion Espacial)

- Núcleo 0: Manejo de pantalla OLED
- Núcleo 1: Tareas principales (WiFi, LoRa, MQTT)
*/

#define idManualLanzamiento "0K09SR"

// Librerías para LoRa, WiFi y MQTT
#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Librerías para OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Librerías para multitarea
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

// Definición de los pines I2C en el TTGO ESP32 OLED
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Definición de tamaños de buffer
#define MQTT_MAX_PACKET_SIZE 128
#define MQTT_TOPIC_SIZE 20

// Definición de núcleos
#define CORE_0 0
#define CORE_1 1
#define PRIORITY_0 1
#define PRIORITY_1 1
#define STACK_SIZE 10000

//Variables de WiFi
#define ssid "UM4"//"El mas lento de todos"//
#define password "delunoalocho"//"deverdad"//
#define WIFI_RETRY_DELAY 5000 // 5 segundos entre intentos de reconexión WiFi

//Variables de MQTT
#define mqttServer "167.234.252.122"
#define mqttPort 1883
#define mqttID "E. Terrena - TTGO ESP 32 OLED"
#define mqttUser "admin"
#define mqttKeepalive 15
#define mqttPassword "root1234"

//Debemos definir los pines que se utilizarán por el módulo LoRa T-Beam ESP32
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14   //14 = ESP32 OLED - 23 = T-BEAM ESP32
#define DIO0 26

//Aquí definimos una frecuencia de operación según nuestra ubicación.
#define BAND 915E6

// Variables para control de tiempo
unsigned long lastWifiRetry = 0;
unsigned long lastMqttRetry = 0;
unsigned long lastDisplayUpdate = 0;
const unsigned long MQTT_RETRY_DELAY = 5000; // 5 segundos entre intentos
const unsigned long DISPLAY_UPDATE_INTERVAL = 3000; // 3 segundos entre actualizaciones de pantalla

int cont = 0;  //Haremos un contador de paquetes enviados
int valorLeido = -1;  // Variable para almacenar el valor leído
int messageCount = 0; // Contador de mensajes recibidos

// Objeto de la pantalla OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

struct Mensaje {
  String sensor;
  String datoLoRa;
};
Mensaje m;

WiFiClient espClient;
PubSubClient client(espClient);

// Variables de estado para la pantalla
bool loraInitialized = false;
unsigned long displayMode = 0; // 0=WiFi, 1=LoRa, 2=MQTT, 3=Mensaje
bool newMessageReceived = false;

// Estructura para compartir datos entre tareas
struct SharedData {
  bool wifiConnected;
  String ipAddress;
  bool mqttConnected;
  bool showNewMessage;
  String messageSensor;
  String messageData;
  int messageCounter;
  SemaphoreHandle_t mutex;
};

SharedData sharedData;

// -------------------- FUNCIONES DE LA PANTALLA OLED --------------------

void setupOLED() {
  // Configurar el pin de reset de la OLED
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  // Inicializar los pines I2C
  Wire.begin(OLED_SDA, OLED_SCL);
  
  // Inicializar la pantalla OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Error al inicializar la pantalla SSD1306"));
    return;
  }
  
  // Limpiar el buffer y configurar propiedades iniciales
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  
  // Mostrar mensaje de bienvenida
  display.println("Estacion Terrena");
  display.println(idManualLanzamiento);
  display.println("Iniciando sistemas...");
  display.println("Dual Core Mode");
  display.display();
  delay(2000);
}

void clearDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
}

void displayWiFiStatus(bool connected, String ip = "") {
  clearDisplay();
  display.println("Estado WiFi:");
  if (connected) {
    display.println("* Conectado");
    display.print("IP: ");
    display.println(ip);
  } else {
    display.println("* Desconectado");
    display.println("Intentando reconexion...");
  }
  display.display();
}

void displayLoRaStatus(bool initialized) {
  clearDisplay();
  display.println("Estado LoRa:");
  if (initialized) {
    display.println("* Inicializado OK");
    display.println("Banda: 915 MHz");
    display.println("SF: 12  BW: 125kHz");
  } else {
    display.println("* Error de inicio");
    display.println("Revisar conexiones");
  }
  display.display();
}

void displayMQTTStatus(bool connected) {
  clearDisplay();
  display.println("Estado MQTT:");
  if (connected) {
    display.println("* Conectado");
    display.println("Servidor: " + String(mqttServer));
    display.println("ID: " + String(mqttID));
  } else {
    display.println("* Desconectado");
    display.println("Intentando reconexion...");
  }
  display.display();
}

void displayReceivedMessage(String sensor, String data) {
  clearDisplay();
  display.println("Mensaje recibido:");
  display.print("Sensor: ");
  display.println(sensor);
  
  // Mostrar los primeros 40 caracteres de datos
  String dataShort = data;
  if (dataShort.length() > 40) {
    dataShort = dataShort.substring(0, 37) + "...";
  }
  display.println(dataShort);
  
  display.display();
}

void displayText(String text, int x = 0, int y = 0, int size = 1) {
  display.setTextSize(size);
  display.setCursor(x, y);
  display.println(text);
  display.setTextSize(1); // Restaurar tamaño por defecto
}

void displayStats(int messageCount) {
  clearDisplay();
  displayText("Estadisticas:", 0, 0);
  displayText("Msgs recibidos: " + String(messageCount), 0, 16);
  displayText("ID: " + String(idManualLanzamiento), 0, 32);
  displayText("Core 0: OLED", 0, 48);
  display.display();
}

// -------------------- FUNCIONES PRINCIPALES --------------------

// Función para verificar y reconectar WiFi
boolean checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastWifiRetry >= WIFI_RETRY_DELAY) {
      Serial.println("WiFi desconectado. Intentando reconexión...");
      
      // Actualizar el estado compartido
      xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
      sharedData.wifiConnected = false;
      xSemaphoreGive(sharedData.mutex);
      
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      lastWifiRetry = currentMillis;
    }
    return false;
  }
  
  // Actualizar el estado compartido
  xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
  if (!sharedData.wifiConnected) {
    sharedData.wifiConnected = true;
    sharedData.ipAddress = WiFi.localIP().toString();
  }
  xSemaphoreGive(sharedData.mutex);
  
  return true;
}

// Función para reconectar MQTT
boolean reconnectMQTT() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMqttRetry >= MQTT_RETRY_DELAY) {
    Serial.println("Intentando conexión MQTT...");
    
    // Actualizar el estado compartido
    xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
    sharedData.mqttConnected = false;
    xSemaphoreGive(sharedData.mutex);
    
    if (client.connect(mqttID, mqttUser, mqttPassword)) {
      Serial.println("Conectado a MQTT");
      
      // Actualizar el estado compartido
      xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
      sharedData.mqttConnected = true;
      xSemaphoreGive(sharedData.mutex);
      
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

void checkConnections() {
  if (!checkWiFiConnection()) {
    return; // Si no hay WiFi, no intentamos MQTT
  }
  
  if (!client.connected()) {
    reconnectMQTT();
  }
}

void setupLoRa() {
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  // Configurar el factor de propagación (SF)
  LoRa.setSpreadingFactor(12);
  // Configurar el ancho de banda (Bandwidth)
  LoRa.setSignalBandwidth(125E3);

  if (!LoRa.begin(BAND)) {
    Serial.println("Error iniciando LoRa");
    loraInitialized = false;
  } else {
    Serial.println("Inicio exitoso de LoRa!");
    loraInitialized = true;
  }
  
  // Actualizar el estado compartido
  xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
  sharedData.showNewMessage = false;
  xSemaphoreGive(sharedData.mutex);
}

Mensaje leerMensaje() {
  String incoming = "";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  Mensaje m;
  int separatorIndex = incoming.indexOf(':');
  
  if (separatorIndex != -1) {
    m.sensor = incoming.substring(0, separatorIndex);
    String datoLoRa = incoming.substring(separatorIndex + 1);
    m.datoLoRa = datoLoRa + "," + idManualLanzamiento;
  } else {
    m.sensor = "ERROR";
    m.datoLoRa = "formato_invalido";
  }
  
  return m;
}

void enviarDatosMQTT(String variable, String dato) {
  if (!client.connected()) {
    Serial.println("No se puede publicar: MQTT desconectado");
    return;
  }

  char msg[MQTT_MAX_PACKET_SIZE];
  char topic[MQTT_TOPIC_SIZE];
  
  // Verificar el tamaño del mensaje
  if (dato.length() >= MQTT_MAX_PACKET_SIZE) {
    Serial.println("Error: Mensaje demasiado largo");
    return;
  }

  dato.toCharArray(msg, MQTT_MAX_PACKET_SIZE);

  const char* topico;
  if (variable == "G") {
    topico = "valuesGPS";
  } else if (variable == "M") {
    topico = "valuesMPL";
  } else if (variable == "A") {
    topico = "valuesBMM";
  } else if (variable == "U") {
    topico = "valuesMPU";
  } else {
    topico = "unknownTopic";
  }

  Serial.print("Publicando en ");
  Serial.print(topico);
  Serial.print(" -> ");
  Serial.println(msg);

  // Intentar publicar y verificar el resultado
  if (client.publish(topico, msg)) {
    //Serial.println("Publicación exitosa");
  } else {
    Serial.println("Error en la publicación");
  }

  client.loop();
}

// -------------------- TAREAS FREERTOS --------------------

// Tarea que se ejecuta en el Core 0 (OLED)
void TaskOLED(void *pvParameters) {
  // Inicializar pantalla OLED
  setupOLED();

  // Mostrar mensaje inicial en la pantalla
  displayText("Core 0: OLED OK", 0, 40);
  display.display();
  delay(2000);
  
  unsigned long lastUpdate = 0;
  int currentMode = 0;
  
  for (;;) {
    bool showMessage = false;
    bool wifiStatus = false;
    bool mqttStatus = false;
    String ip = "";
    String sensorMsg = "";
    String dataMsg = "";
    int msgCount = 0;
    
    // Obtener datos actualizados del mutex
    xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
    showMessage = sharedData.showNewMessage;
    wifiStatus = sharedData.wifiConnected;
    mqttStatus = sharedData.mqttConnected;
    ip = sharedData.ipAddress;
    sensorMsg = sharedData.messageSensor;
    dataMsg = sharedData.messageData;
    msgCount = sharedData.messageCounter;
    xSemaphoreGive(sharedData.mutex);
    
    // Si hay un nuevo mensaje, mostrarlo
    if (showMessage) {
      displayReceivedMessage(sensorMsg, dataMsg);
      
      // Resetear el flag después de mostrar el mensaje
      xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
      sharedData.showNewMessage = false;
      xSemaphoreGive(sharedData.mutex);
      
      lastUpdate = millis();
    }
    // Si no hay mensaje nuevo, rotar la información
    else if (millis() - lastUpdate >= DISPLAY_UPDATE_INTERVAL) {
      switch (currentMode) {
        case 0:
          displayWiFiStatus(wifiStatus, ip);
          break;
        case 1:
          displayLoRaStatus(loraInitialized);
          break;
        case 2:
          displayMQTTStatus(mqttStatus);
          break;
        case 3:
          displayStats(msgCount);
          break;
      }
      
      currentMode = (currentMode + 1) % 4;
      lastUpdate = millis();
    }
    
    // Dar tiempo a otras tareas
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Tarea que se ejecuta en el Core 1 (Principal)
void TaskMain(void *pvParameters) {
  // Configurar LoRa
  setupLoRa();

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
    
    xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
    sharedData.wifiConnected = true;
    sharedData.ipAddress = WiFi.localIP().toString();
    xSemaphoreGive(sharedData.mutex);
  } else {
    Serial.println("\nFalló conexión WiFi inicial - continuando...");
    
    xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
    sharedData.wifiConnected = false;
    xSemaphoreGive(sharedData.mutex);
  }

  // Configurar MQTT
  client.setKeepAlive(mqttKeepalive);
  client.setServer(mqttServer, mqttPort);
  
  if (WiFi.status() == WL_CONNECTED) {
    if (reconnectMQTT()) {
      xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
      sharedData.mqttConnected = true;
      xSemaphoreGive(sharedData.mutex);
    } else {
      Serial.println("Fallo en conexión inicial MQTT - continuando...");
      
      xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
      sharedData.mqttConnected = false;
      xSemaphoreGive(sharedData.mutex);
    }
  }

  for (;;) {
    // Verificar conexiones
    checkConnections();
    
    // Recepción de paquetes
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      m = leerMensaje();
      String sensorI = m.sensor;
      String datoLoRa = m.datoLoRa;
      
      // Incrementar contador de mensajes
      messageCount++;
      
      // Actualizar datos compartidos
      xSemaphoreTake(sharedData.mutex, portMAX_DELAY);
      sharedData.showNewMessage = true;
      sharedData.messageSensor = sensorI;
      sharedData.messageData = datoLoRa;
      sharedData.messageCounter = messageCount;
      xSemaphoreGive(sharedData.mutex);

      // Enviar datos por MQTT si estamos conectados
      if (client.connected()) {
        enviarDatosMQTT(sensorI, datoLoRa);
      } else {
        Serial.println("No se pudo enviar: MQTT desconectado");
      }
    }
    
    client.loop();
    
    // Dar tiempo a otras tareas
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// -------------------- SETUP Y LOOP --------------------

void setup() {
  Serial.begin(115200);
  Serial.println("\nIniciando sistema en modo Dual Core");
  
  // Inicializar mutex para los datos compartidos
  sharedData.mutex = xSemaphoreCreateMutex();
  sharedData.wifiConnected = false;
  sharedData.mqttConnected = false;
  sharedData.showNewMessage = false;
  sharedData.messageCounter = 0;
  
  // Crear tareas para cada núcleo
  xTaskCreatePinnedToCore(
    TaskOLED,    // Función que implementa la tarea
    "TaskOLED",  // Nombre de la tarea
    STACK_SIZE,  // Tamaño del stack
    NULL,        // Parámetros
    PRIORITY_0,  // Prioridad
    NULL,        // Handle de la tarea
    CORE_0);     // Núcleo donde se ejecuta
    
  xTaskCreatePinnedToCore(
    TaskMain,    // Función que implementa la tarea
    "TaskMain",  // Nombre de la tarea
    STACK_SIZE,  // Tamaño del stack
    NULL,        // Parámetros
    PRIORITY_1,  // Prioridad
    NULL,        // Handle de la tarea
    CORE_1);     // Núcleo donde se ejecuta
  
  // Con FreeRTOS no necesitamos más código en setup
}

void loop() {
  // Nada que hacer aquí, todo se maneja en las tareas
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}