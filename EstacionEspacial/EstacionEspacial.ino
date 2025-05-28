// Optimized version of the Space Station code
// Consolidated libraries and definitions
#include <LoRa.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPL3115A2.h>
#include "DFRobot_BMM150.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Pin definitions
struct Pins {
  static const uint8_t GPS_RX = 6;
  static const uint8_t GPS_TX = 5;
  static const uint8_t LORA_SCK = 13;
  static const uint8_t LORA_MISO = 12;
  static const uint8_t LORA_MOSI = 11;
  static const uint8_t LORA_SS = 10;
  static const uint8_t LORA_RST = 9;
  static const uint8_t LORA_DIO0 = 2;
  static const uint8_t LED = 13;
};

// Configuration constants
const uint8_t SLAVE_ADDRESS = 0x08;
const float BAND = 915E6;
const uint32_t GPS_BAUD = 9600;
const float DECLINACION_MAGNETICA = -11.0;
const uint16_t TRANSMISSION_INTERVAL = 1000;  // 1 second
const float G_LSB = 0.00006103515625;

// Sensor objects
SoftwareSerial gpsSerial(Pins::GPS_RX, Pins::GPS_TX);
TinyGPSPlus gps;
Adafruit_MPL3115A2 baro;
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);
MPU6050 mpu;

// Global variables with better scope management
struct SensorData {
  struct {
    double lat = 0;
    double lon = 0;
  } gps;

  struct {
    float altitude = 0;
    float pressure = 0;
    float temperature = 0;
  } mpl;

  struct {
    float compass = 0;
    float trueNorth = 0;
  } bmm;

  struct {
    double yaw = 0;
    double pitch = 0;
    double roll = 0;
    double accelX = 0;
    double accelY = 0;
    double accelZ = 0;
  } mpu;
} sensorData;

// MPU DMP variables
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

class SpaceStation {
public:
  void setup() {
    initializeCommunication();
    initializeSensors();
  }

  void loop() {
    static unsigned long lastSendTime = 0;

    readSensors();

    if (millis() - lastSendTime >= TRANSMISSION_INTERVAL) {
      transmitData();
      lastSendTime = millis();
    }
  }

private:
  void initializeCommunication() {
    Serial.begin(115200);
    gpsSerial.begin(GPS_BAUD);

    // Initialize LoRa
    LoRa.setPins(Pins::LORA_SS, Pins::LORA_RST, Pins::LORA_DIO0);
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setTxPower(20);

    if (!LoRa.begin(BAND)) {
      Serial.println(F("LoRa initialization failed"));
      while (1)
        ;
    }
  }

  void initializeSensors() {
    // Initialize Wire
    Wire.begin();
    Wire.setClock(400000);

    // Initialize MPL3115A2
    if (!baro.begin()) {
      Serial.println(F("MPL3115A2 initialization failed"));
      while (1)
        ;
    }

    // Initialize BMM150
    while (bmm150.begin()) {
      Serial.println(F("BMM150 initialization failed"));
      delay(1000);
    }
    configureBMM150();

    // Initialize MPU6050
    initializeMPU();
  }

  void configureBMM150() {
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_10HZ);
    bmm150.setMeasurementXYZ();
  }

  void initializeMPU() {
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    if (devStatus == 0) {
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.setDMPEnabled(true);

      attachInterrupt(digitalPinToInterrupt(Pins::LORA_DIO0), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
  }

  void readSensors() {
    readGPS();
    readMPL();  //baro
    readBMM();
    readMPU();
  }

  void readGPS() {
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid()) {
          sensorData.gps.lat = gps.location.lat();
          sensorData.gps.lon = gps.location.lng();
        }
      }
    }
  }

  void readMPL() {
    sensorData.mpl.altitude = baro.getAltitude();
    sensorData.mpl.pressure = baro.getPressure();
    sensorData.mpl.temperature = baro.getTemperature();
  }

  void readBMM() {
    float compassDegree = bmm150.getCompassDegree();
    sensorData.bmm.trueNorth = normalizeAngle(compassDegree + DECLINACION_MAGNETICA);
  }

  float normalizeAngle(float angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
  }

  void readMPU() {
    if (!dmpReady) return;

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sensorData.mpu.accelX = ax * G_LSB;
    sensorData.mpu.accelY = ay * G_LSB;
    sensorData.mpu.accelZ = az * G_LSB;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      sensorData.mpu.yaw = ypr[0] * 180 / M_PI;
      sensorData.mpu.pitch = ypr[1] * 180 / M_PI;
      sensorData.mpu.roll = ypr[2] * 180 / M_PI;
    }
  }

  void transmitData() {
    String dataPackets[] = {
      formatGPSData(),
      formatMPLData(),
      formatBMMData(),
      formatMPUData()
    };

    for (const String& packet : dataPackets) {
      transmitPacket(packet);
    }
  }

  String formatGPSData() {
    return String("G:") + String(sensorData.gps.lat, 6) + "," + String(sensorData.gps.lon, 6);
  }

  String formatMPLData() {
    return String("M:") + String(sensorData.mpl.altitude) + "," + String(sensorData.mpl.pressure) + "," + String(sensorData.mpl.temperature);
  }

  String formatBMMData() {
    return String("A:") + String(sensorData.bmm.trueNorth);
  }

  String formatMPUData() {
    return String("U:") + String(sensorData.mpu.accelX) + "," + String(sensorData.mpu.accelY) + "," + String(sensorData.mpu.accelZ) + "," + String(sensorData.mpu.yaw) + "," + String(sensorData.mpu.pitch) + "," + String(sensorData.mpu.roll);
  }

  void transmitPacket(const String& packet) {
    Serial.println("Sending: " + packet);

    LoRa.beginPacket();
    LoRa.print(packet);
    LoRa.endPacket();

    sendToI2CSlave(packet);
  }

  void sendToI2CSlave(const String& data) {
    const int BUFFER_SIZE = 30;  // Usar 30 para dejar espacio para overhead
    int length = data.length();
    int sent = 0;

    while (sent < length) {
      Wire.beginTransmission(SLAVE_ADDRESS);
      for (int i = 0; i < BUFFER_SIZE && (sent + i) < length; i++) {
        Wire.write(data[sent + i]);
      }
      Wire.endTransmission();
      sent += BUFFER_SIZE;
      delay(5);  // Pequeña pausa entre transmisiones
    }

    // Enviar marca de fin de mensaje
    Wire.beginTransmission(SLAVE_ADDRESS);
    Wire.write('\n');
    Wire.endTransmission();
    delay(5);
  }
};

SpaceStation spaceStation;

void setup() {
  spaceStation.setup();
}

void loop() {
  spaceStation.loop();
}