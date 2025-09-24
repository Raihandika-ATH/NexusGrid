#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <PZEM004Tv30.h>

const char* WIFI_SSID = "SSID";
const char* WIFI_PASS = "PASSWORD";
const char* MQTT_SERVER = "broker.hivemq.com";
const int   MQTT_PORT = 1883;
const char* API_HOST = "nexusgrid.rayakademi.my.id";
const String API_PATH = "/api/log_data.php";

const char* TOPIC_POWER_TOTAL = "nexusgrid/data/power_total";
const char* TOPIC_ENERGY_TOTAL = "nexusgrid/data/energy_total";
const char* TOPIC_COST_ESTIMATE = "nexusgrid/data/cost_estimate";
const char* TOPIC_NOTIFICATIONS = "nexusgrid/notifications";
const char* TOPIC_SUBSCRIBE_DEVICE_CMD = "nexusgrid/device/+/command";
const char* TOPIC_SUBSCRIBE_DEVICE_MODE = "nexusgrid/device/+/mode";
const char* TOPIC_SUBSCRIBE_SYSTEM_CMD = "nexusgrid/system/command";

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define PIR_PIN     19

#define RELAY_ON    LOW
#define RELAY_OFF   HIGH

#define LOBBY_LAMP_ID 1
#define LOBBY_LAMP_INDEX 0 // Index dalam array (ID - 1)

PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);

const int NUM_DEVICES = 7;
const int RELAY_PINS[NUM_DEVICES] = {2, 4, 5, 18, 23, 13, 12};

struct Device {
    int id;
    bool state;
    String mode;
};

Device devices[NUM_DEVICES];

float totalPowerActual = 0.0;
float totalEnergyActual = 0.0;

unsigned long lastMotionTime = 0;
const long MOTION_TIMEOUT = 15000;

bool highUsageNotifSent = false;
const float HIGH_USAGE_THRESHOLD_KWH = 0.005;


WiFiClient espClient;
PubSubClient mqttClient(espClient);
unsigned long lastPublishMillis = 0;
const long PUBLISH_INTERVAL = 5000;

void connectWiFi();
void connectMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleDeviceCommand(int deviceId, const String& message);
void handleModeCommand(int deviceId, const String& message);
void handleSystemCommand(const String& message);
void handlePirSensor();
void readAndPublishPzemData();
void publishSingleData(int deviceId, const char* metric, const char* value);
void publishNotification(const char* level, const char* message);
void checkHighUsage();
void logDataToDatabase();

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < NUM_DEVICES; i++) {
        pinMode(RELAY_PINS[i], OUTPUT);
        digitalWrite(RELAY_PINS[i], RELAY_OFF);
    }

    pinMode(PIR_PIN, INPUT);

    for (int i = 0; i < NUM_DEVICES; i++) {
        devices[i] = {i + 1, false, "manual"};
    }
    
    connectWiFi();
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (!mqttClient.connected()) connectMqtt();
    mqttClient.loop();

    handlePirSensor();

    if (millis() - lastPublishMillis >= PUBLISH_INTERVAL) {
        lastPublishMillis = millis();
        readAndPublishPzemData();
        checkHighUsage();
        logDataToDatabase();
    }
}

void connectWiFi() {
    Serial.print("Menghubungkan ke WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi terhubung!");
    Serial.print("Alamat IP: ");
    Serial.println(WiFi.localIP());
}

void connectMqtt() {
    while (!mqttClient.connected()) {
        Serial.print("Mencoba terhubung ke MQTT Broker...");
        String clientId = "ESP32-NexusGrid-Node-" + String(random(0xffff), HEX);
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("terhubung!");

            mqttClient.subscribe(TOPIC_SUBSCRIBE_DEVICE_CMD);
            mqttClient.subscribe(TOPIC_SUBSCRIBE_DEVICE_MODE);
            mqttClient.subscribe(TOPIC_SUBSCRIBE_SYSTEM_CMD);
        } else {
            Serial.printf("gagal, rc=%d. Coba lagi dalam 5 detik\n", mqttClient.state());
            delay(5000);
        }
    }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    message.reserve(length);
    for (unsigned int i = 0; i < length; i++) { message += (char)payload[i]; }
    String topicStr = String(topic);
    Serial.printf("MQTT [%s]: %s\n", topicStr.c_str(), message.c_str());

    if (topicStr.startsWith("nexusgrid/device/")) {
        int deviceId = topicStr.substring(17, topicStr.indexOf('/', 17)).toInt();
        if (deviceId < 1 || deviceId > NUM_DEVICES) return;

        if (topicStr.endsWith("/command")) {
            handleDeviceCommand(deviceId, message);
        } else if (topicStr.endsWith("/mode")) {
            handleModeCommand(deviceId, message);
        }
    } else if (topicStr.equals(TOPIC_SUBSCRIBE_SYSTEM_CMD)) {
        handleSystemCommand(message);
    }
}

void handleDeviceCommand(int deviceId, const String& message) {
    int deviceIndex = deviceId - 1;
    bool newState = message.equalsIgnoreCase("on");

    if (deviceId == LOBBY_LAMP_ID && devices[deviceIndex].mode == "auto") {
        devices[deviceIndex].mode = "manual";
        publishSingleData(deviceId, "mode", "manual");
        Serial.printf("Lampu Lobi (ID %d) dialihkan ke mode manual.\n", deviceId);
    }

    if (devices[deviceIndex].state != newState) {
        devices[deviceIndex].state = newState;
        digitalWrite(RELAY_PINS[deviceIndex], newState ? RELAY_ON : RELAY_OFF);
        publishSingleData(deviceId, "status", newState ? "on" : "off");

    }
}

void handleModeCommand(int deviceId, const String& message) {

    if (deviceId == LOBBY_LAMP_ID) {
        int deviceIndex = deviceId - 1;

        if (devices[deviceIndex].mode.equals(message)) {
            return;
        }

        devices[deviceIndex].mode = message;
        publishSingleData(deviceId, "mode", message.c_str());
        Serial.printf("Mode Lampu Lobi (ID %d) diubah menjadi: %s\n", deviceId, message.c_str());
    }
}

void handleSystemCommand(const String& message) {
    if (message.equals("reset_energy")) {
        if (pzem.resetEnergy()) {
            totalEnergyActual = 0;
            Serial.println("!!! DATA ENERGI UTAMA BERHASIL DIRESET !!!");
            publishNotification("success", "Data energi pada sensor utama berhasil direset.");
        } else {
            Serial.println("!!! GAGAL MERESET DATA ENERGI UTAMA !!!");
            publishNotification("error", "Gagal mereset data energi pada sensor utama.");
        }
    }
}


void handlePirSensor() {

    if (devices[LOBBY_LAMP_INDEX].mode != "auto") {
        return;
    }

    if (digitalRead(PIR_PIN) == HIGH) {
        lastMotionTime = millis();
    }

    bool motionDetected = (millis() - lastMotionTime < MOTION_TIMEOUT);
    bool shouldBeOn = motionDetected;

    if (devices[LOBBY_LAMP_INDEX].state != shouldBeOn) {
        devices[LOBBY_LAMP_INDEX].state = shouldBeOn;
        digitalWrite(RELAY_PINS[LOBBY_LAMP_INDEX], shouldBeOn ? RELAY_ON : RELAY_OFF);
        publishSingleData(LOBBY_LAMP_ID, "status", shouldBeOn ? "on" : "off");
        Serial.printf("Lampu Lobi (ID %d) mode auto -> %s\n", LOBBY_LAMP_ID, shouldBeOn ? "ON" : "OFF");
    }
}

void readAndPublishPzemData() {
    float power = pzem.power();
    float energy = pzem.energy();

    if (isnan(power) || isnan(energy)) {
        Serial.println("Error: Gagal membaca dari sensor PZEM.");
        publishNotification("error", "Koneksi sensor energi (PZEM-004T) terputus.");
        totalPowerActual = 0.0;

        return;
    }
 
    Serial.printf("Daya Saat Ini: %.2f W, Total Energi: %.4f kWh\n", power, energy);


    totalPowerActual = power;
    totalEnergyActual = energy;


    char buffer[20];
    
    dtostrf(totalPowerActual, 4, 2, buffer);
    mqttClient.publish(TOPIC_POWER_TOTAL, buffer, true);
    
    dtostrf(totalEnergyActual, 6, 4, buffer);
    mqttClient.publish(TOPIC_ENERGY_TOTAL, buffer, true);

    long costEstimate = totalEnergyActual * 1445;
    sprintf(buffer, "%ld", costEstimate);
    mqttClient.publish(TOPIC_COST_ESTIMATE, buffer, true);

    Serial.println("--- Data sensor utama dipublikasikan ---");
}

void checkHighUsage() {
    if (totalEnergyActual > HIGH_USAGE_THRESHOLD_KWH && !highUsageNotifSent) {
        publishNotification("warn", "Penggunaan energi harian telah melebihi ambang batas.");
        highUsageNotifSent = true;
    } else if (totalEnergyActual < HIGH_USAGE_THRESHOLD_KWH) {
        highUsageNotifSent = false;
    }
}


void publishSingleData(int deviceId, const char* metric, const char* value) {
    char topic[50];
    snprintf(topic, sizeof(topic), "nexusgrid/device/%d/%s", deviceId, metric);
    mqttClient.publish(topic, value, true);
}

void publishNotification(const char* level, const char* message) {
    JsonDocument doc;
    doc["level"] = level;
    doc["message"] = message;
    String output;
    serializeJson(doc, output);
    mqttClient.publish(TOPIC_NOTIFICATIONS, output.c_str());
}

void logDataToDatabase() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Tidak ada koneksi WiFi untuk mengirim data ke database.");
        return;
    }

    HTTPClient http;
    String serverUrl = "http://" + String(API_HOST) + API_PATH;
    
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    String httpRequestData = "total_power=" + String(totalPowerActual) + "&total_energy=" + String(totalEnergyActual);
    
    int httpResponseCode = http.POST(httpRequestData);
    
    if (httpResponseCode == 200) {
        Serial.printf("HTTP POST success, response code: %d\n", httpResponseCode);
    } else {
        Serial.printf("HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    
    http.end();
}

