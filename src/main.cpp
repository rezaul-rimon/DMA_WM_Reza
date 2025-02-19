#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>

// Configuration Section
#define Fast_LED false
#define DEBUG_MODE true
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }

#define WORK_PACKAGE "1165"
#define GW_TYPE "00"
#define FIRMWARE_UPDATE_DATE "250208" 
#define DEVICE_SERIAL "0001"
#define DEVICE_ID WORK_PACKAGE GW_TYPE FIRMWARE_UPDATE_DATE DEVICE_SERIAL

#define HB_INTERVAL 5*60*1000
#define DATA_INTERVAL 1*60*1000

// WiFi and MQTT reconnection time config
#define WIFI_ATTEMPT_COUNT 30
#define WIFI_ATTEMPT_DELAY 1000
#define MAX_WIFI_ATTEMPTS 2
#define MQTT_ATTEMPT_COUNT 10
#define MQTT_ATTEMPT_DELAY 5000

const char* ssid = "DMA-IR-Bluster";
const char* password = "dmabd987";
const char* mqtt_server = "broker2.dma-bd.com";
const char* mqtt_user = "broker2";
const char* mqtt_password = "Secret!@#$1234";
const char* mqtt_data_topic = "DMA/GPS/PUB";
const char* mqtt_sub_topic = "DMA/GPS/SUB";
const char* mqtt_hb_topic = "DMA/GPS/HB";

WiFiClient espClient;
PubSubClient client(espClient);

#define A9G_PIN 15
bool wifiResetFlag = false;

HardwareSerial A9G(2);  // UART2: RX=16, TX=17

void reconnectWiFi() {
    if (WiFi.status() != WL_CONNECTED) {
        DEBUG_PRINTLN("Attempting WiFi connection...");
        WiFi.begin(ssid, password);
        int attempt = 0;
        while (WiFi.status() != WL_CONNECTED && attempt < WIFI_ATTEMPT_COUNT) {
            vTaskDelay(pdMS_TO_TICKS(WIFI_ATTEMPT_DELAY));
            attempt++;
            DEBUG_PRINT(".");
        }
        DEBUG_PRINTLN("");
        if (WiFi.status() == WL_CONNECTED) {
            DEBUG_PRINTLN("WiFi Connected!");
        } else {
            DEBUG_PRINTLN("WiFi Connection Failed, Restarting...");
            ESP.restart();
        }
    }
}

void reconnectMQTT() {
    if (!client.connected()) {
        char clientId[24];
        snprintf(clientId, sizeof(clientId), "dma_ss_%04X%04X%04X", random(0xffff), random(0xffff), random(0xffff));

        for (int attempt = 0; attempt < MQTT_ATTEMPT_COUNT; attempt++) {
            DEBUG_PRINTLN("Attempting MQTT connection...");
            if (client.connect(clientId, mqtt_user, mqtt_password)) {
                DEBUG_PRINTLN("MQTT connected");
                client.subscribe(mqtt_sub_topic);
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(MQTT_ATTEMPT_DELAY));
        }
        DEBUG_PRINTLN("Max MQTT attempts exceeded, restarting...");
        ESP.restart();
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    DEBUG_PRINTLN("Message arrived: " + message);
}

void networkTask(void *param) {
    WiFi.mode(WIFI_STA);
    reconnectWiFi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(mqttCallback);

    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!client.connected()) {
                reconnectMQTT();
            }
        } else {
            reconnectWiFi();
        }
        client.loop();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void mainTask(void *param) {
  A9G.begin(115200, SERIAL_8N1, 16, 17);
  DEBUG_PRINTLN("Initializing A9G...");
  vTaskDelay(pdMS_TO_TICKS(3000));
  A9G.println("AT");
  vTaskDelay(pdMS_TO_TICKS(1000));
  A9G.println("AT+GPS=1");
  vTaskDelay(pdMS_TO_TICKS(1000));
  A9G.println("AT+GPSRD=1");

  String gpsData = "";
  int a9g_fail_count = 0;  // Count failures before resetting

  for (;;) {
      // Read GPS Data When Available
      if (A9G.available()) {
          String gpsRawData = A9G.readStringUntil('\n');
          if (gpsRawData.startsWith("+GPSRD:$GNGGA")) {
              gpsData = gpsRawData;
              DEBUG_PRINTLN("Extracted GPS Data: " + gpsData);
          }
          a9g_fail_count = 0;  // Reset failure count on success
      } else {
          a9g_fail_count++;  // Increment failure count
          DEBUG_PRINTLN("A9G Not Responding")
          if (a9g_fail_count > 6) {  // Reset only if failed 12 times (~30s)
              DEBUG_PRINTLN("A9G unresponsive! Restarting...");
              digitalWrite(A9G_PIN, HIGH);
              vTaskDelay(pdMS_TO_TICKS(5000));
              digitalWrite(A9G_PIN, LOW);
              vTaskDelay(pdMS_TO_TICKS(2000));

              A9G.begin(115200, SERIAL_8N1, 16, 17);
              DEBUG_PRINTLN("Reinitializing A9G...");
              vTaskDelay(pdMS_TO_TICKS(3000));

              A9G.println("AT");
              vTaskDelay(pdMS_TO_TICKS(1000));
              if (A9G.available() && A9G.readString().indexOf("OK") != -1) {
                  A9G.println("AT+GPS=1");
                  vTaskDelay(pdMS_TO_TICKS(1000));
                  A9G.println("AT+GPSRD=1");
                  a9g_fail_count = 0;  // Reset failure count after restart
              } else {
                  DEBUG_PRINTLN("A9G failed to respond after restart!");
              }
          }
      }

      // Send Heartbeat Every HB_INTERVAL
      static unsigned long last_hb_send_time = 0;
      if (millis() - last_hb_send_time >= HB_INTERVAL) {
          last_hb_send_time = millis();
          if (client.connected()) {
              char hb_data[50];
              snprintf(hb_data, sizeof(hb_data), "%s,wifi_connected", DEVICE_ID);
              client.publish(mqtt_hb_topic, hb_data);
              DEBUG_PRINTLN("Heartbeat sent to MQTT");
          } else {
              DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
          }
      }

      // Send Data Every DATA_INTERVAL
      static unsigned long last_data_send_time = 0;
      if (millis() - last_data_send_time >= DATA_INTERVAL) {
          last_data_send_time = millis();
          if (client.connected()) {
              char data[100];
              snprintf(data, sizeof(data), "%s,%s", DEVICE_ID, gpsData.c_str());
              client.publish(mqtt_data_topic, data);
              DEBUG_PRINTLN("Data sent to MQTT: " + gpsData);
          } else {
              DEBUG_PRINTLN("Failed to publish Data on MQTT");
          }
      }

      vTaskDelay(pdMS_TO_TICKS(5000));  // Run every 5 seconds
  }
}


void setup() {
    Serial.begin(115200);
    DEBUG_PRINT("Device ID: ");
    DEBUG_PRINTLN(DEVICE_ID);
    pinMode(A9G_PIN, OUTPUT);
    digitalWrite(A9G_PIN, LOW);
    xTaskCreatePinnedToCore(networkTask, "Network Task", 8*1024, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(mainTask, "Main Task", 16*1024, NULL, 1, NULL, 1);
}

void loop() { }
