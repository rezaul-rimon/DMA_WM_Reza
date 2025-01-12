// Start Library Include section //
// ---------------------------- //
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>  // WiFiManager library
#include <PubSubClient.h>
#include <FastLED.h>

// End Library Include section //
// ---------------------------- //


//Configuration Section Start
//-------------------------//

// Debug mode Config
#define DEBUG_MODE true
#define DEBUG_PRINT(x)  if (DEBUG_MODE) { Serial.print(x); }
#define DEBUG_PRINTLN(x) if (DEBUG_MODE) { Serial.println(x); }

// Device Config
#define WORK_PACKAGE "1178"
#define GW_TYPE "00"
#define FIRMWARE_UPDATE_DATE "241121" // Format: yymmdd
#define DEVICE_SERIAL "9999"
#define DEVICE_ID WORK_PACKAGE GW_TYPE FIRMWARE_UPDATE_DATE DEVICE_SERIAL

#define HB_INTERVAL 10*1000
#define DATA_INTERVAL 1*15*1000


// WiFi and MQTT reconnection time config
#define WIFI_ATTEMPT_COUNT 30
#define WIFI_ATTEMPT_DELAY 1000
#define WIFI_WAIT_COUNT 60
#define WIFI_WAIT_DELAY 1000
#define MAX_WIFI_ATTEMPTS 2
#define MQTT_ATTEMPT_COUNT 10
#define MQTT_ATTEMPT_DELAY 5000

// WiFi and MQTT attempt counters
int wifiAttemptCount = WIFI_ATTEMPT_COUNT;
int wifiWaitCount = WIFI_WAIT_COUNT;
int maxWifiAttempts = MAX_WIFI_ATTEMPTS;
int mqttAttemptCount = MQTT_ATTEMPT_COUNT;

// MQTT Server Config
const char* mqtt_server = "broker2.dma-bd.com";
const char* mqtt_user = "broker2";
const char* mqtt_password = "Secret!@#$1234";
const char* mqtt_topic = "DMA/MC/PUB";

// LED Conffig
#define DATA_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];


//End Configuration Section//
//-------------------------//


//Start Making instance Section//
//-----------------------------//

//Wifi and MQTT Instance
WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wm;

//FreeRTOS Task instance
TaskHandle_t networkTaskHandle;
TaskHandle_t mainTaskHandle;
TaskHandle_t wifiResetTaskHandle;


//End Making instance Section//
//-----------------------------//


//Start Variable declaretion Section//
//----------------------------------//

// WiFi Reset Button
#define WIFI_RESET_BUTTON_PIN 35
// #define TXB6_PIN 34 // Pin for ammonia sensor

//WiFi Reset Flag
bool wifiResetFlag = false;


//End Variable declaretion Section//
//--------------------------------//



// Start Function Section //
//-----------------------//

// Function to reconnect to WiFi
void reconnectWiFi() {
  leds[0] = CRGB::Red;
  FastLED.show();

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting WiFi connection...");
      WiFi.begin();  // Use saved credentials
      wifiAttemptCount--;
      DEBUG_PRINTLN("Remaining WiFi attempts: " + String(wifiAttemptCount));
      // vTaskDelay(WIFI_ATTEMPT_DELAY / portTICK_PERIOD_MS);
      vTaskDelay(pdMS_TO_TICKS(WIFI_ATTEMPT_DELAY));
    } else if (wifiWaitCount > 0) {
      wifiWaitCount--;
      DEBUG_PRINTLN("WiFi wait... retrying in a moment");
      DEBUG_PRINTLN("Remaining WiFi wait time: " + String(wifiWaitCount) + " seconds");
      vTaskDelay(pdMS_TO_TICKS(WIFI_WAIT_DELAY));
    } else {
      wifiAttemptCount = WIFI_ATTEMPT_COUNT;
      wifiWaitCount = WIFI_WAIT_COUNT;
      maxWifiAttempts--;
      if (maxWifiAttempts <= 0) {
        DEBUG_PRINTLN("Max WiFi attempt cycles exceeded, restarting...");
        ESP.restart();
      }
    }
  }
}

// Function to reconnect to MQTT with a unique client ID
void reconnectMQTT() {
  if (!client.connected()) {
    leds[0] = CRGB::Yellow;
    FastLED.show();

    char clientId[24];  // 1 byte for "dma_em_" + 8 bytes for random hex + null terminator
    snprintf(clientId, sizeof(clientId), "dma_wm_%04X%04X%04X", random(0xffff), random(0xffff), random(0xffff));

    if (mqttAttemptCount > 0) {
      DEBUG_PRINTLN("Attempting MQTT connection...");
      
      if (client.connect(clientId, mqtt_user, mqtt_password)) {  // Use the unique client ID
        DEBUG_PRINTLN("MQTT connected");
        DEBUG_PRINT("Client_ID: ");
        DEBUG_PRINTLN(clientId);

        leds[0] = CRGB::Black;
        FastLED.show();

        char topic[48];
        snprintf(topic, sizeof(topic), "%s/%s", mqtt_topic, DEVICE_ID);
        client.subscribe(topic);
        
      } else {
        DEBUG_PRINTLN("MQTT connection failed");
        DEBUG_PRINTLN("Remaining MQTT attempts: " + String(mqttAttemptCount));
        mqttAttemptCount--;
        vTaskDelay(pdMS_TO_TICKS(MQTT_ATTEMPT_DELAY));
      }
    } else {
      DEBUG_PRINTLN("Max MQTT attempts exceeded, restarting...");
      ESP.restart();
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Print the topic and message for debugging
  DEBUG_PRINTLN("Message arrived on topic: " + String(topic));
  DEBUG_PRINTLN("Message content: " + message);

  // Check if the message is "get_from_sd_card"
  if (message == "get_data_from_sd_card") {
    DEBUG_PRINTLN("Triggering sendToFtp()...");
   
  }

  // Check if the message is "get_from_sd_card"
  if (message == "clear_sd_card") {
    DEBUG_PRINTLN("Triggering sendToFtp()...");
  }
}

// End Function Section //
//----------------------//


// Start FreeRTOS Task Section //
//----------------------//

// Network Task for WiFi and MQTT
void networkTask(void *param) {
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  for (;;) {
    // Check WiFi connection
    if (WiFi.status() == WL_CONNECTED) {
      // Check and reconnect MQTT if necessary
      if (!client.connected()) {
        reconnectMQTT();
      }
    } else {
      // Reconnect WiFi if disconnected
      reconnectWiFi();
    }

    // Loop MQTT client for processing incoming messages
    client.loop();

    // Delay for 100ms before next cycle
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// Task for WiFi reset and WiFiManager setup
void wifiResetTask(void *param) {
  for (;;) {
    // Check if button is pressed (LOW state)
    if (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
      unsigned long pressStartTime = millis(); // Record the time when the button is pressed
      DEBUG_PRINTLN("Button Pressed....");
      leds[0] = CRGB::Blue;
      FastLED.show();

      // Wait for at least 5 seconds to confirm long press
      while (digitalRead(WIFI_RESET_BUTTON_PIN) == LOW) {
        // Check if button is still pressed after 5 seconds
        if (millis() - pressStartTime >= 5000) {
          DEBUG_PRINTLN("5 seconds holding time reached, starting WiFiManager...");

          // Suspend other tasks to avoid conflict
          vTaskSuspend(networkTaskHandle);
          vTaskSuspend(mainTaskHandle);

          DEBUG_PRINTLN("Starting WiFiManager for new WiFi setup...");
          leds[0] = CRGB::Green;
          FastLED.show();

          wm.resetSettings();  // Clear previous settings
          wm.autoConnect("DMA_MC_Setup"); // Start AP for new configuration

          DEBUG_PRINTLN("New WiFi credentials set, Restarting...");
          ESP.restart();  // Restart after WiFi configuration
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to avoid overwhelming the system
      }
    }
    else{
      leds[0] = CRGB::Black;
      FastLED.show();
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100 ms
  }
}


/*********************************************************************/
/*                                  main                             */
/*********************************************************************/

void mainTask(void *param) {
  for (;;) {

    // Get the current time's epoch
    static unsigned long last_hb_send_time = 0;
    if (millis() - last_hb_send_time >= HB_INTERVAL) {
      last_hb_send_time = millis();

        if (client.connected()) {
            char hb_data[50];  // Buffer for the heartbeat data

            // Format the heartbeat message into the buffer
            snprintf(hb_data, sizeof(hb_data), "%s,wifi_connected", DEVICE_ID);

            // Publish the heartbeat message
            client.publish(mqtt_topic, hb_data);
            DEBUG_PRINTLN("Heartbeat published data to mqtt");
            leds[0] = CRGB::Blue;
            FastLED.show();
            vTaskDelay(pdMS_TO_TICKS(1000));
            leds[0] = CRGB::Black;
            FastLED.show();
        } else {
            DEBUG_PRINTLN("Failed to publish Heartbeat on MQTT");
        }
    }

    static unsigned long last_data_send_time = 0;
    if (millis() - last_data_send_time >= DATA_INTERVAL) {
      last_data_send_time = millis();
      
      DEBUG_PRINTLN("Data send");

      leds[0] = CRGB::Green;
      FastLED.show();
      vTaskDelay(pdMS_TO_TICKS(1000));
      leds[0] = CRGB::Black;
      FastLED.show();
    
    }
    DEBUG_PRINTLN("Hello");
    vTaskDelay(pdMS_TO_TICKS(1000));  // Print "Hello" every second
  }
}


void setup() {

  Serial.begin(115200);

  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  delay(1000);

  

  // LED setup
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();

  // timeClient.begin();

  // Button setup
  pinMode(WIFI_RESET_BUTTON_PIN, INPUT_PULLUP);

  // Set up MQTT client
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  // Create tasks
  xTaskCreatePinnedToCore(networkTask, "Network Task", 8*1024, NULL, 1, &networkTaskHandle, 0);
  xTaskCreatePinnedToCore(mainTask, "Main Task", 16*1024, NULL, 1, &mainTaskHandle, 1);
  xTaskCreatePinnedToCore(wifiResetTask, "WiFi Reset Task", 4*1024, NULL, 1, &wifiResetTaskHandle, 1);
}

// Loop function
void loop() {
}




