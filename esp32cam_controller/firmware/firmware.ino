/****************************************************************************************************************************************************
    TITLE: HOW TO BUILD A $9 RSTP VIDEO STREAMER: Using The ESP-32 CAM Board || Arduino IDE - DIY #14
    DESCRIPTION: This sketch creates a video streamer than uses RTSP. You can configure it to either connect to an existing WiFi network or to create
    a new access point that you can connect to, in order to stream the video feed.

    By Frenoy Osburn
    YouTube Video: https://youtu.be/1xZ-0UGiUsY
    BnBe Post: https://www.bitsnblobs.com/rtsp-video-streamer---esp32
 ****************************************************************************************************************************************************/

/********************************************************************************************************************
   Board Settings:
   Board: "ESP32 AI Thinker Module"
*********************************************************************************************************************/

#include "src/OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

#include "src/SimStreamer.h"
#include "src/OV2640Streamer.h"
#include "src/CRtspSession.h"

#include "wifikeys.h"

WebServer server(80);

// Set Static IP address (WIFI connection to host)
//IPAddress local_IP(192, 168, 1, 160);
#include "auto_ip.h"
// Set Gateway IP address (WIFI connection to host)
IPAddress gateway(192, 168, 1, 1);

// Define camera model and Pin Out
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Camera Flash light and Status LED
#define FLASH_LED_PIN 4
#define FLASH_LED_CHANNEL 4
#define FLASH_LED_BRIGHTNESS 20
#define WIFI_LED_PIN 33
OV2640 cam;


void setup()
{
  Serial.begin(115200);
  //while (!Serial);            //wait for serial connection.

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 16000000; //Reduced from 20000000
  config.pixel_format = PIXFORMAT_JPEG;
  //  config.frame_size = FRAMESIZE_SVGA; // 800x600
  //  config.frame_size = FRAMESIZE_SXGA; // 1280 x 1024
  config.frame_size = FRAMESIZE_UXGA; //1600 x 1200
  config.jpeg_quality = 12;
  config.fb_count = 2;

  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, HIGH); //LED Off

  cam.init(config);

  ledcSetup(FLASH_LED_CHANNEL, 5000, 8);
  ledcAttachPin(FLASH_LED_PIN, FLASH_LED_CHANNEL);
  ledcWrite(FLASH_LED_CHANNEL, 0); // FLASH LED Off

  WiFi.mode(WIFI_STA);

  IPAddress local_IP = ip_address();
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);
  IPAddress secondaryDNS(8, 8, 4, 4);
  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Wifi Static IP Failed to configure");
  }
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  IPAddress ip;
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.print("Local IP : ");
  Serial.println(ip);
  Serial.print("Stream Link: rtsp://");
  Serial.print(ip);
  Serial.println(":8554/mjpeg/1");

  server.on("/", HTTP_GET, handle_jpg_stream);
  server.on("/jpg", HTTP_GET, handle_jpg);
  server.onNotFound(handleNotFound);
  server.begin();

}

CStreamer *streamer;
CRtspSession *session;
WiFiClient client; // FIXME, support multiple clients

void loop()
{
  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("WiFi lost"));
    
    digitalWrite(WIFI_LED_PIN, HIGH); //LED Off
    // Attempt to Reconnect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(F("."));
    }
    IPAddress ip;
    ip = WiFi.localIP();
    
    Serial.println(F("WiFi connected"));
    Serial.print("Local IP : ");
    Serial.println(ip);
  } else {
    digitalWrite(WIFI_LED_PIN, LOW); //LED On
  }

}
