#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>

#define PINLED 13
#define PINPIR 12
#define NUMPIXELS 51

const char* ssid = "MischisIndanetz";
const char* password = "123456789";

unsigned long tActivated;
bool active;


typedef struct 
{
  float r;
  float g;
  float b;
  float w;
} rgbw;

Adafruit_NeoPixel neoPixels = Adafruit_NeoPixel(NUMPIXELS, PINLED, NEO_BGRW + NEO_KHZ800);

int gammaLUT[256];

rgbw rgbwBuffer[NUMPIXELS] = {0.0f};

void startOTA();
void updatePixels(rgbw pxls[NUMPIXELS]);


void setup() {
  Serial.begin(115200);
  Serial.println("Booting");
  startOTA();
  neoPixels.begin();

  pinMode(PINPIR, INPUT);

  // Create gamma LUT
  float gamma = 2.0f;
  for (int n=0; n < 256; n++)
  {
    gammaLUT[n] = (int) (powf(((float)n/255.0f), gamma) * 255);
  }
}

void loop() {
  ArduinoOTA.handle();
  int pirState = digitalRead(PINPIR);
  
  unsigned long time = millis();
  unsigned long activeForMs = 60 * 1000; // 1minute

  if (pirState)
  {
    tActivated = millis();
  }

  active = ((millis() - tActivated) < activeForMs);


  for (int n = 0; n < NUMPIXELS; n++)
  {
    rgbwBuffer[n].r = 0.0f;
    rgbwBuffer[n].g = 0.0f;
    rgbwBuffer[n].b = 0.0f;
    rgbwBuffer[n].w = (float)active;
  }

  updatePixels(rgbwBuffer);
}


void updatePixels(rgbw pxls[NUMPIXELS]) 
{
  for (int p = 0; p < NUMPIXELS; p++)
  {
    // Clip Values to 0-1 and apply Gamma LUT Map
     rgbw out = pxls[p];
     out.r = min(max(0.0f, out.r), 1.0f);
     out.g = min(max(0.0f, out.g), 1.0f);
     out.b = min(max(0.0f, out.b), 1.0f);
     int8_t r = gammaLUT[(int)(out.r * 255)];
     int8_t g = gammaLUT[(int)(out.g * 255)];
     int8_t b = gammaLUT[(int)(out.b * 255)];
     int8_t w = gammaLUT[(int)(out.w * 255)];
    // Update LEDs
    neoPixels.setPixelColor(p, r, g, b, w);
  }
  neoPixels.show();
}


void startOTA()
{
 WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}