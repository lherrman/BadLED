#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_NeoPixel.h>

#define PINLED 13
#define PINPIR 12
#define PINPOTI1 2
#define PINPOTI2 4
#define NUMPIXELS 51


const char* ssid = "MischisIndanetz";
const char* password = "123456789";

unsigned long tActivated;
bool active;
float brightnessPoti;
float huePoti;


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
void readInput();
void updatePixels(rgbw pxls[NUMPIXELS]);
void smoothFade(bool active);
void clrBuffer();
rgbw sumBrightness();
rgbw mulColor(float fac, rgbw in);
rgbw addColor(rgbw in1, rgbw in2);
rgbw getColor(float r, float g, float b, float w);



void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  startOTA();
  neoPixels.begin();

  // Init Pins
  pinMode(PINPIR, INPUT);
  //pinMode(PINPOTI1, INPUT);
  //pinMode(PINPOTI2, INPUT);

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
  unsigned long activeForMs = 4 * 60  * 1000; 
  
  if (pirState)
  {
    tActivated = millis();
  }

  active = ((unsigned long)(time - tActivated) < activeForMs);

  smoothFade(active);

  updatePixels(rgbwBuffer);
}


void smoothFade(bool active)
{
  rgbw targetColor = getColor(0.8f, 0.45f, 0.1f, 1.0f);
  float targerBrightness =  targetColor.w * NUMPIXELS;
  float progress = sumBrightness().w / targerBrightness;

  float fadeFactorBase = 0.01;
  float fadeFactorProgress = 0.1; // Factor for fading faster at start of Fade

  unsigned long time = millis();
  unsigned long timeOld = 0;
  if (time - timeOld > 10)
  {
    timeOld = time;

    if (active)
    {
      float factor = fadeFactorBase + (fadeFactorProgress * (1.0f - progress));
      for (int i = 1; i < NUMPIXELS-1; i++)
      {
        rgbwBuffer[0] = targetColor;
        rgbwBuffer[i] = addColor(mulColor((1.0f - factor) , rgbwBuffer[i]), mulColor(factor , rgbwBuffer[i - 1]));
      }
    }
    else{
      float factor = fadeFactorBase + (fadeFactorProgress * (progress));
      for (int i = 1; i < NUMPIXELS-1; i++)
      {
        rgbwBuffer[0] = getColor(0.0f, 0.0f, 0.0f, 0.0f);
        rgbwBuffer[NUMPIXELS-1] = getColor(0.0f, 0.0f, 0.0f, 0.0f);
        rgbwBuffer[i] = addColor(mulColor((1.0f - factor) , rgbwBuffer[i]), mulColor(factor , rgbwBuffer[i + 1]));
      }
    }
  }
}


rgbw sumBrightness()
{
  rgbw ret = {0.0f};
  for (int i = 0; i < NUMPIXELS; i++)
  {
    ret.r += rgbwBuffer[i].r;
    ret.g += rgbwBuffer[i].g;
    ret.b += rgbwBuffer[i].b;
    ret.w += rgbwBuffer[i].w;
  }
  return ret;
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
    neoPixels.setPixelColor(p, b, r, g, w);
  }
  neoPixels.show();
}

void readInput()
{
  brightnessPoti = analogRead(PINPOTI1) / 4095.0f;
  huePoti = analogRead(PINPOTI2) / 4095.0f;
}

rgbw mulColor(float fac, rgbw in)
{
  rgbw out;
  out.r = in.r * fac;
  out.g = in.g * fac;
  out.b = in.b * fac;
  out.w = in.w * fac;
  return out;
}

rgbw addColor(rgbw in1, rgbw in2)
{
  rgbw out;
  out.r = in1.r + in2.r;
  out.g = in1.g + in2.g;
  out.b = in1.b + in2.b;
  out.w = in1.w + in2.w;
  return out;
}

rgbw getColor(float r, float g, float b, float w)
{
  rgbw out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.w = w;
  return out;
}

void clrBuffer()
{
  for (int p = 0; p < NUMPIXELS; p++)
  {
    rgbwBuffer[p] = getColor(0.0f, 0.0f, 0.0f, 0.0f);
  }
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

      // Show Progress on Leds
      clrBuffer();
      for (int n = 0; n < (progress / (total / NUMPIXELS)); n++)
      {
        rgbwBuffer[n] = getColor(0.0f, 0.5f, 0.0f, 0.0f); 
      }
      updatePixels(rgbwBuffer);
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