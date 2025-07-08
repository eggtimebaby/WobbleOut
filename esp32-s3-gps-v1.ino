#include <TinyGPSPlus.h>
#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include <Wire.h>
#include "HWCDC.h"

// USB serial
HWCDC USBSerial;

// GPS setup
TinyGPSPlus gps;
#define GPS_RX 44
#define GPS_TX 43
#define GPS_BAUD 9600

// Display setup
Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 3 /* rotate 90° CCW */, true, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

unsigned long lastGPSCheck = 0;
bool lastHadFix = false;
float lastSpeed = -1;

void setup() {
  USBSerial.begin(115200);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  USBSerial.println("Starting GPS Speedometer");

  if (!gfx->begin()) {
    USBSerial.println("gfx->begin() failed!");
  }

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);

  gfx->fillScreen(BLACK);
  gfx->setTextColor(YELLOW);
  gfx->setTextSize(3);
  drawConnectingScreen();
}

void loop() {
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      updateDisplay();
    }
  }

  if (millis() - lastGPSCheck > 5000 && gps.charsProcessed() < 10) {
    drawNoGPS();
    while (true);  // Stop execution
  }
}

void updateDisplay() {
  if (gps.location.isValid() && gps.speed.isValid()) {
    float currentSpeed = gps.speed.mph();
    currentSpeed = roundf(currentSpeed * 10) / 10.0f;  // Round to 1 decimal

    if (!lastHadFix || currentSpeed != lastSpeed) {
      gfx->fillScreen(BLACK);
      gfx->setTextColor(GREEN);
      gfx->setTextSize(8);  // Large text

      char speedStr[10];
      dtostrf(currentSpeed, 4, 1, speedStr);  // 4 digits, 1 decimal

      // Remove any leading space added by dtostrf
      for (int i = 0; speedStr[i]; ++i) {
        speedStr[i] = toupper(speedStr[i]);  // Force all caps just in case
      }

      centerText(speedStr, gfx->height() / 2 - 40);

      lastSpeed = currentSpeed;
      lastHadFix = true;
    }
  } else {
    if (lastHadFix) {
      drawConnectingScreen();
      lastHadFix = false;
    }
  }
}

void drawConnectingScreen() {
  gfx->fillScreen(BLACK);
  gfx->setTextColor(YELLOW);
  gfx->setTextSize(3);
  centerText("CONNECTING", 60);
  centerText("TO", 110);
  centerText("SATELLITES", 160);
}

void drawNoGPS() {
  gfx->fillScreen(BLACK);
  gfx->setTextColor(RED);
  gfx->setTextSize(3);
  centerText("NO GPS", 80);
  centerText("DETECTED", 120);
  centerText("CHECK WIRING", 160);
}

void centerText(const char *text, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  gfx->getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int16_t x = (gfx->width() - w) / 2;
  gfx->setCursor(x, y);
  gfx->println(text);
}