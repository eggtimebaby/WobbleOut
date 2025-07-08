#include <TinyGPSPlus.h>
#include <Arduino.h>
#include "Arduino_GFX_Library.h"
#include <Wire.h>
#include "HWCDC.h"
#include "SensorQMI8658.hpp"

// Pin definitions
#define LCD_DC 4
#define LCD_CS 5
#define LCD_SCK 6
#define LCD_MOSI 7
#define LCD_RST 8
#define LCD_BL 15
#define LCD_WIDTH 240
#define LCD_HEIGHT 280

// I2C pins for IMU
#define IIC_SDA 11
#define IIC_SCL 10

// Button pin - using BOOT button
#define PWR_BUTTON 0

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

// Speed filtering variables
const float SPEED_THRESHOLD = 3.0;  // MPH - speeds below this are considered stationary
const float SPEED_HYSTERESIS = 1.0; // MPH - prevents rapid on/off switching
bool isMoving = false;

// GPS tracking variables
unsigned long lastGPSCheck = 0;
bool lastHadFix = false;
float lastSpeed = -1;

// IMU variables
SensorQMI8658 qmi;
bool imuAvailable = false;
IMUdata acc;
float accelX, accelY, accelZ;
unsigned long lastIMURead = 0;

// Button handling variables
unsigned long lastButtonPress = 0;
unsigned long buttonPressStart = 0;
bool buttonPressed = false;
int buttonPressCount = 0;
unsigned long lastButtonRelease = 0;
const unsigned long DOUBLE_PRESS_TIMEOUT = 500;  // 500ms for double press
const unsigned long LONG_PRESS_TIMEOUT = 2000;   // 2 seconds for long press
const unsigned long DEBOUNCE_DELAY = 50;         // 50ms debounce

// Trip computer variables
bool tripActive = false;
unsigned long tripStartTime = 0;
float tripMaxSpeed = 0;
float tripDistance = 0;
float lastLat = 0, lastLon = 0;
bool hasLastPosition = false;

// Display modes
enum DisplayMode {
  MODE_SPEED,
  MODE_TRIP_DATA,
  MODE_SETTINGS
};
DisplayMode currentMode = MODE_SPEED;
unsigned long modeStartTime = 0;
const unsigned long TRIP_DATA_DISPLAY_TIME = 5000;  // Show trip data for 5 seconds

void setup() {
  USBSerial.begin(115200);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  USBSerial.println("Starting GPS Speedometer with Speed Threshold Filter");
  USBSerial.printf("Speed threshold: %.1f MPH, Hysteresis: %.1f MPH\n", SPEED_THRESHOLD, SPEED_HYSTERESIS);

  // Initialize button
  pinMode(PWR_BUTTON, INPUT_PULLUP);
  
  // Debug: Check initial button state
  delay(100);  // Let pullup settle
  bool initialState = digitalRead(PWR_BUTTON);
  USBSerial.printf("Using GPIO0 (BOOT button). Initial state: %d (1=released, 0=pressed)\n", initialState);

  // Initialize I2C for IMU
  Wire.begin(IIC_SDA, IIC_SCL);
  initIMU();

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
  // Handle button input
  handleButton();

  // Process GPS data
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      updateTripData();
      updateDisplay();
    }
  }

  // Read IMU data periodically
  if (millis() - lastIMURead > 500) {
    readIMU();
    lastIMURead = millis();
  }

  // Auto-return to speed mode after showing trip data
  if (currentMode == MODE_TRIP_DATA && millis() - modeStartTime > TRIP_DATA_DISPLAY_TIME) {
    currentMode = MODE_SPEED;
    updateDisplay();
  }

  // GPS timeout check
  if (millis() - lastGPSCheck > 5000 && gps.charsProcessed() < 10) {
    drawNoGPS();
    while (true);
  }
}

void handleButton() {
  bool rawButtonState = digitalRead(PWR_BUTTON);
  bool buttonState = !rawButtonState;  // Inverted because of pullup
  
  // Debug output every 2 seconds
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 2000) {
    USBSerial.printf("GPIO0 raw: %d, interpreted: %s\n", 
                     rawButtonState, buttonState ? "PRESSED" : "released");
    lastDebug = millis();
  }
  
  // Button press detection with debounce
  if (buttonState && !buttonPressed && millis() - lastButtonRelease > DEBOUNCE_DELAY) {
    buttonPressed = true;
    buttonPressStart = millis();
    USBSerial.println("Button pressed");
  }
  
  // Button release detection
  if (!buttonState && buttonPressed && millis() - buttonPressStart > DEBOUNCE_DELAY) {
    buttonPressed = false;
    lastButtonRelease = millis();
    unsigned long pressDuration = lastButtonRelease - buttonPressStart;
    
    USBSerial.printf("Button released after %lu ms\n", pressDuration);
    
    // Long press detection
    if (pressDuration >= LONG_PRESS_TIMEOUT) {
      handleLongPress();
      buttonPressCount = 0;  // Reset count after long press
    } else {
      // Short press - count for double press detection
      buttonPressCount++;
      lastButtonPress = millis();
    }
  }
  
  // Check for double press timeout
  if (buttonPressCount > 0 && millis() - lastButtonPress > DOUBLE_PRESS_TIMEOUT) {
    if (buttonPressCount == 1) {
      handleSinglePress();
    } else if (buttonPressCount >= 2) {
      handleDoublePress();
    }
    buttonPressCount = 0;
  }
}

void handleSinglePress() {
  USBSerial.println("Single press detected - Start/Stop trip");
  
  if (tripActive) {
    // Stop trip
    tripActive = false;
    USBSerial.println("Trip stopped");
  } else {
    // Start trip
    tripActive = true;
    tripStartTime = millis();
    tripMaxSpeed = 0;
    tripDistance = 0;
    hasLastPosition = false;
    USBSerial.println("Trip started");
  }
  
  // Brief feedback on display
  gfx->fillScreen(BLACK);
  gfx->setTextColor(tripActive ? GREEN : RED);
  gfx->setTextSize(4);
  centerText(tripActive ? "TRIP ON" : "TRIP OFF", gfx->height() / 2);
  delay(1000);
  
  // Return to normal display
  currentMode = MODE_SPEED;
  updateDisplay();
}

void handleDoublePress() {
  USBSerial.println("Double press detected - Show trip data");
  currentMode = MODE_TRIP_DATA;
  modeStartTime = millis();
  drawTripData();
}

void handleLongPress() {
  USBSerial.println("Long press detected - Settings mode");
  currentMode = MODE_SETTINGS;
  drawSettings();
  
  // Stay in settings for 3 seconds, then return to speed mode
  delay(3000);
  currentMode = MODE_SPEED;
  updateDisplay();
}

void updateTripData() {
  if (!tripActive || !gps.location.isValid() || !gps.speed.isValid()) {
    return;
  }
  
  // Apply the same speed filtering to trip data
  float rawSpeed = gps.speed.mph();
  float filteredSpeed = getFilteredSpeed(rawSpeed);
  
  // Update max speed with filtered value
  if (filteredSpeed > tripMaxSpeed) {
    tripMaxSpeed = filteredSpeed;
  }
  
  // Only calculate distance when actually moving (filtered speed > 0)
  if (filteredSpeed > 0 && hasLastPosition) {
    double distanceMeters = gps.distanceBetween(
      lastLat, lastLon, 
      gps.location.lat(), gps.location.lng()
    );
    tripDistance += distanceMeters * 0.000621371;  // Convert to miles
  }
  
  // Always update position for next calculation
  lastLat = gps.location.lat();
  lastLon = gps.location.lng();
  hasLastPosition = true;
}

// Speed filtering function
float getFilteredSpeed(float rawSpeed) {
  float filteredSpeed = 0.0;
  
  // Apply speed threshold with hysteresis
  if (!isMoving) {
    // Currently stationary - need higher speed to start moving
    if (rawSpeed > SPEED_THRESHOLD + SPEED_HYSTERESIS) {
      isMoving = true;
      filteredSpeed = rawSpeed;
      USBSerial.printf("Movement detected: %.1f MPH\n", rawSpeed);
    } else {
      filteredSpeed = 0.0;  // Force to zero when stationary
    }
  } else {
    // Currently moving - need lower speed to stop
    if (rawSpeed < SPEED_THRESHOLD) {
      isMoving = false;
      filteredSpeed = 0.0;
      USBSerial.printf("Stopped: %.1f MPH\n", rawSpeed);
    } else {
      filteredSpeed = rawSpeed;
    }
  }
  
  return filteredSpeed;
}

void updateDisplay() {
  if (currentMode == MODE_SPEED) {
    drawSpeedDisplay();
  }
  // Other modes are drawn when triggered
}

void drawSpeedDisplay() {
  if (gps.location.isValid() && gps.speed.isValid()) {
    float rawSpeed = gps.speed.mph();
    float displaySpeed = getFilteredSpeed(rawSpeed);
    
    // Round to nearest 0.1
    displaySpeed = roundf(displaySpeed * 10) / 10.0f;

    if (!lastHadFix || displaySpeed != lastSpeed) {
      gfx->fillScreen(BLACK);
      gfx->setTextColor(GREEN);
      gfx->setTextSize(8);

      char speedStr[10];
      dtostrf(displaySpeed, 4, 1, speedStr);
      centerText(speedStr, gfx->height() / 2 - 40);

      // Trip indicator
      if (tripActive) {
        gfx->setTextColor(YELLOW);
        gfx->setTextSize(2);
        centerText("TRIP ACTIVE", 20);
      }

      // Movement status indicator
      gfx->setTextColor(isMoving ? GREEN : RED);
      gfx->setTextSize(1);
      centerText(isMoving ? "MOVING" : "STATIONARY", 35);

      // Satellite count
      if (gps.satellites.isValid()) {
        gfx->setTextColor(YELLOW);
        gfx->setTextSize(2);
        char satStr[20];
        snprintf(satStr, sizeof(satStr), "Satellites: %d", gps.satellites.value());
        centerText(satStr, gfx->height() - 50);
      }

      // IMU orientation
      if (imuAvailable) {
        gfx->setTextColor(CYAN);
        gfx->setTextSize(1);
        char orientStr[30];
        
        if (accelY > 7) {
          snprintf(orientStr, sizeof(orientStr), "Portrait");
        } else if (accelY < -7) {
          snprintf(orientStr, sizeof(orientStr), "Portrait Inverted");
        } else if (accelX > 7) {
          snprintf(orientStr, sizeof(orientStr), "Landscape Left");
        } else if (accelX < -7) {
          snprintf(orientStr, sizeof(orientStr), "Landscape Right");
        } else {
          snprintf(orientStr, sizeof(orientStr), "Flat");
        }
        
        gfx->setCursor(5, gfx->height() - 10);
        gfx->print(orientStr);
      }

      // Debug info in corner (remove this after testing if desired)
      gfx->setTextColor(CYAN);
      gfx->setTextSize(1);
      char debugStr[30];
      snprintf(debugStr, sizeof(debugStr), "Raw: %.1f", rawSpeed);
      gfx->setCursor(gfx->width() - 60, gfx->height() - 10);
      gfx->print(debugStr);

      lastSpeed = displaySpeed;
      lastHadFix = true;
    }
  } else {
    if (lastHadFix) {
      drawConnectingScreen();
      lastHadFix = false;
    }
  }
}

void drawTripData() {
  gfx->fillScreen(BLACK);
  gfx->setTextColor(CYAN);
  gfx->setTextSize(3);
  centerText("TRIP DATA", 30);
  
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  
  // Trip time
  if (tripActive) {
    unsigned long tripTime = (millis() - tripStartTime) / 1000;
    char timeStr[20];
    snprintf(timeStr, sizeof(timeStr), "Time: %02lu:%02lu", tripTime / 60, tripTime % 60);
    centerText(timeStr, 80);
  } else {
    centerText("Trip: STOPPED", 80);
  }
  
  // Max speed (now filtered)
  char maxSpeedStr[20];
  dtostrf(tripMaxSpeed, 4, 1, maxSpeedStr);
  char maxStr[30];
  snprintf(maxStr, sizeof(maxStr), "Max: %s MPH", maxSpeedStr);
  centerText(maxStr, 120);
  
  // Distance
  char distStr[30];
  dtostrf(tripDistance, 4, 2, distStr);
  char distanceStr[40];
  snprintf(distanceStr, sizeof(distanceStr), "Dist: %s mi", distStr);
  centerText(distanceStr, 160);
  
  gfx->setTextColor(YELLOW);
  gfx->setTextSize(1);
  centerText("Auto-return in 5 seconds", 220);
}

void drawSettings() {
  gfx->fillScreen(BLACK);
  gfx->setTextColor(MAGENTA);
  gfx->setTextSize(6);
  centerText("SETTINGS", gfx->height() / 2);
  
  gfx->setTextColor(WHITE);
  gfx->setTextSize(2);
  centerText("Coming soon...", gfx->height() / 2 + 60);
}

void drawConnectingScreen() {
  gfx->fillScreen(BLACK);
  gfx->setTextColor(YELLOW);
  gfx->setTextSize(3);
  centerText("CONNECTING", 60);
  centerText("TO", 110);
  centerText("SATELLITES", 160);
  
  if (gps.satellites.isValid()) {
    gfx->setTextColor(WHITE);
    gfx->setTextSize(2);
    char satStr[20];
    snprintf(satStr, sizeof(satStr), "Found: %d", gps.satellites.value());
    centerText(satStr, 200);
  }
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

// IMU Functions (unchanged from your original code)
void initIMU() {
  USBSerial.println("Initializing IMU...");
  
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
    USBSerial.println("IMU initialization failed!");
    imuAvailable = false;
    return;
  }
  
  USBSerial.println("IMU found!");
  USBSerial.printf("Chip ID: 0x%02X\n", qmi.getChipID());
  
  qmi.configAccelerometer(
    SensorQMI8658::ACC_RANGE_4G,
    SensorQMI8658::ACC_ODR_1000Hz,
    SensorQMI8658::LPF_MODE_0,
    true);
  
  qmi.enableAccelerometer();
  
  imuAvailable = true;
  USBSerial.println("IMU initialized successfully");
}

void readIMU() {
  if (!imuAvailable) return;
  
  if (qmi.getDataReady()) {
    if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
      accelX = acc.x;
      accelY = acc.y;
      accelZ = acc.z;
    }
  }
}