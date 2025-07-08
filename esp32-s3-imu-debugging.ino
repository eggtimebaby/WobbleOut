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

      // Movement status indicator (simplified)
      gfx->setTextColor(isMoving ? GREEN : RED);
      gfx->setTextSize(2);
      gfx->setCursor(10, 35);
      gfx->print(isMoving ? "M" : "S");

      // Satellite count
      if (gps.satellites.isValid()) {
        gfx->setTextColor(YELLOW);
        gfx->setTextSize(2);
        char satStr[20];
        snprintf(satStr, sizeof(satStr), "Satellites: %d", gps.satellites.value());
        centerText(satStr, gfx->height() - 50);
      }

      // IMU visualization - Simple accelerometer bar graph
      if (imuAvailable) {
        // Draw IMU status and simple visualization
        gfx->setTextColor(CYAN);
        gfx->setTextSize(1);
        gfx->setCursor(10, 50);
        gfx->print("IMU:");
        
        // Calculate total acceleration magnitude
        float totalAccel = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        
        // Draw acceleration bars (X, Y, Z)
        int barX = 45;
        int barY = 50;
        int barHeight = 8;
        int barWidth = 30;
        
        // X acceleration bar (red)
        gfx->fillRect(barX, barY, barWidth, barHeight, BLACK);
        gfx->drawRect(barX, barY, barWidth, barHeight, RED);
        int xBarFill = map(constrain(abs(accelX), 0, 10), 0, 10, 0, barWidth-2);
        if (xBarFill > 0) {
          gfx->fillRect(barX+1, barY+1, xBarFill, barHeight-2, RED);
        }
        gfx->setCursor(barX + barWidth + 2, barY);
        gfx->print("X");
        
        // Y acceleration bar (green)
        barY += 12;
        gfx->fillRect(barX, barY, barWidth, barHeight, BLACK);
        gfx->drawRect(barX, barY, barWidth, barHeight, GREEN);
        int yBarFill = map(constrain(abs(accelY), 0, 10), 0, 10, 0, barWidth-2);
        if (yBarFill > 0) {
          gfx->fillRect(barX+1, barY+1, yBarFill, barHeight-2, GREEN);
        }
        gfx->setCursor(barX + barWidth + 2, barY);
        gfx->print("Y");
        
        // Z acceleration bar (blue)
        barY += 12;
        gfx->fillRect(barX, barY, barWidth, barHeight, BLACK);
        gfx->drawRect(barX, barY, barWidth, barHeight, BLUE);
        int zBarFill = map(constrain(abs(accelZ), 0, 10), 0, 10, 0, barWidth-2);
        if (zBarFill > 0) {
          gfx->fillRect(barX+1, barY+1, zBarFill, barHeight-2, BLUE);
        }
        gfx->setCursor(barX + barWidth + 2, barY);
        gfx->print("Z");
        
        // Total acceleration indicator
        gfx->setCursor(85, 50);
        char accelStr[15];
        snprintf(accelStr, sizeof(accelStr), "%.1fg", totalAccel);
        gfx->print(accelStr);
        
        // Simple tilt indicator using largest axis
        gfx->setCursor(135, 50);
        if (abs(accelX) > abs(accelY) && abs(accelX) > abs(accelZ)) {
          gfx->print(accelX > 0 ? "R" : "L");  // Right/Left tilt
        } else if (abs(accelY) > abs(accelZ)) {
          gfx->print(accelY > 0 ? "U" : "D");  // Up/Down tilt
        } else {
          gfx->print("F");  // Flat
        }
      } else {
        gfx->setTextColor(RED);
        gfx->setTextSize(1);
        gfx->setCursor(10, 50);
        gfx->print("IMU: OFFLINE");
      }

      // Debug info in corner (remove this after testing if desired)
      gfx->setTextColor(CYAN);
      gfx->setTextSize(1);
      char debugStr[50];
      snprintf(debugStr, sizeof(debugStr), "Raw: %.1f IMU: %.1f,%.1f,%.1f", rawSpeed, accelX, accelY, accelZ);
      gfx->setCursor(5, gfx->height() - 10);
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
  
  // First, let's scan I2C to see what's available
  USBSerial.println("Scanning I2C bus...");
  Wire.beginTransmission(0x6A);
  if (Wire.endTransmission() == 0) {
    USBSerial.println("Device found at 0x6A");
  }
  Wire.beginTransmission(0x6B);
  if (Wire.endTransmission() == 0) {
    USBSerial.println("Device found at 0x6B");
  }
  
  // Try both possible I2C addresses
  bool foundAt6A = false, foundAt6B = false;
  
  // Try 0x6A first
  if (qmi.begin(Wire, 0x6A, IIC_SDA, IIC_SCL)) {
    USBSerial.printf("IMU responded at 0x6A, Chip ID: 0x%02X\n", qmi.getChipID());
    foundAt6A = true;
  }
  
  // Try 0x6B if 0x6A didn't work
  if (!foundAt6A && qmi.begin(Wire, 0x6B, IIC_SDA, IIC_SCL)) {
    USBSerial.printf("IMU responded at 0x6B, Chip ID: 0x%02X\n", qmi.getChipID());
    foundAt6B = true;
  }
  
  if (!foundAt6A && !foundAt6B) {
    USBSerial.println("IMU initialization failed at both addresses!");
    imuAvailable = false;
    return;
  }
  
  uint8_t chipID = qmi.getChipID();
  USBSerial.printf("Found IMU with Chip ID: 0x%02X\n", chipID);
  
  // Try different configuration approaches based on chip ID
  bool configSuccess = false;
  
  if (chipID == 0x05) {
    USBSerial.println("Standard QMI8658 detected, using normal config...");
    configSuccess = qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_1000Hz,
      SensorQMI8658::LPF_MODE_0,
      true);
  } else if (chipID == 0x7C) {
    USBSerial.println("Different chip ID detected, trying alternative configs...");
    
    // Try different configurations
    configSuccess = qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_2G,
      SensorQMI8658::ACC_ODR_250Hz,
      SensorQMI8658::LPF_MODE_0,
      true);
    
    if (!configSuccess) {
      USBSerial.println("First alt config failed, trying second...");
      configSuccess = qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_8G,
        SensorQMI8658::ACC_ODR_500Hz,
        SensorQMI8658::LPF_MODE_1,
        false);
    }
  } else {
    USBSerial.printf("Unknown chip ID 0x%02X, trying default config...\n", chipID);
    configSuccess = qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_250Hz,
      SensorQMI8658::LPF_MODE_0,
      true);
  }
  
  if (!configSuccess) {
    USBSerial.println("All accelerometer configurations failed!");
    imuAvailable = false;
    return;
  }
  
  USBSerial.println("Accelerometer configured successfully, enabling...");
  if (!qmi.enableAccelerometer()) {
    USBSerial.println("Failed to enable accelerometer");
    imuAvailable = false;
    return;
  }
  
  // Try to read initial values with longer delay
  USBSerial.println("Waiting for IMU to stabilize...");
  delay(500);  // Longer delay for stabilization
  
  // Try multiple read attempts
  bool dataReady = false;
  for (int i = 0; i < 10; i++) {
    if (qmi.getDataReady()) {
      dataReady = true;
      break;
    }
    delay(100);
  }
  
  if (dataReady) {
    USBSerial.println("IMU data ready after initialization");
    if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
      accelX = acc.x;
      accelY = acc.y;
      accelZ = acc.z;
      USBSerial.printf("Initial IMU readings - X: %.2f, Y: %.2f, Z: %.2f\n", accelX, accelY, accelZ);
      imuAvailable = true;
      USBSerial.println("IMU initialized successfully");
    } else {
      USBSerial.println("Failed to read initial accelerometer data");
      imuAvailable = false;
    }
  } else {
    USBSerial.println("IMU data not ready after multiple attempts");
    imuAvailable = false;
  }
}

void readIMU() {
  if (!imuAvailable) return;
  
  static unsigned long lastIMUDebug = 0;
  
  if (qmi.getDataReady()) {
    if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
      accelX = acc.x;
      accelY = acc.y;
      accelZ = acc.z;
      
      // Debug output every 2 seconds
      if (millis() - lastIMUDebug > 2000) {
        USBSerial.printf("IMU Data - X: %.2f, Y: %.2f, Z: %.2f, Total: %.2f\n", 
                         accelX, accelY, accelZ, 
                         sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ));
        lastIMUDebug = millis();
      }
    } else {
      // Debug if accelerometer read fails
      if (millis() - lastIMUDebug > 2000) {
        USBSerial.println("IMU: getAccelerometer() failed");
        lastIMUDebug = millis();
      }
    }
  } else {
    // Debug if data not ready
    if (millis() - lastIMUDebug > 5000) {  // Less frequent for this one
      USBSerial.println("IMU: getDataReady() returned false");
      lastIMUDebug = millis();
    }
  }
}