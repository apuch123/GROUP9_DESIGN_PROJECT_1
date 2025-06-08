#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_BMP280.h>
#include "FreeRTOS.h"
#include "task.h"

// --- Pin Configuration ---
#define OLED_SDA_PIN PB9
#define OLED_SCL_PIN PB8
#define BMP_SDA PB11
#define BMP_SCL PB10
#define OLED_ADDR 0x3C
#define BMP280_ADDR 0x76

#define LDR_PIN PA0
#define LED_PIN PA5
#define BUTTON_PIN PC14
#define BUZZER_PIN PA6
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// --- I2C Instances ---
TwoWire WireOLED(OLED_SDA_PIN, OLED_SCL_PIN);
TwoWire WireBMP(BMP_SDA, BMP_SCL); // Custom I2C instance

// --- Global Variables ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &WireOLED, -1);
Adafruit_BMP280 bmp(&WireBMP);

enum DisplayMode { MODE_ENVIRONMENT, MODE_CALIBRATION };
DisplayMode currentMode = MODE_ENVIRONMENT;

struct SystemState {
  int ldrRaw = 0;
  float ldrVoltage = 0;
  float temperature = 0;
  float pressure = 0;
  bool ledState = false;
  bool buttonPressed = false;
  float tempOffset = 0;
  float pressOffset = 0;
} state;

// Mutex for shared state protection (optional here, but good practice)
SemaphoreHandle_t xStateMutex;

unsigned long startMillis;

// Forward declarations
void SensorTask(void *pvParameters);
void DisplayTask(void *pvParameters);
void CommunicationTask(void *pvParameters);
void UserInterfaceTask(void *pvParameters);
void HeartbeatTask(void *pvParameters);

void setup() {
  Serial1.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  WireOLED.begin();
  WireOLED.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial1.println("OLED Init Failed!");
    while(1);
  }

  WireBMP.begin();
  WireBMP.setClock(400000);
  if (!bmp.begin(0x76)) {
    Serial1.println("BMP280 not found at 0x76! Trying 0x77...");
    if (!bmp.begin(0x77)) {
      Serial1.println("BMP280 not found! Check wiring.");
      while(1);
    }
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("System Ready");
  display.display();
  delay(1000);

  startMillis = millis();

  // Create mutex
  xStateMutex = xSemaphoreCreateMutex();

  // Create FreeRTOS tasks
  xTaskCreate(SensorTask, "SensorTask", 256, NULL, 2, NULL);
  xTaskCreate(DisplayTask, "DisplayTask", 512, NULL, 1, NULL);
  xTaskCreate(CommunicationTask, "CommunicationTask", 256, NULL, 1, NULL);
  xTaskCreate(UserInterfaceTask, "UserInterfaceTask", 128, NULL, 3, NULL);
  xTaskCreate(HeartbeatTask, "HeartbeatTask", 128, NULL, 4, NULL);

  // Start the scheduler
  vTaskStartScheduler();

  // Should never reach here
  while(1);
}

void loop() {
  // Empty, FreeRTOS scheduler is running
}

// --- Task Implementations ---

// Sensor Reading Task: samples sensors every 1000ms
void SensorTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);

  for (;;) {
    // Read sensors
    int ldrRawLocal = analogRead(LDR_PIN);
    ldrRawLocal = 4095 - ldrRawLocal; // invert

    float vcc = 3.3;
    float ldrVoltageLocal = ldrRawLocal * (vcc / 4095.0);
    const float LDR_REF_RESISTOR = 10000.0;
    float ldrResistance = 0;

    if (ldrVoltageLocal > 0.01 && ldrVoltageLocal < (vcc - 0.01)) {
      ldrResistance = (vcc - ldrVoltageLocal) * LDR_REF_RESISTOR / ldrVoltageLocal;
    } else {
      ldrResistance = (ldrVoltageLocal <= 0.01) ? INFINITY : 0;
    }

    float temperatureLocal = bmp.readTemperature();
    float pressureLocal = bmp.readPressure() / 100.0F;

    // Alarm logic
    const int LIGHT_ALARM_THRESHOLD = 33;
    if (temperatureLocal > LIGHT_ALARM_THRESHOLD) {
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }

    // Update shared state with mutex
    if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
      state.ldrRaw = ldrRawLocal;
      state.ldrVoltage = ldrVoltageLocal;
      state.temperature = temperatureLocal;
      state.pressure = pressureLocal;
      xSemaphoreGive(xStateMutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Display Task: updates OLED every 1000ms
void DisplayTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);

  for (;;) {
    int ldrRawLocal;
    float ldrVoltageLocal, temperatureLocal, pressureLocal, tempOffsetLocal, pressOffsetLocal;
    DisplayMode modeLocal;

    if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
      ldrRawLocal = state.ldrRaw;
      ldrVoltageLocal = state.ldrVoltage;
      temperatureLocal = state.temperature;
      pressureLocal = state.pressure;
      tempOffsetLocal = state.tempOffset;
      pressOffsetLocal = state.pressOffset;
      modeLocal = currentMode;
      xSemaphoreGive(xStateMutex);
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(getClock());

    if (modeLocal == MODE_ENVIRONMENT) {
      display.println(" Environment Monitor");
      display.println("---------------------");
      display.printf("     L: %d\n", ldrRawLocal);
      display.print("     T: ");
      display.print(temperatureLocal, 1);
      display.println(" C");
      display.print("     P: ");
      display.print(pressureLocal, 1);
      display.println(" hPa");
    } else {
      display.println("  Calibration Mode");
      display.println("---------------------");
      display.print("    T Offset: ");
      display.print(tempOffsetLocal, 1);
      display.println();
      display.print("    P Offset: ");
      display.print(pressOffsetLocal, 1);
      display.println();
    }
    display.display();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Communication Task: sends data over UART every 2000ms
void CommunicationTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2000);

  for (;;) {
    int ldrRawLocal;
    float ldrVoltageLocal, temperatureLocal, pressureLocal;

    if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
      ldrRawLocal = state.ldrRaw;
      ldrVoltageLocal = state.ldrVoltage;
      temperatureLocal = state.temperature;
      pressureLocal = state.pressure;
      xSemaphoreGive(xStateMutex);
    }

    Serial1.print(getClock());
    Serial1.print(" | LDR: ");
    Serial1.print(ldrRawLocal);
    Serial1.print(" (");
    Serial1.print(ldrVoltageLocal, 2);
    Serial1.print("V) | Temp: ");
    Serial1.print(temperatureLocal, 1);
    Serial1.print("% | Pres: ");
    Serial1.print(pressureLocal, 1);
    Serial1.println("hPa");

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// User Interface Task: monitors button presses, debounce 300ms
void UserInterfaceTask(void *pvParameters) {
  const TickType_t debounceDelay = pdMS_TO_TICKS(300);
  TickType_t lastPressTime = 0;

  for (;;) {
    bool buttonPressed = (digitalRead(BUTTON_PIN) == LOW);
    TickType_t now = xTaskGetTickCount();

    if (buttonPressed && (now - lastPressTime > debounceDelay)) {
      lastPressTime = now;

      // Toggle mode safely
      taskENTER_CRITICAL();
      currentMode = (currentMode == MODE_ENVIRONMENT) ? MODE_CALIBRATION : MODE_ENVIRONMENT;
      taskEXIT_CRITICAL();

      Serial1.println(currentMode == MODE_ENVIRONMENT ? "Mode: Environment" : "Mode: Calibration");
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Poll every 50 ms
  }
}

// Heartbeat Task: toggles LED every 500ms
void HeartbeatTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(500);

  bool ledStateLocal = false;

  for (;;) {
    ledStateLocal = !ledStateLocal;
    digitalWrite(LED_PIN, ledStateLocal);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Helper function to get formatted clock string
String getClock() {
  unsigned long elapsed = (millis() - startMillis) / 1000;
  unsigned long startSec = 20, startMin = 32, startHour = 4;
  elapsed += startSec + startMin * 60 + startHour * 3600;

  unsigned int sec = elapsed % 60;
  unsigned int min = (elapsed / 60) % 60;
  unsigned int hour = (elapsed / 3600) % 24;

  char buffer[9];
  sprintf(buffer, "       %02u:%02u:%02u", hour, min, sec);
  return String(buffer);
}
