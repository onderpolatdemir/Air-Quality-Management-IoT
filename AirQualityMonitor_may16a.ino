#include "arduino_secrets.h"
#include <Arduino_ConnectionHandler.h>
#include <ArduinoIoTCloud.h>
#include "thingProperties.h"  

#include <Wire.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>

#define DHT_PIN 4
#define MQ135_PIN 34
#define RGB_RED_PIN 25
#define RGB_GREEN_PIN 26
#define RGB_BLUE_PIN 27
#define RELAY_PIN 16

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

#define MQ135_CALIBRATION_SAMPLE_TIMES 50
#define MQ135_CALIBRATION_SAMPLE_INTERVAL 500

#define AIR_QUALITY_GOOD 800
#define AIR_QUALITY_MODERATE 1200

#define HISTORY_SIZE 450  
#define PREDICTION_WINDOW 900  
#define TREND_WINDOW 30  

float airQualityHistory[HISTORY_SIZE];
unsigned long timeHistory[HISTORY_SIZE];
int historyIndex = 0;
bool historyFull = false;

float prediction = 0;
float trendSlope = 0;
float shortTermAverage = 0;
float longTermAverage = 0;

String previousVentilationSpeed = "off";
bool speedChanged = false;

unsigned long previousMillis = 0;
const long interval = 2000;  

int mq135Readings[5] = {0};
int readIndex = 0;
int mq135Sum = 0;
int mq135Average = 0;

void setup() {
  Serial.begin(9600);
  delay(1500);

  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); 

  initSensors();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Starting system...");
  display.display();

  initProperties();


  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  display.println("Connecting to Cloud...");
  display.display();
}

void loop() {
  ArduinoCloud.update();
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    readSensors();
    storeHistoricalData();
    updatePrediction();
    updateDisplay();
    updateRGBIndicator();
    controlVentilation();
  }
}

void initSensors() {
  dht.begin();
  Serial.println("MQ135 warming up. For best results, allow several minutes for stabilization.");
  Serial.println("Taking initial readings for calibration...");
  
  float mq135Sum = 0;
  for (int i = 0; i < MQ135_CALIBRATION_SAMPLE_TIMES; i++) {
        int reading = analogRead(MQ135_PIN);
    mq135Sum += reading;
    
    Serial.print("Calibration reading ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(MQ135_CALIBRATION_SAMPLE_TIMES);
    Serial.print(": ");
    Serial.println(reading);
    
    delay(MQ135_CALIBRATION_SAMPLE_INTERVAL / 10);  // Reduced to speed up initial setup
  }
  
  float mq135Baseline = mq135Sum / MQ135_CALIBRATION_SAMPLE_TIMES;
  Serial.print("MQ135 baseline value: ");
  Serial.println(mq135Baseline);
  Serial.println("Sensor initialization complete. Monitoring for stabilization...");
}

void readSensors() {
  float tempHumidity = dht.readHumidity();
  float tempTemperature = dht.readTemperature();
  
  if (isnan(tempHumidity) || isnan(tempTemperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    humidity = tempHumidity;
    temperature = tempTemperature;
  }
  
  mq135Sum = mq135Sum - mq135Readings[readIndex];
  
  mq135Readings[readIndex] = analogRead(MQ135_PIN);
  
  mq135Sum = mq135Sum + mq135Readings[readIndex];
  
  readIndex = (readIndex + 1) % 5;
  
  mq135Average = mq135Sum / 5;
  
  int tempAirQuality = map(mq135Average, 0, 4095, 400, 2000);
   
  int airQualityPercentage = map(tempAirQuality, 400, 2000, 0, 100);
  
  airQuality = airQualityPercentage;
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.print(" %, Air Quality: ");
   Serial.print(airQualityPercentage);
  Serial.print("%, Raw PPM: ");
  Serial.print(tempAirQuality);
  Serial.println(" ppm CO2 eq.");
}

void storeHistoricalData() {
  airQualityHistory[historyIndex] = airQuality;
  timeHistory[historyIndex] = millis();
  
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  if (historyIndex == 0) {
    historyFull = true;
  }
}

void updatePrediction() {
  if (!historyFull && historyIndex < TREND_WINDOW) {
    return; 
  }
  
  float shortTermSum = 0;
  float longTermSum = 0;
  int shortTermCount = 0;
  int longTermCount = 0;
  
  int currentIndex = (historyIndex - 1 + HISTORY_SIZE) % HISTORY_SIZE;
  for (int i = 0; i < HISTORY_SIZE; i++) {
    int index = (currentIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
    if (i < TREND_WINDOW) {
      shortTermSum += airQualityHistory[index];
      shortTermCount++;
    }
    if (i < HISTORY_SIZE) {
      longTermSum += airQualityHistory[index];
      longTermCount++;
    }
  }
  
  shortTermAverage = shortTermSum / shortTermCount;
  longTermAverage = longTermSum / longTermCount;
  
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (int i = 0; i < TREND_WINDOW; i++) {
    int index = (currentIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
    float x = i;  
    float y = airQualityHistory[index];
    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumX2 += x * x;
  }
  

  trendSlope = (TREND_WINDOW * sumXY - sumX * sumY) / (TREND_WINDOW * sumX2 - sumX * sumX);

  float currentValue = airQualityHistory[currentIndex];
  float trendPrediction = currentValue + (trendSlope * (PREDICTION_WINDOW / 2000.0));  
  
  prediction = (0.4 * currentValue) + (0.4 * trendPrediction) + (0.2 * longTermAverage);
  
  prediction = constrain(prediction, 0, 100);

  predictedAirQuality = prediction;

    Serial.print("Current: ");
  Serial.print(currentValue);
  Serial.print(" Trend: ");
  Serial.print(trendSlope);
  Serial.print(" Predicted: ");
  Serial.println(prediction);
}


void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Air Quality Monitor");
  display.drawLine(0, 10, display.width(), 10, WHITE);
  
  display.setCursor(0, 15);
  display.print("Temp:");
  display.print(temperature);
  display.println(" C");
  
  display.setCursor(0, 25);
  display.print("Hum:");
  display.print(humidity);
  display.println(" %");
  
  display.setCursor(64, 15);
  display.print("AQ:");
  display.print(airQuality);
  display.println("%");
  
  display.setCursor(60, 25);
  display.print("Pred:");
  display.print(prediction, 1);  // Show only 1 decimal place
  display.println("%");
  
  display.setCursor(0, 45);
  display.print("Status: ");

  int tempAirQuality = map(airQuality, 0, 100, 400, 2000);
  if (tempAirQuality < AIR_QUALITY_GOOD) {
    display.println("GOOD");
    airQualityStatus = "GOOD";
  } else if (tempAirQuality < AIR_QUALITY_MODERATE) {
    display.println("MODERATE");
    airQualityStatus = "MODERATE";
  } else {
    display.println("POOR");
    airQualityStatus = "POOR";
  }
  
  display.setCursor(0, 55);
  display.print("Vent: ");
  display.print(ventilationStatus ? "ON" : "OFF");
  if (ventilationStatus) {
    display.print(" (");
    display.print(ventilationSpeed);
    display.print(")");
  }
  
  display.display();
}

void updateRGBIndicator() {
  int tempAirQuality = map(airQuality, 0, 100, 400, 2000);
  if (tempAirQuality < AIR_QUALITY_GOOD) {
    setRGBColor(0, 255, 0);
  } else if (tempAirQuality < AIR_QUALITY_MODERATE) {
    setRGBColor(0, 0, 255);
  } else {
    setRGBColor(255, 0, 0);
  }
}

void setRGBColor(int r, int g, int b) {
  analogWrite(RGB_RED_PIN, r);
  analogWrite(RGB_GREEN_PIN, g);
  analogWrite(RGB_BLUE_PIN, b);
}

void controlVentilation() {

    static unsigned long lastVentilationChange = 0;
    if (millis() - lastVentilationChange < 60000) {  
        return;
    }

    int currentReading = analogRead(MQ135_PIN);

    int airQualityPPM = map(currentReading, 0, 4095, 400, 2000);

    if (airQualityPPM > AIR_QUALITY_GOOD && !ventilationStatus) {
      ventilationStatus = true;
      digitalWrite(RELAY_PIN, HIGH);
      Serial.println("Ventilation activated (based on current reading)");
      
      if (airQualityPPM > 1200) { 
        ventilationSpeed = "high";
        Serial.println("Ventilation speed set to high");
      } else {
        ventilationSpeed = "medium";
        Serial.println("Ventilation speed set to medium");
      }
    } 
    else if (airQualityPPM <= AIR_QUALITY_GOOD - 50 && ventilationStatus) {
      ventilationStatus = false;
      ventilationSpeed = "off";
      digitalWrite(RELAY_PIN, LOW);
      Serial.println("Ventilation deactivated");
    }
    else if (ventilationStatus) {
      static int lastSpeedChange = 0;
      if (millis() - lastSpeedChange > 10000) {  
        String newSpeed = ventilationSpeed;  
   
        if (airQualityPPM > 1200) {  
          if (ventilationSpeed != "high") {
            ventilationSpeed = "high";
            Serial.println("Ventilation speed set to high");
            lastSpeedChange = millis();
          }
        } 
        else {
          if (ventilationSpeed != "medium") {
            ventilationSpeed = "medium";
            Serial.println("Ventilation speed set to medium");
            lastSpeedChange = millis();
          }
        }
            
      previousVentilationSpeed = newSpeed;
    }
  }
}

void onVentilationStatusChange() {
  Serial.print("Ventilation status changed to: ");
  Serial.println(ventilationStatus);
  digitalWrite(RELAY_PIN, ventilationStatus ? HIGH : LOW);
}

void onVentilationSpeedChange() {
  Serial.print("Ventilation speed changed to: ");
  Serial.println(ventilationSpeed);
}