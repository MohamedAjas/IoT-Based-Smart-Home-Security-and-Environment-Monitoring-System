#include <Arduino.h>

#include <WiFi.h>
#include <Wire.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremote.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

// ========== WiFi and ThingsBoard ==========
#define WIFI_SSID     "Wokwi-GUEST"
#define WIFI_PASSWORD ""
#define TB_SERVER     "thingsboard.cloud"
#define TB_TOKEN      "1bY45oHhIF4R9PBzWZxd"

// ========== Pin Definitions ==========
#define DHTPIN        14
#define DHTTYPE       DHT22
#define TRIG_PIN      12
#define ECHO_PIN      13
#define PIR_PIN       26
#define IR_PIN        27
#define BUZZER_PIN    25
#define LED_PIN       33
#define BUTTON_PIN    32

// ========== Display ==========
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ========== Sensor Setup ==========
DHT dht(DHTPIN, DHTTYPE);
IRrecv irrecv(IR_PIN);
decode_results results;

// ========== ThingsBoard ==========
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient);

// ========== Variables ==========
float temperature, humidity, distance;
long duration;
bool motionDetected = false;
bool buttonPressed = false;
String lastIRCommand = "";

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi!");
}

void connectToThingsBoard() {
  while (!tb.connected()) {
    Serial.println("Connecting to ThingsBoard...");
    if (tb.connect(TB_SERVER, TB_TOKEN)) {
      Serial.println("Connected to ThingsBoard!");
    } else {
      Serial.println("Failed, retrying...");
      delay(1000);
    }
  }
}

void readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
}

void readDHT() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
}

void updateOLED() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.print("Temp: ");
  oled.print(temperature);
  oled.println(" C");
  oled.print("Humidity: ");
  oled.print(humidity);
  oled.println(" %");
  oled.print("Distance: ");
  oled.print(distance);
  oled.println(" cm");
  oled.print("Motion: ");
  oled.println(motionDetected ? "YES" : "NO");
  oled.display();
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print("C H:");
  lcd.print(humidity);
  lcd.setCursor(0, 1);
  lcd.print("D:");
  lcd.print(distance);
  lcd.print("cm PIR:");
  lcd.print(motionDetected ? "Y" : "N");
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  lcd.init();
  lcd.backlight();
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  irrecv.enableIRIn();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  connectToWiFi();
  connectToThingsBoard();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
  if (!tb.connected()) {
    connectToThingsBoard();
  }

  tb.loop();

  // Read Sensors
  readDHT();
  readUltrasonic();
  motionDetected = digitalRead(PIR_PIN);
  buttonPressed = digitalRead(BUTTON_PIN) == LOW;

  // IR Remote
  if (irrecv.decode(&results)) {
    lastIRCommand = String(results.value, HEX);
    Serial.print("IR: ");
    Serial.println(lastIRCommand);
    irrecv.resume();  // Receive next
  }

  // Output Control
  digitalWrite(LED_PIN, motionDetected || buttonPressed);
  digitalWrite(BUZZER_PIN, motionDetected ? HIGH : LOW);

  // Update Display
  updateOLED();
  updateLCD();

  // Send to ThingsBoard
  tb.sendTelemetryFloat("temperature", temperature);
  tb.sendTelemetryFloat("humidity", humidity);
  tb.sendTelemetryFloat("distance_cm", distance);
  tb.sendTelemetryBool("motion_detected", motionDetected);
  tb.sendTelemetryBool("button_pressed", buttonPressed);
  tb.sendTelemetryString("last_ir_command", lastIRCommand);

  delay(2000);
}
