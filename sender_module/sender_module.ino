#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// IMU
MPU9250_asukiaaa myIMU;

// IMU I2C Pins
#define MPU_SDA 21
#define MPU_SCL 22

// LoRa pins
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26

void setup() {
  Serial.begin(115200);

  // OLED Reset
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(20);
  digitalWrite(OLED_RESET, HIGH);

  // I2C for OLED and IMU
  Wire.begin(MPU_SDA, MPU_SCL);

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (true);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();

  // IMU Init
  myIMU.setWire(&Wire);
  myIMU.beginAccel();
  myIMU.beginGyro();
  delay(200);

  display.setCursor(0, 10);
  display.println("MPU Ready");

  // LoRa Init
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(868E6)) {
    display.println("LoRa init failed!");
    display.display();
    while (true);
  }

  display.println("LoRa started!");
  display.display();
  delay(1000);
}

void loop() {
  // Update IMU values
  myIMU.accelUpdate();
  myIMU.gyroUpdate();

  float ax = myIMU.accelX();
  float ay = myIMU.accelY();
  float az = myIMU.accelZ();
  float gx = myIMU.gyroX();
  float gy = myIMU.gyroY();
  float gz = myIMU.gyroZ();

  // Temperature from MPU register
  Wire.beginTransmission(myIMU.address); // Usually 0x68
  Wire.write(0x41); // TEMP_OUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(myIMU.address, 2);
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  float tempC = rawTemp / 333.87 + 21.0;

  // Format LoRa message
  String message = String(ax, 2) + "," + String(ay, 2) + "," + String(az, 2) + "," +
                   String(gx, 2) + "," + String(gy, 2) + "," + String(gz, 2) + "," +
                   String(tempC, 2);

  // Send via LoRa
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();

  // Display on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Sending IMU Data:");
  display.printf("Ax: %.1f Ay: %.1f\nAz: %.1f\n", ax, ay, az);
  display.printf("Gx: %.1f Gy: %.1f\nGz: %.1f\n", gx, gy, gz);
  display.printf("Temp: %.1f C\n", tempC);
  display.display();

  // Print to Serial (optional)
  Serial.println("Sent: " + message);

  delay(1000); // Send every second
}
