#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Pins
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// LoRa Pins
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

  // Initialize I2C for OLED
  Wire.begin(21, 22);

  // OLED Init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Receiver Starting...");
  display.display();

  // LoRa Init
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(433E6)) {
    display.println("LoRa init failed!");
    display.display();
    while (true);
  }

  display.setCursor(0, 10);
  display.println("LoRa Ready!");
  display.display();
  delay(1000);
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }

    Serial.println("Received: " + received);

    // Try to split the string into values
    float ax, ay, az, gx, gy, gz, tempC;
    int parsed = sscanf(received.c_str(), "%f,%f,%f,%f,%f,%f,%f",
                        &ax, &ay, &az, &gx, &gy, &gz, &tempC);

    display.clearDisplay();
    display.setCursor(0, 0);
    if (parsed == 7) {
      display.println("Received Data:");
      display.printf("Ax: %.1f Ay: %.1f\nAz: %.1f\n", ax, ay, az);
      display.printf("Gx: %.1f Gy: %.1f\nGz: %.1f\n", gx, gy, gz);
      display.printf("Temp: %.1f C\n", tempC);
    } else {
      display.println("Invalid Data:");
      display.println(received);
    }
    display.display();
  }
}
