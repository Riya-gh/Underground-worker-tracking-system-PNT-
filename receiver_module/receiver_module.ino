/* Receiver: TTGO LoRa receiver that forwards POS packets to Serial (USB)
   When a LoRa packet like "POS,1.234,0.567,12.3" arrives, it prints it to Serial.
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// LoRa pins
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26

void setup() {
  Serial.begin(115200);
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(20);
  digitalWrite(OLED_RESET, HIGH);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    while(true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Receiver Starting...");
  display.display();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(868E6)) {
    display.println("LoRa init failed!");
    display.display();
    while (true);
  }
  display.println("LoRa Ready!");
  display.display();
  delay(300);
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    // Print to Serial so laptop can read it
    Serial.println(received);

    // Also show on OLED basic info
    float px=0, py=0, yaw=0;
    int parsed = sscanf(received.c_str(), "POS,%f,%f,%f", &px, &py, &yaw);
    display.clearDisplay();
    display.setCursor(0, 0);
    if (parsed==3) {
      display.printf("POS: X=%.2f Y=%.2f\nYaw: %.1f\n", px, py, yaw);
    } else {
      display.println("Received:");
      display.println(received);
    }
    display.display();
  }
}
