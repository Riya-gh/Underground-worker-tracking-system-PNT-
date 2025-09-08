/* 
   TTGO ESP32 LoRa Receiver 
   Receives indoor position data and forwards to laptop via Serial
   Data format: POS,x_meters,y_meters,heading_degrees
   NO GPS - Pure indoor position tracking!
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Configuration
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// LoRa Configuration
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26

// Statistics
int packetsReceived = 0;
unsigned long lastPacketTime = 0;
float lastX = 0.0f, lastY = 0.0f, lastHeading = 0.0f;

void setup() {
  Serial.begin(115200);
  
  // Initialize OLED
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(20);
  digitalWrite(OLED_RESET, HIGH);
  
  Wire.begin(21, 22); // SDA, SCL for OLED

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED initialization failed!");
    while(true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Indoor Position");
  display.println("Receiver Starting...");
  display.display();

  // Initialize LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  
  if (!LoRa.begin(868E6)) {
    display.println("LoRa: FAILED!");
    display.display();
    Serial.println("LoRa initialization failed!");
    while (true);
  }

  display.println("LoRa: Ready!");
  display.println("Waiting for data...");
  display.display();
  
  Serial.println("Indoor Position Receiver Ready!");
  Serial.println("Forwarding LoRa data to laptop...");
  
  delay(1000);
}

void loop() {
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    String receivedData = "";
    
    // Read the packet
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    
    // Forward to laptop via Serial (this is what your Python code reads)
    Serial.println(receivedData);
    
    // Parse and display position data
    float x = 0, y = 0, heading = 0;
    int rssi = LoRa.packetRssi();
    
    if (receivedData.startsWith("POS,")) {
      int parsed = sscanf(receivedData.c_str(), "POS,%f,%f,%f", &x, &y, &heading);
      
      if (parsed == 3) {
        packetsReceived++;
        lastPacketTime = millis();
        lastX = x;
        lastY = y;
        lastHeading = heading;
        
        // Update display with position info
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println("POSITION RECEIVED:");
        display.printf("X: %.2f meters\n", x);
        display.printf("Y: %.2f meters\n", y);
        display.printf("Heading: %.1f°\n", heading);
        display.printf("RSSI: %d dBm\n", rssi);
        display.printf("Packets: %d\n", packetsReceived);
        display.println("(Indoor coordinates)");
        display.display();
        
        // Debug info to Serial
        Serial.printf("Parsed: X=%.2fm, Y=%.2fm, H=%.1f°, RSSI=%ddBm\n", 
                     x, y, heading, rssi);
      } else {
        // Parsing failed
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Parse Error:");
        display.println(receivedData);
        display.display();
        
        Serial.printf("Parse failed: %s\n", receivedData.c_str());
      }
    } else {
      // Unknown data format
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Unknown data:");
      display.println(receivedData);
      display.display();
      
      Serial.printf("Unknown: %s\n", receivedData.c_str());
    }
  }
  
  // Check for timeout (no data received recently)
  if (packetsReceived > 0 && (millis() - lastPacketTime) > 5000) {
    // No data for 5 seconds
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("NO DATA");
    display.printf("Last position:\n");
    display.printf("X: %.2f m\n", lastX);
    display.printf("Y: %.2f m\n", lastY);
    display.printf("H: %.1f°\n", lastHeading);
    display.printf("Packets: %d\n", packetsReceived);
    display.println("Check sender!");
    display.display();
  }
  
  delay(100); // Small delay to prevent overwhelming the display
}