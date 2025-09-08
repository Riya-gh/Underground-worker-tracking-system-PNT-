/* 
   TTGO ESP32 LoRa Sender with MPU6500/MPU9250
   Indoor Position Tracking using PDR (Pedestrian Dead Reckoning)
   Sends: POS,x_meters,y_meters,heading_degrees
   NO GPS - Pure indoor tracking!
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Configuration
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);

// IMU Configuration
MPU9250_asukiaaa myIMU;
#define MPU_SDA 21
#define MPU_SCL 22

// LoRa Configuration
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26

// Calibration Parameters
const int CAL_SAMPLES = 500;
const float G = 9.80665f;

// Step Detection Parameters (tune these for your walking)
float STEP_THRESHOLD = 0.20f;        // Threshold to detect step start
float STEP_LOWER = 0.08f;            // Threshold for step end
unsigned long MIN_STEP_INTERVAL_MS = 300; // Minimum time between steps (ms)

// Step Length (tune this to match your stride)
bool USE_FIXED_STEP_LENGTH = true;
float STEP_LENGTH = 0.70f; // meters - ADJUST THIS FOR YOUR STRIDE

// Runtime Variables
float gyroBiasZ = 0.0f;
float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;
bool calibrated = false;

unsigned long lastMicros = 0;
float heading_degrees = 0.0f;        // Current heading in degrees
float posX = 0.0f, posY = 0.0f;      // Position in meters (LOCAL coordinates)

// Step detection state
bool inStepWindow = false;
unsigned long lastStepTime = 0;
float lastPeak = 0.0f;
int stepCount = 0;

void calibrateSensors() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.println("Calibrating IMU...");
  display.println("Keep device STILL!");
  display.display();

  float sumGx = 0, sumGy = 0, sumGz = 0;
  float sumAx = 0, sumAy = 0, sumAz = 0;

  for (int i = 0; i < CAL_SAMPLES; ++i) {
    myIMU.accelUpdate();
    myIMU.gyroUpdate();
    sumAx += myIMU.accelX();
    sumAy += myIMU.accelY();
    sumAz += myIMU.accelZ();
    sumGx += myIMU.gyroX();
    sumGy += myIMU.gyroY();
    sumGz += myIMU.gyroZ();
    delay(4);
  }
  
  accelBiasX = sumAx / CAL_SAMPLES;
  accelBiasY = sumAy / CAL_SAMPLES;
  accelBiasZ = sumAz / CAL_SAMPLES;
  gyroBiasZ = sumGz / CAL_SAMPLES;

  calibrated = true;

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Calibration Done!");
  display.printf("Gyro Bias: %.2f\n", gyroBiasZ);
  display.printf("Accel Bias: %.2f g\n", accelBiasZ);
  display.println("Ready to track!");
  display.display();
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize OLED
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(20);
  digitalWrite(OLED_RESET, HIGH);
  
  Wire.begin(MPU_SDA, MPU_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED initialization failed!");
    while (true);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Indoor Position");
  display.println("Tracker Starting...");
  display.display();

  // Initialize IMU
  myIMU.setWire(&Wire);
  myIMU.beginAccel();
  myIMU.beginGyro();
  delay(200);

  display.setCursor(0,20);
  display.println("IMU: Ready");
  display.display();

  // Initialize LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(868E6)) {
    display.println("LoRa: FAILED!");
    display.display();
    while (true);
  }
  
  display.println("LoRa: Ready");
  display.display();
  delay(1000);

  // Calibrate sensors
  calibrateSensors();
  lastMicros = micros();
  
  Serial.println("Indoor Position Tracker Ready!");
  Serial.println("Data format: POS,x_meters,y_meters,heading_degrees");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // Read IMU data
  myIMU.accelUpdate();
  myIMU.gyroUpdate();

  float ax_g = myIMU.accelX();
  float ay_g = myIMU.accelY();
  float az_g = myIMU.accelZ();

  float gx = myIMU.gyroX();
  float gy = myIMU.gyroY();
  float gz = myIMU.gyroZ();

  // Remove bias
  float gz_corrected = gz - gyroBiasZ;

  // Update heading using gyroscope (integrate angular velocity)
  float gz_rad_per_sec = gz_corrected * (PI / 180.0f);
  float heading_radians = heading_degrees * (PI / 180.0f);
  heading_radians += gz_rad_per_sec * dt;
  
  // Normalize to 0-360 degrees
  heading_degrees = heading_radians * (180.0f / PI);
  if (heading_degrees < 0) heading_degrees += 360.0f;
  if (heading_degrees >= 360) heading_degrees -= 360.0f;

  // Step detection using acceleration magnitude
  float accel_magnitude = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float deviation_from_1g = fabs(accel_magnitude - 1.0f);

  unsigned long currentTime = millis();

  // State machine for step detection
  if (!inStepWindow) {
    // Looking for step start
    if (deviation_from_1g > STEP_THRESHOLD && 
        (currentTime - lastStepTime) > MIN_STEP_INTERVAL_MS) {
      inStepWindow = true;
      lastPeak = deviation_from_1g;
    }
  } else {
    // In step window - track peak and wait for drop
    if (deviation_from_1g > lastPeak) {
      lastPeak = deviation_from_1g;
    }
    
    if (deviation_from_1g < STEP_LOWER) {
      // Step confirmed!
      inStepWindow = false;
      lastStepTime = currentTime;
      stepCount++;

      // Calculate position update
      float step_length = STEP_LENGTH;
      
      // Convert heading to standard mathematical convention (0° = East, 90° = North)
      float math_heading = 90.0f - heading_degrees;
      float heading_rad = math_heading * (PI / 180.0f);
      
      // Update position
      float dx = step_length * cos(heading_rad);
      float dy = step_length * sin(heading_rad);
      posX += dx;
      posY += dy;

      // Create and send LoRa packet
      char packet[100];
      snprintf(packet, sizeof(packet), "POS,%.3f,%.3f,%.1f", posX, posY, heading_degrees);

      // Send via LoRa
      LoRa.beginPacket();
      LoRa.print(packet);
      LoRa.endPacket();

      // Update display with POSITION info (not lat/long!)
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("INDOOR POSITION:");
      display.printf("X: %.2f m\n", posX);
      display.printf("Y: %.2f m\n", posY);
      display.printf("Heading: %.1f°\n", heading_degrees);
      display.printf("Steps: %d\n", stepCount);
      display.println("(Local coordinates)");
      display.display();

      // Serial debug
      Serial.println(packet);
      Serial.printf("Step %d: X=%.2fm, Y=%.2fm, H=%.1f°\n", stepCount, posX, posY, heading_degrees);
    }
  }

  delay(10); // ~100Hz loop rate
}void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
