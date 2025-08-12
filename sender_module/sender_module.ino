/* Sender: TTGO + MPU6500
   PDR: step detection + gyro-integrated heading.
   Sends POS packets via LoRa when a step is detected:
   Format sent: POS,<x_m>,<y_m>,<yaw_deg>
*/

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
#define MPU_SDA 21
#define MPU_SCL 22

// LoRa pins (same as your code)
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26

// Calibration & PDR parameters — tune these
const int CAL_SAMPLES = 500;
const float G = 9.80665f;

// Step detection thresholds (g units)
float STEP_THRESHOLD = 0.20f;        // amplitude above (|mag_g - 1.0|) to detect step start — tune
float STEP_LOWER = 0.08f;            // lower threshold for step end
unsigned long MIN_STEP_INTERVAL_MS = 300; // minimum time between steps

// Step length model: choose fixed or dynamic
bool USE_FIXED_STEP_LENGTH = true;
float STEP_LENGTH = 0.68f; // meters — tune for your stride
// if dynamic, we'll use a simple amplitude-based heuristic:
// L = K * pow(peak_g, 0.25). You can experiment with this.
float DYN_K = 0.45f;

// runtime variables
float gyroBiasZ = 0.0f;
float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;
bool calibrated = false;

unsigned long lastMicros = 0;
float yaw = 0.0f;            // radians
float posX = 0.0f, posY = 0.0f; // meters

// step detection state
bool inStepWindow = false;
unsigned long lastStepTime = 0;
float lastPeak = 0.0f;

void calibrateSensors() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Calibrating...");
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
    delay(4); // ~250 Hz sampling during calibration
  }
  accelBiasX = sumAx / CAL_SAMPLES;
  accelBiasY = sumAy / CAL_SAMPLES;
  accelBiasZ = sumAz / CAL_SAMPLES; // note: if sensor was flat, accelBiasZ ≈ +1.0 (g)
  gyroBiasZ = sumGz / CAL_SAMPLES;

  calibrated = true;

  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("Cal done\nGbiasZ: %.2f\nAbiasZ: %.2f g\n", gyroBiasZ, accelBiasZ);
  display.display();
  delay(800);
}

void setup() {
  Serial.begin(115200);
  // OLED reset
  pinMode(OLED_RESET, OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(20);
  digitalWrite(OLED_RESET, HIGH);
  // I2C
  Wire.begin(MPU_SDA, MPU_SCL);

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

  // IMU init
  myIMU.setWire(&Wire);
  myIMU.beginAccel();
  myIMU.beginGyro();
  delay(200);

  display.setCursor(0,10);
  display.println("MPU Ready");
  display.display();

  // LoRa init
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  if (!LoRa.begin(868E6)) {
    display.println("LoRa init failed!");
    display.display();
    while (true);
  }
  display.println("LoRa started!");
  display.display();
  delay(200);

  // calibrate while still
  calibrateSensors();
  lastMicros = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6f;
  if (dt <= 0) dt = 0.001f;
  lastMicros = now;

  // update IMU
  myIMU.accelUpdate();
  myIMU.gyroUpdate();

  // raw readings
  float ax_g = myIMU.accelX(); // in g
  float ay_g = myIMU.accelY();
  float az_g = myIMU.accelZ();

  float gx = myIMU.gyroX();
  float gy = myIMU.gyroY();
  float gz = myIMU.gyroZ();

  // remove biases computed at startup
  float gz_corr = gz - gyroBiasZ;

  // integrate yaw using gyro Z (deg/s -> rad/s)
  float gz_rad = gz_corr * (PI / 180.0f);
  yaw += gz_rad * dt; // radians
  // normalize yaw to -PI..PI
  if (yaw > PI) yaw -= 2*PI;
  else if (yaw < -PI) yaw += 2*PI;

  // step detection using acceleration magnitude
  float mag_g = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float mag_no_g = fabs(mag_g - 1.0f); // how far magnitude is from 1 g

  unsigned long nowMillis = millis();

  // peak capture / simple hysteresis state-machine
  if (!inStepWindow) {
    // look for rising above threshold
    if (mag_no_g > STEP_THRESHOLD && (nowMillis - lastStepTime) > MIN_STEP_INTERVAL_MS) {
      inStepWindow = true;
      lastPeak = mag_no_g;
    }
  } else {
    // we're in a candidate step window; update peak and wait for drop below lower threshold
    if (mag_no_g > lastPeak) lastPeak = mag_no_g;
    if (mag_no_g < STEP_LOWER) {
      // step confirmed
      inStepWindow = false;
      lastStepTime = nowMillis;

      // estimate step length
      float stepLen = STEP_LENGTH;
      if (!USE_FIXED_STEP_LENGTH) {
        // dynamic (simple heuristic). peak is in g; ensure positive
        float ampl = max(0.02f, lastPeak);
        stepLen = DYN_K * pow(ampl, 0.25f); // fine-tune DYN_K
        if (stepLen < 0.3f) stepLen = 0.3f;
        if (stepLen > 1.5f) stepLen = 1.5f;
      }

      // update position using current yaw (yaw=0 -> +X)
      float dx = stepLen * cos(yaw);
      float dy = stepLen * sin(yaw);
      posX += dx;
      posY += dy;

      // prepare message and send via LoRa
      char buf[120];
      // send x,y in meters and yaw in degrees
      float yaw_deg = yaw * 180.0f / PI;
      snprintf(buf, sizeof(buf), "POS,%.3f,%.3f,%.1f", posX, posY, yaw_deg);

      // LoRa send
      LoRa.beginPacket();
      LoRa.print(buf);
      LoRa.endPacket();

      // display
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Step detected!");
      display.printf("X: %.2f m\nY: %.2f m\nYaw: %.1f\n", posX, posY, yaw_deg);
      display.display();

      // debug serial
      Serial.println(buf);
    }
  } // end step window

  // optional small delay for loop cadence
  delay(10); // ~100 Hz loop
}
