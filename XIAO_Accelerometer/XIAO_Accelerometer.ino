#include <bluefruit.h>
#include <LSM6DS3.h>
#include <math.h>

#define ACC_SERVICE_UUID 0xFFE0
#define CMD_CHARACTERISTIC_UUID 0xFFE2
#define DATA_CHARACTERISTIC_UUID 0xFFE3

#define CMD_START 0x01
#define CMD_STOP 0x02
#define CMD_DOWNLOAD 0x03
// Analog pin connected to battery
#define VBAT_MV_PER_LSB (3600.0F / 1023.0F) // 3.6V reference voltage
#define VBAT_DIVIDER 2.0F                   // Voltage divider on the board
#define VBAT_OFFSET 0.1F                    // Compensation for voltage drop

LSM6DS3 imu(I2C_MODE, 0x6A);

typedef struct
{
  uint32_t timestamp;
  float velocity; // Velocity in m/s
} DataPoint;

#define MAX_DATA_POINTS 2500 // Adjusted for nRF52840 memory constraints
DataPoint dataBuffer[MAX_DATA_POINTS];
int dataIndex = 0;
bool isRecording = false;

// Advertising parameters
#define ADVERTISING_INTERVAL_MS 100 // Fast advertising interval (100ms)
#define ADVERTISING_TIMEOUT_MS 0    // No timeout (continuous advertising)
#define ADVERTISING_POWER 4         // Default tx power (values from -40 to +8)
#define CONN_INTERVAL_MIN 8         // Minimum acceptable connection interval (10ms)
#define CONN_INTERVAL_MAX 16        // Maximum acceptable connection interval (20ms)

BLEService imuService(ACC_SERVICE_UUID);
BLECharacteristic cmdCharacteristic(CMD_CHARACTERISTIC_UUID, BLEWrite | BLERead, 5);
BLECharacteristic dataCharacteristic(DATA_CHARACTERISTIC_UUID, BLERead, sizeof(DataPoint));

// Battery Service
BLEService batteryService(0x180F);                                        // Battery service UUID
BLECharacteristic batteryLevelChar(0x2A19, BLERead | BLENotify, 1, true); // Battery level characteristic

bool isConnected = false;        // Track connection status status
bool restartAdvertising = false; // Flag to restart advertising after disconnect

// Variables for orientation and velocity calculation
float roll = 0.0, pitch = 0.0, yaw = 0.0;
float previousAcceleration = 0.0;
float currentVelocity = 0.0;
unsigned long lastUpdateTime = 0;
const float velocityDecayFactor = 0.95; // Faster decay for a 70kg individual

// Complementary filter constants
const float alpha = 0.98; // Weight for gyroscope data

// Constants for improved velocity calculation
#define ACCEL_THRESHOLD 0.05       // Threshold to detect movement
#define STILLNESS_DURATION 200     // Time in ms to recognize stillness
#define VELOCITY_HISTORY_SIZE 10   // Size of the velocity smoothing buffer
#define DEFAULT_STRIDE_LENGTH 0.75 // Default stride length in meters

// Variables for improved velocity calculation
float velocityHistory[VELOCITY_HISTORY_SIZE] = {0};
int velocityHistoryIndex = 0;
bool wasStill = true;
unsigned long stillStartTime = 0;
float peakAcceleration = 0;
float strideFrequency = 0;
float strideLength = DEFAULT_STRIDE_LENGTH; // Define stride length variable
unsigned long lastPeakTime = 0;
unsigned long lastZeroVelocityUpdate = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing IMU...");
  if (imu.begin() != 0)
  {
    Serial.println("Failed to initialize IMU");
    while (1)
      ;
  }
  Serial.println("IMU initialized successfully");

  Serial.println("Initializing BLE...");
  Bluefruit.begin();
  Bluefruit.setName("IMU");
  Bluefruit.setTxPower(ADVERTISING_POWER);

  // Set connection interval (how often BLE sends packets)
  // Important for reliable connection and reconnection
  Bluefruit.Periph.setConnInterval(CONN_INTERVAL_MIN, CONN_INTERVAL_MAX);

  imuService.begin();
  cmdCharacteristic.begin();
  cmdCharacteristic.setWriteCallback(commandCallback);
  dataCharacteristic.begin();

  // Setup battery service
  batteryService.begin();
  batteryLevelChar.begin();
  batteryLevelChar.write8(getBatteryLevel()); // Initialize battery level

  // Add battery service to advertising
  Bluefruit.Advertising.addService(batteryService);

  // Initialize cmdCharacteristic with the default status (idle)
  uint8_t initialStatus = 0; // 0 = idle
  cmdCharacteristic.write(&initialStatus, sizeof(initialStatus));

  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  // Configure advertising
  Bluefruit.Advertising.addService(imuService);

  // More comprehensive advertising setup for better reconnection
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();

  // Set advertising parameters for better discoveryng
  Bluefruit.Advertising.restartOnDisconnect(true);                                         // No timeout for fast advertising
  Bluefruit.Advertising.setInterval(ADVERTISING_INTERVAL_MS, ADVERTISING_INTERVAL_MS * 2); // Fast advertisingimeout (continuous advertising)
  Bluefruit.Advertising.setFastTimeout(0);                                                 // No timeout for fast advertising

  Bluefruit.Advertising.start();
  Serial.println("BLE initialized and advertising started");
}

void loop()
{
  // Check if we need to restart advertising (after disconnection)
  if (restartAdvertising && !Bluefruit.Advertising.isRunning())
  {
    Serial.println("Restarting advertising to improve reconnection...");
    Bluefruit.Advertising.start();
    restartAdvertising = false;
  }

  if (isRecording)
  {
    if (dataIndex < MAX_DATA_POINTS)
    {
      unsigned long currentTime = millis();
      float dt = (currentTime - lastUpdateTime) / 1000.0; // Time delta in seconds
      lastUpdateTime = currentTime;

      // Read accelerometer and gyroscope data
      float accelX = imu.readFloatAccelX();
      float accelY = imu.readFloatAccelY();
      float accelZ = imu.readFloatAccelZ();
      float gyroX = imu.readFloatGyroX();
      float gyroY = imu.readFloatGyroY();
      float gyroZ = imu.readFloatGyroZ();

      // Calculate orientation using complementary filter
      float accelPitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / M_PI;
      float accelRoll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / M_PI;

      pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * accelPitch;
      roll = alpha * (roll + gyroX * dt) + (1 - alpha) * accelRoll;
      yaw += gyroZ * dt;

      // Calculate global acceleration
      float accelForward = accelX * cos(pitch * M_PI / 180.0) +
                           accelY * sin(roll * M_PI / 180.0) +
                           accelZ * cos(roll * M_PI / 180.0);

      // Calculate the magnitude of acceleration
      float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

      // Apply a high-pass filter to remove low-frequency noise
      static float previousAccelMagnitude = 0.0;
      float filteredAccelMagnitude = accelMagnitude - previousAccelMagnitude;
      previousAccelMagnitude = accelMagnitude;

      // Track peak acceleration for stride frequency calculation
      if (filteredAccelMagnitude > peakAcceleration)
      {
        peakAcceleration = filteredAccelMagnitude;
      }

      // Peak detection for stride frequency calculation
      if (filteredAccelMagnitude > ACCEL_THRESHOLD && previousAccelMagnitude <= ACCEL_THRESHOLD)
      {
        if (lastPeakTime > 0)
        {
          float timeBetweenPeaks = (currentTime - lastPeakTime) / 1000.0; // in seconds
          if (timeBetweenPeaks > 0.2 && timeBetweenPeaks < 2.0)
          {                                           // Valid stride time range
            strideFrequency = 1.0 / timeBetweenPeaks; // Hz
          }
        }
        lastPeakTime = currentTime;
        peakAcceleration = 0; // Reset peak for next stride
      }

      // Zero-velocity update detection
      bool isStill = abs(filteredAccelMagnitude) < ACCEL_THRESHOLD;

      if (isStill)
      {
        if (!wasStill)
        {
          // Transition to stillness
          stillStartTime = currentTime;
          wasStill = true;
        }
        else if (currentTime - stillStartTime > STILLNESS_DURATION)
        {
          // Apply zero-velocity update after sustained stillness
          if (currentTime - lastZeroVelocityUpdate > 1000)
          {                         // Limit ZUPTs to once per second
            currentVelocity *= 0.5; // Gradual decay during ZUPT
            if (currentVelocity < 0.05)
            {
              currentVelocity = 0.0;
            }
            lastZeroVelocityUpdate = currentTime;
          }
        }
      }
      else
      {
        wasStill = false;
      }

      // Use stride information when available
      if (strideFrequency > 0 && strideLength > 0)
      {
        float strideVelocity = strideFrequency * strideLength;
        // Blend with accelerometer-based velocity
        if (strideVelocity > 0)
        {
          currentVelocity = currentVelocity * 0.7 + strideVelocity * 0.3;
        }
      }

      // Process filtered acceleration for forward motion only
      if (filteredAccelMagnitude > 0.0)
      {
        // Integrate filtered acceleration with adaptive gain
        float adaptiveGain = map(filteredAccelMagnitude, 0, 2.0, 0.5, 1.0);
        currentVelocity += filteredAccelMagnitude * dt * adaptiveGain;
      }

      // Apply velocity decay with dynamic factor based on acceleration
      float dynamicDecayFactor = map(filteredAccelMagnitude, 0, 1.0, 0.93, 0.98);
      currentVelocity *= dynamicDecayFactor;

      // Ensure velocity doesn't go negative
      if (currentVelocity < 0.0)
      {
        currentVelocity = 0.0;
      }

      // Apply smoothing using a moving average
      velocityHistory[velocityHistoryIndex] = currentVelocity;
      velocityHistoryIndex = (velocityHistoryIndex + 1) % VELOCITY_HISTORY_SIZE;

      float smoothedVelocity = 0;
      for (int i = 0; i < VELOCITY_HISTORY_SIZE; i++)
      {
        smoothedVelocity += velocityHistory[i];
      }
      smoothedVelocity /= VELOCITY_HISTORY_SIZE;
      currentVelocity = smoothedVelocity;

      // Store the velocity in the data buffer
      dataBuffer[dataIndex++] = {currentTime, currentVelocity};
      Serial.print("Velocity: ");
      Serial.print(currentVelocity);
      Serial.println(" m/s");
      delay(50); // Sampling rate of 20Hz
    }
    else
    {
      Serial.println("Data buffer full, stopping recording");
      stopRecording();
    }
  }

  // Update battery level periodically (every 30 seconds)
  static unsigned long lastBatteryUpdate = 0;
  if (millis() - lastBatteryUpdate > 30000)
  {
    uint8_t batteryLevel = getBatteryLevel();
    batteryLevelChar.write8(batteryLevel);
    Serial.print("Battery level updated: ");
    Serial.print(batteryLevel);
    Serial.println("%");
    lastBatteryUpdate = millis();

    // Make sure advertising is running for reconnection
    if (!isConnected && !Bluefruit.Advertising.isRunning())
    {
      Serial.println("Restarting advertising during battery update...");
      Bluefruit.Advertising.start();
    }
  }

  // If disconnected and buffer is full, wait for reconnection to upload data
  if (!isConnected && dataIndex == MAX_DATA_POINTS)
  {
    Serial.println("Buffer full, waiting for reconnection to upload data...");
    delay(1000); // Avoid busy-waiting
  }
}

// Reset velocity calculation parameters
void resetVelocityCalculation()
{
  currentVelocity = 0.0;
  wasStill = true;
  stillStartTime = 0;
  peakAcceleration = 0;
  strideFrequency = 0;
  lastPeakTime = 0;
  lastZeroVelocityUpdate = 0;

  for (int i = 0; i < VELOCITY_HISTORY_SIZE; i++)
  {
    velocityHistory[i] = 0;
  }
  velocityHistoryIndex = 0;
}

void sendDeviceStatus()
{
  uint8_t status = isRecording ? 1 : 0; // 1 if recording, 0 otherwise
  cmdCharacteristic.write(&status, sizeof(status));
}

void sendBufferSize()
{
  uint16_t bufferSize = dataIndex; // Number of data points in the buffer
  Serial.print("Sending buffer size: ");
  Serial.println(bufferSize);

  // Ensure the buffer size is sent reliably
  bool success = cmdCharacteristic.write((uint8_t *)&bufferSize, sizeof(bufferSize));
  if (!success)
  {
    Serial.println("Failed to send buffer size.");
  }
  delay(50); // Allow time for the browser to read the buffer size
}

void sendAllDataInChunks()
{
  if (dataIndex == 0)
  {
    dataCharacteristic.write((uint8_t *)"", 0); // No data
    return;
  }

  const size_t chunkSize = 20; // Each chunk contains 1 data point (4 bytes timestamp + 4 bytes velocity)
  uint8_t buffer[chunkSize];

  for (int i = 0; i < dataIndex; i++)
  {
    uint32_t timestamp = dataBuffer[i].timestamp;
    float velocity = dataBuffer[i].velocity;

    memcpy(buffer, &timestamp, sizeof(timestamp));
    memcpy(buffer + sizeof(timestamp), &velocity, sizeof(velocity));

    dataCharacteristic.write(buffer, chunkSize);
    delay(10); // Small delay to avoid overwhelming BLE
  }

  // Send an empty chunk as an end-of-data marker
  dataCharacteristic.write((uint8_t *)"", 0);
}

void commandCallback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
  if (len < 1)
    return;
  uint8_t command = data[0];
  Serial.print("Received command: ");
  Serial.println(command, HEX);
  switch (command)
  {
  case CMD_START:
    startRecording();
    break;
  case CMD_STOP:
    stopRecording();
    break;
  case CMD_DOWNLOAD:
    sendBufferSize(); // Send the buffer size before starting the download
    sendAllDataInChunks();
    break;
  default:
    Serial.println("Unknown command received");
    break;
  }
}

void startRecording()
{
  isRecording = true;
  dataIndex = 0;
  resetVelocityCalculation();
  roll = pitch = yaw = 0.0;
  lastUpdateTime = millis();
  sendDeviceStatus(); // Notify browser of status change
  Serial.println("Recording started");
}

void stopRecording()
{
  isRecording = false;
  sendDeviceStatus(); // Notify browser of status change
  Serial.println("Recording stopped");
}

void onConnect(uint16_t conn_handle)
{
  isConnected = true;
  Serial.println("Device connected");

  // Send the current battery level on connection
  uint8_t batteryLevel = getBatteryLevel();
  batteryLevelChar.write8(batteryLevel);
  Serial.print("Battery level sent on connection: ");
  Serial.print(batteryLevel);
  Serial.println("%");

  // If buffer is full, upload data to the browser
  if (dataIndex == MAX_DATA_POINTS)
  {
    Serial.println("Uploading buffered data to browser...");
    sendAllDataInChunks();
  }
}

void onDisconnect(uint16_t conn_handle, uint8_t reason)
{
  isConnected = false;
  Serial.print("Device disconnected. Reason: 0x");
  Serial.println(reason, HEX);

  // Set flag to restart advertising in the main loop
  restartAdvertising = true;

  // Start fast advertising immediately to improve reconnection
  if (!Bluefruit.Advertising.isRunning())
  {
    Serial.println("Restarting advertising after disconnect...");
    Bluefruit.Advertising.start();
  }
}

uint8_t getBatteryLevel()
{
  // Read the battery voltage from PIN_VBAT
  int vbat_raw = analogRead(PIN_VBAT);

  // Convert raw value to millivolts
  float vbat_mv = vbat_raw * VBAT_MV_PER_LSB * VBAT_DIVIDER;

  // Convert millivolts to battery percentage (3.0V-4.2V range for LiPo)
  // 4.2V = 100%, 3.0V = 0%
  float battery_level = map(vbat_mv, 3000, 4200, 0, 100);

  // Constrain the percentage to 0-100 range
  battery_level = constrain(battery_level, 0, 100);

  return (uint8_t)battery_level;
}

// Helper function to map float values
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}