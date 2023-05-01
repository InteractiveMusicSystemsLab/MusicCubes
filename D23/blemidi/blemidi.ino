/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This example requires following library
 * - https://github.com/FortySevenEffects/arduino_midi_library/ (MIDI Library)
 *
 * For BLE MIDI Setup on PC/mobile checkout our learn guide
 * - https://learn.adafruit.com/wireless-untztrument-using-ble-midi/ble-midi-setup
 */

#include <Arduino.h>
#include <bluefruit.h>
#include <MIDI.h>
#include <LSM6DS3.h>
#include <MadgwickAHRS.h>
#include <base64.hpp>

#define CC_YAW 0
#define CC_PITCH 1
#define CC_ROLL 2
#define CC_CHANGE_CHANNEL 55
#define MIDI_DICE 59
#define SYSEX_BUFFER_SIZE 1024

BLEDis bledis;
BLEMidi blemidi;
LSM6DS3 imu(I2C_MODE, 0x6A);
BLEUart bleuart;
// initialize a Madgwick filter:
Madgwick filter;
// sensor's sample rate is fixed at:
const int sensorRate = 104;
const float gyroThresh = 0.15;
const float accelThresh = 0.013;
const int initialDetectTime = 1000; // time to allow CC signals from other cubes to prevent interference.
int channel = 2;  // this can change if interference detected
unsigned long microsPerReading, microsPrevious;

// Create a new instance of the Arduino MIDI Library,
// and attach BluefruitLE MIDI as the transport.
MIDI_CREATE_BLE_INSTANCE(blemidi);

void setup() {
  Serial.begin(115200);

  unsigned long start_time = millis();
  while (!Serial && (millis() - start_time < 1000)) {
    delay(10);
  }

  // Generate a unique Base64-encoded string based on device's UID
  uint8_t uid[16];
  memcpy(uid, (const void *)NRF_FICR->DEVICEADDR, sizeof(uid));
  unsigned char unique_string[11];
  encode_base64(uid, sizeof(uid), unique_string);
  unique_string[9] = '\0';
  unique_string[10] = '\0';
  // Print the unique string
  Serial.println((char *)unique_string);

  Serial.println("Adafruit Bluefruit52 MIDI over Bluetooth LE Example");

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);  // Check bluefruit.h for supported values

  // Setup the on board blue LED to be enabled on CONNECT
  Bluefruit.autoConnLed(true);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Initialize MIDI, and listen to all MIDI channels
  // This will also call blemidi service's begin()
  MIDI.begin(MIDI_CHANNEL_OMNI);

  // Attach the handleNoteOn function to the MIDI Library. It will
  // be called whenever the Bluefruit receives MIDI Note On messages.
  MIDI.setHandleNoteOn(handleNoteOn);

  // Do the same for MIDI Note Off messages.
  MIDI.setHandleNoteOff(handleNoteOff);

  // MIDI.setHandleControlChange(handleControlChange);


  // Set up and start advertising
  startAdv((char *)unique_string);

  // Start MIDI read loop
  Scheduler.startLoop(midiRead);

  imu.settings.gyroEnabled = 1;              // Can be 0 or 1
  imu.settings.gyroRange = 2000;             // Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  imu.settings.gyroSampleRate = sensorRate;  // Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  imu.settings.gyroBandWidth = 200;          // Hz.  Can be: 50, 100, 200, 400;
  // imu.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  // imu.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 2;                // Max G force readable.  Can be: 2, 4, 8, 16
  imu.settings.accelSampleRate = sensorRate;  // Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  imu.settings.accelBandWidth = 200;          // Hz.  Can be: 50, 100, 200, 400;
  // imu.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  // imu.settings.accelFifoDecimation = 1;  //set 1 for on /1
  imu.settings.tempEnabled = 1;

  imu.begin();

  filter.begin(sensorRate);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / sensorRate;
  microsPrevious = micros();
}

void startAdv(char *unique_string) {

  char name[30];
  strcpy(name, "Cube ");
  strcat(name, unique_string);

  Bluefruit.setName(name);
  // Set General Discoverable Mode flag
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);

  // Advertise TX Power
  Bluefruit.Advertising.addTxPower();

  // Advertise BLE MIDI Service
  Bluefruit.Advertising.addService(blemidi);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
	 * - Enable auto advertising if disconnected
	 * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
	 * - Timeout for fast mode is 30 seconds
	 * - Start(timeout) with timeout = 0 will advertise forever (until connected)
	 *
	 * For recommended advertising interval
	 * https://developer.apple.com/library/content/qa/qa1931/_index.html
	 */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}

void handleNoteOn(byte channel, byte pitch, byte velocity) {
  // Log when a note is pressed.
  // Serial.printf("Note on: channel = %d, pitch = %d, velocity - %d", channel, pitch, velocity);
  // Serial.println();
}

void handleNoteOff(byte channel, byte pitch, byte velocity) {
  // Log when a note is released.
  // Serial.printf("Note off: channel = %d, pitch = %d, velocity - %d", channel, pitch, velocity);
  // Serial.println();
}

void handleControlChange(byte ccChannel, byte control, byte value) {
  Serial.printf("cc: channel = %d, pitch = %d, velocity - %d", ccChannel, control, value);
  Serial.println();
  if (control == CC_CHANGE_CHANNEL) {
    channel = ccChannel;
  }
}

// Set up filter coefficients (adjust as needed)
float alpha = 1;
float dt = 0.005;

float heading = 0;
float pitch = 0;
float roll = 0;

int prevOutputH, prevOutputP, prevOutputR;
int outputH = 64;
int outputP = 64;
int outputR = 64;

// Variables to store the previous readings.
float prev_ax = 0, prev_ay = 0, prev_az = 0;
float prev_gx = 0, prev_gy = 0, prev_gz = 0;

int stillCounter = 0;
int movingCounter = 0;
bool isMoving;
int lastDice = 0;
bool sendingMidi = false;
uint32_t lastNotConnectedTime = 0;
void loop() {
  // Don't continue if we aren't connected.
  if (!Bluefruit.connected()) {
    lastNotConnectedTime = millis();
    return;
  }

  // Don't continue if the connected device isn't ready to receive messages.
  if (!blemidi.notifyEnabled()) {
    return;
  }
  unsigned long microsNow;

  float ax, ay, az;  // Acceleration readings
  float gx, gy, gz;  // Gyroscope readings

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (!sendingMidi && millis() - lastNotConnectedTime > initialDetectTime) {
    sendingMidi = true;
    Serial.println("NOW SENDING MIDI");
  }

  // check if the IMU is ready to read:
  if (microsNow - microsPrevious >= microsPerReading) {
    // read accelerometer & gyrometer:
    ax = imu.readFloatAccelX();
    ay = imu.readFloatAccelY();
    az = imu.readFloatAccelZ();
    gx = imu.readFloatGyroX();
    gy = imu.readFloatGyroY();
    gz = imu.readFloatGyroZ();

    // update the filter, which computes orientation:
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;

    // Calculate the change in acceleration and angular velocity.
    float delta_ax = absolute_difference(ax, prev_ax);
    float delta_ay = absolute_difference(ay, prev_ay);
    float delta_az = absolute_difference(az, prev_az);
    float delta_gx = absolute_difference(gx, prev_gx);
    float delta_gy = absolute_difference(gy, prev_gy);
    float delta_gz = absolute_difference(gz, prev_gz);

    // Update the previous values.
    prev_ax = ax;
    prev_ay = ay;
    prev_az = az;
    prev_gx = gx;
    prev_gy = gy;
    prev_gz = gz;

    // Calculate the magnitudes of the change in acceleration and angular velocity.
    float delta_accel_magnitude = magnitude(delta_ax, delta_ay, delta_az);
    float delta_gyro_magnitude = magnitude(delta_gx, delta_gy, delta_gz);

    bool nowMoving = false;
    if (delta_accel_magnitude < accelThresh && delta_gyro_magnitude < gyroThresh) {
      nowMoving = false;
    } else {
      nowMoving = true;
    }

    if (nowMoving) {
      stillCounter = 0;
      movingCounter++;
    } else {
      movingCounter = 0;
      stillCounter++;
    }

    if (stillCounter > 10 && isMoving) {
      isMoving = false;
      movingCounter = 0;
      // Serial.println("STILL");
      int pitch = MIDI_DICE + calculateDice(ax, ay, az);
      if (sendingMidi && pitch != lastDice) {
        MIDI.sendNoteOn(pitch, 60, channel);
        lastDice = pitch;
      }
      
    } else if (movingCounter > 10 && !isMoving) {
      isMoving = true;
      stillCounter = 0;
      Serial.println("MOVING");
    }

    outputH = (int)((heading / 180.0 + 1) / 2 * 127);
    outputR = (int)((roll / 180.0 + 1) / 2 * 127);
    outputP = (int)((pitch / 180.0 + 1) / 2 * 127);
    if (isMoving && sendingMidi) {
      if (lastDice != 0) {
        MIDI.sendNoteOff(lastDice, 0, channel);
        lastDice = 0;
      }
      if (outputH != prevOutputH) {
        prevOutputH = outputH;
        MIDI.sendControlChange(CC_YAW, (int)outputH, channel);
      }
      if (outputR != prevOutputR) {
        prevOutputR = outputR;
        MIDI.sendControlChange(CC_ROLL, (int)outputR, channel);
      }
      if (outputP != prevOutputP) {
        prevOutputP = outputP;
        MIDI.sendControlChange(CC_PITCH, (int)outputP, channel);
      }
    }
  }
  // midiRead should be run as often as possible
  midiRead();
}

void midiRead() {
  // Don't continue if we aren't connected.
  if (!Bluefruit.connected()) {
    return;
  }

  // Don't continue if the connected device isn't ready to receive messages.
  if (!blemidi.notifyEnabled()) {
    return;
  }

  // read any new MIDI messages
  MIDI.read();
}

// Function to calculate the absolute difference between two values.
float absolute_difference(float a, float b) {
  return abs(a - b);
}

int calculateDice(float x, float y, float z) {
  const float threshold = 0.8;
  if (x > threshold) {
    return 1;
  } else if (x < -threshold) {
    return 6;
  } else if (y > threshold) {
    return 2;
  } else if (y < -threshold) {
    return 5;
  } else if (z > threshold) {
    return 3;
  } else if (z < -threshold) {
    return 4;
  } else {
    // In case the accelerometer is not in a stable position
    return -1;
  }
}

// Function to calculate the magnitude of a vector.
float magnitude(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}


byte sysexBuffer[SYSEX_BUFFER_SIZE];
uint16_t sysexIndex = 0;
bool sysexActive = false;

// ableton doesn't route SysEx back to other cubes so doesn't work
// void handleSysEx(byte* sysex, uint16_t length, bool complete) {
//   if (complete) {
//     // Add the last chunk of the SysEx message to the buffer
//     memcpy(&sysexBuffer[sysexIndex], sysex, length);
//     sysexIndex += length;
//     sysexActive = false;
//     // Call the function to process the complete SysEx message
//     processSysEx(sysexBuffer, sysexIndex);
//     // Reset the buffer and index for the next message
//     memset(sysexBuffer, 0, SYSEX_BUFFER_SIZE);
//     sysexIndex = 0;
//   } else {
//     // Add the current chunk of the SysEx message to the buffer
//     memcpy(&sysexBuffer[sysexIndex], sysex, length);
//     sysexIndex += length;
//     sysexActive = true;
//   }
// }

// void processSysEx(byte* sysex, unsigned int length) {
//   // Process the complete SysEx message
//   // For example, print the data to the serial monitor
//   Serial.println("SysEx message received:");
//   for (int i = 0; i < length; i++) {
//     Serial.print(sysex[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.println();
// }