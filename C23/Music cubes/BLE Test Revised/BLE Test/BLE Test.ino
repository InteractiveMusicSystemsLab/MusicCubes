#include <ArduinoBLE.h>
#include <LSM6DS3.h>
#include <Wire.h>

LSM6DS3 myIMU(I2C_MODE, 0x6A);
BLEService accelService("1101");
BLEFloatCharacteristic accelCharX("0099b53a-b473-11ed-afa1-0242ac120002", BLERead | BLENotify);
BLEFloatCharacteristic accelCharY("0099b8a0-b473-11ed-afa1-0242ac120002", BLERead | BLENotify);
BLEFloatCharacteristic accelCharZ("0099ba12-b473-11ed-afa1-0242ac120002", BLERead | BLENotify);

void setup() {
  // Serial.begin(9600);
  // while (!Serial)
  //   ;

  myIMU.begin();

  // if (myIMU.begin() != 0) {
  //   Serial.println("Device error");
  // } else {
  //   Serial.println("Device OK!");
  // }

  pinMode(LED_BUILTIN, OUTPUT);
  if (!BLE.begin()) {
    // Serial.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName("AccelerometerData");
  BLE.setAdvertisedService(accelService);
  accelService.addCharacteristic(accelCharX);
  accelService.addCharacteristic(accelCharY);
  accelService.addCharacteristic(accelCharZ);
  BLE.addService(accelService);


  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    // Serial.print("Connected to central: ");
    // Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      float accelx = myIMU.readFloatAccelX();
      float accely = myIMU.readFloatAccelY();
      float accelz = myIMU.readFloatAccelZ();
      //float accelyRaw = map(accely, 0, 1023, 0, 100);
      //float accelzRaw = map(accelz, 0, 1023, 0, 100);

      // Serial.print("Accelx = ");
      // Serial.println(accelx);
      // Serial.print("Accely = ");
      // Serial.println(accely);
      // Serial.print("Accelz = ");
      // Serial.println(accelz);
      accelCharX.writeValue(accelx);
      accelCharY.writeValue(accely);
      accelCharZ.writeValue(accelz);
      delay(200);
    }
  }

  digitalWrite(LED_BUILTIN, LOW);
  // Serial.print("Disconnected from central: ");
  // Serial.println(central.address());
}
