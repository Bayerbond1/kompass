// André Kuhlmann @KuhlTime
// kuhlti.me
// 26.09.2019

#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <ICM_20948.h>
#include <FastLED.h>
#include <Esp.h>
#include <EEPROM.h>
#include <Wifi.h>
#include <ArduinoOTA.h>

// CONFIG
// ------------------------------------------------------------------------------------------
// general
#define VERSION "1.0.0"
#define INTERVAL 30

// storage
// (4 Byte per Float) 4*6
#define EEPROM_SIZE 24

// wifi
#define SSID      "24.ERROR"
#define PASSWORD  "41454307339034761748"

// ota
#define OTA           true
#define OTA_DURATION  10000

// serial
#define BAUDRATE  115200
#define VERBOSE   true

// imu
float offset[3] = {-20.93, 6.75, 18.9};
float scale[3] = {0.9875070982396364, 1.0351190476190477, 0.9791666666666667};

// components
#define NEOPIXEL_PIN  13
#define NUMPIXELS     12
#define BRIGHTNESS    10
#define BUTTONPIN     A5  // Should corrospond to Pin 12 on the Lolin Lite

// bluetooth
#define DEVICE_NAME   "Navitron"
#define SERVICE_UUID  "60b8cb4f-49c8-44a9-8276-2b150f4303a3"
#define CHAR_UUID     "bd8a4cd3-e0f4-47d9-aa5b-27389e0e4c20"
// ------------------------------------------------------------------------------------------


// VARIABLES
// ------------------------------------------------------------------------------------------
// general
#define PI 3.14159

// ota
unsigned long timer = 0;
bool processOta = OTA;

// components
ICM_20948_I2C icm;
CRGB leds[NUMPIXELS];

// icm calibration values
bool calibrated = false;
float wantedHeading = 0.0;

// bluetooth
bool doConnect = false;
bool connected = false;
bool doScan = false;

BLEUUID serviceUUID(SERVICE_UUID);
BLEUUID charUUID(CHAR_UUID);

BLEAdvertisedDevice* bleDevice;
BLERemoteCharacteristic* remoteCharacteristic;
// ------------------------------------------------------------------------------------------

// MARK: FUNCTIONS
// ------------------------------------------------------------------------------------------
void vprint(String text) {
  if (VERBOSE) {
    Serial.print(text);
  }
}

void vprintln(String text) {
  if (VERBOSE) {
    Serial.println(text);
  }
}

void allLeds(CRGB color) {
  for (size_t i=0; i < NUMPIXELS; i++) {
    leds[i] = color;
  }
}

void reboot() {
  vprintln("Detected Button Press. Restarting");

  FastLED.clear();
  allLeds(CRGB::Red);
  FastLED.show();

  sleep(2);

  FastLED.clear();
  FastLED.show();  

  ESP.restart();
}

float* extrema(float* x, float* y, float* z) {
  int dataPoints = int(sizeof(x) / sizeof(x[0]));

  float minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;

  for (size_t i=0; i < dataPoints; i++) {
    if (x[i] < minX) {
      minX = x[i];
    } else if (x[i] > maxX) {
      maxX = x[i];
    }

    if (y[i] < minY) {
      minY = y[i];
    } else if (y[i] > maxY) {
      maxY = y[i];
    }

    if (z[i] < minZ) {
      minZ = z[i];
    } else if (z[i] > maxZ) {
      maxZ = z[i];
    }
  }

  // FIXME: Outputs pointer to a localy (local to this function) stored variable
  float pOutput[6] = {minX, maxX, minY, maxY, minZ, maxZ};

  vprint("minX: ");
  vprint(String(minX));
  vprint("maxX: ");
  vprint(String(maxX));
  vprint("minY: ");
  vprint(String(minY));
  vprint("maxY: ");
  vprint(String(maxY));
  vprint("minZ: ");
  vprint(String(minZ));
  vprint("maxZ: ");
  vprintln(String(maxZ));

  return pOutput;
}

void calibrate() {
  int interval = 50;
  int dataPoints = 400;

  FastLED.clear();
  FastLED.show();

  float x[dataPoints];
  float y[dataPoints];
  float z[dataPoints];
  
  for (size_t i=0; i < dataPoints; i++) {
    icm.getAGMT();

    int led = int((float(i) / float(dataPoints)) * float(NUMPIXELS));
    vprintln(String(led));
    leds[led] = CRGB::Yellow;
    FastLED.show();

    x[i] = icm.magX();
    y[i] = icm.magY();
    z[i] = icm.magZ();

    //vprint("Captured Data Point ");
    //vprintln(String(i));
    delay(interval);
  }

  float minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;

  // get biggest and lowest value
  // new loop to have the top execution for capturing only and no more logical operations
  float* pExtreme = extrema(x, y, z);

  minX = pExtreme[0];
  maxX = pExtreme[1];
  minY = pExtreme[2];
  maxY = pExtreme[3];
  minZ = pExtreme[4];
  maxZ = pExtreme[5];

  // TODO: Offset calculation
  offset[0] = (maxX + minX) / 2;
  offset[1] = (maxY + minY) / 2;
  offset[2] = (maxZ + minZ) / 2;

  for (size_t i = 0; i < dataPoints; i++)
  {
    x[i] = x[i] - offset[0];
    y[i] = y[i] - offset[1];
    z[i] = z[i] - offset[2];
  }
  
  // redetermine the extreme values
  pExtreme = extrema(x, y, z);

  minX = pExtreme[0];
  maxX = pExtreme[1];
  minY = pExtreme[2];
  maxY = pExtreme[3];
  minZ = pExtreme[4];
  maxZ = pExtreme[5];

  // TODO: Scale calculation
  float avgDeltaAxis[3];

  avgDeltaAxis[0] = (maxX - minX) / 2;
  avgDeltaAxis[1] = (maxY - minY) / 2;
  avgDeltaAxis[2] = (maxZ - minZ) / 2;

  float avgDelta = (avgDeltaAxis[0] + avgDeltaAxis[1] + avgDeltaAxis[2]) / 3;

  scale[0] = avgDelta / avgDeltaAxis[0];
  scale[1] = avgDelta / avgDeltaAxis[1];
  scale[2] = avgDelta / avgDeltaAxis[2];

  // TODO: Store Values in EEPROM

  FastLED.clear();
  FastLED.show();
}

// returns the heading without titl compensation
float noTiltCompensationHeading(float mag[3]) {
  // heading value -179 to 180
  float heading = atan2(-mag[1], mag[0]);

  // correct heading for values from 0 to 359
  if (heading < 0) {
    heading = 2*PI + heading;
  }

  return heading;
}
// ------------------------------------------------------------------------------------------


// MARK: WiFi
// ------------------------------------------------------------------------------------------
void initializeWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    vprintln("Connection to WiFi Wailed!");
  }
}
// ------------------------------------------------------------------------------------------


// MARK: OTA
// ------------------------------------------------------------------------------------------
void initializeOTA() {
  ArduinoOTA.setHostname("Navitron");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";
      
      vprintln("Start updating " + type);
    })
    .onEnd([]() {
      vprintln("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      if (VERBOSE)
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      if (VERBOSE) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)    Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)  Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)  Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)      Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();
}
// ------------------------------------------------------------------------------------------


// MARK: BLE
// ------------------------------------------------------------------------------------------
class MyClientCallback: public BLEClientCallbacks {
  
  // MARK: Connected
  void onConnect(BLEClient* client) {
      vprintln("Connected to BLE Server");
  }

  // MARK: Disconnected
  void onDisconnect(BLEClient* client) {
    connected = false;
    vprintln("Disconnected from BLE Server");
  }
};

class MyAdvertisedDeviceCallback: public BLEAdvertisedDeviceCallbacks {
  // This fuction gets called for each advertising BLE server
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    vprintln(advertisedDevice.toString().c_str());

    // When the device matches the one we are searching for
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      bleDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;
    }
  }
};

// Handles recieved BLE Data
void notifyCallback(
  BLERemoteCharacteristic* remoteBLECharacteristic,
  uint8_t* data,
  size_t length,
  bool isNotify
) {

  // logging
  vprint("Notify callback for charactersitc: ");
  vprint(remoteBLECharacteristic->getUUID().toString().c_str());
  vprint(" of data length ");
  vprintln(String(length));
  vprint("Data: ");
  vprintln(String((char *) data));

  // handeling data
  String message = (char *) data;
  wantedHeading = message.toFloat();
}

bool connectToServer() {
  vprint("Creating a connection to ");
  vprintln(bleDevice->getAddress().toString().c_str());

  BLEClient* client = BLEDevice::createClient();
  client->setClientCallbacks(new MyClientCallback());
  client->connect(bleDevice);

  BLERemoteService* remoteService = client->getService(serviceUUID);

  // failed to find service with the set serviceUUID on the server
  if (remoteService == nullptr) {
    vprint("Failed to find service on server. ServiceUUID: ");
    vprintln(serviceUUID.toString().c_str());

    client->disconnect();
    return false;
  }

  remoteCharacteristic = remoteService->getCharacteristic(charUUID);

  // failed to find characteristic uuid on the server
  if (remoteCharacteristic == nullptr) {
    vprint("Failed to find the characterstic UUID on the server. CharacteristicUUID: ");
    vprintln(charUUID.toString().c_str());

    client->disconnect();
    return false;
  }

  if (remoteCharacteristic->canRead()) {
    String value = remoteCharacteristic->readValue().c_str();
    vprint("The characterstic value is: ");
    vprintln(value);
  }

  // connect the characteristic with notification handler function
  if (remoteCharacteristic->canNotify()) {
    remoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}
// ------------------------------------------------------------------------------------------

// MARK: SETUP
// ------------------------------------------------------------------------------------------
void setup() {

  // MARK: Serial Port
  // when the verbose flag is enabled start the serial port
  if (VERBOSE) {
    Serial.begin(BAUDRATE);

    // wait port is opened
    while (!Serial);
  }

  vprint("Navitron v");
  vprintln(VERSION);

  // MARK: OTA
  if (OTA) {
    initializeWifi();
  }

  // MARK: Storage
  EEPROM.begin(EEPROM_SIZE);

  // MARK: ICM-20948
  Wire.begin();
  Wire.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    icm.begin(Wire, 1);

    vprint("ICM-Status: ");
    vprintln(icm.statusString());

    if (icm.status != ICM_20948_Stat_Ok) {
      vprintln("Trying to reinit the IMU.");
      delay(500);
    } else {
      initialized = true;
    }
  }

  // MARK: FastLED
  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds, NUMPIXELS);
  FastLED.setBrightness(BRIGHTNESS);

  // MARK: Button
  pinMode(BUTTONPIN, INPUT);

  // Button Pressed on Boot
  if (digitalRead(BUTTONPIN) == HIGH) {
    vprintln("Starting Calibration process!");
    calibrate();
  }

  // MARK: BLE
  BLEDevice::init(DEVICE_NAME);

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallback());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}
// ------------------------------------------------------------------------------------------


// LOOP
// ------------------------------------------------------------------------------------------
void loop() {
  if (timer == 0) {
    timer = millis();
  }

  // MARK: OTA
  // run the handle function for OTA_DURATION
  if (OTA && processOta) {
    unsigned int ellapsedTime = millis() - timer;

    if (ellapsedTime >= OTA_DURATION) {
      processOta = false;
    } else {
      ArduinoOTA.handle();
      return;
    }
  }

  // MARK: Button
  // watch the button state
  if (digitalRead(BUTTONPIN) == HIGH) {
    reboot();
  }

  // MARK: BLE
  if (doConnect) {
    if (connectToServer()) {
      vprintln("Connected to the Peripheral");
    } else {
      vprintln("Failed to connect to the Peripheral");
    }
    doConnect = false;
  }

  // MARK: Processing
  if (icm.dataReady()) {
    // updates the values
    icm.getAGMT();

    float rawMag[3] = {icm.magX(), icm.magY(), icm.magZ()};
    float mag[3];
    
    // correct magenetometer data
    for (int i=0; i < 3; i++) {
      mag[i] = (rawMag[i] - offset[i]) * scale[i];
    }

    float headingDeg = noTiltCompensationHeading(mag) * 180 / PI;
    float headingDifference = wantedHeading - headingDeg;

    if (headingDifference > 180.0) {
      headingDifference -= 360.0;
    } else if (headingDifference <= -180) {
      headingDifference += 360;
    }

    // calculate led
    // right or left turn
    // TODO: Recheck math. Little off sometimes
    int led = round(sqrt(pow(NUMPIXELS * headingDifference / 360.0, 2)));
    // left turn
    if (headingDifference < 0) {
      led = round(NUMPIXELS - led) - 1;
    }

    // set led
    FastLED.clear();
    leds[led] = CRGB::Cyan;
    FastLED.show();

    vprint(String(headingDifference));
    vprint(" - led: ");
    vprintln(String(led));
    delay(INTERVAL);
  }
}
// ------------------------------------------------------------------------------------------