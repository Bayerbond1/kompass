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
