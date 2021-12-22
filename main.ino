/*
 * measures Angle using gy-80
 * attach g_int2 to A0
 * gyro and acc
 * output: yaw, pitch, roll, as in an aircraft, also rotations matrix possible (converts from north-west-up to x,y,z of sensor)
 */
#include <Wire.h>
#include <ICM20948_WE.h>
#include <FastLED.h>
#include <BasicLinearAlgebra.h>
#include "BLEDevice.h"
using namespace BLA;

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24
#define MATLAB 0  //use with rotation_test.m
#define ANGLE 1
#define GYRO 0
#define ACC 0
#define TIMING 0
#define POSITION 0
#define VELOCITY 0
#define KOMPASS 1
#define ORIANTATION_MATRIX 0
#define PYTHON 0
#define DELAYS 0

#define DEBUG 0

// LEDs
#define AD0_VAL 1
#define LED_DATA 18
#define NUM_LEDS 16
#define BRIGHTNESS 5
CRGB leds[NUM_LEDS];

#define ICM20948_ADDR 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// BLUETOOTH
static BLEUUID serviceUUID("60B8CB4F-49C8-44A9-8276-2B150F4303A3");
static BLEUUID    charUUID("BD8A4CD3-E0F4-47D9-AA5B-27389E0E4C20");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

// VARIABLES
float wantedHeading = 0.0;
float gyro_x=0, gyro_y=0, gyro_z=0;  //value of gyro
float gyro_x_init, gyro_y_init, gyro_z_init;   //offset of gyro
float roll=0, pitch=0, yaw=0;  //rotation estimate by gyro
float X_out, Y_out, Z_out;  //acc in g
float acc_cal;
BLA::Matrix<3, 3> Orientation_Gyro = {1,0,0,  //combined oriantation represented by matrix (each colum is unit vector of drone in earth coordinate system), converts from drone to earth
                                      0,1,0,
                                      0,0,1};
BLA::Matrix<3, 1> gravity_axis; //used in print
BLA::Matrix<3, 1> z_axis = {0,  //unitvector, z-axis, pointing up
                              0,
                              1};
BLA::Matrix<3, 1> rotation_axis;
BLA::Matrix<3, 1> pos = {0,0,0};
BLA::Matrix<3, 1> vel = {0,0,0};
//BLA::Matrix<3, 1> Acc_z_axis = {X_out,  //columvector of gravity
//                                Y_out,
//                                Z_out};
//int minX = -50; int minY = -11; int minZ = -38; int maxX =  19; int maxY = 55; int maxZ = 24;BLA::Matrix<3, 1> mag_vector;
//corected for differend orientation
//int minX = -50; int minY = -55; int minZ = -24; int maxX =  19; int maxY = 11; int maxZ = 38; //red with external compass

//int minX = -47; int minY = -46; int minZ = -71; int maxX =  40; int maxY = 44; int maxZ = 22;
int minX = -47; int minY = -44; int minZ = -22; int maxX =  40; int maxY = 46; int maxZ = 71;

BLA::Matrix<3, 1> mag_vector;
BLA::Matrix<3, 1> north = {0,0,0};
float magnitute=1;
float x,y,z; //triple axis data, temp move to update
float x_norm, y_norm, z_norm;
float yaw_compass_error = 0;
float yaw_compass;
float acc_error_angle; //angle between z axis and accaleration

long steps = micros();
long last_acc=steps;
long last_disp = steps;
long last_gyro = steps;
long last_mag = steps;
long last_led =steps;
float delta_t_acc = 0;
int delta_t=20; //time between measuments in ms
long timer = 0;
int disp = 0; //aid display function

volatile byte new_gyro_num = 0;

// MARK: - BLUETOOTH

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

// -----------------

// MARK: - FUNCTIONS

void sensorLoop() {
  //wait for new data
  while(!new_gyro_num) {
    check_data();
  }

  myIMU.readSensor();

  new_gyro_num=0;
  update_gyro();
  update_gyro_angle();
    
  //calc combined angle/read acc
  if(micros()-last_acc>10000)
  {
    delta_t_acc = (float)(micros()-last_acc)/1000000;
    last_acc+=10000;
    read_acc();
    update_acc(0.01);
  }

  //read magnetometer
  if(micros()-last_mag>=50000){
    last_mag+=50000;
    magnometor_upadate(0.05);  //c.. complementrary factor, c=0.005,-> tau=10s, 1-c=exp(-Ts/tau), tau vorgeben, c berechnen
  }

  //update led
  if(micros()-last_led>=50000){
    last_led+=50000;
    update_leds();  //c.. complementrary factor, c=0.005,-> tau=10s, 1-c=exp(-Ts/tau), tau vorgeben, c berechnen
  }
  
  eular_angels_form_rotation_matrix(Orientation_Gyro);
  //update controller / display msg
  const long plot_delay = 500000;
  if(micros()-last_disp>=plot_delay){
    last_disp += plot_delay;
    disp_values();
  }
}

void check_data() //check if the interrupt for new Data is high
{
    /*delayMicroseconds(100); //interrupt
   if(analogRead(A0)>256)
   {
    new_gyro_data_avalible();
   }*/
   //Serial.print("1 ");
   if(last_gyro<micros()-8192)  //time based
   {
    last_gyro+=8192;
    new_gyro_data_avalible();
   }
   delayMicroseconds(100);
}

void new_gyro_data_avalible() //trigger this, when the interrupt happens
{
  new_gyro_num=1;
  delta_t=micros()-steps;
  steps=micros();
}

// -----------------

void setup(){
  Wire.begin();
  Serial.begin(115200);

  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }
  else{
    Serial.println("ICM20948 is connected");
  }
  
  //setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  init_acc();
  init_HMC5883();
  calibrate();
  //init_orientation();
  read_acc();
  update_acc(1);
  delay(100);
  read_acc();
  update_acc(1);
  magnometor_upadate(1);
    
  steps = micros();
  last_acc=steps;
  last_disp = steps;
  last_gyro = steps;
  last_mag=steps;
  last_led=steps;
  timer=0;
  eular_angels_form_rotation_matrix(Orientation_Gyro);
  disp_values();
  FastLED.addLeds<WS2812B, LED_DATA, GRB>(leds, NUM_LEDS); // RGB, GRB, BGR

  // BLUETOOTH
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  if (connected) {
    sensorLoop();
  }
}
