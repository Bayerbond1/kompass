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

//LEDs
#define AD0_VAL 1
#define LED_DATA 18
#define NUM_LEDS 16
#define BRIGHTNESS 5
CRGB leds[NUM_LEDS];

#define ICM20948_ADDR 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// bluetooth
#define DEVICE_NAME   "Navitron"
#define SERVICE_UUID  "60b8cb4f-49c8-44a9-8276-2b150f4303a3"
#define CHAR_UUID     "bd8a4cd3-e0f4-47d9-aa5b-27389e0e4c20"

bool doConnect = false;
bool connected = false;
bool doScan = false;

BLEUUID serviceUUID(SERVICE_UUID);
BLEUUID charUUID(CHAR_UUID);

BLEAdvertisedDevice* bleDevice;
BLERemoteCharacteristic* remoteCharacteristic;


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

  // MARK: BLE
  BLEDevice::init(DEVICE_NAME);

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallback());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);

    
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
}

void loop() {
  // MARK: BLE
  if (doConnect) {
    if (connectToServer()) {
      vprintln("Connected to the Peripheral");
    } else {
      vprintln("Failed to connect to the Peripheral");
    }
    doConnect = false;
  }

  while(!new_gyro_num){check_data();}  //wait for new data

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
