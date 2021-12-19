void init_acc()
{
  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  //myIMU.autoOffsets();
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setAccSampleRateDivider(0);
  //init the acc
  Serial.println("complete");
}

void calibrate()
{
  Serial.print("calibration ");
  float len = 300;
  float x=0;  //acc
  float y=0;
  float z=0;

  gyro_x_init=0;  //gyro
  gyro_y_init=0;
  gyro_z_init=0;
  
  for(int i = 0;i<len; i++)
  {
    //acc
  myIMU.readSensor();
  xyzFloat magValue = myIMU.getMagValues();
  float X=magValue.x;
  float Y=magValue.y;
  float Z=magValue.z;
  Z+=(0.0376/9.81);
  x+=X;
  y+=Y;
  z+=Z;


  //gyro
  //while(!new_gyro_num){check_data();} //not neccesary
    //new_gyro_num=0;
    update_gyro();
    
    gyro_x_init += gyro_x;
    gyro_y_init += gyro_y;
    gyro_z_init += gyro_z;
  
  delay(10);
  }
  float acc_x_init = x/len;
  float acc_y_init = y/len;
  float acc_z_init = z/len;
  acc_cal = sqrt(acc_x_init*acc_x_init+acc_y_init*acc_y_init+acc_z_init*acc_z_init);
  Serial.print(acc_cal);
  gyro_x_init /=len;
  gyro_y_init /=len;
  gyro_z_init /=len;
  
  Serial.println(" complete ");
}

//init
int setupL3G4200D(int scale){
  //gyro
  Serial.print("starting up L3G4200D");
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setGyrDLPF(ICM20948_DLPF_0);  
  Serial.println(" complete");
}

void init_orientation(){
  read_acc();
  
  BLA::Matrix<3, 1> acc_z_axis = {X_out, Y_out, Z_out}; //where is the gravity vector pointing relative to the sensor?
  
  //clac error angle
  BLA::Matrix<1, 1> absoulte = (~acc_z_axis)*acc_z_axis;
  BLA::Matrix<1, 1> angle_temp = (~acc_z_axis)*z_axis/sqrt(absoulte(0,0));  //angle between acc and z-axis (measure for error)
  acc_error_angle = acos(angle_temp(0,0));
  
  //correct error/complementary filter
  if(acc_error_angle>0.0000000001){
    //calc rotation axis
    BLA::Matrix<3, 1> rotation_axis = CrossProduct(acc_z_axis, z_axis); //order?
    float norm = 1/(sin(acc_error_angle)*sqrt(absoulte(0,0)));
    rotation_axis(0,0) = rotation_axis(0,0)*norm;
    rotation_axis(1,0) = rotation_axis(1,0)*norm;
    rotation_axis(2,0) = rotation_axis(2,0)*norm;
  
    float c=1;  //use complete acc info
    BLA::Matrix<3, 3> rotation_mat = RotMatFromAngleAndAxis(acc_error_angle*c, rotation_axis);
    Orientation_Gyro = rotation_mat;
  }
}

void init_HMC5883(){
  if (!myIMU.initMagnetometer()) {
    Serial.println("Magnetometer does not respond");
  }
  else {
    Serial.println("Magnetometer is connected");
  }
  myIMU.setMagOpMode(AK09916_CONT_MODE_100HZ);
}