//hardcoded for simplicity
//#define X_MIN -483
//#define Y_MIN -980
//#define Z_MIN -763
//#define X_MAX 659
//#define Y_MAX 136
//#define Z_MAX 111

void magnometor_upadate(float c){ //c.. complementrary factor, c=0.005,-> tau=10s, 1-c=exp(-Ts/tau), tau vorgeben, c berechnen
  myIMU.readSensor();
  xyzFloat magValue = myIMU.getMagValues();
  //Serial.println(magValue.x);

  //compass is oriented differend to acc/gyro
  x=magValue.x;
  y=-magValue.y;
  z=-magValue.z;
  
  x_norm = (float)2*(x-minX)/(maxX-minX)-1;
  y_norm = (float)2*(y-minY)/(maxY-minY)-1;
  z_norm = (float)2*(z-minZ)/(maxZ-minZ)-1;

  //x_norm = 0;
  //y_norm = 0.999;
  //z_norm = 0;
  
  //magnitute = sqrt(x_norm*x_norm + y_norm*y_norm + z_norm*z_norm);
  mag_vector = {(float)x_norm,(float)y_norm,(float)z_norm}; //mag_vector in local coordinates
  north = Orientation_Gyro*mag_vector;
  yaw_compass_error = atan2(north(1), north(0));// atan2(1,0);//
  yaw_compass = -atan2(y_norm, x_norm);
  
  //BLA::Matrix<3,1> Up = {0, 0, 1};
  //BLA::Matrix<3,1> Axis = Orientation_Gyro.Inverse()*Up;
  //BLA::Matrix<3,3> Rot_mat = RotMatFromAngleAndAxis(yaw_compass_error*c, Axis);
  BLA::Matrix<3,3> Rot_mat = RotMatFromAngle(0,0, -yaw_compass_error*c);
  Orientation_Gyro = Rot_mat*Orientation_Gyro;
}