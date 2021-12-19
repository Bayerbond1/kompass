void update_gyro()
{   
    //read x
    xyzFloat gyr = myIMU.getGyrValues();
    gyro_x = gyr.x*DEG_TO_RAD;
    gyro_y = gyr.y*DEG_TO_RAD;
    gyro_z = gyr.z*DEG_TO_RAD;

    //gyro_x=0;
    //gyro_y=0;
    //gyro_z=0;

}

void gyro_correct_offset()
{
  //gyro_x -= gyro_x_init;
  //gyro_y -= gyro_y_init;
  //gyro_z -= gyro_z_init;
}

void update_gyro_angle()
{
  gyro_correct_offset();
  
  timer+=delta_t;
  //roll_gyro, pitch_gyro, yaw-gyro: rotation estimate by gyro
  //gyro_x, ...: omega around x-axis
  //Orientation_Gyro: matrix for orientation
  BLA::Matrix<3,3> delta_rot = RotMatFromAngle(gyro_x*delta_t/1000000, gyro_y*delta_t/1000000, gyro_z*delta_t/1000000);

  Orientation_Gyro = Orientation_Gyro*delta_rot;  //=(Orientation_Gyro*delta_rot*Orientation_Gyro.inverse())*Orientation_Gyro, drehung um fundamentale Achse
}