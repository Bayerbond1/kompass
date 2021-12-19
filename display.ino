void disp_values()
{
  if(MATLAB!=0){
    Serial << Orientation_Gyro<<","<< delta_t<<","<< north;
  }
  if(ANGLE!=0){
    Serial.print(" roll: ");
    Serial.print(roll*RAD_TO_DEG);
    Serial.print(" pitch: ");
    Serial.print(pitch*RAD_TO_DEG);
    Serial.print(" yaw: ");
    Serial.print(yaw*RAD_TO_DEG);
  }
  if(GYRO!=0){
    Serial.print(" gyro_x [deg/s]: ");
    Serial.print(gyro_x*RAD_TO_DEG);
    Serial.print(" gyro_y: ");
    Serial.print(gyro_y*RAD_TO_DEG);
    Serial.print(" gyro_z: ");
    Serial.print(gyro_z*RAD_TO_DEG);
  }
  if(ACC!=0){
    Serial.print(" ACC_x: ");
    Serial.print(X_out);
    Serial.print(" ACC_y: ");
    Serial.print(Y_out);
    Serial.print(" ACC_z: ");
    Serial.print(Z_out);
    disp = 0; //further output elsewhere
  
  }
  if(TIMING!=0)
  {
    Serial.print(" delta_t: ");
    Serial.print((float)delta_t/1000000, 4);
    Serial.print(" time [s]: ");
    Serial.print((double)micros()/1000000, 3);
    Serial.print(" timer: ");
    Serial.print((double)timer/1000000, 3);
    Serial.print(" last_gyro: ");
    Serial.print((double)last_gyro/1000000, 3);
    Serial.print(" last_acc: ");
    Serial.print((double)last_acc/1000000, 3);
    Serial.print(" last_mag: ");
    Serial.print((double)last_mag/1000000, 3);
    Serial.print(" last_disp: ");
    Serial.print((double)last_disp/1000000, 3);
    
  }
  if(POSITION!=0)
  {
    Serial.print(" x[m]: ");
    Serial.print(pos(0), 3);
    Serial.print(" y: ");
    Serial.print(pos(1), 3);
    Serial.print(" z: ");
    Serial.print(pos(2), 3);
  }
  if(VELOCITY!=0)
  {
    Serial.print(" vx: ");
    Serial.print(vel(0), 3);
    Serial.print(" vy: ");
    Serial.print(vel(1), 3);
    Serial.print(" vz: ");
    Serial.print(vel(2)), 3;
    BLA::Matrix<3, 1> acc_vector = Orientation_Gyro*gravity_axis*9.81/acc_cal-z_axis*9.81;
    Serial.print(" ax: ");
    Serial.print(acc_vector(0), 5);
    Serial.print(" ay: ");
    Serial.print(acc_vector(1), 5);
    Serial.print(" az: ");
    Serial.print(acc_vector(2), 5);
  }
  if(KOMPASS!=0){
    Serial.print(" yaw_compass_error: ");
    Serial.print(((double)yaw_compass_error*RAD_TO_DEG));
    Serial.print(" yaw_compass: ");
    Serial.print(((double)yaw_compass*RAD_TO_DEG));
    Serial << " mag_vector: " << mag_vector<< " north: " << north;//<<" x_norm: "<< x_norm <<" y_norm: "<< y_norm <<" z_norm: "<< z_norm << " x: "<< x << " y: "<< y <<" z: "<< z;
  }
  if(ORIANTATION_MATRIX!=0)
  {
    Serial << " " <<Orientation_Gyro << " " << determinant(Orientation_Gyro)<< " "<< Orientation_Gyro(2,0);
  }
  if(PYTHON!=0)
  {
    Serial.print(yaw*RAD_TO_DEG);
    Serial.print(",");
    Serial.print(yaw_compass_error*RAD_TO_DEG);
    Serial.print(",");
    Serial.print((float)yaw_compass*RAD_TO_DEG);
    Serial.print(",");
    Serial.print((float)(micros()-last_disp)/1000000, 3);
  }
 Serial.println();
}