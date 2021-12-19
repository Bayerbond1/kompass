void read_acc()//in g    
{
  // === Read acceleromter data === //
  xyzFloat gVal = myIMU.getGValues();

  X_out = gVal.x;
  Y_out = gVal.y;
  Z_out = gVal.z;
}

void update_acc(float c){   //c.. complementrary factor, 1-c=0.999,-> tau=10s, 1-c=exp(-Ts/tau), tau vorgeben, c berechnen
  gravity_axis = {X_out, Y_out, Z_out}; //where is the gravity vector pointing relative to the sensor?
  BLA::Matrix<3, 1> acc_z_axis = Orientation_Gyro*gravity_axis;
  //acc_z_axis = Orientation_Gyro*acc_z_axis; //this should be pointing straight up in earth coordinate system, (the sensor is accalerated upwards, asuming no gyro-integration-error)
  
  //clac error angle
  BLA::Matrix<1, 1> absoulte = (~acc_z_axis)*acc_z_axis;
  BLA::Matrix<1, 1> angle_temp = (~acc_z_axis)*z_axis/sqrt(absoulte(0,0));  //angle between acc and z-axis (measure for error)
  acc_error_angle = acos(angle_temp(0,0));
  
  //correct error/complementary filter
  if(acc_error_angle>0.00001){
    //calc rotation axis
    rotation_axis = CrossProduct(acc_z_axis, z_axis); //order?
    float norm = 1/(sin(acc_error_angle)*sqrt(absoulte(0,0)));
    rotation_axis(0,0) = rotation_axis(0,0)*norm;
    rotation_axis(1,0) = rotation_axis(1,0)*norm;
    rotation_axis(2,0) = rotation_axis(2,0)*norm;

    if(absoulte(0,0)<2){
      c=-c*(absoulte(0,0)-2)*absoulte(0,0);   //korrekturfaktor fall betrag der Beschleunigung ungleich 1
    }
    else
    {
      c=0;
    }
    BLA::Matrix<3, 3> rotation_mat = RotMatFromAngleAndAxis(acc_error_angle*c, rotation_axis);
    Orientation_Gyro = rotation_mat*Orientation_Gyro;

    //update velocity and position
    BLA::Matrix<3, 1> at = (Orientation_Gyro*gravity_axis*9.81/acc_cal-z_axis*9.81)*delta_t_acc;
    vel = vel + at;
    pos = pos + vel*0.01-at*delta_t_acc*0.5;
    
  }
  if(disp)
    {
      disp=0;
      /*Serial.print(" absoulte: ");
      Serial.print(absoulte(0,0)); */
      Serial.print(" Error_angle: ");
      Serial.print(acc_error_angle*RAD_TO_DEG);
      /*Serial.print(" det: ");
      Serial.print(determinant(rotation_mat));
      Serial.print(" norm: ");
      Serial.print(norm, 10);*/
      Serial <<" rotation_axis: " << rotation_axis; 
    }     
}