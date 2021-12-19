float determinant(BLA::Matrix<3,3> m)
{
  return m(0,0)*m(1,1)*m(2,2)
         +m(0,1)*m(1,2)*m(2,0)
         +m(0,2)*m(1,0)*m(2,1)
         -m(0,0)*m(1,2)*m(2,1)
         -m(0,1)*m(1,0)*m(2,2)
         -m(0,2)*m(1,1)*m(2,0);
}

//calculate rotation matix from angles (Angles as drawn on the gyro)
BLA::Matrix<3,3> RotMatFromAngle(float x, float y, float z)
{
  BLA::Matrix<3,3> Rotx =   {1,       0,      0,
                            0,        cos(x), -sin(x),
                            0,        sin(x), cos(x)};
                                                      
  BLA::Matrix<3,3> Roty =   {cos(y),  0,      sin(y),
                             0,       1,      0,
                             -sin(y), 0,      cos(y)};
                            
  BLA::Matrix<3,3> Rotz =   {cos(z), -sin(z), 0,
                             sin(z),  cos(z), 0,
                             0,       0,      1};
                             
  Rotx = Roty*Rotx;
  Rotx = Rotz*Rotx;
  /*Serial.print(Rotx(0,0), 10);
  Serial.print(" ");
  Serial.print(Roty(0,0), 10);
  Serial.print(" ");
  Serial.println(Rotz(0,0), 10);*/
  return Rotx;
}

//convert rotation Matrix to Angle (as drawn on the gyro)
void eular_angels_form_rotation_matrix(BLA::Matrix<3,3> RotationMatrix)
{
  float R00 = RotationMatrix(0,0);
  float R10 = RotationMatrix(1,0);
  float R20 = RotationMatrix(2,0);
  float R21 = RotationMatrix(2,1);
  float R22 = RotationMatrix(2,2);
  
  
  pitch = asin(R20);  //pitch ist mathematisch negative, entgenen der rechten-Hand-Regel um die Y-Achse
  yaw = atan2(R10, R00);  //yaw ist mat. positve
  float c=cos(pitch);
  if(c>0.0001){
    roll = atan2(R21/c, R22/c);
  }
  else{ //gimbal lock, dont chance angles
  }
}

//create rotation matrix from rotation axis and angle
BLA::Matrix<3,3> RotMatFromAngleAndAxis(float angle, BLA::Matrix<3,1> Axis){
  float c = cos(angle);
  float s = sin(angle);

  float c_=1-c;
  
  
  float x=Axis(0,0);
  float y=Axis(1,0);
  float z=Axis(2,0);

  BLA::Matrix<3,3> Rot = {c+x*x*c_, x*y*c_-z*s, x*z*c_+y*s,
                          y*x*c_+z*s, c+y*y*c_, y*z*c_-x*s,
                          z*x*c_-y*s, z*y*c_+x*s, c+z*z*c_};
  return Rot;
  
}

BLA::Matrix<3,1> CrossProduct(BLA::Matrix<3,1> A, BLA::Matrix<3,1> B){
  float x = A(1,0)*B(2,0)-A(2,0)*B(1,0);
  float y = A(2,0)*B(0,0)-A(0,0)*B(2,0);
  float z = A(0,0)*B(1,0)-A(1,0)*B(0,0);
  BLA::Matrix<3,1> C = {x,y,z};
  return C;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}