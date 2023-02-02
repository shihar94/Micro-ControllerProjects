#ifndef IMU_H
#define IMU_H
float dT = 0.01;
float yaw , pitch , roll;
float JF1,JF2,JF3,JF4;
float n0,n1,n2,n3;
float quatmult[4] = {n0,n1,n2,n3};
float beta = .009;
float q_dot1,q_dot2,q_dot3,q_dot4;
float q_dot_[4];
float q[4]  = {1,0,0,0};

#include "mpu_6050.h"
void quarternion2euler(float q[4]);
void gradientMatrix(float quarternion[4],float accel_vector[4]);
void gradientMatrix(float quarternion[4],float accel_vector[4]);
void quarternionMultiplication(float q[4] ,float w[4]);
void q_dotF();
void q_estimateFunction(float dT,float q[4]);



void madgwickFilter(float q_[4],float w_v[4],float a_v[4]){
    quarternion2euler(q_);
    gradientMatrix(q_,a_v);
    quarternionMultiplication(q_ ,w_v);
    q_dotF();
    q_estimateFunction( dT,q_);
}

void quarternion2euler(float q[4]){
  float q_0 = q[0];
  float q_1 = q[1];
  float q_2 = q[2];
  float q_3 = q[3];

  yaw  = atan2((2*q_0*q_3)-(2*q_1*q_2),1-(2*q_2*q_2)-(2*q_3*q_3));
  pitch = asin((2*q_0*q_2) - (2*q_3*q_1));
  roll = atan2((2*q_0*q_1) + (2*q_2*q_3), -(2*q_1*q_1) -(2*q_2*q_2)+1);
}


void gradientMatrix(float quarternion[4],float accel_vector[4]){
    float q1 = quarternion[0];
    float q2 = quarternion[1];
    float q3 = quarternion[2];
    float q4 = quarternion[3];

    float F1,F2,F3;
    F1  = 2*(q2*q4-q1*q3)-accel_vector[1];
    F2  = 2*(q1*q2+q3*q4)-accel_vector[2];
    F3  = 2*(0.5-q2*q2-q3*q3)- accel_vector[3];

   //float JF1,JF2,JF3,JF4;
    JF1 = (-2*q3*F1) + (2*q2*F2) + (0*F3);
    JF2 = (2*q4*F1) + (2*q1*F2) + (-4*q2*F3);
    JF3 = (-2*q1*F1) + (2*q4*F2) + (-4*q3*F3);
    JF4 = (2*q2*F1) + (2*q3*F2) + (0*F3);  

    float norm;
    norm = sqrt ((JF1*JF1) + (JF2*JF2) +(JF2*JF2) + (JF3*JF3));

    JF1 = JF1/norm;
    JF2 = JF2/norm;
    JF3 = JF3/norm;
    JF4 = JF4/norm;
    
}


void quarternionMultiplication(float q[4] ,float w[4]){
    float w1 = q[0];
    float x1 = q[1];
    float y1 = q[2];
    float z1 = q[3];

    float w2 = w[0];
    float x2 = w[1];
    float y2 = w[2];
    float z2 = w[3];

   // float n0,n1,n2,n3;
    n0 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    n1 = w1*x2 + w2*x1 + y1*z2 - y2*z1;
    n2 = w1*y2 + w2*y1 - x1*z2 + x2*z1;
    n3 = w2*z1 - x2*y1 + x1*y2 + z2*w1;

    n0 = 0.5*n0;
    n1 = 0.5*n1;
    n2 = 0.5*n2;
    n3 = 0.5*n3;
}

void q_dotF(){
  q_dot_[0] = n0 - beta*JF1;
  q_dot_[1] = n1 - beta*JF2;
  q_dot_[2] = n2 - beta*JF3;
  q_dot_[3] = n3 - beta*JF4;
}


void q_estimateFunction(float dT,float q[4]){
  q[0] = q[0] + q_dot_[0]*dT;
  q[1] = q[1] + q_dot_[1]*dT;
  q[2] = q[2] + q_dot_[2]*dT;
  q[3] = q[3] + q_dot_[3]*dT;
}
#endif 

/*
    Serial.println("=======================================================");
    //Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());
    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());  
    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());
    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());
    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    Serial.println("=======================================================\n");
    */