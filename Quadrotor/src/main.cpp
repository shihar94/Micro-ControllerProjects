#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include "imu.h"
#include "motor.h"
#include "pid.h"


MPU6050 mpu6050(Wire);

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pin_setup();
}

void loop() {
  

  if(millis() - timer > 10){
    mpu6050.update();
    float norm_accel = sqrt((mpu6050.getAccX()*mpu6050.getAccX())+ (mpu6050.getAccY()*mpu6050.getAccY())+(mpu6050.getAccZ()*mpu6050.getAccZ()));

   float a[4]    = {0,mpu6050.getAccX()/norm_accel ,mpu6050.getAccY()/norm_accel,mpu6050.getAccZ()/norm_accel};
   float w_a[4]  = {0,mpu6050.getGyroX()*3.14/180,mpu6050.getGyroY()*3.14/180,mpu6050.getGyroZ()*3.14/180};
   madgwickFilter(q,w_a,a); 

  /*
   if((set_point-yaw)>0.1){
       int out = controll_signal(yaw);
       //turnRight_m(out);
   }else{
       stop(0);
   }/*else if(yaw-set_point<0.1){
       int out = controll_signal(yaw);
       turnLeft_m(out);
   }*/

  ///forward_m();
   //turnLeft_m(100);
   //Serial.println(output);
   //Serial.println(error);
  Serial.print(roll*180/3.14);
   Serial.print("roll  ");
   Serial.print(pitch*180/3.14);
   Serial.print("pitch ");
   Serial.print(yaw*180/3.14);
   Serial.println("yaw  ");
   Serial.println(" ");
   timer = millis();
    
  }

}