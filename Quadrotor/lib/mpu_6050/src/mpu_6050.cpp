#include "mpu_6050.h"

mpu_6050::mpu_6050(){
    //just a constructor 
    //Serial.println("Initialising MPU 6050 Sensor");
}

mpu_6050::~mpu_6050(){
    //just a destructor 
   // Serial.println("Closing Down Sensor");
}


void mpu_6050::getAccelData(){
        Wire.beginTransmission(I2C_MPU);
        Wire.write(ACCEL_REG);
        Wire.endTransmission();
        Wire.requestFrom(I2C_MPU,6);
        while(Wire.available()<6);
        //16384 is because for +- 2g range
        accel_rawx = (Wire.read()<<8|Wire.read())/16384.0;
        accel_rawy = (Wire.read()<<8|Wire.read())/16384.0;
        accel_rawz = (Wire.read()<<8|Wire.read())/16384.0;
}

void mpu_6050::getGyroData(){
        Wire.beginTransmission(I2C_MPU);
        Wire.write(GYRO_REG);
        Wire.endTransmission();
        Wire.requestFrom(I2C_MPU,6);
        while(Wire.available()<6);
        //65.5 is because for +- 500 deg/s range 
        gyro_rawx = (Wire.read()<<8|Wire.read())/65.5;
        gyro_rawy = (Wire.read()<<8|Wire.read())/65.5;
        gyro_rawz = (Wire.read()<<8|Wire.read())/65.5;
}

void mpu_6050::sendAccelData(){

}

void mpu_6050::sendAngleData(){

}

void mpu_6050::init(){
    Wire.beginTransmission(I2C_MPU);
    Wire.write(PWR_REG);
    Wire.write(0b00000000);
    Wire.endTransmission();

    Wire.beginTransmission(I2C_MPU);
    Wire.write(GYRO_SET_REG);
    Wire.write(0x00000000);
    Wire.endTransmission();

    Wire.beginTransmission(I2C_MPU);
    Wire.write(ACCEL_SET_REG);
    Wire.write(0x00000000);
    Wire.endTransmission();
}


void mpu_6050::println(){
    Serial.print("Gyro (deg)");
    Serial.print(" X=");
    Serial.print(gyro_rawx);
    Serial.print(" Y=");
    Serial.print(gyro_rawy);
    Serial.print(" Z=");
    Serial.print(gyro_rawz);
    Serial.print(" Accel (g)");
    Serial.print(" X=");
    Serial.print(accel_rawx);
    Serial.print(" Y=");
    Serial.print(accel_rawy);
    Serial.print(" Z=");
    Serial.println(accel_rawz);
   
}
/*
@brief this calculates the biases for accel
@param class public variables
*/

void mpu_6050::accelBias(int biasSize = 200){
 
   for (int i =0;i<biasSize;i++){
        mpu_6050::getAccelData();
       gx_bias = gx_bias + accel_rawx;
       ay_bias = ay_bias + accel_rawy;
       az_bias = az_bias + accel_rawz;
   }

    gx_bias = gx_bias/biasSize;
    ay_bias = ay_bias/biasSize;
    az_bias = az_bias/biasSize;

    //Serial.println("ax bias");
    //Serial.println(gx_bias);
}

void mpu_6050::gyroBias(int biasSize = 3000){
   
  
   for (int i =0;i<biasSize;i++){
       mpu_6050::getGyroData();
       gx_bias = gx_bias + gyro_rawx;
       gy_bias = gy_bias + gyro_rawy;
       gz_bias = gz_bias + gyro_rawz;
       Serial.println(i);
   }

    gx_bias = gx_bias/biasSize;
    gy_bias = gy_bias/biasSize;
    gz_bias = gz_bias/biasSize;

  //  Serial.println("gx bias");
   // Serial.println(gx_bias);
}

void mpu_6050::sendGyroData(float &gx,float &gy,float &gz){
    //mpu_6050::gyroBias( biasSize);
    gx = gyro_rawx - gx_bias;
    gy= gyro_rawy - gy_bias;
    gz = gyro_rawz - gz_bias;

}


/*
@brief This converts the raw values obtained to angle and ancceleration 
angles will be converted to quarternions then it will be extracted to avoid gimbal lock 
@param class public variables
*/



