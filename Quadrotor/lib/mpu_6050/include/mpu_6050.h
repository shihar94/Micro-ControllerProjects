#ifndef MPU_6050_H
#define MPU_6050_H

#define I2C_MPU          0x68
#define GYRO_REG         0x43       
#define ACCEL_REG        0x3B
#define PWR_REG          0x6B
#define GYRO_SET_REG     0x1B
#define ACCEL_SET_REG    0x1C

// need to do the modelling of the offsets for gyro and accel 
//accel looks fine but gyro has some issues as it looks



#include <Arduino.h>
#include <Wire.h>
#include<math.h>

class mpu_6050{
    public:
        float gyro_rawx ,gyro_rawy , gyro_rawz;
        float accel_rawx , accel_rawy , accel_rawz;
        float euler_yaw , euler_pitch, euler_roll;
        float q0,q1,q2,q3; // quarternions 
        float ax_bias=0, ay_bias=0 , az_bias=0;
        float gx_bias=0, gy_bias=0 , gz_bias=0;
    public:
        mpu_6050();   // constructor  
        ~mpu_6050(); //destructor
        void init();
        void  getGyroData();//get gyro from registers
        void getAccelData(); //get accel from Register
        void sendAngleData();  //normalise and send the ypr data
        void sendAccelData();   // normalise and send the acceleration data
        void angle_processing(); //process the raw values for meaningful angles to upload to filtering purpose
        void println();
        void accelBias(int biasSize = 200);
        void gyroBias(int biasSize = 3000);
        void sendGyroData(float &gx,float &gy,float &gz);

};
#endif