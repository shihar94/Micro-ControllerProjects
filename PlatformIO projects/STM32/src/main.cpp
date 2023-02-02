#include <mbed.h>
#include "mpu6050.h"

DigitalOut led(LED1);
 //i2c insttance for mpu6050
MPU6050 mpu(I2C_SDA,I2C_SCL); 

int main() {
  led = 1;
  // put your setup code here, to run once:

  while(1) {
   uint8_t add =  mpu.testConnection();
   if(add == 104){
     led = 0;
   }
   //printf("I am %d\r\n",add);
  }
}