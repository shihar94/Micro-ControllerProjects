#ifndef PID_H
#define PID_H 


float ki , kd;
float kp = 3.75;
int pwm_min = 90;
int pwm_max = 150;


float  error=0; 
float set_point = 1.57;

float epsilon = 0.0001; //tolerance level for stability 
//int output:


void clamp(int &output){
    if (output > pwm_max){
        output = pwm_max;
    }else if(output < pwm_min){
        output = pwm_min;
    }else{
        output = output;
    }
}

void map(){

}

int controll_signal(float current_position){
    error = set_point - current_position;
    int output = kp*error;
    clamp(output);
    return output;
}

#endif