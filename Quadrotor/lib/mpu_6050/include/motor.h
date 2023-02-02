#ifndef MOTOR_H
#define MOTOR_H


#define R_pwm 10 
#define L_pwm 3
#define R_F 4
#define R_B 5
#define L_F 9
#define L_B 8

int speed_pwm = 150;

void pin_setup(){
    pinMode(R_pwm,OUTPUT);
    pinMode(L_pwm,OUTPUT);
    pinMode(R_F,OUTPUT);
    pinMode(R_B,OUTPUT);
    pinMode(L_F,OUTPUT);
    pinMode(L_B,OUTPUT);

}

void forward_m(){
    analogWrite(R_pwm,speed_pwm);
    analogWrite(L_pwm,speed_pwm);
    
    digitalWrite(R_F,HIGH);
    digitalWrite(R_B,LOW);

    digitalWrite(L_F,HIGH);
    digitalWrite(L_B,LOW);
}

void backward_m(int turn_Speed){
    analogWrite(R_pwm,turn_Speed);
    analogWrite(L_pwm,turn_Speed);
    
    digitalWrite(R_B,HIGH);
    digitalWrite(R_F,LOW);

    digitalWrite(L_B,HIGH);
    digitalWrite(L_F,LOW);
}

void turnRight_m(int turn_Speed){
    analogWrite(R_pwm,turn_Speed);
    analogWrite(L_pwm,turn_Speed);
    
    digitalWrite(R_F,HIGH);
    digitalWrite(R_B,LOW);

    digitalWrite(L_B,HIGH);
    digitalWrite(L_F,LOW);
}


void turnLeft_m(int turn_Speed){
    analogWrite(R_pwm,turn_Speed);
    analogWrite(L_pwm,turn_Speed);
    
    digitalWrite(R_F,LOW);
    digitalWrite(R_B,HIGH);

    digitalWrite(L_B,LOW);
    digitalWrite(L_F,HIGH);
}

void stop(int turn_Speed){
    analogWrite(R_pwm,turn_Speed);
    analogWrite(L_pwm,turn_Speed);
    
    digitalWrite(R_F,LOW);
    digitalWrite(R_B,LOW);

    digitalWrite(L_B,LOW);
    digitalWrite(L_F,LOW);
}



#endif 