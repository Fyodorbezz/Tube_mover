#include "define.h"

#include <Wire.h>
#include <Adafruit_PCF8574.h>

Adafruit_PCF8574 pcf;

Motor_controll claw_motor(1, MOTOR3_DIR, MOTOR3_SPEED, MOTOR3_ENC_A, MOTOR3_ENC_B, MOTOR3_INV, MOTOR3_ENC_INV, MOTOR3_ENC_RESOLUTION, MOTOR3_D1, MOTOR3_D2, MOTOR3_kP, MOTOR3_kI, MOTOR3_kD);
Motor_controll gimble_motor(2, MOTOR2_DIR, MOTOR2_SPEED, MOTOR2_ENC_A, MOTOR2_ENC_B, MOTOR2_INV, MOTOR2_ENC_INV, MOTOR2_ENC_RESOLUTION, MOTOR2_D1, MOTOR2_D2, MOTOR2_kP, MOTOR2_kI, MOTOR2_kD);
Motor_controll magasine_motor(3, MOTOR1_DIR, MOTOR1_SPEED, MOTOR1_ENC_A, MOTOR1_ENC_B, MOTOR1_INV, MOTOR1_ENC_INV, MOTOR1_ENC_RESOLUTION, MOTOR1_D1, MOTOR1_D2, MOTOR2_kP, MOTOR2_kI, MOTOR2_kD);

Claw_move_data claw_move_data;

void motor3_enc_tick(){
  claw_motor.enc_tick();
}

void motor2_enc_tick(){
  gimble_motor.enc_tick();
}

void motor1_enc_tick(){
  magasine_motor.encoder_ticks += 2*sign(magasine_motor.motor_duty);
}

void home(){
  magasine_motor.set_duty(-MOTOR1_D1);
  while(digitalRead(MAGASINE_ENDSTOP) != 1){

  }
  magasine_motor.set_duty(0);
  magasine_motor.motor_angle = 0;
  magasine_motor.encoder_ticks = 0;
  magasine_motor.encoders_tick_last = 0;

  gimble_motor.set_duty(-MOTOR2_D1);
  while(digitalRead(GIMBLE_ENDSTOP) != 1){

  }
  gimble_motor.set_duty(0);
  gimble_motor.motor_angle = 0;
  gimble_motor.encoder_ticks = 0;
  gimble_motor.encoders_tick_last = 0;

  claw_motor.set_duty(-MOTOR3_D1);
  while(digitalRead(CLAW_ENDSTOP) != 1){

  }
  claw_motor.set_duty(0);
  claw_motor.motor_angle = 0;
  claw_motor.encoder_ticks = 0;
  claw_motor.encoders_tick_last = 0;

  gimble_motor.send_stop_flag = 1;
  claw_motor.send_stop_flag = 1;
  magasine_motor.send_stop_flag = 1;
}

unsigned long long serial_flush_timer=0;

void setup(){
  Serial.begin(115200);

  attachInterrupt(0, motor3_enc_tick, CHANGE);
  attachInterrupt(1 , motor3_enc_tick, CHANGE);

  attachPCINT(digitalPinToPCINT(MOTOR2_ENC_A), motor2_enc_tick, CHANGE);
  attachPCINT(digitalPinToPCINT(MOTOR2_ENC_B) , motor2_enc_tick, CHANGE);

  attachPCINT(digitalPinToPCINT(MOTOR1_ENC_A), motor1_enc_tick, CHANGE);
  //attachPCINT(digitalPinToPCINT(MOTOR1_ENC_B) , motor1_enc_tick, CHANGE);

  //while(!pcf.begin(0x3F ,&Wire)){
  //}
  delay(200);
  //pcf.pinMode(0, INPUT);
  //pcf.pinMode(1, INPUT);
  //pcf.pinMode(2, INPUT);
  //pcf.pinMode(3, INPUT);



  /*while(1){
    left_motor.tick();
    Serial.print(double(gimble_motor.motor_angle));
    Serial.print(" ");
    Serial.println(double(gimble_motor.motor_speed));
  }*/

  //Serial.println("Speed,Target,Error,Error_sum,Error_rate,Duty");
  //claw_motor.set_duty(-100);
  //home();
  //magasine_motor.target_angle = -10;
  
  while(1){
    gimble_motor.tick();
    claw_motor.tick();
    magasine_motor.tick();
  

    /*Serial.print(double(claw_motor.motor_angle));
    Serial.print("\t");
    Serial.print(double(gimble_motor.motor_angle));
    Serial.print("\t");
    Serial.println(double(magasine_motor.motor_angle));*/
  
    /*if (Serial.available()){
      char order = Serial.read();
      if (order == 'M'){
        gimble_motor.move_by(Serial.parseInt());
      }
      if (order == 'P'){
        //left_motor.motor_kP = Serial.parseFloat();
      }
      if (order == 'D'){
        //left_motor.motor_kD = Serial.parseFloat();
      }
      if (order == 'I'){
        //left_motor.motor_kI = Serial.parseFloat();
      }
    }*/

    if (claw_motor.send_stop_flag == 1){
      Serial.write((byte*)&claw_motor.stop_data, sizeof(claw_motor.stop_data));
      claw_motor.send_stop_flag = 0;
    }
    if (gimble_motor.send_stop_flag == 1){
      Serial.write((byte*)&gimble_motor.stop_data, sizeof(gimble_motor.stop_data));
      gimble_motor.send_stop_flag = 0;
    }
    if (magasine_motor.send_stop_flag == 1){
      Serial.write((byte*)&magasine_motor.stop_data, sizeof(magasine_motor.stop_data));
      magasine_motor.send_stop_flag = 0;
    }

    if (Serial.available() >= sizeof(claw_move_data)){
      Serial.readBytes((byte*)&claw_move_data, sizeof(Claw_move_data));
      //Serial.print(claw_move_data.motor_number);
      //Serial.print(" ");
      //Serial.println(claw_move_data.angle);
      serial_flush_timer = millis();
      if(claw_move_data.motor_number == 1){
        claw_motor.target_angle = claw_move_data.angle;
        claw_motor.motor_stop_flag = 0;
      }
      else if(claw_move_data.motor_number == 2){
        gimble_motor.target_angle = claw_move_data.angle;
        gimble_motor.motor_stop_flag = 0;
      }
      else if(claw_move_data.motor_number == 3){
        magasine_motor.target_angle = claw_move_data.angle;
        magasine_motor.motor_stop_flag = 0;
      }
      else if(claw_move_data.motor_number == 4 && claw_move_data.angle == 1){
        home();
      }
    }

    if (Serial.available() == 0){
      serial_flush_timer = millis();
    }

    if (millis() - serial_flush_timer > 500){
      while(Serial.available() > 0) {
        char t = Serial.read();
      }
    }


    
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}