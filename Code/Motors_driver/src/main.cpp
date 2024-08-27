#include "define.h"

Motor_controll right_motor(motor3_table, MOTOR3_DIR, MOTOR3_SPEED, MOTOR3_ENC_A, MOTOR3_ENC_B, MOTOR3_INV, MOTOR3_ENC_INV, MOTOR3_ENC_RESOLUTION, MOTOR3_A, MOTOR3_B, MOTOR3_kP, MOTOR3_kI, MOTOR3_kD);
Motor_controll left_motor(motor2_table, MOTOR2_DIR, MOTOR2_SPEED, MOTOR2_ENC_A, MOTOR2_ENC_B, MOTOR2_INV, MOTOR2_ENC_INV, MOTOR2_ENC_RESOLUTION, MOTOR2_A, MOTOR2_B, MOTOR2_kP, MOTOR2_kI, MOTOR2_kD);

Motor_move_data motor_move_data;
Enc_data enc_data;

unsigned long long serial_flush_timer=0;

void motor3_enc_tick(){
  right_motor.enc_tick();
}

void motor2_enc_tick(){
  left_motor.enc_tick();
}


void setup(){
  Serial.begin(115200);

  attachInterrupt(0, motor3_enc_tick, CHANGE);
  attachInterrupt(1 , motor3_enc_tick, CHANGE);

  attachPCINT(digitalPinToPCINT(MOTOR2_ENC_A), motor2_enc_tick, CHANGE);
  attachPCINT(digitalPinToPCINT(MOTOR2_ENC_B) , motor2_enc_tick, CHANGE);

  //left_motor.generate_slope();

  /*while(1){
    left_motor.tick();
    Serial.print(double(left_motor.motor_angle));
    Serial.print(" ");
    Serial.println(double(left_motor.motor_speed));
  }*/

  //Serial.println("Speed,Target,Error,Error_sum,Error_rate,Duty");

  //left_motor.PID_target_speed = 600;
  //right_motor.PID_target_speed = 600;

  delay(1000);
  //Serial.flush();
  Serial.println(sizeof(Enc_data));
  while(1){
    right_motor.tick();
    left_motor.tick();

    if (left_motor.new_enc_val){
      left_motor.new_enc_val = 0;
      enc_data.motor_number = 0;
      enc_data.angle = int32_t(left_motor.motor_angle*1000);
      Serial.write((byte*)&enc_data, sizeof(Enc_data));
    }

    if (right_motor.new_enc_val){
      right_motor.new_enc_val = 0;
      enc_data.motor_number = 1;
      enc_data.angle = int32_t(right_motor.motor_angle*1000);
      Serial.write((byte*)&enc_data, sizeof(Enc_data));
    }

    if (Serial.available() >= sizeof(Motor_move_data)){
      Serial.readBytes((byte*)&motor_move_data, sizeof(Motor_move_data));
      serial_flush_timer = millis();
      if(motor_move_data.motor_number == 0){
        left_motor.PID_target_speed = motor_move_data.speed;
      }
      else if(motor_move_data.motor_number == 1){
        right_motor.PID_target_speed = motor_move_data.speed;
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

    /*if (Serial.available()){
      char order = Serial.read();
      if (order == 'S'){
        left_motor.PID_target_speed = Serial.parseInt();
      }
      if (order == 'P'){
        left_motor.motor_kP = Serial.parseFloat();
      }
      if (order == 'D'){
        left_motor.motor_kD = Serial.parseFloat();
      }
      if (order == 'I'){
        left_motor.motor_kI = Serial.parseFloat();
      }
    }*/


    
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
