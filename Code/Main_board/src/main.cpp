#include <define.h>
#include <WiFi.h>
const char* ssid = "esp32";
const char* password = "12344321";

WiFiServer server(80);
IPAddress ip(192, 168, 1, 1);
IPAddress mask(255, 255, 0, 0);

Enc_data enc_data;
Motor_stop_data stop_data;
unsigned long long serial_flush_timer=0;
unsigned long long serial_flush_timer2=0;



Claw_move_data home;

Robot robot;

/*comand_type:
1 - move_forward_till_line_on_line
2 - move_forward_distance_on_line
3 - move_forward_free
4 - move_forward_till_line_right_shift
5 - move_forward_till_line_left_shift
6 - rotate
7 - grab_pipe_same_high
8 - grab_pipe_above
9 - grab_pipe_below
10 - put_pipe*/

int16_t simple_commands_list1[60][3] = {{6, -90}, {9, 0}, {6, 90}, {1, 2,1}, {6, 90}, {1,1,1}, {12, 0}};
//int16_t simple_commands_list1[40][3] = {{1, 2, 1}};

//int16_t simple_commands_list[40][3] = {{9, 0}, {6, -90}, {1, 2, 1}, {6, -90}, {1, 1, 1}, {6, 90}, {3, 315, 1}, {3, 320, 0}, {2, 250, 0}, {3, 260, 0}, {3, 295, 0}, {6, -90}, {1, 3, 1}, {6, -90}, {1, 2, 1}, {9, 0}, {3, -600}, {6, 90}, {1, 1, 1}, {6, -90}, {2, 315, 1}, {2, 320, 0}, {2, 500, 0}, {2, 260, 0}, {1, 295, 1}, {12, 0}, {6, -90}, {1, 1, 1},{2, 315, 1}, {9, 1}, {6, 90}, {12, 0}, {6, 90}, {1, 1, 1}, {6, -90}, {6, 90}, {1, 1, 1}, {6, -90}, {12, 0}};
//int16_t simple_commands_list[40][3] = {{2, 315, 1}, {2, 320, 0}, {2, 500, 0}, {2, 260, 0}, {2, 280, 0}, {6, -90}, {1, 1, 1},{2, 315, 1}, {9, 1}, {6, 90}, {12, 0}, {6, 90}, {1, 1, 1}, {6, -90}, {12, 0}, {6, 90}, {1, 1, 1}, {6, -90}, {12, 0}};
//int16_t simple_commands_list[40][3] = {{6, 720}, {9, 0}, {9, 0}, {6, 180}};
//int16_t simple_commands_list[40][3] = {/*{2, 315, 1}, {2, 320, 0}, {2, 250, 0},*/ {2, 260, 0}, {2, 295, 0}};
//int16_t simple_commands_list[40][3] = {{6, -90}, {9, 0}, {6, 90}, {1, 2, 1}, {6, -90}, {1, 1, 1}, {6, 90}, {2, 665, 1}, {9, 0}, {6, 180}, {2, 300, 0}, {2, 260, -1}, {2, 295, 0}, {6, -90}, {1, 1, 1}, {12, 0}, {6, -90}, {1, 1, 1}, {6, 90}, {12, 0}};
//int16_t simple_commands_list[40][3] = {{9, 0}, {1, 1, 1}, {2, 665, 1}, {9, 0}, {6, 180}, {2, 200, 0}, {2, 390, -1}, {6, -90}, {1, 1, 1}, {12, 0}};
//int16_t simple_commands_list[40][3] = {{6, -90}, {9, 0}, {6, 90}, {1, 2,1}, {6, 90}, {1,1,1}, {12, 0}};
//int16_t simple_commands_list[40][3] = {{2, 100, 0}, {2, 555, 1}, {9, 0}, {6, 180}, {2, 100, 0}, {2, 470, -1}, {6, -90}, {1, 1, 1}, {12, 0}};
int16_t simple_commands_list2[60][3] = {{6, -90}, {9, 0}, {6, 90}, {1, 2, 1}, {2, 555, 1}, {2, 100, 0}, {9, 0},
{6, 180}, {2, 100, 0}, {2, 485, -1}, {6, -90}, {1, 1, 1}, {6, -90}, {1, 1, 1}, {6, 90}, {12, 0}, {6, -90}, {1, 1, 1},
{6, 90}, {12, 0}, {6, -90}, {1, 2, 1}, {6, 90}, {1, 1, 1}, {6, -90}, {1, 1, 1}, {6, -90}, {2, 555, 1},
{2, 1060,0}, {2, 455, -1}, {6, -90}, {1, 3, 1}, {9, 0}, {6, 180}, {1, 3, 1}, {6, 90}, {2, 555, 1},
{2, 1060, 0}, {2, 435, -1}, {6, 90}, {1, 1, 1}, {6, 90}, {1, 1, 1}, {6, -90}, {1, 1, 1}, {6, -90}, {12, 0}};
//int16_t simple_commands_list2[60][3] = {{2, 300, 0}};
//int16_t simple_commands_list2[40][3] = {{12,0}};
//int16_t simple_commands_list2[40][3] = {{1, 2, 1}};
//int16_t simple_commands_list[40][3] = {{9, 0}, {9, 0}, {12, 0}, {12, 0}};
//int16_t simple_commands_list[40][3] = {{12, 0}};
int16_t simple_commands_list[60][3];
int16_t simple_commends_col = 52;
//int16_t simple_commends_col = 1;




/*comand_type:
1 - move_forward_till_line_on_line
2 - move_forward_distance_on_line
3 - move_forward_free
4 - move_forward_till_line_right_shift
5 - move_forward_till_line_left_shift
6 - rotate
7 - rotate_2
8 - transport_position
9 - grab_pipe_below
10 - put_pipe*/




//int16_t commands_list[40][2] = {{1, 2}, {6, -90}, {2, 300}, {6, 90}, {3, 300}, {6, 90}, {2, 300}, {6, 90}, {1, 3}, {6, 180}};
//int16_t commands_list[40][2] = {{10, 0}, {1, 3}, {6, -90}, {3, 300}, {6, -90}, {3, 300}, {6, 90}, {2, 300}, {6, 90}, {1, 3}, {6, 180}};
//int16_t commands_list[40][2] = {{10, 0}, {6, 90}, {1, 3}, {6, -90}, {2, 630}, {6, 180}, {2, 550}, {6, -90}, {1, 2}, {6, -90}, {1, 2}, {6, 90}, {1, 1}, {6, 90}, {6, 90}, {1, 1}, {6, 90}, {1, 4}};
//int16_t commands_list[40][3] = {{10, 0, 0}, {2, 315, 0}, {2, 270, 0}, {2, 295, 0}};
//int16_t commands_list[40][2] = {{10, 0}, {2, 315}, {2, 320}};
//int16_t commands_list[40][2] = {{10, 0}, {2, 260}, {2, 295}};
//int16_t commands_list[40][2] = {{10, 0},{8, 0}, {2, 100}, {9, 0}, {6, 180}, {6, 90}, {1, 3}, {6, -90}, {2, 616}, {6, 180}, {2, 616}, {6, -90}, {1, 2}, {6, -90}, {1, 2}, {6, 90}, {1, 1}, {6, 90}, {6, 90}, {1, 1}, {6, 90}, {1, 4}};
int16_t commands_list[200][3] = {};
int16_t commands_col = 0;

uint64_t sensor_ask_timer = 0;
uint64_t movement_tick_timer = 0;

int16_t cur_command_number = 0;

bool robot_state = 0;
int tmp_com_col = 0;

void execute_command(){
  if (robot.movement_type == 0 && cur_command_number < commands_col){
    int16_t cur_command = commands_list[cur_command_number][0];
    Serial.println();
    Serial.print(cur_command_number);
    Serial.print(" ");
    Serial.println(commands_col);
    Serial.flush();
    switch (cur_command)
    {
    case 1:
      robot.move_forward_line_crosses(commands_list[cur_command_number][1], 0, commands_list[cur_command_number][2]);
      break;
    case 2:
      robot.move_forward_line_distance(commands_list[cur_command_number][1], 0, commands_list[cur_command_number][2]);
      break;
    case 3:
      robot.move_forward_distance(commands_list[cur_command_number][1], commands_list[cur_command_number][2]);
      break;
    case 4:
      robot.move_forward_line_crosses(commands_list[cur_command_number][1], 4, commands_list[cur_command_number][2]);
      break;
    case 5:
      robot.move_forward_line_crosses(commands_list[cur_command_number][1], -4, commands_list[cur_command_number][2]);
      break;
    case 6:
      robot.rotate(commands_list[cur_command_number][1]);
      break;
    case 9:
      robot.open_claw(commands_list[cur_command_number][1]);
      break;
    case 10:
      robot.close_claw(commands_list[cur_command_number][1]);
      break;
    case 11:
      robot.move_magasine(commands_list[cur_command_number][1]);
      break;
    case 12:
      robot.open_claw_half();
      break;
    case 13:
      robot.gimble_in_position(commands_list[cur_command_number][1]);
      break;
    case 14:
      robot.open_only_claw(commands_list[cur_command_number][1]);
      break;
    case 15:
      robot.transport_position();
      break;
    case 16:
      delay(1000);
    }

    cur_command_number ++;
  }
}



void setup() {
  bool btn = digitalRead(BUTTON_PIN);
  if (btn == 0){
    robot.phase = 1;
  }
  for (int i=0; i<60; i++){
    for (int j=0; j<3; j++){
      if (btn == 0){
        simple_commands_list[i][j] = simple_commands_list1[i][j];
      }
      else{
        simple_commands_list[i][j] = simple_commands_list2[i][j];
      }
    }
  }

  //
  Serial1.begin(115200, SERIAL_8N1, MOTOTRS_DRIVER_RX, MOTOTRS_DRIVER_TX);
  Serial2.begin(115200, SERIAL_8N1, CLAW_DRIVER_RX, CLAW_DRIVER_TX);
  Serial.begin(115200);

  Serial.println("Start");
  WiFi.softAP(ssid, password);
  delay(100);
  WiFi.softAPConfig(ip, ip, mask);
  server.begin();
  bool recieved_data = 1;

  while(!recieved_data){
    WiFiClient client = server.available();
    int length;
    int value;
    bool phase = 0;
    int counter = 0;
    Serial.println("Start");
    if (client){
      Serial.println("Client");
      while (client.connected()) {
        Serial.println("Connected");
        if ((length = client.available() / 4) > 0) {
          for (int i = 0; i < length; i++) {
            client.readBytes((char*)&value, sizeof(value));
            simple_commands_list[counter/2][phase] = value;
            phase = !phase;
            counter ++;
          }
          simple_commends_col = length/2;
          recieved_data = 1;
          // client.println("200");
        }
        delay(1);
      }
    }
  }

  for (int i=0; i<simple_commends_col; i++){
    if (simple_commands_list[i][0] == 1){
      simple_commands_list[i][2] = 1;
    }
    if (simple_commands_list[i][0] == 2 && simple_commands_list[i][1] == 315){
      simple_commands_list[i][2] = 1;
    }
    if (simple_commands_list[i][0] == 2 && simple_commands_list[i][1] == 260){
      simple_commands_list[i][2] = -1;
    }
    Serial.print(simple_commands_list[i][0]);
    Serial.print(" ");
    Serial.println(simple_commands_list[i][1]);
  }

  pinMode(SENSOR_CNTRL_1, OUTPUT);
  pinMode(SENSOR_CNTRL_2, OUTPUT);
  pinMode(SENSOR_CNTRL_3, OUTPUT);
  pinMode(SENSOR_SIGNAL_1, INPUT);
  pinMode(SENSOR_SIGNAL_2, INPUT);
  pinMode(BUTTON_PIN, INPUT);

  
  home.motor_number = 4;
  home.angle = 1;

  claw_setup();

  robot.tubes_inside == 0;

  commands_list[commands_col][0] = 15;
  commands_list[commands_col][1] = 0;
  commands_list[commands_col][2] = 0;
  commands_col ++;

  for (int i = 0; i<simple_commends_col; i++){
    if (simple_commands_list[i][0] <= 8){
      commands_list[commands_col][0] = simple_commands_list[i][0];
      commands_list[commands_col][1] = simple_commands_list[i][1];
      commands_list[commands_col][2] = simple_commands_list[i][2];
      commands_col ++;
    }
    else if (simple_commands_list[i][0] == 9){
      if (robot.tubes_inside == 0 && robot.phase == 0){
        commands_list[commands_col][0] = 11;
        commands_list[commands_col][1] = 3;
        commands_col ++;
        commands_list[commands_col][0] = 9;
        commands_list[commands_col][1] = simple_commands_list[i][1];
        commands_col ++;
        commands_list[commands_col][0] = 2;
        if (simple_commands_list[i][1] == 0){
          commands_list[commands_col][1] = SAME_HEIGHT_GRABBING_DISTANCE;
        }
        else{
          commands_list[commands_col][1] = ABOVE_GRABBING_DISTANCE;
        }
        commands_list[commands_col][2] = 0;
        commands_col ++;
        commands_list[commands_col][0] = 10;
        commands_list[commands_col][1] = 0;
        commands_col ++;
        robot.tubes_inside++;
        commands_list[commands_col][0] = 3;
        if (simple_commands_list[i][1] == 1){
          commands_list[commands_col][1] = -400;
        }
        else{
          commands_list[commands_col][1] = -100;
        }
        commands_col ++;
        commands_list[commands_col][0] = 12;
        commands_list[commands_col][1] = 0;
        commands_col ++;
        commands_list[commands_col][0] = 11;
        commands_list[commands_col][1] = 1;
        commands_col ++;
        commands_list[commands_col][0] = 13;
        commands_list[commands_col][1] = 2;
        commands_col ++;
        commands_list[commands_col][0] = 14;
        commands_list[commands_col][1] = 1;
        commands_col ++;
        commands_list[commands_col][0] = 11;
        commands_list[commands_col][1] = 3;
        commands_col ++;
      }
      else if (robot.tubes_inside == 1 || robot.phase == 1){
        commands_list[commands_col][0] = 9;
        commands_list[commands_col][1] = simple_commands_list[i][1];
        commands_col ++;
        commands_list[commands_col][0] = 2;
        if (simple_commands_list[i][1] == 0){
          commands_list[commands_col][1] = SAME_HEIGHT_GRABBING_DISTANCE;
        }
        else{
          commands_list[commands_col][1] = ABOVE_GRABBING_DISTANCE;
        }
        commands_list[commands_col][2] = 0;
        commands_col ++;
        commands_list[commands_col][0] = 10;
        commands_list[commands_col][1] = 1;
        commands_col ++;
        robot.tubes_inside++;
        commands_list[commands_col][0] = 3;
        if (simple_commands_list[i][1] == 1){
          commands_list[commands_col][1] = -400;
        }
        else{
          commands_list[commands_col][1] = -100;
        }
        commands_col ++;

        
      }
    }
    else if (simple_commands_list[i][0] == 12){
      if (robot.tubes_inside == 1 && robot.phase == 0){
        commands_list[commands_col][0] = 11;
        commands_list[commands_col][1] = 1;
        commands_col ++;
        commands_list[commands_col][0] = 12;
        commands_list[commands_col][1] = 0;
        commands_col ++;
        commands_list[commands_col][0] = 13;
        commands_list[commands_col][1] = 1;
        commands_col ++;
        commands_list[commands_col][0] = 11;
        commands_list[commands_col][1] = 3;
        commands_col ++;
        commands_list[commands_col][0] = 14;
        commands_list[commands_col][1] = 1;
        commands_col ++;
        commands_list[commands_col][0] = 1;
        commands_list[commands_col][1] = 1;
        commands_list[commands_col][2] = 2;
        commands_col ++;
        commands_list[commands_col][0] = 13;
        commands_list[commands_col][1] = 0;
        commands_col ++;
        commands_list[commands_col][0] = 14;
        commands_list[commands_col][1] = 0;
        commands_col ++;
        commands_list[commands_col][0] = 3;
        commands_list[commands_col][1] = -220;
        commands_col ++;
        commands_list[commands_col][0] = 14;
        commands_list[commands_col][1] = 1;
        commands_col ++;
        commands_list[commands_col][0] = 13;
        commands_list[commands_col][1] = 2;
        commands_col ++;
        robot.phase = 1;
      }
      else{
       commands_list[commands_col][0] = 1;
        commands_list[commands_col][1] = 1;
        commands_list[commands_col][2] = 2;
        commands_col ++;
        commands_list[commands_col][0] = 13;
        commands_list[commands_col][1] = 0;
        commands_col ++;
        commands_list[commands_col][0] = 14;
        commands_list[commands_col][1] = 0;
        commands_col ++;
        commands_list[commands_col][0] = 3;
        commands_list[commands_col][1] = -220;
        commands_col ++;
        commands_list[commands_col][0] = 14;
        commands_list[commands_col][1] = 1;
        commands_col ++; 
        commands_list[commands_col][0] = 13;
        commands_list[commands_col][1] = 2;
        commands_col ++;
      }
      robot.tubes_inside --;
    }
  }

  /*while(1){
    Serial.print(digitalRead(34));
    Serial.print(" ");
    Serial.print(digitalRead(35));
    Serial.print(" ");
    Serial.println(digitalRead(32));
  }*/
     
  /*left_motor.motor_number = 0;
  right_motor.motor_number = 1;
  claw.motor_number =  0;
  gimble.motor_number = 1;
  magasine.motor_number = 2;
  delay(5000);

  right_motor.speed = 370;
  left_motor.speed = 400;
  Serial1.write((byte*)&left_motor, sizeof(Motor_move_data));
  Serial1.write((byte*)&right_motor, sizeof(Motor_move_data));
  delay(3100);
  right_motor.speed = 0;
  left_motor.speed = 0;
  Serial1.write((byte*)&left_motor, sizeof(Motor_move_data));
  Serial1.write((byte*)&right_motor, sizeof(Motor_move_data));

  
  claw.angle = 55;
  Serial2.write((byte*)&claw, sizeof(Claw_move_data));

  delay(1000);

  gimble.angle = 90;
  Serial2.write((byte*)&gimble, sizeof(Claw_move_data));

  delay(1000);

  right_motor.speed = -630;
  left_motor.speed = -700;
  Serial1.write((byte*)&left_motor, sizeof(Motor_move_data));
  Serial1.write((byte*)&right_motor, sizeof(Motor_move_data));
  delay(1000);
  right_motor.speed = 0;
  left_motor.speed = 0;
  Serial1.write((byte*)&left_motor, sizeof(Motor_move_data));
  Serial1.write((byte*)&right_motor, sizeof(Motor_move_data));*/
}

bool flag = 0;



void loop() {
  /*right_motor.speed = 700;
  left_motor.speed = 700;
  //Serial1.write((byte*)&left_motor, sizeof(Motor_move_data));
  //Serial1.write((byte*)&right_motor, sizeof(Motor_move_data));
  gimble.angle = 70;
  claw.angle = 30;
  Serial2.write((byte*)&gimble, sizeof(Claw_move_data));
  Serial2.write((byte*)&claw, sizeof(Claw_move_data));

  delay(1000);

  //right_motor.speed = 0;
 //left_motor.speed = 0;
  //Serial1.write((byte*)&left_motor, sizeof(Motor_move_data));
  //Serial1.write((byte*)&right_motor, sizeof(Motor_move_data));

  //delay(1000);*/
  
  if(Serial1.available() >= sizeof(Enc_data)){
    serial_flush_timer = millis();
    Serial1.readBytes((byte*)&enc_data, sizeof(Enc_data));
    if(enc_data.motor_number == 0){
      robot.left_motor_angle = enc_data.angle/1000.0;
      robot.left_motor_update = 1;
    }
    if(enc_data.motor_number == 1){
      robot.right_motor_angle = enc_data.angle/1000.0;
      robot.right_motor_update = 1;
    }
  }

  if (Serial1.available() == 0){
    serial_flush_timer = millis();
  }

  if (millis() - serial_flush_timer > 10){
    while(Serial1.available() > 0) {
      char t = Serial1.read();
    }
  }
  //Serial.print(Serial2.available());
  //Serial.print(" ");
  //Serial.println(sizeof(stop_data));
  if(Serial2.available() >= sizeof(stop_data)){
    serial_flush_timer2 = millis();
    Serial2.readBytes((byte*)&stop_data, sizeof(stop_data));
    //Serial.print(stop_data.motor_number);
    //Serial.print(" ");
    //Serial.println(stop_data.stop);
    if (stop_data.stop == 1){
      robot.motors_ready[stop_data.motor_number-1] = 1;
    }
  }

  if (Serial2.available() == 0){
    serial_flush_timer2 = millis();
  }

  if (millis() - serial_flush_timer2 > 10){
    while(Serial2.available() > 0) {
      char t = Serial2.read();
    }
  }

  if (robot_state == 0 && digitalRead(BUTTON_PIN) == 0){
    Serial.println("Start");
    Serial2.write((byte*)&home, sizeof(home));
    robot.movement_type = 6;
    robot_state = 1;
  }

  if (robot_state){
    execute_command();
    /*Serial.print(robot.motors_ready[0]);
    Serial.print(" ");
    Serial.print(robot.motors_ready[1]);
    Serial.print(" ");
    Serial.println(robot.motors_ready[2]);*/
  }

  robot.odometry(Serial);
  if (millis() - sensor_ask_timer > 1){
    robot.get_sensor_value(Serial);
    sensor_ask_timer = millis();
  }
  if(millis() - movement_tick_timer > 20){
    robot.movement_tick(Serial);
    movement_tick_timer = millis();
  }

}