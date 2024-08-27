#include <define.h>

Claw_move_data claw;
Claw_move_data gimble;
Claw_move_data magasine;

void claw_setup(){
    claw.motor_number =  1;
    gimble.motor_number = 2;
    magasine.motor_number = 3;
}

int16_t sign(long double val){
    if (val < 0){
        return -1;
    }
    return 1;
}

void Robot::move_forward_line_distance(int32_t distance, int16_t Line_error, int brake){
    prev_x = target_x;
    prev_y = target_y;
    prev_a = target_a;

    target_x += distance*cos(target_a);
    target_y += distance*sin(target_a);

    target_x_proj = (target_x * cos(-target_a+1*PI/2) - target_y * sin(-target_a+1*PI/2));
    target_y_proj = ((target_x * sin(-target_a+1*PI/2) + target_y * cos(-target_a+1*PI/2)));
    prev_x_proj = (prev_x * cos(-target_a+1*PI/2) - prev_y * sin(-target_a+1*PI/2));
    prev_y_proj = ((prev_x * sin(-target_a+1*PI/2) + prev_y * cos(-target_a+1*PI/2)));

    left_motor_base_value = left_motor_angle;
    right_motor_base_value = right_motor_angle;

    error_sum_1 = 0;
    error_sum_2 = 0;
    last_error_1 = 0;
    last_error_2 = 0;

    brake_mode = brake;

    mode_change_timer = millis();

    movement_axis = (int(degrees(target_a))%360)%180 >= 90;
    movement_type = 2;
    on_cross2 = (brake == 0);
}

void Robot::move_forward_distance(int32_t distance, bool brake){
    prev_x = target_x;
    prev_y = target_y;
    prev_a = target_a;

    target_x += distance*cos(target_a);
    target_y += distance*sin(target_a);

    target_x_proj = (target_x * cos(-target_a+1*PI/2) - target_y * sin(-target_a+1*PI/2));
    target_y_proj = ((target_x * sin(-target_a+1*PI/2) + target_y * cos(-target_a+1*PI/2)));
    prev_x_proj = (prev_x * cos(-target_a+1*PI/2) - prev_y * sin(-target_a+1*PI/2));
    prev_y_proj = ((prev_x * sin(-target_a+1*PI/2) + prev_y * cos(-target_a+1*PI/2)));

    left_motor_base_value = left_motor_angle;
    right_motor_base_value = right_motor_angle;

    error_sum_1 = 0;
    error_sum_2 = 0;
    last_error_1 = 0;
    last_error_2 = 0;

    on_cross2 = 0;

    movement_axis = (int(degrees(target_a))%360)%180 >= 90;
    movement_type = 3;

    direction = sign(distance);

    brake_mode = brake;

    kP = kP2;
    kI = kI2;
    kD = kD2;
    last_error = 0;
    error_sum = 0;
}

void Robot::move_forward_line_crosses(int32_t crosses, int16_t Line_error, int After_move){
    prev_x = target_x;
    prev_y = target_y;
    prev_a = target_a;

    for (int i=0; i<crosses; i++){
        crosses_cors[i][0] = ((i+1)*300 - 115)*cos(target_a) + target_x;
        crosses_cors[i][1] = ((i+1)*300 - 115)*sin(target_a) + target_y;
    }

    target_x += (crosses*300 - 115)*cos(target_a);
    target_y += (crosses*300 - 115)*sin(target_a);

    target_x_proj = (target_x * cos(-target_a+1*PI/2) - target_y * sin(-target_a+1*PI/2));
    target_y_proj = ((target_x * sin(-target_a+1*PI/2) + target_y * cos(-target_a+1*PI/2)));
    prev_x_proj = (prev_x * cos(-target_a+1*PI/2) - prev_y * sin(-target_a+1*PI/2));
    prev_y_proj = ((prev_x * sin(-target_a+1*PI/2) + prev_y * cos(-target_a+1*PI/2)));

    target_cross_counbt = crosses;

    movement_type = 1;

    error_sum_1 = 0;
    error_sum_2 = 0;
    last_error_1 = 0;
    last_error_2 = 0;

    left_motor_base_value = left_motor_angle;
    right_motor_base_value = right_motor_angle;

    after_move = After_move;

    cross_count = 0;
    on_cross = 0;
    on_cross2 = 1;

    kP = kP1;
    kI = kI1;
    kD = kD1;
    last_error = 0;
    error_sum = 0;
}


void Robot::rotate(long double angle){
    prev_x = target_x;
    prev_y = target_y;
    prev_a = target_a;

    movement_axis = (int(radians(target_a))%360)%180 >= 90;
    target_a += radians(angle);

    target_x_proj = (target_x * cos(-target_a+1*PI/2) - target_y * sin(-target_a+1*PI/2));
    target_y_proj = ((target_x * sin(-target_a+1*PI/2) + target_y * cos(-target_a+1*PI/2)));
    prev_x_proj = (prev_x * cos(-target_a+1*PI/2) - prev_y * sin(-target_a+1*PI/2));
    prev_y_proj = ((prev_x * sin(-target_a+1*PI/2) + prev_y * cos(-target_a+1*PI/2)));

    movement_type = 4;
}

void Robot::transport_position(){
    gimble.angle = TRANSPORT_ANGLE;
    claw.angle = OPEN_CLAW_VALUE;
    magasine.angle = MAGASINE_SLOT_3_ANGLE;

    Serial2.write((byte*)&gimble, sizeof(gimble));
    Serial2.write((byte*)&claw, sizeof(claw));
    Serial2.write((byte*)&magasine, sizeof(magasine));
    motors_ready[0] = 0;
    motors_ready[1] = 0;
    motors_ready[2] = 0;

    movement_type = 6;
}

void Robot::open_claw_half(){
    claw.angle = HALF_CLOSE_CLAW_VALUE;
    Serial2.write((byte*)&claw, sizeof(claw));
    motors_ready[0] = 0;

    movement_type = 7;
}

void Robot::gimble_in_position(int16_t position){
    if (position == 0){
        gimble.angle = STANDART_TUBE_PICK_ANGLE;
    }
    else if (position == 1){
        gimble.angle = PLACE_IN_MAGASINE_ANGLE;
    }
    else if (position == 2){
        gimble.angle = TRANSPORT_ANGLE;
    }
    
    Serial2.write((byte*)&gimble, sizeof(gimble));
    motors_ready[1] = 0;

    movement_type = 9;
}
void Robot::open_only_claw(int16_t position){
    if (position == 0){
        claw.angle = OPEN_CLAW_VALUE;
    }
    else if (position == 1){
        claw.angle = CLOSE_CLAW_VALUE;
    }
    
    Serial2.write((byte*)&claw, sizeof(claw));
    motors_ready[0] = 0;

    movement_type = 7;
}

void Robot::move_magasine(int16_t position){
    if (position == 0){
        magasine.angle = MAGASINE_SLOT_1_ANGLE;
    }
    if (position == 1){
        magasine.angle = MAGASINE_SLOT_2_ANGLE;
    }
    if (position == 3){
        magasine.angle = MAGASINE_SLOT_3_ANGLE;
    }
    if (position == 4){
        magasine.angle = 60;
    }

    Serial2.write((byte*)&magasine, sizeof(magasine));
    motors_ready[2] = 0;

    movement_type = 8;
}

void Robot::open_claw(int16_t type){
    if (type == 0){
        gimble.angle = STANDART_TUBE_PICK_ANGLE;
    }
    else if (type == 1){
        gimble.angle = UPPER_TUBE_PICK_ANGLE;
    }
    else if (type == 2){
        gimble.angle = LOWER_TUBE_PICK_ANGLE;
    }
    gimble.angle = STANDART_TUBE_PICK_ANGLE;
    claw.angle = OPEN_CLAW_VALUE;

    Serial2.write((byte*)&gimble, sizeof(gimble));
    Serial2.write((byte*)&claw, sizeof(claw));
    motors_ready[0] = 0;
    motors_ready[1] = 0;

    movement_type = 5;
}

void Robot::close_claw(int16_t type){
    if (type <= 0){
        gimble.angle = PLACE_IN_MAGASINE_ANGLE;
    }
    else{
        gimble.angle = TRANSPORT_ANGLE;
    }
    claw.angle = CLOSE_CLAW_VALUE;

    Serial2.write((byte*)&gimble, sizeof(gimble));
    Serial2.write((byte*)&claw, sizeof(claw));
    motors_ready[0] = 0;
    motors_ready[1] = 0;

    movement_type = 5;
}


void Robot::set_motor_speed(uint16_t number, u_int16_t speed){
    if (number == 0){
        left_motor.speed = speed;
        Serial1.write((byte*)&left_motor, sizeof(Motor_move_data));
    }
    if (number == 1){
        right_motor.speed = speed;
        Serial1.write((byte*)&right_motor, sizeof(Motor_move_data));
    }
}

void Robot::movement_tick(Stream &SerialX){
    if (sensor_update && odometry_update){
        int16_t target_speed = BASIC_SPEED;
        float error = 0;

        long double x_proj = 0, y_proj = 0;

        x_proj = (x * cos(-target_a+1*PI/2) - y * sin(-target_a+1*PI/2));
        y_proj = ((x * sin(-target_a+1*PI/2) + y * cos(-target_a+1*PI/2)));

        

        if (movement_type == 1){
            if (sensor_sum > 2){
                /*error = (left_motor_angle - left_motor_base_value)*1.12 - (right_motor_angle - right_motor_base_value);
                if (!on_cross2){
                    kP = kP2;
                    kI = kI2;
                    kD = kD2;
                }
                on_cross2 = 1;*/
                error = 0;
            }
            else{
                error = (line_error - target_line_error)*LINE_COEF;
                if (on_cross2){
                    kP = kP1;
                    kI = kI1;
                    kD = kD1;
                }
                on_cross2 = 0;
            }

            target_speed = 300 + MAIN_AXIS_COEF*abs(y_proj-prev_y_proj);

            if (sensor_sum > 6){
                if (on_cross == 0){
                    x = crosses_cors[cross_count][0];
                    y = crosses_cors[cross_count][1];
                    a = target_a;
                }
                on_cross = 1;
            }
            if (sensor_sum < 4 && on_cross){
                cross_count += 1;
                on_cross = 0;
            }
            if (cross_count == target_cross_counbt){
                if (after_move == 1){
                    //115
                    move_forward_line_distance(115, target_line_error, 0);
                }
                else if (after_move == 2){
                    //115
                    move_forward_line_distance(40, target_line_error, 0);
                }
                else{
                    target_speed = 0;
                    movement_type = 0;
                    set_motor_speed(0, 0);
                    set_motor_speed(1, 0);
                }
            }
        }
        else if (movement_type == 2){
            if (brake_mode != 0 && (abs(y_proj-prev_y_proj) < 100 || abs(target_y_proj-y_proj) < 100 || brake_mode == -1)){
                error = (left_motor_angle - left_motor_base_value)*1.14 - (right_motor_angle - right_motor_base_value);
                if (!on_cross2){
                    kP = kP2;
                    kI = kI2;
                    kD = kD2;
                    error_sum_1 = error_sum;
                    error_sum = error_sum_2;
                    last_error_1 = last_error;
                    last_error = last_error_2;
                    left_motor_base_value = left_motor_angle;
                    right_motor_base_value = right_motor_angle;
                }
                on_cross2 = 1;
                Serial.println(error);
            }
            else if(brake_mode != 0){
                error = (line_error_cut - target_line_error)*LINE_COEF;
                if (on_cross2){
                    kP = kP1;
                    kI = kI1;
                    kD = kD1;
                    error_sum_2 = error_sum;
                    error_sum = error_sum_1;
                    last_error_2 = last_error;
                    last_error = last_error_1;
                }
                on_cross2 = 0;
            }
            else if(brake_mode == 0){
                error = (line_error2 - target_line_error)*LINE_COEF;
                if (on_cross2){
                    kP = kP1;
                    kI = kI1;
                    kD = kD1;
                    error_sum_2 = error_sum;
                    error_sum = error_sum_1;
                    last_error_2 = last_error;
                    last_error = last_error_1;
                }
                on_cross2 = 0;
            }

            if (abs(y_proj-prev_y_proj) < abs(target_y_proj-y_proj)){
                target_speed = 300 + MAIN_AXIS_COEF*abs(y_proj-prev_y_proj);
            }
            else{
                target_speed = 200 + MAIN_AXIS_COEF*abs(target_y_proj-y_proj);
            }
            if (abs(target_y_proj-y_proj) < 5){
                target_speed = 0;
                movement_type = 0;
                if (brake_mode == 1){
                    set_motor_speed(0, 20);
                    set_motor_speed(1, 20);
                }
                else if (brake_mode == -1){
                    set_motor_speed(0, -20);
                    set_motor_speed(1, -20);
                }
                else{
                    set_motor_speed(0, 0);
                    set_motor_speed(1, 0);
                }
                
            }
            
        }
        else if (movement_type == 3){

            /*error = -(target_x_proj-x_proj) * SIDE_AXIS_COEF;*/

            error = ((left_motor_angle - left_motor_base_value)*1.12 - (right_motor_angle - right_motor_base_value));

            if (abs(y_proj-prev_y_proj) < abs(target_y_proj-y_proj)){
                target_speed = (400 + MAIN_AXIS_COEF*abs(y_proj-prev_y_proj))*direction;
            }
            else{
                target_speed = (300 + MAIN_AXIS_COEF*abs(target_y_proj-y_proj))*direction;
            }

            if (abs(target_y_proj-y_proj) < 5){
                target_speed = 0;
                movement_type = 0;
                if (brake_mode){
                    set_motor_speed(0, 20);
                    set_motor_speed(1, 20);
                }
                else{
                    set_motor_speed(0, 0);
                    set_motor_speed(1, 0);
                }
            }
        }
        else if (movement_type == 4){

            if (abs(a-prev_a) < abs(target_a-a)){
                target_speed = 200 + ANGLE_COEF*abs(a-prev_a);
            }
            else{
                target_speed = 200 + ANGLE_COEF*abs(target_a-a);
            }

            if (movement_axis == 0){
            //    error = (target_x-x) * SIDE_AXIS_COEF;
            }
            if (movement_axis == 1){
            //    error = (target_y-y) * SIDE_AXIS_COEF;
            }

            if (abs(target_a-a) < radians(2)){
                target_speed = 0;
                movement_type = 0;
                set_motor_speed(0, 0);
                set_motor_speed(1, 0);
            }

        }
        else if(movement_type == 5){
            if (motors_ready[0] && motors_ready[1]){
                movement_type = 0;
            }
        }
        else if(movement_type == 6){
            if (motors_ready[0] == 1 && motors_ready[1] == 1 && motors_ready[2] == 1){
                movement_type = 0;
            }
        }
        else if(movement_type == 7){
            if (motors_ready[0]){
                movement_type = 0;
            }
        }
        else if(movement_type == 8){
            if (motors_ready[2]){
                movement_type = 0;
            }
        }
        else if(movement_type == 9){
            if (motors_ready[1]){
                movement_type = 0;
            }
        }

        error = PID(error);

        if (target_speed > BASIC_SPEED){
            target_speed = BASIC_SPEED;
        }
        if (target_speed < -BASIC_SPEED){
            target_speed = -BASIC_SPEED;
        }

        if (error > 300){
            error = 300;
        }
        if (error < -300){
            error = -300;
        }
        SerialX.print(target_speed);
        SerialX.print(" ");
        SerialX.print(error);
        SerialX.print(" ");
        SerialX.print(movement_type);
        SerialX.print(" ");
        SerialX.print(movement_axis);
        SerialX.print(" ");
        SerialX.print(cross_count);
        SerialX.print("\t");
        SerialX.print(double(target_x));
        SerialX.print(" ");
        SerialX.print(double(target_y));
        SerialX.print(" ");
        SerialX.print(double(degrees(target_a)));
        SerialX.print("\t");
        SerialX.print(double(target_x_proj));
        SerialX.print(" ");
        SerialX.print(double(target_y_proj));
        SerialX.print("\t");
        SerialX.print(double(kP));
        SerialX.print(" ");
        SerialX.print(double(line_error));
        SerialX.print("\t");
        SerialX.print(double(x_proj));
        SerialX.print(" ");
        SerialX.print(double(y_proj));
        SerialX.print("\t");
        SerialX.print(double(x));
        SerialX.print(" ");
        SerialX.print(double(y));
        SerialX.print(" ");
        SerialX.print(double(degrees(a)));
        SerialX.print("\t");
        SerialX.print(double(left_motor_angle - left_motor_base_value)*1.12);
        SerialX.print(" ");
        SerialX.print(double(right_motor_angle - right_motor_base_value));
        SerialX.print("\t");

        for (int i=0; i<16; i++){
            SerialX.print(sensor_values[i]);
            SerialX.print(" ");
        }
        SerialX.println();

        if (movement_type == 1 || movement_type == 2 || movement_type == 3){
            set_motor_speed(0, target_speed-error);
            set_motor_speed(1, target_speed+error);
        }
        if (movement_type == 4){
            set_motor_speed(0, -target_speed*sign(target_a - a));
            set_motor_speed(1, target_speed*sign(target_a - a));
        }

        sensor_update = 0;
        odometry_update = 0;
    }
}
