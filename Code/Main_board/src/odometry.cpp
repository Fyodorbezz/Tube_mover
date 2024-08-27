#include <define.h>

void Robot::odometry(Stream &Serialx){
    if (left_motor_update && right_motor_update){
        long double L1, L2, dx, dy, da, r;

        L1 = radians(left_motor_angle - left_motor_angle_last)*WHEEL_RADIUS*1.12;//1.096;
        L2 = radians(right_motor_angle - right_motor_angle_last)*WHEEL_RADIUS;

        if (L1 == L2){
            x += L1 * cos(a);
            y += L1 * sin(a);
        }
        else {
            r = PLATFORM_WIDTH * (1 + (2 * L1) / (L2 - L1));
            da = (L2 - L1) / (2 * PLATFORM_WIDTH);
            dx = r * sin(da);
            dy = r * (1 - cos(da));

            x += (dx * cos(a) - dy * sin(a));
            y += (dx * sin(a) + dy * cos(a));
            a += da;
        }
        /*Serialx.print(double(radians(left_motor_angle)));
        Serialx.print(" ");
        Serialx.print(double(radians(left_motor_angle)*WHEEL_RADIUS));
        Serialx.print(" ");
        Serialx.print(double(right_motor_angle*WHEEL_RADIUS));
        Serialx.print(" ");
        Serialx.print(double(x));
        Serialx.print(" ");
        Serialx.print(double(y));
        Serialx.print(" ");
        Serialx.println(degrees(double(a)));*/

        left_motor_update = 0;
        right_motor_update = 0;

        left_motor_angle_last = left_motor_angle;
        right_motor_angle_last = right_motor_angle;

        odometry_update = 1;
    }
}

void Robot::get_sensor_value(Stream &Serialx){
    sensor_raw_values[8+current_sensor_number] = analogRead(SENSOR_SIGNAL_1);
    sensor_raw_values[current_sensor_number] = analogRead(SENSOR_SIGNAL_2);
    sensor_values[8+current_sensor_number] = int((float(sensor_raw_values[8+current_sensor_number]-min_values[8+current_sensor_number])*100)/(max_values[8+current_sensor_number]-min_values[8+current_sensor_number]));
    sensor_values[current_sensor_number] = int((float(sensor_raw_values[current_sensor_number]-min_values[current_sensor_number])*100)/(max_values[current_sensor_number]-min_values[current_sensor_number]));

    sensor_values2[8+current_sensor_number] = int((float(sensor_raw_values[8+current_sensor_number]-min_values2[8+current_sensor_number])*100)/(max_values2[8+current_sensor_number]-min_values2[8+current_sensor_number]));
    sensor_values2[current_sensor_number] = int((float(sensor_raw_values[current_sensor_number]-min_values2[current_sensor_number])*100)/(max_values2[current_sensor_number]-min_values2[current_sensor_number]));
    
    //sensor_values[13] = 0;

    //Serialx.println(digitalRead(SENSOR_SIGNAL_1));

    current_sensor_number ++;
    if (current_sensor_number >= 8){
        current_sensor_number = 0;
        

        sensor_sum = 0;
        float sensor_weighted_sum = 0;
        float sensor_weighted_sum2 = 0;
        int16_t sensor_count = 0;
        for (int16_t i = 0; i < 16; i++){
            sensor_sum += (sensor_values[i] > 70);
            sensor_sum2 += (sensor_values2[i] > 70);
            if (sensor_values2[i] >= 40){
                sensor_weighted_sum2 += ((i + int(i>7)) - 8) * sensor_values2[i];
            }
            if (sensor_values[i] >= 40){
                sensor_weighted_sum += ((i + int(i>7)) - 8) * sensor_values[i];
            }
            sensor_count += 1;

            //Serialx.print(sensor_raw_values[i]);
            //Serialx.print(", ");
        }
        if (sensor_count == 0){
            line_error = 0;
        }
        else{
            line_error = sensor_weighted_sum/sensor_count;
            line_error2 = sensor_weighted_sum2/sensor_count;
        }
        sensor_weighted_sum = 0;
        sensor_count = 0;
        for (int16_t i = 4; i < 12; i++){
            if (sensor_values2[i] >= 40){
                sensor_weighted_sum += ((i + int(i>7)) - 8) * sensor_values2[i];
            }
            sensor_count += 1;
        }
        if (sensor_count == 0){
            line_error_cut = 0;
        }
        else{
            line_error_cut = sensor_weighted_sum/sensor_count;
        }
        //Serialx.print(" ");
        //Serialx.print(sensor_weighted_sum);
        //Serialx.print(" ");
        //Serialx.print(sensor_count);
        //Serialx.println(" ");
        
        sensor_update = 1;
        //Serialx.println(line_error);
    }

    /*digitalWrite(SENSOR_CNTRL_1, bool(current_sensor_number%2));
    digitalWrite(SENSOR_CNTRL_2, bool((current_sensor_number / 2)%2));
    digitalWrite(SENSOR_CNTRL_3, bool(current_sensor_number / 4));*/
    digitalWrite(SENSOR_CNTRL_1, bool(current_sensor_number%2));
    digitalWrite(SENSOR_CNTRL_2, bool((current_sensor_number / 2)%2));
    digitalWrite(SENSOR_CNTRL_3, bool(current_sensor_number / 4));

    /*Serialx.print(bool(current_sensor_number%2));
    Serialx.print(" ");
    Serialx.print(bool((current_sensor_number / 2)%2));
    Serialx.print(" ");
    Serialx.println(bool(current_sensor_number / 4));*/
}
