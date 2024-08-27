#include <Arduino.h>
#include <PinChangeInterrupt.h>



#define MOTOR1_DIR 7
#define MOTOR1_SPEED 6
#define MOTOR1_ENC_A A0
#define MOTOR1_ENC_B A1
#define MOTOR1_INV 1
#define MOTOR1_ENC_INV 1
#define MOTOR1_ENC_RESOLUTION 0.2

#define MOTOR1_D1 50
#define MOTOR1_D2 20


#define MOTOR2_DIR 8
#define MOTOR2_SPEED 9
#define MOTOR2_ENC_A 4
#define MOTOR2_ENC_B 5
#define MOTOR2_INV 1
#define MOTOR2_ENC_INV 1
#define MOTOR2_ENC_RESOLUTION 0.2
#define MOTOR2_A 0.036
#define MOTOR2_B 8.99
#define MOTOR2_kP 0.05
#define MOTOR2_kD 0.03
#define MOTOR2_kI 0.01

#define MOTOR2_D1 70
#define MOTOR2_D2 40

#define MOTOR3_DIR A2
#define MOTOR3_SPEED 10
#define MOTOR3_ENC_A 2
#define MOTOR3_ENC_B 3
#define MOTOR3_INV 0
#define MOTOR3_ENC_INV 1
#define MOTOR3_ENC_RESOLUTION 0.6
#define MOTOR3_A 0.10
#define MOTOR3_B 6.77
#define MOTOR3_kP 0.15
#define MOTOR3_kD 0.07
#define MOTOR3_kI 0.03

#define MOTOR3_D1 100
#define MOTOR3_D2 100


#define MOTOR4_DIR A3
#define MOTOR4_SPEED 11
#define MOTOR4_ENC_A 12
#define MOTOR4_ENC_B 13
#define MOTOR4_INV 0
#define MOTOR4_ENC_INV 0
#define MOTOR4_ENC_RESOLUTION 360/(6*100)

#define MINIMAL_SPEED 1

#define CLAW_ENDSTOP A3
#define GIMBLE_ENDSTOP 11
#define MAGASINE_ENDSTOP 12

const char enct_table [16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

/*static const short motor1_table [256] = 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

static const short motor2_table [256] = 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

static const short motor3_table [256] = 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 

static const short motor4_table [256] = 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};*/


short sign(int value){
    if (value > 0){
        return 1;
    }
    if (value == 0){
        return 0;
    }
    return -1;
}

struct Motor_stop_data{
    int32_t motor_number;
    int32_t stop;
};

struct Claw_move_data{
    public:
    short motor_number;
    short angle;
};

class Motor_controll{
    public:
    long long int encoder_ticks = 0;
    byte encoder_state = 0;

    long long int encoders_tick_last = 0;
    long long int speed_calcuation_time = 0;
    long long int acelearation_calcuation_time = 0;

    long double motor_angle = 0;
    long double motor_speed = 0;
    long double motor_speed_last = 0;
    long double motor_axeleration = 0;

    long double target_axeleration = 0;
    long double PID_target_speed = 0;
    long double target_speed = 0;
    long double target_angle = 0;
    bool motor_mode = 0;
    /* 0 - target speed
       1 - target position*/

    short int motor_duty = 0;

    float motor_kP = 0;
    float motor_kD = 0;
    float motor_kI = 0;

    double last_error = 0;
    double error_sum = 0;

    float error;
    float error_rate;
    float eP;
    float eD;
    float eI;
    float PID_result;
    float slope_result;

    unsigned long long speed_calculation_timer = 0;
    unsigned long long PID_timer = 0; 

    //const short* motor_table;
    const byte motor_dir_pin;
    const byte motor_speed_pin;
    const byte motor_enc_A_pin;
    const byte motor_enc_B_pin;
    const bool motor_inv;
    const bool enc_inv;
    const float enc_resolution;
    const float motor_A;
    const float motor_B;
    short result;

    bool motor_stop_flag = 1;
    bool send_stop_flag = 0;
    Motor_stop_data stop_data;
    
    
    Motor_controll(int16_t Motor_number, const byte Motor_dir_pin, const byte Motor_speed_pin, 
    const byte Motor_enc_A_pin, const byte Motor_enc_B_pin, const bool Motor_inv, const bool Enc_inv, 
    const float Enc_resolution, const float Motor_A, const float Motor_B, const float Motor_kP, const float Motor_kI, const float Motor_kD): 
    /*motor_table(Motor_table)*/ motor_dir_pin(Motor_dir_pin), motor_speed_pin(Motor_speed_pin), motor_enc_A_pin(Motor_enc_A_pin),
    motor_enc_B_pin(Motor_enc_B_pin), motor_inv(Motor_inv), enc_inv(Enc_inv), enc_resolution(Enc_resolution), motor_A(Motor_A), motor_B(Motor_B),
    motor_kD (Motor_kD), motor_kI (Motor_kI), motor_kP (Motor_kP){
        stop_data.motor_number = Motor_number;
        stop_data.stop = 1;
        pinMode(motor_dir_pin, OUTPUT);
        pinMode(motor_speed_pin, OUTPUT);
        pinMode(Motor_enc_A_pin, INPUT);
        pinMode(Motor_enc_B_pin, INPUT);
    }

    void enc_tick(){
        encoder_state = ((encoder_state << 2) & 0b1100) | ((digitalRead(motor_enc_A_pin) << 1) & 0b10) | (digitalRead(motor_enc_B_pin)& 0b1);
        encoder_ticks += enct_table[encoder_state] * (1-2*enc_inv);
    }

    void calculate_speed(){
        motor_angle = encoder_ticks*enc_resolution;
        motor_speed = ((encoder_ticks-encoders_tick_last)*enc_resolution)/((millis() - speed_calcuation_time)/1000.0);
        encoders_tick_last = encoder_ticks;
        speed_calcuation_time = millis();

        motor_axeleration = (motor_speed-motor_speed_last)/(millis() - acelearation_calcuation_time);
        motor_speed_last = motor_speed;
        acelearation_calcuation_time = millis();
    }

    void set_duty(){
        if (motor_duty > 0){
            digitalWrite(motor_dir_pin, 1-motor_inv);
        }
        else{
            digitalWrite(motor_dir_pin, 0+motor_inv);
        }
        analogWrite(motor_speed_pin, abs(motor_duty));
    }

    void set_duty(short Motor_duty){
        motor_duty = Motor_duty;
        if (motor_duty > 0){
            digitalWrite(motor_dir_pin, 1-motor_inv);
        }
        else{
            digitalWrite(motor_dir_pin, 0+motor_inv);
        }
        analogWrite(motor_speed_pin, abs(motor_duty));
    }

    void PID_controll(){
        error = (PID_target_speed - motor_speed);
        eP = error*motor_kP;
        error_rate = error-last_error;
        eD = error_rate*motor_kD;
        last_error = error;
        error_sum += error;
        if (error_sum > 10000){
            error_sum = 10000;
        }
        if (error_sum < -10000){
            error_sum = -10000;
        }
        eI = error_sum * motor_kI;
        PID_result = eP + eD + eI;
        slope_result = PID_target_speed*motor_A + motor_B;
        result = int(PID_result + slope_result + 0.5);
        set_duty(result); 
    }

    void path_planner(){
        if (abs(PID_target_speed) < abs(target_speed)){
            PID_target_speed += target_axeleration*sign(target_speed);
        }
        if (abs(PID_target_speed) > abs(target_speed)){
            PID_target_speed = target_speed;
        }

        if (motor_mode && abs((target_speed*target_speed)/(2*abs(target_axeleration))) > abs(target_angle-motor_angle) && abs(PID_target_speed) > abs(MINIMAL_SPEED)){
            PID_target_speed -= target_axeleration*sign(target_speed);
        }

        if (abs(target_angle-motor_angle) <= 1 && motor_mode){
            target_speed = 0;
            PID_target_speed = 0;
            target_angle = motor_angle;

            motor_duty = 0;
            set_duty();
        }
    }

    void generate_table(){
        for (int i=5; i<80; i+=2){
            set_duty(i);
            unsigned long long int timer = millis();
            short counter = 0;
            while(1){
                if (millis() - timer > 20){
                    calculate_speed();
                    timer = millis();
                    counter ++;
                    if (counter >= 100){
                        break;
                    } 
                }
            }
            //Serial.print(int(motor_speed+0.5));
            //Serial.print(", ");
        }
    }

    void stop_at_angle(){
        double duty;
        duty = (target_angle-motor_angle)*5;

        if (abs(duty) > motor_A){
            duty = motor_A*sign(duty);
        }

        if (abs(duty) < motor_B){
            duty = motor_B*sign(duty);
        }

        //Serial.println(duty);

        if(abs(target_angle-motor_angle) <= 2){
            set_duty(0);
            if (!motor_stop_flag){
                motor_stop_flag = 1;
                send_stop_flag = 1;
            }

        }
        else{
            motor_stop_flag = 0;
            set_duty(duty);
        }
    }

    void move_by(float angle){
        target_angle += angle;
        if (angle > 0){
            set_duty(motor_A);
        }
        else{
            set_duty(motor_B);
        }
        
    }

    void generate_slope(){
        set_duty(30);
        delay(2);
        unsigned long long int timer = millis();
        short counter = 0;
        long double speed_sum = 0;
        while(1){
            if (millis() - timer > 20){
                calculate_speed();
                timer = millis();
                counter ++;
                speed_sum += motor_speed;
                if (counter >= 100){
                    break;
                } 
            }
        }
        //Serial.println(int(speed_sum/counter+0.5));
        set_duty(100);
        delay(2);
        timer = millis();
        counter = 0;
        speed_sum = 0;
        while(1){
            if (millis() - timer > 20){
                calculate_speed();
                timer = millis();
                counter ++;
                speed_sum += motor_speed;
                if (counter >= 100){
                    break;
                } 
            }
        }
        //Serial.println(int(speed_sum/counter+0.5));
    }

    void tick(){
        if (millis() - speed_calculation_timer > 2){
            calculate_speed();
            speed_calculation_timer = millis();
        }
        if (millis() - PID_timer > 5){
            stop_at_angle();

            /*Serial.print(float(motor_angle));
            Serial.print(" ");
            Serial.print(float(motor_duty));
            Serial.print(" ");
            Serial.println(float(target_angle));*/


            /*PID_controll();
            PID_timer = millis();

            Serial.print(double(motor_speed));
            Serial.print(",");
            Serial.print(double(PID_target_speed));
            Serial.print(",");
            Serial.print(double(error));
            Serial.print(",");
            Serial.print(double(error_sum)/10);
            Serial.print(",");
            Serial.print(double(error_rate));
            Serial.print(",");
            Serial.println(double(motor_duty));*/
        }
    }

};