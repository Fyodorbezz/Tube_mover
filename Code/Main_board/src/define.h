#include <Arduino.h>

#define MOTOTRS_DRIVER_RX 27
#define MOTOTRS_DRIVER_TX 14

#define CLAW_DRIVER_RX 16
#define CLAW_DRIVER_TX 17

#define SENSOR_CNTRL_1 33
#define SENSOR_CNTRL_2 25
#define SENSOR_CNTRL_3 13

#define SENSOR_SIGNAL_1 34
#define SENSOR_SIGNAL_2 32

#define BUTTON_PIN 4

//#define WHEEL_RADIUS 46/2
//#define WHEEL_RADIUS 37/2
#define WHEEL_RADIUS 34.5/2
//#define PLATFORM_WIDTH 226/2.0
//#define PLATFORM_WIDTH 215/2.0
#define PLATFORM_WIDTH 206/2.0

#define LINE_COEF 40
#define MAIN_AXIS_COEF 10
#define SIDE_AXIS_COEF 1
#define ANGLE_COEF 5.747*100

#define BASIC_SPEED 500

#define OPEN_CLAW_VALUE -5
#define CLOSE_CLAW_VALUE 42
#define HALF_CLOSE_CLAW_VALUE 24
#define STANDART_TUBE_PICK_ANGLE 2
#define UPPER_TUBE_PICK_ANGLE 31
#define LOWER_TUBE_PICK_ANGLE -11
#define PLACE_IN_MAGASINE_ANGLE 372
#define TRANSPORT_ANGLE 300
#define MAGASINE_SLOT_3_ANGLE 200
#define MAGASINE_SLOT_2_ANGLE 90
#define MAGASINE_SLOT_1_ANGLE 20
#define SAME_HEIGHT_GRABBING_DISTANCE 100
#define ABOVE_GRABBING_DISTANCE 160

struct Motor_move_data{
    public:
    short motor_number;
    short speed;
};

struct Claw_move_data{
    public:
    short motor_number;
    short angle;
};

struct Enc_data{
    public:
    int32_t motor_number;
    int32_t angle;
};

struct Motor_stop_data{
    int32_t motor_number;
    int32_t stop;
};

int16_t sign(long double val);

void claw_setup();

class Robot{
    public:
    long double x, y, a = PI/2;
    long double target_x = 0, target_y = 0, target_a = PI/2;
    long double prev_x = 0, prev_y = 0, prev_a = PI/2;
    long double target_x_proj, target_y_proj, prev_x_proj, prev_y_proj;
    bool movement_axis = 0;
    int16_t movement_type = 0;

    int16_t sensor_values[16];
    int16_t sensor_values2[16];
    int16_t sensor_raw_values[16];
    int16_t min_values[16] = {1261, 1286, 1377, 1637, 1104, 1147, 944, 1175, 3184, 2219, 2897, 3345, 2570, 2757, 2326, 2734,};
    int16_t max_values[16] = {1887, 1945, 1937, 2063, 1817, 1830, 1823, 1863, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,};
    int16_t min_values2[16] = {1117, 1196, 1303, 1602, 1008, 1041, 887, 1114, 3192, 2171, 2837, 3375, 2486, 4095, 2266, 2445,};
    int16_t max_values2[16] = {1981, 2037, 2006, 2096, 1950, 1998, 2001, 1980, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095,};
    int16_t middle_values[16];
    float line_error = 0;
    float line_error_cut = 0;
    int16_t sensor_sum = 0;
    int16_t sensor_sum2 = 0;
    float line_error2 = 0;
    bool sensor_update = 0;
    int16_t current_sensor_number=0;
    int16_t current_sensor_number_last=0;
    int16_t target_line_error = 0;
    bool on_cross=0;
    bool on_cross2=0;
    int16_t cross_count = 0;
    int16_t target_cross_counbt = 0;
    int16_t after_move = 0;

    double crosses_cors[8][2];

    bool motors_ready[3] = {0, 0, 0};
    int16_t tubes_inside = 0;

    long double left_motor_angle=0;
    long double left_motor_angle_last=0;
    bool left_motor_update = 0;

    long double right_motor_angle=0;
    long double right_motor_angle_last=0;
    bool right_motor_update = 0;

    long double left_motor_base_value = 0, right_motor_base_value;

    bool odometry_update = 0;

    Motor_move_data left_motor;
    Motor_move_data right_motor;

    //double kP1 = 7.25, kD1 = 0.3, kI1 = 0.3;
    double kP1 = 0.4, kD1 = 0.04, kI1 = 0.02;
    double kP2 = 5, kD2 = 1.5, kI2 = 1.5;

    long double error_sum_1 = 0, error_sum_2 = 0;
    long double last_error_1 = 0, last_error_2 = 0;

    double kP = 0, kD = 0, kI = 0;
    long double error, last_error, error_sum, error_rate, eP, eD, eI;
    int brake_mode = 1;
    bool aftermove = 0;
    int16_t direction = 0;

    bool phase = 0;

    uint64_t mode_change_timer = 0;

    Robot(){
        left_motor.motor_number = 0;
        right_motor.motor_number = 1;

        for (int16_t i = 0; i<16; i++){
            middle_values[i] = (max_values[i] + min_values[i])/2;
        }
    }

    void odometry(Stream &Serialx);
    void get_sensor_value(Stream &SerialX);

    void move_forward_line_distance(int32_t distance, int16_t Line_error, int brake);
    void move_forward_line_crosses(int32_t crosses, int16_t Line_error, int After_move);
    void move_forward_distance(int32_t distance, bool brake);

    void rotate(long double angle);

    void open_claw(int16_t type);
    void close_claw(int16_t type);
    void transport_position();
    void open_claw_half();
    void move_magasine(int16_t position);
    void gimble_in_position(int16_t position);
    void open_only_claw(int16_t position);

    void set_motor_speed(uint16_t number, u_int16_t speed);

    void movement_tick(Stream &SerialX);

    double PID(double error){
        eP = error*kP;
        error_rate = error-last_error;
        eD = error_rate*kD;
        last_error = error;
        error_sum += error;
        if (error_sum > 10000){
            error_sum = 10000;
        }
        if (error_sum < -10000){
            error_sum = -10000;
        }
        if (sign(error) != sign(last_error)){
            error_sum = 0;
        }
        eI = error_sum * kI;
        return (eP + eD + eI);
    }
    
};