#include <Arduino.h>
#include "can_serial.h"
#include "driver/pcnt.h"

#define ENC_LEFT_FRONT_A 5
#define ENC_LEFT_FRONT_B 22

const int pwm_left_front_pin1 = 25;
const int pwm_left_front_pin2 = 26;

#define ENC_RIGHT_FRONT_A 4
#define ENC_RIGHT_FRONT_B 21

const int pwm_right_front_pin1 = 32;
const int pwm_right_front_pin2 = 33;

#define ENC_LEFT_BACK_A 19
#define ENC_LEFT_BACK_B 35

const int pwm_left_back_pin1 = 12;
const int pwm_left_back_pin2 = 13;

#define ENC_RIGHT_BACK_A 18
#define ENC_RIGHT_BACK_B 23

const int pwm_right_back_pin1 = 27;
const int pwm_right_back_pin2 = 14;

// #define RXD2 16
// #define TXD2 17

// TaskHandle_t Task_Run_CAN; // Task_Run_CAN
// CAN_manager* MainCAN = new CAN_manager(CAN_BAUD_SPEED, CAN_TX, CAN_RX, CAN_FRAME, CAN_ID, CAN_SEND_SIZE);
// Main_controller* MainCtrl = new Main_controller(MainCAN);
CanSerial CSerial(Serial);

DataPacket data_recieve_can_serial; 
DataPacket data_send_can_serial;

// Time interval for measurements in milliseconds
const int interval = 50;
long previousMillis = 0;
long currentMillis = 0;

////////////////// Motor Controller Variables and Constants ///////////////////
 
// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;
 
// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 2000; //620: origin - 800: for left wheel real - 810: for right wheel real
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.035;
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.27;

// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 9100; // Originally 2880
   
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 0; // about 0.1 m/s
const int PWM_MAX = 255; // about 0.172 m/s

// var of left front wheel
float Kp_lf = 4.2;
float Ki_lf = 0.1;
float Kd_lf = 0.3;

int16_t tickLeftFront = 0;
double velLeftFront = 0;
int pwmLeftFrontBase = 0;
int pwmLeftFrontReq = 0;
int pwmLeftFrontOut = 0;
int cmdvel_left_front = 0;
double time_cmdvel_leftfront_recv = 0;
int is_recv_left_front = 0;
int left_front_dir = 0;
int pre_left_front_dir = 0;

long lastTimeLeftFront = 0;
float lastError_leftfront = 0;
float integral_leftfront = 0;

// var of right front wheel
float Kp_rf = 5.0; // Proportional gain
float Ki_rf = 0.2; // Integral gain
float Kd_rf = 1.0; // Derivative gain

int16_t tickRightFront = 0;
double velRightFront = 0;
int pwmRightFrontBase = 0;
int pwmRightFrontReq = 0;
int pwmRightFrontOut = 0;
int cmdvel_right_front = 0;
double time_cmdvel_rightfront_recv = 0;
int is_recv_right_front = 0;
int right_front_dir = 0;
int pre_right_front_dir = 0;

long lastTimeRightFront = 0;
float lastError_rightfront = 0;
float integral_rightfront = 0;

// var of left back wheel
float Kp_lb = 4.2;
float Ki_lb = 0.1;
float Kd_lb = 0.3;

int16_t tickLeftBack = 0;
double velLeftBack = 0;
int pwmLeftBackBase = 0;
int pwmLeftBackReq = 0;
int pwmLeftBackOut = 0;
int cmdvel_left_back = 0;
double time_cmdvel_leftback_recv = 0;
int is_recv_left_back = 0;
int left_back_dir = 0;
int pre_left_back_dir = 0;

long lastTimeLeftBack = 0;
float lastError_leftback = 0;
float integral_leftback = 0;

// var of right back wheel
float Kp_rb = 4.5;
float Ki_rb = 0.1;
float Kd_rb = 0.5;

int16_t tickRightBack = 0;
double velRightBack = 0;
int pwmRightBackBase = 0;
int pwmRightBackReq = 0;
int pwmRightBackOut = 0;
int cmdvel_right_back = 0;
double time_cmdvel_rightback_recv = 0;
int is_recv_right_back = 0;
int right_back_dir = 0;
int pre_right_back_dir = 0;

long lastTimeRightBack = 0;
float lastError_rightback = 0;
float integral_rightback = 0;

int is_recv_cmdvel = 0;

// Calculate the left front wheel linear velocity in m/s every time a 
void calc_vel_left_front(){
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (32768 + tickLeftFront - prevLeftCount) % 32768;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (32768 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per millisecond
  velLeftFront = 2*numOfTicks/TICKS_PER_METER/(millis()-prevTime);

  // Calculate right wheel velocity in RPM
  velLeftFront = velLeftFront*30000.0/PI/WHEEL_RADIUS;

  // Keep track of the previous tick count
  prevLeftCount = tickLeftFront;
 
  // Update the timestamp
  prevTime = millis();
 
}

void calc_vel_right_front(){
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (32768 + tickRightFront - prevRightCount) % 32768;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (32768 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per millisecond
  velRightFront = 2*numOfTicks/TICKS_PER_METER/(millis()-prevTime);

  // Calculate right wheel velocity in RPM
  velRightFront = velRightFront*30000.0/PI/WHEEL_RADIUS;

  // Keep track of the previous tick count
  prevRightCount = tickRightFront;
 
  // Update the timestamp
  prevTime = millis();
 
}

// Calculate the left back wheel linear velocity in m/s every time a 
void calc_vel_left_back(){
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (32768 + tickLeftBack - prevLeftCount) % 32768;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (32768 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per millisecond
  velLeftBack = 2*numOfTicks/TICKS_PER_METER/(millis()-prevTime);

  // Calculate right wheel velocity in RPM
  velLeftBack = velLeftBack*30000.0/PI/WHEEL_RADIUS;

  // Keep track of the previous tick count
  prevLeftCount = tickLeftBack;
 
  // Update the timestamp
  prevTime = millis();
 
}

void calc_vel_right_back(){
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (32768 + tickRightBack - prevRightCount) % 32768;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (32768 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per millisecond
  velRightBack = 2*numOfTicks/TICKS_PER_METER/(millis()-prevTime);

  // Calculate right wheel velocity in RPM
  velRightBack = velRightBack*30000.0/PI/WHEEL_RADIUS;

  // Keep track of the previous tick count
  prevRightCount = tickRightBack;
 
  // Update the timestamp
  prevTime = millis();
 
}

int gain_dir(int x){
   if( x > 0){
      return 1;                                   // tiến trước 
   }
   else if( x < 0){
      return 2;                                   // lùi
   }
   else{                                         // đứng yên
      return 0;
   }
}

void set_pwm_values(int channel1, int channel2, int dir, int pwm_val) {
    if (dir == 1) { // tiến trước
      ledcWrite(channel1, pwm_val);
      ledcWrite(channel2, 0);
    }
    else if (dir == 2) { // lùi sau
      ledcWrite(channel1, 0);
      ledcWrite(channel2, pwm_val);
    }
    else {
      ledcWrite(channel1, 0);
      ledcWrite(channel2, 0);
    }
}

void pwm_pin_setup(){
  // --
  ledcSetup(0, 2000, 8);
  ledcAttachPin(pwm_left_front_pin1, 0);
  ledcWrite(0, 0);

  ledcSetup(1, 2000, 8);
  ledcAttachPin(pwm_left_front_pin2, 1);
  ledcWrite(1, 0);

  // **
  ledcSetup(2, 2000, 8);
  ledcAttachPin(pwm_right_front_pin1, 2);
  ledcWrite(2, 0);

  ledcSetup(3, 2000, 8);
  ledcAttachPin(pwm_right_front_pin2, 3);
  ledcWrite(3, 0);

  // --
  ledcSetup(4, 2000, 8);
  ledcAttachPin(pwm_left_back_pin1, 4);
  ledcWrite(4, 0);

  ledcSetup(5, 2000, 8);
  ledcAttachPin(pwm_left_back_pin2, 5);
  ledcWrite(5, 0);

  // **
  ledcSetup(6, 2000, 8);
  ledcAttachPin(pwm_right_back_pin1, 6);
  ledcWrite(6, 0);

  ledcSetup(7, 2000, 8);
  ledcAttachPin(pwm_right_back_pin2, 7);
  ledcWrite(7, 0);
}

void setupPCNT(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t pcntConfig;
  pcntConfig.pulse_gpio_num = pinA;
  pcntConfig.ctrl_gpio_num = pinB;
  pcntConfig.channel = PCNT_CHANNEL_0;
  pcntConfig.unit = unit;
  pcntConfig.pos_mode = PCNT_COUNT_INC;
  pcntConfig.neg_mode = PCNT_COUNT_DEC;
  pcntConfig.lctrl_mode = PCNT_MODE_REVERSE;
  pcntConfig.hctrl_mode = PCNT_MODE_KEEP;
  pcntConfig.counter_h_lim = 32767;
  pcntConfig.counter_l_lim = -32768;

  pcnt_unit_config(&pcntConfig);

  // Bật bộ lọc tín hiệu để chống nhiễu (chặn xung < 1000 ns)
  pcnt_set_filter_value(unit, 1000); // nanosec
  pcnt_filter_enable(unit);

  // Xóa bộ đếm
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

int computePID(float Kp, float Ki, float Kd, float target_vel, float real_vel, long lastTime, float last_error, float integral) {
  // static float last_error = 0;
  // static float integral = 0;

  float error = target_vel - real_vel;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // đổi ra giây
  lastTime = now;

  integral += error * dt;
  integral = constrain(integral, -100, 100);

  float derivative = (error - last_error) / dt;
  last_error = error;

  int output = Kp * error + Ki * integral + Kd * derivative;
  return output;
}

void setup() {
    CSerial.begin(115200);
    // Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 

    // pin setup  
    setupPCNT(PCNT_UNIT_0, ENC_LEFT_FRONT_A, ENC_LEFT_FRONT_B);
    setupPCNT(PCNT_UNIT_1, ENC_RIGHT_FRONT_A, ENC_RIGHT_FRONT_B);

    setupPCNT(PCNT_UNIT_2, ENC_LEFT_BACK_A, ENC_LEFT_BACK_B);
    setupPCNT(PCNT_UNIT_3, ENC_RIGHT_BACK_A, ENC_RIGHT_BACK_B);

    pwm_pin_setup();
}

void loop() {

    if (CSerial.readPacket(data_send_can_serial)){
        left_front_dir = data_send_can_serial.data[0];
        cmdvel_left_front = data_send_can_serial.data[1];

        right_front_dir = data_send_can_serial.data[2];
        cmdvel_right_front = data_send_can_serial.data[3];

        left_back_dir = data_send_can_serial.data[4];
        cmdvel_left_back = data_send_can_serial.data[5];

        right_back_dir = data_send_can_serial.data[6];
        cmdvel_right_back = data_send_can_serial.data[7];

        // Serial2.printf("left_front_dir = %d, cmdvel_left_front = %d\n", left_front_dir, cmdvel_left_front);
        // Serial2.printf("right_front_dir = %d, cmdvel_right_front = %d\n", right_front_dir, cmdvel_right_front);
        // Serial2.printf("left_back_dir = %d, cmdvel_left_back = %d\n", left_back_dir, cmdvel_left_back);
        // Serial2.printf("right_back_dir = %d, cmdvel_right_back = %d\n", right_back_dir, cmdvel_right_back);

        is_recv_cmdvel = 1;
    }
  
    pcnt_get_counter_value(PCNT_UNIT_0, &tickLeftFront);
    pcnt_get_counter_value(PCNT_UNIT_1, &tickRightFront);
    pcnt_get_counter_value(PCNT_UNIT_2, &tickLeftBack);
    pcnt_get_counter_value(PCNT_UNIT_3, &tickRightBack);

    // Record the time
    currentMillis = millis();
    
    // If the time interval has passed, publish the number of ticks, and calculate the velocities.
    if (currentMillis - previousMillis > interval) {
        previousMillis = currentMillis;
    
        // Publish tick counts to topics
        // leftFrontTickPub.publish( &left_front_tick);
        // rightFrontTickPub.publish( &right_front_tick);
        // leftBackTickPub.publish( &left_back_tick);
        // rightBackTickPub.publish( &right_back_tick);

        calc_vel_left_front();
        // leftFrontVelPub.publish(&left_front_vel);

        calc_vel_right_front();
        // rightFrontVelPub.publish(&right_front_vel);

        calc_vel_left_back();
        // leftBackVelPub.publish(&left_back_vel);

        calc_vel_right_back();
        // rightBackVelPub.publish(&right_back_vel);

    }


    // control left front motor
    if (is_recv_cmdvel == 1){
        // determine direction of wheel and calculate PWM request
        // left_front_dir = gain_dir(cmdvel_left_front);

        pwmLeftFrontBase = map(abs(cmdvel_left_front), 0, 60, 0, 255);

        int pid_compensate = computePID(Kp_lf, Ki_lf, Kd_lf, abs(cmdvel_left_front), abs(velLeftFront), lastTimeLeftFront, lastError_leftfront, integral_leftfront);
        pwmLeftFrontReq = pwmLeftFrontBase + pid_compensate;

        // Calculate the output PWM value by making slow changes to the current value
        if (abs(pwmLeftFrontReq) > pwmLeftFrontOut) {
            pwmLeftFrontOut += PWM_INCREMENT;
        }
        else if (abs(pwmLeftFrontReq) < pwmLeftFrontOut) {
            pwmLeftFrontOut -= PWM_INCREMENT;
        }
        
        // Conditional operator to limit PWM output at the maximum 
        pwmLeftFrontOut = constrain(pwmLeftFrontOut, PWM_MIN, PWM_MAX);

        // Case 1: robot stop => reset variables
        if (left_front_dir == 0 || pwmLeftFrontReq == 0) {
            pwmLeftFrontReq = 0;
            pwmLeftFrontOut = 0;
            lastError_leftfront = 0.0;
            integral_leftfront = 0.0;

        }

        // Case 2: robot change dir => stop robot before running
        if (pre_left_front_dir != left_front_dir){
            pre_left_front_dir = left_front_dir;
            pwmLeftFrontReq = 0;
            pwmLeftFrontOut = 0;
            lastError_leftfront = 0.0;
            integral_leftfront = 0.0;

        }
        
        // Case 3: real wheel vel more than target vel => stop robot before running
        // float delta_vel = abs(velLeftFront) - abs(cmdvel_left_front);
        // if (delta_vel > 5) {
        //     pwmLeftFrontReq = 0;
        //     pwmLeftFrontOut = 0;
        // }

        // Stop the car if there are no cmd_vel messages
        // if((millis()/1000) - time_cmdvel_leftfront_recv > 1) {
        // pwmLeftFrontReq = 0;
        // pwmLeftFrontOut = 0;  
        // }

        // out pwm to motor
        set_pwm_values(0, 1, left_front_dir, pwmLeftFrontOut);
    }
    else{
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }

    // control right front motor
    if (is_recv_cmdvel == 1){
        // determine direction of wheel and calculate PWM request
        // right_front_dir = gain_dir(cmdvel_right_front);
        pwmRightFrontBase = map(abs(cmdvel_right_front), 0, 60, 0, 255);

        // use pid for calculate pwm value
        int pid_compensate = computePID(Kp_rf, Ki_rf, Kd_rf, abs(cmdvel_right_front), abs(velRightFront), lastTimeRightFront, lastError_rightfront, integral_rightfront);
        pwmRightFrontReq = pwmRightFrontBase + pid_compensate;

        // Calculate the output PWM value by making slow changes to the current value
        if (abs(pwmRightFrontReq) > pwmRightFrontOut) {
            pwmRightFrontOut += PWM_INCREMENT;
        }
        else if (abs(pwmRightFrontReq) < pwmRightFrontOut) {
            pwmRightFrontOut -= PWM_INCREMENT;
        }
        
        // Conditional operator to limit PWM output at the maximum 
        pwmRightFrontOut = constrain(pwmRightFrontOut, PWM_MIN, PWM_MAX);

        // Case 1: robot stop => reset variables
        if (right_front_dir == 0 || pwmRightFrontReq == 0) {
            pwmRightFrontReq = 0;
            pwmRightFrontOut = 0;
            lastError_rightfront = 0.0;
            integral_rightfront = 0.0;

        }

        // Case 2: robot change dir => stop robot before running
        if (pre_right_front_dir != right_front_dir){
            pre_right_front_dir = right_front_dir;
            pwmRightFrontReq = 0;
            pwmRightFrontOut = 0;
            lastError_rightfront = 0.0;
            integral_rightfront = 0.0;   
        }
        
        // Case 3: real wheel vel more than target vel => stop robot before running
        // float delta_vel = abs(velRightFront) - abs(cmdvel_right_front);
        // if (delta_vel > 5) {
        //     pwmRightFrontReq = 0;
        //     pwmRightFrontOut = 0;
        // }

        // Stop the car if there are no cmd_vel messages
        // if((millis()/1000) - time_cmdvel_rightfront_recv > 1) {
        // pwmRightFrontReq = 0;
        // pwmRightFrontOut = 0;  
        // }

        // out pwm to motor
        set_pwm_values(2, 3, right_front_dir, pwmRightFrontOut);
    }
    else{
        ledcWrite(2, 0);
        ledcWrite(3, 0);    
    }

    // control left back motor
    if (is_recv_cmdvel == 1){
        // determine direction of wheel and calculate PWM request
        // left_back_dir = gain_dir(cmdvel_left_back);
        pwmLeftBackBase = map(abs(cmdvel_left_back), 0, 60, 0, 255);

        int pid_compensate = computePID(Kp_lb, Ki_lb, Kd_lb, abs(cmdvel_left_back), abs(velLeftBack), lastTimeLeftBack, lastError_leftback, integral_leftback);
        pwmLeftBackReq = pwmLeftBackBase + pid_compensate;

        // Calculate the output PWM value by making slow changes to the current value
        if (abs(pwmLeftBackReq) > pwmLeftBackOut) {
            pwmLeftBackOut += PWM_INCREMENT;
        }
        else if (abs(pwmLeftBackReq) < pwmLeftBackOut) {
            pwmLeftBackOut -= PWM_INCREMENT;
        }
        
        // Conditional operator to limit PWM output at the maximum 
        pwmLeftBackOut = constrain(pwmLeftBackOut, PWM_MIN, PWM_MAX);

        // Case 1: robot stop => reset variables
        if (left_back_dir == 0 || pwmLeftBackReq == 0) {
            pwmLeftBackReq = 0;
            pwmLeftBackOut = 0;
            lastError_leftback = 0.0;
            integral_leftback = 0.0;
        }

        // Case 2: robot change dir => stop robot before running
        if (pre_left_back_dir != left_back_dir){
            pre_left_back_dir = left_back_dir;
            pwmLeftBackReq = 0;
            pwmLeftBackOut = 0; 
            lastError_leftback = 0.0;
            integral_leftback = 0.0;    
        }
        
        // Case 3: real wheel vel more than target vel => stop robot before running
        // float delta_vel = abs(velLeftBack) - abs(cmdvel_left_back);
        // if (delta_vel > 5) {
        //     pwmLeftBackReq = 0;
        //     pwmLeftBackOut = 0;
        // }

        // Stop the car if there are no cmd_vel messages
        // if((millis()/1000) - time_cmdvel_leftback_recv > 1) {
        // pwmLeftBackReq = 0;
        // pwmLeftBackOut = 0;  
        // }

        // out pwm to motor
        set_pwm_values(4, 5, left_back_dir, pwmLeftBackOut);
    }
    else{
        ledcWrite(4, 0);
        ledcWrite(5, 0);    
    }

    // control right back motor
    if (is_recv_cmdvel == 1){
        // determine direction of wheel and calculate PWM request
        // right_back_dir = gain_dir(cmdvel_right_back);
        pwmRightBackBase = map(abs(cmdvel_right_back), 0, 60, 0, 255);

        int pid_compensate = computePID(Kp_rb, Ki_rb, Kd_rb, abs(cmdvel_right_back), abs(velRightBack), lastTimeRightBack, lastError_rightback, integral_rightback);
        pwmRightBackReq = pwmRightBackBase + pid_compensate;

        // Calculate the output PWM value by making slow changes to the current value
        if (abs(pwmRightBackReq) > pwmRightBackOut) {
            pwmRightBackOut += PWM_INCREMENT;
        }
        else if (abs(pwmRightBackReq) < pwmRightBackOut) {
            pwmRightBackOut -= PWM_INCREMENT;
        }
        
        // Conditional operator to limit PWM output at the maximum 
        pwmRightBackOut = constrain(pwmRightBackOut, PWM_MIN, PWM_MAX);

        // Case 1: robot stop => reset variables
        if (right_back_dir == 0 || pwmRightBackReq == 0) {
            pwmRightBackReq = 0;
            pwmRightBackOut = 0;
            lastError_rightback = 0.0;
            integral_rightback = 0.0;

        }

        // Case 2: robot change dir => stop robot before running
        if (pre_right_back_dir != right_back_dir){
            pre_right_back_dir = right_back_dir;
            pwmRightBackReq = 0;
            pwmRightBackOut = 0; 
            lastError_rightback = 0.0;
            integral_rightback = 0.0;    
        }
        
        // Case 3: real wheel vel more than target vel => stop robot before running
        // float delta_vel = abs(velRightBack) - abs(cmdvel_right_back);
        // if (delta_vel > 5) {
        //     pwmRightBackReq = 0;
        //     pwmRightBackOut = 0;
        // }

        // Stop the car if there are no cmd_vel messages
        // if((millis()/1000) - time_cmdvel_rightback_recv > 1) {
        // pwmRightBackReq = 0;
        // pwmRightBackOut = 0;  
        // }

        // out pwm to motor
        set_pwm_values(6, 7, right_back_dir, pwmRightBackOut);
    }
    else{
        ledcWrite(6, 0);
        ledcWrite(7, 0);    
    }
    delay(1);
}