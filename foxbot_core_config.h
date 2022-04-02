/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Description: Core configuration file for foxbot robot.
 *  Author: Bart van der Haagen 
 *
 *  Serial communication is enabled with the following command:
 *  rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
 */

#ifndef FOXBOT_CORE_CONFIG_H_
#define FOXBOT_CORE_CONFIG_H_

// enable this line for Arduino Due
#define USE_USBCON

/* Include librairies */
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <Timer.h>
#include <MedianFilter.h>

/* Global parameters definitions*/
#define FREQUENCY_RATE 				119 		// [ms] default 50ms
#define FREQUENCY_ODOMETRY 				152 		// [ms] default 250ms
#define FREQUENCY_ROSPINONCE 				175 		// [ms]
#define FREQUENCY_CONTROLLER 				74 		// [ms] default 50ms
#define FREQUENCY_JSP                                 100

/* Rate computing parameters */
#define RATE_DIRECTION_MEDIAN_FILTER_SIZE 		3
#define RATE_CONV 					0.0073882 	// [inc] -> [rad]
// 853 (234) inc per wheel rotation
#define RATE_AVERAGE_FILTER_SIZE 			4

/* Rate controller parameters */
#define PWM_MAX				 	4095            // 12 bit
#define RATE_CONTROLLER_KP 				130.0 		 // default 150
#define RATE_CONTROLLER_KD 				5000000000000.0 //4500000000000
#define RATE_CONTROLLER_KI 				0.00005 	 //0.00001
#define RATE_INTEGRAL_FREEZE				250
#define RATE_CONTROLLER_MIN_PWM 			-500
#define RATE_CONTROLLER_MAX_PWM 			500

/* Mechanical parameters */
#define WHEEL_RADIUS 					0.04 	// [m]

// distance between the two wheels
#define BASE_LENGTH 					0.35 		// [m]  0.288

// define joint limits	
#define ADCVALUE 0.001256637
#define SHOULDER_ZERO    0.0
#define SHOULDER_RATIO   3.141
#define ELBOW_ZER0       1200.0
#define ELBOW_RATIO      0.6				

/* Define frequency loops */
Timer _frequency_rate(FREQUENCY_RATE);
Timer _frequency_odometry(FREQUENCY_ODOMETRY);
Timer _frequency_rospinonce(FREQUENCY_ROSPINONCE);
Timer _frequency_controller(FREQUENCY_CONTROLLER);
Timer _frequency_jsp(FREQUENCY_JSP);

/* Define median filter for direction */
MedianFilter motor_right_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);
MedianFilter motor_left_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);
MedianFilter motor_back_right_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);
MedianFilter motor_back_left_direction_median_filter(RATE_DIRECTION_MEDIAN_FILTER_SIZE);

/* Define pins */


// motor A (fornt_right)
const byte motorRightEncoderPinA = 33;
const byte motorRightEncoderPinB = 19;
const byte enMotorRight = 13;
const byte in1MotorRight = 2;   //26 C1 M1
const byte in2MotorRight = 3;   //28 C2 M2

// motor B (frond_left)
const byte motorLeftEncoderPinA = 22;
const byte motorLeftEncoderPinB = 23;
const byte enMotorLeft = 12;
const byte in1MotorLeft = 4;    //30
const byte in2MotorLeft = 5;    //32

// motor C (back_right)
const byte motorBackRightEncoderPinA = 37;
const byte motorBackRightEncoderPinB = 32;
const byte enMotorBackRight = 10;
const byte in1MotorBackRight = 6;   //26 C1 M1
const byte in2MotorBackRight = 7;   //28 C2 M2

// motor D (back_left)
const byte motorBackLeftEncoderPinA = 24;
const byte motorBackLeftEncoderPinB = 25;
const byte enMotorBackLeft = 11;
const byte in1MotorBackLeft = 26;    //30
const byte in2MotorBackLeft = 27;    //32


/* Define motors variables */
// right
volatile int motor_right_inc;
int motor_right_direction;
int motor_right_filtered_direction;
float motor_right_filtered_inc_per_second;
float motor_right_rate_est;
float motor_right_rate_ref;
int motor_right_check_dir;
int motor_right_pwm_rate;
unsigned long motor_right_prev_time;
int pwmMotorRight = 0;

// left
volatile int motor_left_inc;
int motor_left_direction;
int motor_left_filtered_direction;
float motor_left_filtered_inc_per_second;
float motor_left_rate_est;
float motor_left_rate_ref;
int motor_left_check_dir;
int motor_left_pwm_rate;
unsigned long motor_left_prev_time;
int pwmMotorLeft = 0;

// backright
volatile int motor_back_right_inc;
int motor_back_right_direction;
int motor_back_right_filtered_direction;
float motor_back_right_filtered_inc_per_second;
float motor_back_right_rate_est;
float motor_back_right_rate_ref;
int motor_back_right_check_dir;
int motor_back_right_pwm_rate;
unsigned long motor_back_right_prev_time;
int pwmMotorBackRight = 0;

// backleft
volatile int motor_back_left_inc;
int motor_back_left_direction;
int motor_back_left_filtered_direction;
float motor_back_left_filtered_inc_per_second;
float motor_back_left_rate_est;
float motor_back_left_rate_ref;
int motor_back_left_check_dir;
int motor_back_left_pwm_rate;
unsigned long motor_back_left_prev_time;
int pwmMotorBackLeft = 0;


/* Define controllers variables PAN/TILT
volatile double shoulder_des = 0;
volatile double error_prev_shoulder = 0;
volatile double error_sum_shoulder = 0;
volatile double PID_shoulder = 0;
volatile double sensorValuePosShoulder = 0;

volatile double elbow_des = 0;
volatile double error_prev_elbow = 0;
volatile double error_sum_elbow = 0;
volatile double PID_elbow = 0;
volatile double sensorValuePosElbow = 0;
*/

char *as[] = {"right_front_wheel_joint" , "left_front_wheel_joint" , "right_back_wheel_joint" , "left_back_wheel_joint"}; /*"pan_joint", "tilt_joint" ,*/
float positions_tab[4];
float velocities_tab[4];
float efforts_tab[4];


/* Define controllers variables */
// right
unsigned long controler_motor_right_prev_time;
float controler_motor_right_prev_epsilon = 0.0;
float controler_motor_right_int = 0.0;
// left
unsigned long controler_motor_left_prev_time;
float controler_motor_left_prev_epsilon = 0.0;
float controler_motor_left_int = 0.0;
// back_right
unsigned long controler_motor_back_right_prev_time;
float controler_motor_back_right_prev_epsilon = 0.0;
float controler_motor_back_right_int = 0.0;
// back_left
unsigned long controler_motor_back_left_prev_time;
float controler_motor_back_left_prev_epsilon = 0.0;
float controler_motor_back_left_int = 0.0;

/* Mixer variable */
float linear_velocity_ref_x;
float linear_velocity_ref_y;
float angular_velocity_ref_z;
float linear_velocity_est_x;
float linear_velocity_est_y;
float angular_velocity_est_z;


const int pan_rate_est_pin = A0;
const int tilt_rate_est_pin = A1;

float joint_last_time;

float yaw_est;
unsigned long odom_prev_time;


/* ROS Nodehanlde */
ros::NodeHandle  nh;

/* Prototype function */
void rateControler(const float rate_ref, const float rate_est, int & pwm_rate,
                   unsigned long & prev_time, float & previous_epsilon,
                   float & integral_epsilon);
float runningAverage(float prev_avg, const float val, const int n);
void setMotorRateAndDirection(int pwm_ref, const float rate_ref,
                              const byte enMotor, const byte in1Motor, const byte in2Motor);

/* Velocity command subscriber */
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", commandVelocityCallback);


/* Joint_state subscriber
void messageCbShoulder( const std_msgs::Float32& toggle_msg){shoulder_des = toggle_msg.data;}
ros::Subscriber<std_msgs::Float32> sub_shoulder("pan_joint", messageCbShoulder );

void messageCbElbow( const std_msgs::Float32& toggle_msg){elbow_des = toggle_msg.data;}
ros::Subscriber<std_msgs::Float32> sub_elbow("tilt_joint", messageCbElbow );
 */


/* Odometry publisher */
nav_msgs::Odometry wheel_odom;
ros::Publisher odom_publisher("odom", &wheel_odom);

/*Joint state publisher */
sensor_msgs::JointState joints;
ros::Publisher joint_state_publisher("/joint_states", &joints);

/* wheel_quat in clicks  */
std_msgs::Float32 right_rate_est;
ros::Publisher wheel_quat_pub_rf("motor_right_rate_est", &right_rate_est);

std_msgs::Float32 left_rate_est;
ros::Publisher wheel_quat_pub_lf("motor_left_rate_est", &left_rate_est);

std_msgs::Float32 back_right_rate_est;
ros::Publisher wheel_quat_pub_rb("motor_back_right_rate_est", &back_right_rate_est);

std_msgs::Float32 back_left_rate_est;
ros::Publisher wheel_quat_pub_lb("motor_back_left_rate_est", &back_left_rate_est);

/* pan_position in clicks */
//std_msgs::Float32 pan_est;
//ros::Publisher pan_rate_pub("pan_rate_est", &pan_est);

/* tilt_position in clicks */
//std_msgs::Float32 tilt_est;
//ros::Publisher tilt_rate_pub("tilt_rate_est", &tilt_est);

/* further debugging posibilities */
geometry_msgs::Point debug_1;
ros::Publisher debug_publisher_1("debug_1", &debug_1);




template <typename type>
type sign(type value) {
    return type((value>0)-(value<0));
}


#endif // FOXBOT_CORE_CONFIG_H_
