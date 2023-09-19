#include <MPU6500_WE.h>
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "motor.h"
#include "encoder.h"
#include "pidIr.h"
#include "LPF.h"
#include "HPF.h"

// To enter debug mode uncomment this line below.
//#define MONITOR_OMEGA
//#define DEBUG
//#define FILTER
//#define DEBUG_ODOM

//Constants
#define LOOPTIME 10 //in ms (1000/LOOPTIME hz)
#define MAX_PWM 255 //in range of 0-255
#define WHEEL_DISTANCE 26.4 //in cm
#define WHEEL_RADIUS (6.5/2.0) //in cm
#define WHEEL_ENC 897.0 //in ppr
#define RADS_TO_RPM 9.55
#define DEGS_TO_RPM (1.0/6.0)
#define DISTANCE_TO_CENTROID sqrt(pow(65.0,2.0)+pow(130.0,2.0))
#define ANGLE_TO_CENTROID atan2(65.0,130.0)
#define MPU6500_ADDR 0x68

// IMU declaration
MPU6500_WE myIMU = MPU6500_WE(MPU6500_ADDR);

// Motor pin assignment
Motor motor_kiri(5,4,9); // Motor(int RPWM, int LPWM, int EN);
Motor motor_kanan(8,7,10);

// Encoder pin assignment
Encoder enc_kiri(2,3); // Encoder(int pin_a, int pin_b);
Encoder enc_kanan(11,12);

// Encoder callback function
void callbackKiA(){enc_kiri.doEncoderA();}
void callbackKiB(){enc_kiri.doEncoderB();}
void callbackKaA(){enc_kanan.doEncoderA();}
void callbackKaB(){enc_kanan.doEncoderB();}

// Create PID object
pidIr pid_left_omega(1.1,0.006,0.0);//Kp = 1.0, Ki = 0.006, Kd = 0.0
pidIr pid_right_omega(1.1,0.006,0.0);

// Create LPF object
float fc = 3; //fc = cut-off frequency (in Hz)
LPF omega_left_lp(fc);
LPF omega_right_lp(fc);
LPF dlpf(1);
HPF dhpf(0.5);

// Timestamp variables
volatile long curr_millis;
volatile long prev_millis;
float Ts;

// Angle variables
volatile long curr_left_angle;
volatile long curr_right_angle;
volatile long prev_left_angle;
volatile long prev_right_angle;

// Omega variables
volatile long left_omega;
volatile long right_omega;
volatile long prev_left_omega = 0;
volatile long prev_right_omega = 0;

// Filtered variables
volatile long filtered_left_omega;
volatile long filtered_right_omega;

// Command variables
float target_omega_ka = 0;
float target_omega_ki = 0;

// Control action (PWM)
int pwm_ka;
int pwm_ki;

// Robot Pose Variables
// Encoder
float pos_x_enc = 0;
float pos_y_enc = 0;
float theta_enc = 0;

//IMU
float prev_wz_imu_gyr = 0;

float curr_vx_imu_global = 0;
float curr_vy_imu_global = 0;
float prev_vx_imu_global = 0;
float prev_vy_imu_global = 0;

float curr_ax_imu_global = 0;
float curr_ay_imu_global = 0;
float prev_ax_imu_global = 0;
float prev_ay_imu_global = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

float pos_x_imu = 0;
float pos_y_imu = 0;
float theta_imu = 0;

//ROS communication
ros::NodeHandle nh;

void velCallback(const geometry_msgs::Twist& vel){
  float vx = vel.linear.x; //in m/s
  float wz = vel.angular.z; //in rad/s

  // Speed limit
  if (abs(vx) > 0.7){
    vx = abs(vx)/vx*0.7;
  } 
  
  if (abs(wz) > 3.1){
    wz = abs(vx)/vx*3.1;
  }

  // Inverse Kinematics
  target_omega_ki = (vx*100.0/WHEEL_RADIUS - wz*WHEEL_DISTANCE/(2.0*WHEEL_RADIUS))*RADS_TO_RPM; //in RPM
  target_omega_ka = (vx*100.0/WHEEL_RADIUS + wz*WHEEL_DISTANCE/(2.0*WHEEL_RADIUS))*RADS_TO_RPM; //in RPM

  // Logging the Velocity Command ---------------------------------------------------------- //
  //String linSpeedString = String("Linear speed: " + String(vx,3) + " m/s");
  //String angSpeedString = String("Angular speed: " + String(wz,3) + " rad/s");
  //String OmegaString = String(String(millis()) + ", Left RPM: " + String(target_omega_ki,3) + " RPM, Right RPM: " + String(target_omega_ka,3) + " RPM.");
  //String OmegaString = String(String(millis()) + ", speed: " + String(vx,3) + " m/s, turn: " + String(wz,3) + " rad/s.");
  String PoseString = String(String(millis()) + ": " + /*String(posX_IMU, 3) + ", " + String(posY_IMU, 3) + ", " + String(degree(-Theta_IMU), 3) + ", " +*/ String(pos_x_enc, 3) + ", " + String(pos_y_enc, 3) + ", " + String(degree(theta_imu), 3));
  
  //char linSpeedInfo[30]; linSpeedString.toCharArray(linSpeedInfo, 30);
  //char angSpeedInfo[30]; angSpeedString.toCharArray(angSpeedInfo, 30);
  //char OmegaInfo[100]; OmegaString.toCharArray(OmegaInfo, 100);
  char PoseInfo[100]; PoseString.toCharArray(PoseInfo, 100);

  //nh.loginfo(linSpeedInfo);
  //nh.loginfo(angSpeedInfo);
  //nh.loginfo(OmegaInfo);
  nh.loginfo(PoseInfo);
  // ---------------------------------------------------------------------------------------- //
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", velCallback);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  nh.initNode();
  nh.subscribe(vel_sub);

  // Start to set the pin mode
  motor_kiri.start();
  motor_kanan.start();
  enc_kiri.start(callbackKiA, callbackKiB);
  enc_kanan.start(callbackKaA, callbackKaB);

  //**************************************** MPU6500 *******************************************//
  Wire.begin();
  if(!myIMU.init()){
      Serial.println("MPU6500 does not respond");
  } else{
      Serial.println("MPU6500 is connected");
  }

  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!");

  myIMU.enableGyrDLPF();
  myIMU.setGyrDLPF(MPU6500_DLPF_6);
  myIMU.setSampleRateDivider(5);
  myIMU.setGyrRange(MPU6500_GYRO_RANGE_500);
  myIMU.setAccRange(MPU6500_ACC_RANGE_2G);
  myIMU.enableAccDLPF(true);
  myIMU.setAccDLPF(MPU6500_DLPF_6);
  //myIMU.enableAccAxes(MPU6500_ENABLE_XYZ);
  //myIMU.enableGyrAxes(MPU6500_ENABLE_XYZ);
  //*******************************************************************************************//

  // This will only run in debug mode
  #ifdef DEBUG
  Serial.print(F("Left_Angle:")); Serial.print("\t");
  Serial.print(F("Right_Angle:")); Serial.print("\t");
  Serial.print(F("Left_Omega:")); Serial.print("\t");
  Serial.print(F("Right_Omega:")); Serial.print("\t");
  Serial.print(F("Target_Angle:")); Serial.print("\t");
  Serial.print(F("Target_Omega:")); Serial.print("\t");
  Serial.print(F("Left_PWM:")); Serial.print("\t");
  Serial.print(F("Right_PWM:")); Serial.print("\t");
  Serial.println();
  #endif 

  #ifdef MONITOR_OMEGA
  Serial.print(F("Left_Omega:")); Serial.print("\t");
  Serial.print(F("Right_Omega:")); Serial.print("\t");
  Serial.print(F("Left_PWM:")); Serial.print("\t");
  Serial.print(F("Right_PWM:")); Serial.print("\t");
  Serial.print(F("Target_Omega_Ki:")); Serial.print("\t");
  Serial.print(F("Target_Omega_Ka:")); Serial.print("\t");
  Serial.println();
  #endif

  #ifdef FILTER
  Serial.print(F("Left_Omega:")); Serial.print("\t");
  Serial.print(F("Right_Omega:")); Serial.print("\t");
  Serial.print(F("Filtered_Left_Omega:")); Serial.print("\t");
  Serial.print(F("Filtered_Right_Omega:")); Serial.print("\t");
  Serial.println();
  #endif

  #ifdef DEBUG_ODOM
  Serial.print(F("Left_Angle:")); Serial.print("\t");
  Serial.print(F("Right_Angle:")); Serial.print("\t");
  Serial.print(F("Pos_X:")); Serial.print("\t");
  Serial.print(F("Pos_Y:")); Serial.print("\t");
  Serial.print(F("Theta:")); Serial.print("\t");
  Serial.println();
  #endif
}

void loop() {
  curr_millis = millis();   // Bookmark the time 

  // Start timed loop for everything else (in ms)
  if (curr_millis - prev_millis >= LOOPTIME) {
    Ts = curr_millis - prev_millis; //Ts = time sampling in ms

    // Robot Pose
    getPose();
    sendPoseToSerial();

    nh.spinOnce();

    //if (curr_millis > 3000){
      //target_omega_ki = 120.0;
      //target_omega_ka = 120.0;

      //------------------------------------- Rotate Motor ------------------------------------------//
      // Compute pwm (uncomment what is needed)
      pwm_ki = pid_left_omega.compute(target_omega_ki,filtered_left_omega,MAX_PWM,Ts);
      pwm_ka = pid_right_omega.compute(target_omega_ka,filtered_right_omega,MAX_PWM,Ts);

      //pwm_ki = 0;
      //pwm_ka = 0;

      if (target_omega_ki == 0 && target_omega_ka == 0){
        motorStop();
      } else {
        motorGo(pwm_ki,pwm_ka);
      }  
      //--------------------------------------------------------------------------------------------//
    //}
    
    // Saving the last value
    prev_right_angle = curr_right_angle;
    prev_left_angle = curr_left_angle; 
    prev_right_omega = filtered_right_omega;
    prev_left_omega = filtered_left_omega; 
    prev_millis = curr_millis;

    //------------------------------------- Print Something --------------------------------------//
    #ifdef FILTER
    Serial.print(left_omega); Serial.print("\t");
    Serial.print(right_omega); Serial.print("\t");
    Serial.print(filtered_left_omega); Serial.print("\t");
    Serial.print(filtered_right_omega); Serial.print("\t");
    Serial.println();
    #endif

    // Print result
    #ifdef DEBUG
    Serial.print(curr_left_angle); Serial.print("\t");
    Serial.print(curr_right_angle); Serial.print("\t");
    Serial.print(filtered_left_omega); Serial.print("\t");
    Serial.print(filtered_right_omega); Serial.print("\t");
    Serial.print(target_angle/20.0); Serial.print("\t");
    Serial.print(target_omega/20.0); Serial.print("\t");
    Serial.print(pwm_ki); Serial.print("\t");
    Serial.print(pwm_ka); Serial.print("\t");
    Serial.println();
    #endif

    #ifdef MONITOR_OMEGA
    Serial.print(filtered_left_omega); Serial.print("\t");
    Serial.print(filtered_right_omega); Serial.print("\t");
    Serial.print(pwm_ki); Serial.print("\t");
    Serial.print(pwm_ka); Serial.print("\t");
    Serial.print(target_omega_ki); Serial.print("\t");
    Serial.print(target_omega_ka); Serial.print("\t");
    Serial.println();
    #endif

    #ifdef DEBUG_ODOM
    Serial.print(curr_left_angle); Serial.print("\t");
    Serial.print(curr_right_angle); Serial.print("\t");
    Serial.print(posX); Serial.print("\t");
    Serial.print(posY); Serial.print("\t");
    Serial.print(degree(Theta)); Serial.print("\t");
    Serial.println();
    #endif
    
    //--------------------------------------------------------------------------------------------//
  }
}

float radian (float degree){
  return degree/180.0*PI;
}

float degree (float radian){
  return radian*180.0/PI;
}

//-------------------------------- Get Pose Algorithm ----------------------------------//
void getPose (){
  //---------------------------------- IMU ---------------------------------------//
  xyzFloat acc = myIMU.getGValues();
  xyzFloat gyr = myIMU.getGyrValues();

  roll += radian(getEulAngle(gyr.y)); //in rad
  pitch += radian(getEulAngle(gyr.x)); //in rad
  yaw += radian(getEulAngle(gyr.z)); //in rad

  theta_imu = yaw;

  //Manual motion interrupt
  float gyr_z = gyr.z;
  if (abs(gyr.z) < 0.09){
    gyr_z = 0.0;
  }

  //Position estimation
  float acc_x_imu_local = 9.8*acc.y - 9.8*sin(pitch); //+ radian(gyr_z-prev_gyr_z)/(Ts/1000.0)*DISTANCE_TO_CENTROID*sin(ANGLE_TO_CENTROID) - pow(radian(gyr_z),2.0)*DISTANCE_TO_CENTROID*cos(ANGLE_TO_CENTROID); 
  float acc_x_filter = dhpf.filter(acc_x_imu_local, Ts/1000.0); //in m/s2
  acc_x_filter = dlpf.filter(acc_x_filter, Ts/1000.0); //in m/s2

  // in global frame
  curr_ax_imu_global = acc_x_filter*cos(theta_imu); //in m/s2
  curr_ay_imu_global = acc_x_filter*sin(theta_imu);

  curr_vx_imu_global += ((curr_ax_imu_global + prev_ax_imu_global)/2.0)*Ts/1000.0; curr_vx_imu_global = round(curr_vx_imu_global*100.0)/100.0; //Truncate the decimal points into 2 places
  curr_vy_imu_global += ((curr_ay_imu_global + prev_ay_imu_global)/2.0)*Ts/1000.0; curr_vy_imu_global = round(curr_vy_imu_global*100.0)/100.0;
  
  pos_x_imu += ((curr_vx_imu_global + prev_vx_imu_global)/2.0)*(Ts/1000.0);
  pos_y_imu += ((curr_vy_imu_global + prev_vy_imu_global)/2.0)*(Ts/1000.0);

  // saving the previous value
  prev_ay_imu_global = curr_ay_imu_global;
  prev_ax_imu_global = curr_ax_imu_global;
  prev_vy_imu_global = curr_vy_imu_global;
  prev_vx_imu_global = curr_vx_imu_global;
  float prev_wz_imu_gyr = gyr_z;

  //-------------------------------------- Encoder -----------------------------------------//
  // Wheel Angle 
  curr_left_angle = enc_kiri.getPos()/WHEEL_ENC*360.0; //in deg
  curr_right_angle = enc_kanan.getPos()/WHEEL_ENC*360.0;
  
  //Angle change
  double dAngL = radian(curr_left_angle - prev_left_angle);
  double dAngR = radian(curr_right_angle - prev_right_angle);
  double a = (dAngR + dAngL)/(dAngR - dAngL)*WHEEL_DISTANCE/2.0*0.01; // in m

  //Updating the pose of the vehicle for diff. drive
  if (pow((dAngL - dAngR),2) < 0.0001) { //
    pos_x_enc -= WHEEL_RADIUS/2.0*(dAngR + dAngL)*cos(theta_enc)*0.01; // in m
    pos_y_enc -= WHEEL_RADIUS/2.0*(dAngR + dAngL)*sin(theta_enc)*0.01; // in m
  } else if ((pow(abs(dAngL-dAngR)-abs(2*dAngL),2) < 0.0001) || (pow(abs(dAngL-dAngR)-abs(2*dAngR),2) < 0.0001)) {
    theta_enc -= (dAngR - dAngL)*WHEEL_RADIUS/WHEEL_DISTANCE;
  } else {
    theta_enc -= (dAngR - dAngL)*WHEEL_RADIUS/WHEEL_DISTANCE;
    pos_x_enc += a*sin(theta_enc) - a*sin(theta_enc + (dAngR - dAngL)*WHEEL_RADIUS/WHEEL_DISTANCE);
    pos_y_enc -= a*cos(theta_enc) + a*cos(theta_enc + (dAngR - dAngL)*WHEEL_RADIUS/WHEEL_DISTANCE);
  }

  //Theta = radian(0.9815*degree(Theta) - 0.43);
  theta_enc = warpAngle(theta_enc);

  /*
  // Error Correction
  float posX_corrected;
  float posY_corrected;
  if (abs(degree(Theta)) <= 5 || abs(degree(Theta)) >= 175){
    posX_corrected = 0.9691*posX + 3.6042/100.0;
  } else if (abs(degree(Theta)) >= 85 || abs(degree(Theta)) <= 95) {
    posY_corrected = 0.9691*posY + 3.6042/100.0;
  }*/
  
  // Omega
  left_omega = (curr_left_angle - prev_left_angle)/(Ts/1000.0)*DEGS_TO_RPM; //in RPM
  right_omega = (curr_right_angle - prev_right_angle)/(Ts/1000.0)*DEGS_TO_RPM; //in RPM

  // Filtered
  filtered_left_omega = omega_left_lp.filter(left_omega, Ts/1000.0); //in RPM
  filtered_right_omega = omega_right_lp.filter(right_omega, Ts/1000.0);
}
//----------------------------------------------------------------------------------------------------//

float getEulAngle(float rpy_speed){
  if (abs(rpy_speed) < 0.9) {
    rpy_speed = 0.0;
  }
  double eul_angle = rpy_speed*Ts/1000;
  
  return eul_angle;
}

void motorGo (int ki, int ka) {
  // Rotate motor
  motor_kiri.setEnable(ki);
  motor_kanan.setEnable(ka);
    
  motor_kiri.rotate();
  motor_kanan.rotate();
}

void motorStop (){
  motorGo(0,0);
  resetPID();
}

void resetPID(){
  pid_left_omega.reset();
  pid_right_omega.reset();
}

float warpAngle(float angle){
  angle = fmod(angle, 2.0*PI);
  if (angle > PI) {
    angle -= 2.0*PI;
  } else if (angle < -PI){
    angle += 2.0*PI;
  }
  return angle;
}

void sendPoseToSerial(){
  Serial3.print(pos_x_enc);
  Serial3.print(", ");
  Serial3.print(pos_y_enc);
  Serial3.print(", ");
  Serial3.print(theta_imu);
  Serial3.print(", ");
  Serial3.print(target_omega_ki);
  Serial3.print(", ");
  Serial3.println(target_omega_ka);
}
