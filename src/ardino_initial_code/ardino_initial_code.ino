#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

// Global variables and ROS setup
ros::NodeHandle nodeHandle;

// Constants for servo control
const int minSteering = 1000;
const int maxSteering = 2000;
const int brk_delay = 500;
const int speed_cap = 2000;

// Output pins for Arduino, specific to servo control
static const int STEERING_OUT = 2; // Steering servo output
static const int THROTTLE_OUT = 3; // Throttle servo output

// Adjusted pins from Code2, as requested
static const int ADDITIONAL_OUT_1 = 7;  // Was 4, now 7
static const int ADDITIONAL_OUT_2 = 9;  // Was 5, now 9
static const int ADDITIONAL_OUT_3 = 11; // Was 6, now 11
static const int ADDITIONAL_OUT_4 = 13; // Was 7, now 13

// Analog input pin
const int analogInPin = A0;
int sensorValue = 0;

// Other global variables
int servo_values[6];
unsigned long last_msg_time = 0;
unsigned long water_strike = 0;
unsigned long brk_timer = 0;
unsigned long turn_boost = 0;
bool prev_dir = 0;
int emergency_stop = 0;
unsigned long last_emergency_stop = 0;
float throttle_cmd;

// Servo objects
Servo steeringServo;
Servo throttleServo;

// Function declarations
float fmap(float toMap, float in_min, float in_max, float out_min, float out_max);
float fw_to_rev(float th);
float rev_to_fw();
void failSafeActive();
void driveCallback(const std_msgs::Float32MultiArray& control_msg);
//void waypointCallback( const std_msgs::Int8& msg); 

// ROS Subscribers and Publishers
ros::Subscriber<std_msgs::Float32MultiArray> driveSubscriber("/cmd_vel1", &driveCallback);
//ros::Subscriber<std_msgs::Bool> waypointSubscriber("/waypoint", &waypointCallback);
std_msgs::Int16MultiArray rpmVal;
ros::Publisher rpmVal_data("rpmVal_data", &rpmVal);
std_msgs::Int16MultiArray optiFlow;
ros::Publisher optiFlow_data("optiFlow_data", &optiFlow);
std_msgs::Int8 waterHit;
ros::Publisher water_log("water_log", &waterHit);

void setup() {
  Serial.begin(57600); // Adjust as necessary for integration

  // ROS initialization
  nodeHandle.initNode();
  nodeHandle.subscribe(driveSubscriber);
  nodeHandle.advertise(water_log);

  // Linking servos to outputs
  steeringServo.attach(STEERING_OUT);
  throttleServo.attach(THROTTLE_OUT);

  // Setup for additional outputs adjusted for new pin assignments
  pinMode(ADDITIONAL_OUT_1, OUTPUT);
  pinMode(ADDITIONAL_OUT_2, OUTPUT);
  pinMode(ADDITIONAL_OUT_3, OUTPUT);
  pinMode(ADDITIONAL_OUT_4, OUTPUT);

  Serial.println("Initialization Finished");
}

void loop() {
  // Initial functionality for ROS and servo control
  if(emergency_stop == 0 || emergency_stop == 2)
  {
    unsigned long cur_millis = millis();
    if((millis() - last_msg_time) > 1000){
      failSafeActive();
    } 
    else {
      int throttle_speed = servo_values[1];
      if(throttle_speed > speed_cap)
      {
        throttle_speed = speed_cap;
      }
      
      int steering_level = servo_values[0];
      /*
      if(steering_level > 1660)
      {
        steering_level = 2000;
        throttle_speed = ((throttle_speed - 1500) * 3) + 1500;
        if(throttle_speed > 2000)
        {
          throttle_speed = 2000;
        }
        else if(throttle_speed < 1000)
        {
          throttle_speed = 1000;
        }
      }
      else if(steering_level < 1340)
      {
        steering_level = 1000;
        throttle_speed = ((throttle_speed - 1500) * 3) + 1500;
        if(throttle_speed > 2000)
        {
          throttle_speed = 2000;
        }
        else if(throttle_speed < 1000)
        {
          throttle_speed = 1000;
        }
      }
      */
      /*
      //boost on waypoint strike
      if(millis() < turn_boost + 1000)
      {
        throttle_speed = 2000; // max speed 
      }
      */
      steeringServo.writeMicroseconds(steering_level);
      
      throttleServo.writeMicroseconds(throttle_speed);
    }
    digitalWrite(ADDITIONAL_OUT_1, LOW);
    digitalWrite(ADDITIONAL_OUT_2, LOW);
    digitalWrite(ADDITIONAL_OUT_3, LOW);
    digitalWrite(ADDITIONAL_OUT_4, LOW);
  }

  if(emergency_stop == 0)
  {
    // Updated Water_sensor code functionality with new pin assignments
    sensorValue = analogRead(analogInPin);
    //Serial.print("sensor = ");
    //Serial.println(sensorValue);
    if((sensorValue >= 100)) {
        //digitalWrite(ADDITIONAL_OUT_1, HIGH);
        //digitalWrite(ADDITIONAL_OUT_2, HIGH);
        //digitalWrite(ADDITIONAL_OUT_3, HIGH);
        //digitalWrite(ADDITIONAL_OUT_4, HIGH);
        waterHit.data = 1; 
        water_log.publish(&waterHit);    
        //emergency_stop = 1;
        water_strike = millis();
    } else {
      waterHit.data = 0;
      water_log.publish(&waterHit);
    }
  }
  
  
  //while emergency stop is 1, the rover should lock up. Set emergency stop to 2 after 1 minute.
  if(emergency_stop == 1)
  {
    failSafeActive();
    if((millis() - water_strike) > 10000)
    {
      emergency_stop = 0;
      digitalWrite(ADDITIONAL_OUT_1, LOW);
      digitalWrite(ADDITIONAL_OUT_2, LOW);
      digitalWrite(ADDITIONAL_OUT_3, LOW);
      digitalWrite(ADDITIONAL_OUT_4, LOW);
    }
  }
  //while emergency stop is 2, the rover should just finish the course.
  nodeHandle.spinOnce();
  delayMicroseconds(100);

}

//maps a float from -1 to 1 to 1000 to 2000
float fmap(float toMap, float in_min, float in_max, float out_min, float out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//failsafe for handling sudden forwards to reverse
float fw_to_rev(float th){
  if (brk_timer == 0){
    brk_timer = millis();
  }

  if((millis() - brk_timer) < brk_delay ){
    return th;
  }
  else{

    if((millis() - brk_timer) < (brk_delay + 200)){
      return 0.0;
    }
    else{
      brk_timer = 0;
      prev_dir = 1;
      return th;
    }

  }
}

//failsafe for reverse to forwards
float rev_to_fw(){
  if (brk_timer == 0){
    brk_timer = millis();
  }

  if((millis() - brk_timer) < (brk_delay + 100) ){
    return 0.0;
  }
  else{
      brk_timer = 0;
      prev_dir = 0;
      return 0.0;
  }

}

//failsafe for if we have no input- puts throttle to 0 and steering to a neutral place
void failSafeActive(){
  throttleServo.writeMicroseconds(1500);
  steeringServo.writeMicroseconds(1600);
}

//waypoint callback
/*
void waypointCallback(const std_msgs::Bool& msg) {
  //start a timer when a true comes down
  if(msg.data)
  {
    turn_boost = millis();
    waterHit.data = true; 
    water_log.publish(&waterHit); 
  }
  else
  {
    waterHit.data = false; 
    water_log.publish(&waterHit); 
  }
}
*/

//main method for reading from ros
void driveCallback( const std_msgs::Float32MultiArray&  control_msg ){
  //timestamp the  last ros message
  last_msg_time = millis();
  
  if(control_msg.data[2] == 1.0){
    emergency_stop = 0;
  }
/*
  //if emergency stop is triggered
  if(control_msg.data[2] == 1.0)
  {
    emergency_stop = 1;
//    last_emergency_stop = millis();
//    //flip to 1 for water sensor shut off
  }
  else if(control_msg.data[2] == 0.0){
    emergency_stop = 0;
  }
*/
  //Handle for steering and throttle command
  //Map steering and throttle command to servo output
  float steer_cmd = fmap(control_msg.data[0], -1.0, 1.0, minSteering, maxSteering);
  
  if (steer_cmd < minSteering) {
    steer_cmd = minSteering;
  }
  if (steer_cmd > maxSteering) {
    steer_cmd = maxSteering ;
  }
  
  //write Servo output for steering
  servo_values[0] = steer_cmd;

  //Handle for throttle command
  if ((control_msg.data[1] == 0)){
    throttle_cmd = 0;
    //smt_Throttle = 0;
  }

  // ESC forward continue
  if ((control_msg.data[1] >= 0) && (prev_dir == 0)){
    throttle_cmd = control_msg.data[1];
    //smt_Throttle = smt_Throttle + 0.2 * (throttle_cmd - smt_Throttle);
  }
  
  //ESC reverse continue 
  if ((control_msg.data[1] < 0 ) && (prev_dir == 1)){
    throttle_cmd = control_msg.data[1];
    //smt_Throttle = smt_Throttle + 0.2 * (throttle_cmd - smt_Throttle);
  }

  //From forward to rev
  if ((control_msg.data[1] < 0 ) && (prev_dir == 0)){
    throttle_cmd = fw_to_rev(control_msg.data[1]);
    //smt_Throttle = throttle_cmd;
  }

  //From rev to forward
  if ((control_msg.data[1] > 0 ) && (prev_dir == 1)){ 
    throttle_cmd = rev_to_fw();
    //smt_Throttle = throttle_cmd;
  }

  servo_values[1] = fmap(throttle_cmd, -1.0, 1.0, minSteering, maxSteering);

}
