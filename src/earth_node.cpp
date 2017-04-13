#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>         //set position 용
#include <mavros_msgs/CommandBool.h>           //arm용
#include <mavros_msgs/SetMode.h>               //offboard 모드 설정용
#include <mavros_msgs/State.h>                 //mavros 메세지 활용용
#include "math.h"                              //수식 입력용
#include <sensor_msgs/NavSatFix.h>             //GPS 긁어오기용

double r = 8;
double theta;
double count = 0.0;
double wn = 0.05;

double sun_x = 0.0;
double sun_y = 0.0;
double sun_z = 6.0;

double earth_x = 0.0;
double earth_y = 0.0;
double earth_z = 5.0;

int mode = -1;

geometry_msgs::PoseStamped earth_pos_set(double x, double y, double z);

mavros_msgs::State current_state; //자기 상태 pub용 메세지
void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

geometry_msgs::PoseStamped current_position; //자기기준 local 위치 긁어오기 메세지
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  current_position = *msg;
}

sensor_msgs::NavSatFix current_GPS; //자기 global 위치 긁어오기 메세지
void GPS_cb(const sensor_msgs::NavSatFixConstPtr& msg) { current_GPS = *msg; }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "earth_node");

  ros::NodeHandle nh;
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros_earth/setpoint_position/local", 10);

  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros_earth/local_position/pose", 10, position_cb);
  ros::Subscriber global_pos_sub = nh.subscribe("/mavros_earth/global_position/raw/fix", 10, GPS_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros_earth/state", 10, state_cb);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros_earth/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros_earth/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz 아니면 OFFBOARD 취소 됨
  ros::Rate rate(10.0); // period 0.1 s

  // earth_node의 mode 라는 para를 mode 라는 변수에 넣고 값은 -1
  nh.param("earth_node/mode", mode, -1);

  // wait for FCU connection
  while (ros::ok() && current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  // switch mode to OFFBOARD
  mavros_msgs::SetMode offb_set_mode; //mode change 용
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // arming 
  mavros_msgs::CommandBool arm_cmd;   //arm용
  arm_cmd.request.value = true;

  // initial position
  geometry_msgs::PoseStamped pos_set;  //자기 목표 위치 설정용
  pos_set.pose.position.x = earth_x;
  pos_set.pose.position.y = earth_y;
  pos_set.pose.position.z = earth_z;

  // send a few setpoints before starting
  for (int i = 40; ros::ok() && i > 0; --i)  {
    local_pos_pub.publish(pos_set);
    ros::spinOnce();
    rate.sleep();
  }

  ros::Time last_request = ros::Time::now();

  while (ros::ok()){
    nh.param("earth_node/mode", mode, -1);

    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0))){ // OFFBOARD 가 아니면 3초에 한번씩 갈김
      offb_set_mode.request.custom_mode = "OFFBOARD";
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.success){
        ROS_INFO("earth Offboard enabled");
      }
      last_request = ros::Time::now();
    }

    if (mode == 0){ // arm & takeoff
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))){
      	arm_cmd.request.value = true; // arming
        if (arming_client.call(arm_cmd) && arm_cmd.response.success){
          ROS_INFO("earth armed");
        }
        last_request = ros::Time::now();
      }
      else{ // take off
      	pos_set = earth_pos_set(earth_x, earth_y, earth_z);
      	local_pos_pub.publish(pos_set);
      }
    }
    else if (mode == 1){ // round round
    	theta = wn * count * 0.05; // 0.4rad/s if wn=0.4 in rate 20
    	pos_set = earth_pos_set(sun_x + r * sin(theta), sun_y + r * cos(theta), sun_z);
    	local_pos_pub.publish(pos_set);
    	//나중을 위한 GPS pub
        //ROS_INFO("lon : %f, lat : %f, alt : %f", current_GPS.longitude, current_GPS.latitude,current_GPS.altitude);
        count++;
    }
    else if (mode == 2){ // going home
    	pos_set = earth_pos_set(0, 0, 5);
    	local_pos_pub.publish(pos_set);
    }
    else if (mode == 3){ // landing & disarm
          offb_set_mode.request.custom_mode = "AUTO.LAND";
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

geometry_msgs::PoseStamped earth_pos_set(double x, double y, double z){
	geometry_msgs::PoseStamped position;
	earth_x = x;
	earth_y = y;
	earth_z = z;
	position.pose.position.x = earth_x;
	position.pose.position.y = earth_y;
	position.pose.position.z = earth_z;
	position.header.stamp = ros::Time::now();
	return position;
}