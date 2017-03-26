#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>         //set position 용
#include <mavros_msgs/CommandBool.h>           //arm용
#include <mavros_msgs/SetMode.h>               //offboard 모드 설정용
#include <mavros_msgs/State.h>                 //mavros 메세지 활용용
#include "geometry_msgs/TwistStamped.h"        //set velocity 용
#include "math.h"                              //수식 입력용
#include "solar_sys_formation/msgCoordinate.h" //메세지 정의용
#include <sensor_msgs/NavSatFix.h>             //GPS 긁어오기용

double r = 8;
double theta;
double count = 0.0;
double wn = 0.05;

float sun_x = 0.0;
float sun_y = 0.0;
float sun_z = 6.0;

float earth_x = 0.0;
float earth_y = 0.0;
float earth_z = 6.0;

int mode = -1;

solar_sys_formation::msgCoordinate msg; //자기 위치 pub용 메세지
mavros_msgs::CommandBool arm_cmd;   //arm용
mavros_msgs::SetMode offb_set_mode; //mode change 용
mavros_msgs::State current_state; //자기 상태 pub용 메세지
void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

geometry_msgs::PoseStamped current_position; //자기기준 local 위치 긁어오기 메세지
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_position = *msg;
}

sensor_msgs::NavSatFix current_GPS; //자기 global 위치 긁어오기 메세지
void GPS_cb(const sensor_msgs::NavSatFixConstPtr& msg) { current_GPS = *msg; }

geometry_msgs::PoseStamped pos_set;  //자기 목표 위치 설정용
geometry_msgs::TwistStamped vel_set; //자기 목표속도 설정용

int main(int argc, char** argv)
{
  ros::init(argc, argv, "earth_node");

  ros::NodeHandle nh;

  ros::Publisher ros_coordinate_pub =
      nh.advertise<solar_sys_formation::msgCoordinate>("ros_coordinate_msg",
                                                       100);

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros_earth/setpoint_position/local", 10);

  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(
      "mavros_earth/setpoint_velocity/cmd_vel", 10);

  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "mavros_earth/local_position/pose", 10, position_cb);

  ros::Subscriber global_pos_sub =
      nh.subscribe("/mavros_earth/global_position/raw/fix", 10, GPS_cb);

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros_earth/state", 10, state_cb);

  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros_earth/cmd/arming");

  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros_earth/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz 아니면 OFFBOARD 취소 됨
  ros::Rate rate(20.0); // period 0.05 s

  // earth_node의 mode 라는 para를 mode 라는 변수에 넣고 값은 -1
  nh.param("earth_node/mode", mode, -1);

  // wait for FCU connection
  while (ros::ok() && current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  // switch mode to OFFBOARD
  offb_set_mode.request.custom_mode = "OFFBOARD";

  // initial position
  pos_set.pose.position.x = earth_x;
  pos_set.pose.position.y = earth_y;
  pos_set.pose.position.z = earth_z;

  // send a few setpoints before starting
  for (int i = 40; ros::ok() && i > 0; --i)
  {
    local_pos_pub.publish(pos_set);
    ros::spinOnce();
    rate.sleep();
  }

  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {
    nh.param("earth_node/mode", mode, -1);

    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request >
         ros::Duration(3.0))) // OFFBOARD 가 아니면 3초에 한번씩 갈김
    {
      offb_set_mode.request.custom_mode = "OFFBOARD";

      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
      {
        ROS_INFO("earth Offboard enabled");
      }
      last_request = ros::Time::now();
    }

    if (mode == 0) // arm & takeoff
    { // arming
      arm_cmd.request.value = true;

      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("earth armed");
        }

        last_request = ros::Time::now();
      }
      else
      { // take off
        pos_set.pose.position.x = earth_x;
        pos_set.pose.position.y = earth_y;
        pos_set.pose.position.z = earth_z;
        local_pos_pub.publish(pos_set);
      }
    }

    else if (mode == 1) // round round
    { // getting own position
      theta = wn * count * 0.05; // 0.4rad/s if wn=0.4 in rate 20
      earth_x = sun_x + r * sin(theta);
      earth_y = sun_y + r * cos(theta);
      earth_z = sun_z;

      // goto own position
      pos_set.pose.position.x = earth_x;
      pos_set.pose.position.y = earth_y;
      pos_set.pose.position.z = earth_z;
      local_pos_pub.publish(pos_set);

      // publish current earth position to moon
      msg.earth_x = current_position.pose.position.x;
      msg.earth_y = current_position.pose.position.y;
      msg.earth_z = current_position.pose.position.z;
      ros_coordinate_pub.publish(msg);

       //나중을 위한 GPS pub
      //ROS_INFO("lon : %f, lat : %f,alt : %f", current_GPS.longitude, current_GPS.latitude,current_GPS.altitude);

      count++;
    }

    else if (mode == 2) // going home
    {
      earth_x = 0;
      earth_y = 0;
      earth_z = 6;
      pos_set.pose.position.x = earth_x;
      pos_set.pose.position.y = earth_y;
      pos_set.pose.position.z = earth_z;
      local_pos_pub.publish(pos_set);
    }

    else if (mode == 3) // landing & disarm
    { //  속도로 하거나 모드로 둘 다 가능
      /*
      pos_set.pose.position.x = 0;
      pos_set.pose.position.y = 0;
      pos_set.pose.position.z = 0;
      local_pos_pub.publish(pos_set);

      vel_set.twist.linear.x = 0;
      vel_set.twist.linear.y = 0;
      vel_set.twist.linear.z = -0.7;
      local_vel_pub.publish(vel_set);
      */
      offb_set_mode.request.custom_mode = "AUTO.LAND";
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
