#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>         //set position 용
#include <mavros_msgs/CommandBool.h>           //arm용
#include <mavros_msgs/SetMode.h>               //OFFBOARD 모드 설정용
#include <mavros_msgs/State.h>                 //mavros 메세지 활용용
#include "solar_sys_formation/msgCoordinate.h" //메세지 정의용

// Parameter
int mode = 0;

double coor[3] = {
    0.0,
};
double com_x = 0.0;
double com_y = 0.0;
double com_z = 2.5;

solar_sys_formation::msgCoordinate msg; //자기 위치 pub용 메세지
mavros_msgs::SetMode set_mode;          // mode change 용
geometry_msgs::PoseStamped pose_set;    //자기 목표 위치 설정용
mavros_msgs::CommandBool arm_cmd;       // arm용

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }
geometry_msgs::PoseStamped
    current_position; //자기기준 local 위치 긁어오기 메세지
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_position = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_node");

  ROS_INFO("Control test node executed");

  ros::NodeHandle nh;

  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "mavros/setpoint_position/local", 10);

  ros::Publisher ros_coordinate_pub =
      nh.advertise<solar_sys_formation::msgCoordinate>("ros_coordinate_msg",
                                                       100);

  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "mavros/local_position/pose", 10, position_cb);

  ros::Subscriber state_sub =
      nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

  ros::ServiceClient arming_client =
      nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0); // period 0.05 s

  // wait for FCU connection
  while (ros::ok() && current_state.connected)
  {
    ROS_INFO("Drone connected");
    ros::spinOnce();
    rate.sleep();
  }

  nh.setParam("test_node/x", 0.0);
  nh.setParam("test_node/y", 0.0);
  nh.setParam("test_node/z", 2.5);

  // selfie initial position
  pose_set.pose.position.x = 0;
  pose_set.pose.position.y = 0;
  pose_set.pose.position.z = 2.5;

  // send a few setpoints before starting
  for (int i = 40; ros::ok() && i > 0; --i)
  {
    local_pos_pub.publish(pose_set);
    ros::spinOnce();
    rate.sleep();
  }

  // switch mode to OFFBOARD
  set_mode.request.custom_mode = "OFFBOARD";

  ros::Time last_request = ros::Time::now();

  while (ros::ok())
  {

    if (current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      set_mode.request.custom_mode = "OFFBOARD";
      if (set_mode_client.call(set_mode) && set_mode.response.success)
      {
        ROS_INFO("OFFBOARD enabled");
      }
      else
      {
        ROS_INFO("OFFBOARD disabled");
      }
      last_request = ros::Time::now();
    }

    nh.getParam("test_node/x", com_x);
    nh.getParam("test_node/y", com_y);
    nh.getParam("test_node/z", com_z);

    if (com_z != 0)
    {
      arm_cmd.request.value = true;

      if (!current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("armed");
        }

        last_request = ros::Time::now();
      }


      if (com_x != coor[0] | com_y != coor[1] | com_z != coor[2])
      {
        ROS_INFO("(target position : %lf, %lf, %lf)", com_x, com_y, com_z);
        pose_set.pose.position.x = com_x;
        pose_set.pose.position.y = com_y;
        pose_set.pose.position.z = com_z;
      }
      local_pos_pub.publish(pose_set);
    }
    if (com_x == 0 && com_y == 0 && com_z == 0)
    {
      set_mode.request.custom_mode = "AUTO.LAND";
    }
    /*
    ROS_INFO("cur_position : %lf, %lf, %f", current_position.pose.position.x,
             current_position.pose.position.y,
             current_position.pose.position.z);
             */

    coor[0] = com_x;
    coor[1] = com_y;
    coor[2] = com_z;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
