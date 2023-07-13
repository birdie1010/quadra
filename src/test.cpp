#include <ros.h>
#include<sensor_msgs/JointState.h>

float* pose;
int value=30;

ros::NodeHandle quad_ard;

void pose_setter(const sensor_msgs::JointState &joint_msg)
{
  // pose=joint_msg.position;
  value+=20;
}
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", &pose_setter );