#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>

#include "gazebo_sensors_tutorial/getpiecepose.h"


tf2_ros::Buffer tfBuffer;

bool getPiecePose(gazebo_sensors_tutorial::getpiecepose::Request  &req, gazebo_sensors_tutorial::getpiecepose::Response &res)
{

  static geometry_msgs::TransformStamped transformStamped;
  ROS_INFO("The pose of piece %s is:", req.id.c_str());

  //Get the pose from world to the detected aruco marker
  for(int i=0; i<3; i++){
    try{
      transformStamped = tfBuffer.lookupTransform(req.id, "world", ros::Time(0));//, ros::Duration(0.1));
    }
    catch (tf2::TransformException ex ){
      ROS_ERROR("%d - %s",i,ex.what());
      //exception due to not yet available tf. Publish joint_states and retry.
      continue;
    }
  }
  res.stampedpose.transform.translation.x = transformStamped.transform.translation.x;
  res.stampedpose.transform.translation.y = transformStamped.transform.translation.y;
  res.stampedpose.transform.translation.z = transformStamped.transform.translation.z;
  res.stampedpose.transform.rotation.x = transformStamped.transform.rotation.x;
  res.stampedpose.transform.rotation.y = transformStamped.transform.rotation.y;
  res.stampedpose.transform.rotation.z = transformStamped.transform.rotation.z;
  res.stampedpose.transform.rotation.w = transformStamped.transform.rotation.w;

  ROS_INFO("x: %f", res.stampedpose.transform.translation.x);
  ROS_INFO("y: %f", res.stampedpose.transform.translation.y);
  ROS_INFO("z: %f", res.stampedpose.transform.translation.z);
  ROS_INFO("qx: %f", res.stampedpose.transform.rotation.x);
  ROS_INFO("qy: %f", res.stampedpose.transform.rotation.y);
  ROS_INFO("qz: %f", res.stampedpose.transform.rotation.z);
  ROS_INFO("qw: %f", res.stampedpose.transform.rotation.w);

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception");
  ros::NodeHandle n;

  //The node provides a service to get the pose of a given piece identified by its ID (string)
  ros::ServiceServer serviceGerPiecePose = n.advertiseService("gazebo_sensors_tutorial/getpiecepose", getPiecePose);

  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::spin();
  return 0;
}
