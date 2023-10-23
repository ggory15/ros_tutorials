#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

using namespace std;


geometry_msgs::TransformStamped ts_aruco_201;
geometry_msgs::TransformStamped ts_aruco_582;


// CallBack function Aruco 582 Pose subscribion
void aruco582Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_582.header.stamp = ros::Time::now();
  ts_aruco_582.transform.translation.x = msg.pose.position.x;
  ts_aruco_582.transform.translation.y = msg.pose.position.y;
  ts_aruco_582.transform.translation.z = msg.pose.position.z - 0.05; //height of cube is 10 cm
  ts_aruco_582.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_582.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_582.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_582.transform.rotation.w = msg.pose.orientation.w;
}

// CallBack function Aruco 201 Pose subscribion
void aruco201Callback(geometry_msgs::PoseStamped msg)
{
  ts_aruco_201.header.stamp = ros::Time::now();
  ts_aruco_201.transform.translation.x = msg.pose.position.x;
  ts_aruco_201.transform.translation.y = msg.pose.position.y;
  ts_aruco_201.transform.translation.z = msg.pose.position.z - 0.02; //height of cube is 4 cm
  ts_aruco_201.transform.rotation.x = msg.pose.orientation.x;
  ts_aruco_201.transform.rotation.y = msg.pose.orientation.y;
  ts_aruco_201.transform.rotation.z = msg.pose.orientation.z;
  ts_aruco_201.transform.rotation.w = msg.pose.orientation.w;
}


int main(int argc, char** argv){

  ros::init(argc, argv, "aruco_frames_detection");

  ros::NodeHandle n;

  tf2_ros::TransformBroadcaster tfb_aruco_201;
  tf2_ros::TransformBroadcaster tfb_aruco_582;

  // Subscribe to all the Aruco pose messages:
  ros::Subscriber sub_582 = n.subscribe("/aruco_single_582/pose", 1000, aruco582Callback);
  ros::Subscriber sub_201 = n.subscribe("/aruco_single_201/pose", 1000, aruco201Callback);

  ros::Duration(1.0).sleep();

  // Initialize all transformStamped messages.
  ts_aruco_582.header.stamp = ros::Time::now();
  ts_aruco_582.header.frame_id = "world";
  ts_aruco_582.child_frame_id = "frame_582";
  ts_aruco_582.transform.translation.x = 0;
  ts_aruco_582.transform.translation.y = 0;
  ts_aruco_582.transform.translation.z = 0;
  ts_aruco_582.transform.rotation.x = 0;
  ts_aruco_582.transform.rotation.y = 0;
  ts_aruco_582.transform.rotation.z = 0;
  ts_aruco_582.transform.rotation.w = 1;

  ts_aruco_201.header.stamp = ros::Time::now();
  ts_aruco_201.header.frame_id = "world";
  ts_aruco_201.child_frame_id = "frame_201";
  ts_aruco_201.transform.translation.x = 0;
  ts_aruco_201.transform.translation.y = 0;
  ts_aruco_201.transform.translation.z = 0;
  ts_aruco_201.transform.rotation.x = 0;
  ts_aruco_201.transform.rotation.y = 0;
  ts_aruco_201.transform.rotation.z = 0;
  ts_aruco_201.transform.rotation.w = 1;

  ros::Rate rate(10.0);
  while (n.ok()){
    // Publish of aruco_582TF.
    tfb_aruco_582.sendTransform(ts_aruco_582);

    // Publish of aruco_201 TF.
    tfb_aruco_201.sendTransform(ts_aruco_201);

    rate.sleep();

    ros::spinOnce();
  }
  return 0;
};
