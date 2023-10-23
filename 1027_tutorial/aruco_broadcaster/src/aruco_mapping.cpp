// C++
#include <vector>
#include <cstdint>

#include <ros/ros.h>

#include <tf2/transform_storage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aruco_msgs/Marker.h>
#include <aruco_msgs/MarkerArray.h>
#include <algorithm>


static std::vector<int> markerList;
static std::string camera_frame, aruco_frame;
static bool publish_all;

void markersCallback(const aruco_msgs::MarkerArray &marker_info){

  static tf2_ros::TransformBroadcaster br; //we want _only_ one broadcaster

  tf2::Transform _cameraToMarker, _CameraToReference, _referenceToMarker;
  geometry_msgs::TransformStamped transformStamped;

  if(publish_all)
  {
      for( uint i = 0; i < marker_info.markers.size();i++)
      {
          tf2::fromMsg(marker_info.markers.at(i).pose.pose, _cameraToMarker);
          transformStamped.header.stamp = ros::Time::now();
          transformStamped.header.frame_id = camera_frame;
          transformStamped.child_frame_id = aruco_frame + "_" + std::to_string(marker_info.markers.at(i).id);
          transformStamped.transform = tf2::toMsg(_cameraToMarker);
          br.sendTransform(transformStamped);
      }
  }
  else
  {
      for (unsigned int i = 0; i < marker_info.markers.size(); ++i)
      {
          if(std::find(markerList.begin(), markerList.end(), marker_info.markers.at(i).id) != markerList.end())
          {
              tf2::fromMsg(marker_info.markers.at(i).pose.pose, _cameraToMarker);
              transformStamped.header.stamp = ros::Time::now();
              transformStamped.header.frame_id = camera_frame;
              transformStamped.child_frame_id = aruco_frame + "_" + std::to_string(marker_info.markers.at(i).id);
              transformStamped.transform = tf2::toMsg(_cameraToMarker);
              br.sendTransform(transformStamped);
          }
      }
   }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"aruco_broadcaster");
  ros::NodeHandle nh;
  
  ros::Publisher markers_pub;
  ros::Subscriber camera_aruco_tf_subs;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener _tfListener(tfBuffer);


  camera_aruco_tf_subs = nh.subscribe("/aruco_marker_publisher/markers", 1, markersCallback);

  if (nh.hasParam("markerList"))
  {
    nh.getParam("markerList", markerList);
    if (markerList.empty())
    {
      ROS_WARN_STREAM("Aruco broadcaster will publish all the arucos found");
      publish_all = true;
    }
    else
    {
       ROS_WARN_STREAM("markerList has size of: " << markerList.size() << std::endl);
       publish_all = false;
    }
  }
  else
  {
    ROS_INFO_STREAM("No param named markerList");
    publish_all = true;
  }

  if (nh.hasParam("camera_frame"))
  {
    nh.getParam("camera_frame", camera_frame);
    if(camera_frame.empty())
    {
      ROS_WARN_STREAM("Parameter camera frame has not reference");
      return false;
    }
    else
    {
      ROS_WARN_STREAM("Parameter camera_frame named " + camera_frame);
    }
  }
  else
  {
    ROS_INFO_STREAM("No param given in camera_frame");
  }

  if (nh.hasParam("aruco_frame"))
  {
    nh.getParam("aruco_frame", aruco_frame);
    if (aruco_frame.empty())
    {
      ROS_WARN_STREAM("Parameter aruco_frame is empty");
      return false;
    }
    else
    {
      ROS_WARN_STREAM("Parameter aruco_frame is named " + aruco_frame);
    }
  }
  else
  {
    ROS_INFO_STREAM("No param given in aruco_frame");
  }

  ros::spin();
  return 0;
  
}
