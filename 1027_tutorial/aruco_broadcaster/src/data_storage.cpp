//C++
#include <iostream>
#include <fstream>
#include <cmath>

//ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_msgs/TFMessage.h"
#include <geometry_msgs/TransformStamped.h>
#include "transformpool.h"

//Eigen Lib
#include <Eigen/Dense>
#include <Eigen/Geometry>

//geometry
//#include <tf2/>


std::vector<int> markerFixList;

std::vector<std::vector<double>> translation;
std::vector<std::vector<double>> rotation;


std::map<uint, transformPool> markerAvg;


void markerTransformCallback(const tf2_msgs::TFMessage &transform){

  Eigen::Affine3d Transform;

  for(unsigned int i = 0; i < transform.transforms.size(); ++i){
    std::string id;
    id = transform.transforms.at(i).child_frame_id;
    id.erase(id.begin(), id.end()-3);

    auto it = markerAvg.find(std::stoi(id));

    if (it != markerAvg.end()){
      Eigen::Affine3d t(Eigen::Translation3d(transform.transforms[i].transform.translation.x,
                                             transform.transforms[i].transform.translation.y,
                                             transform.transforms[i].transform.translation.z));

      Eigen::Quaterniond q;

      q = Eigen::Quaterniond(transform.transforms[i].transform.rotation.w,
                             transform.transforms[i].transform.rotation.x,
                             transform.transforms[i].transform.rotation.y,
                             transform.transforms[i].transform.rotation.z);

      q.normalize();
      Eigen::Affine3d aq = Eigen::Affine3d(q);
      Transform = t * aq;

      it->second.addtrans(Transform, 1.0);
    }
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "data_node");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Subscriber tf_marker = n.subscribe("/tf", 1000, markerTransformCallback);

  std::vector<int> markerFixList;
  std::vector<std::vector<double>> markerPose;
  std::string param_name;
  int iteration = 0;
  double translation_error, rot_e1, rot_e2, rotation_error_dist;
  Eigen::Quaterniond rot;

  if (n.hasParam("markerFixList"))
  {
    n.getParam("markerFixList", markerFixList);
    if (markerFixList.empty())
    {
      ROS_WARN_STREAM("Parameter vector has not correct size");
      return false;
    }
    else {
      for (unsigned int i = 0; i < markerFixList.size(); ++i) {
        transformPool Pool;
        markerAvg.insert(std::pair<uint,transformPool>(uint(markerFixList[i]),Pool));
      }
    }
  }
  else{
    ROS_INFO_STREAM("No param named markerFixList");
    return false;
  }

  for (unsigned int i = 0; i < markerFixList.size(); ++i) {

    param_name = "markerFixPos_" + std::to_string(markerFixList[i]);
    std::vector<double> tempVect;
    if (!n.hasParam(param_name)){
      ROS_WARN_STREAM("No marker Pose param given");
    }
    else{
      n.getParam(param_name, tempVect);
    }

    markerPose.push_back(tempVect);

  }

  while (iteration < 10){

    iteration = iteration + 1;

    ros::spinOnce();
    r.sleep();
  }

  for (int i = 0; i < markerFixList.size(); ++i) {

   translation_error = sqrt(pow(markerPose[i][0] - markerAvg[markerFixList[i]].aritMean.translation().x(), 2) +
        pow(markerPose[i][1] - markerAvg[markerFixList[i]].aritMean.translation().y(), 2) +
        pow(markerPose[i][2] - markerAvg[markerFixList[i]].aritMean.translation().z(), 2));

   rot = markerAvg[markerFixList[i]].aritMean.rotation();

   rot_e1 = sqrt(pow(markerPose[i][3] - rot.x(), 2)
        + pow(markerPose[i][4] - rot.y(), 2)
        + pow(markerPose[i][5] - rot.z(), 2)
        + pow(markerPose[i][6] - rot.w(), 2));


   rot_e2 = sqrt(pow(markerPose[i][3] + rot.x(), 2)
        + pow(markerPose[i][4] + rot.y(), 2)
        + pow(markerPose[i][5] + rot.z(), 2)
        + pow(markerPose[i][6] + rot.w(), 2));

    rotation_error_dist = std::min(rot_e1, rot_e2);

    std::cout << "Translation error of ID " << markerFixList[i] << " is " << translation_error << std::endl;

    std::cout << "Rotation error of ID " << markerFixList[i] << " is " << rotation_error_dist << std::endl;

  }

}
