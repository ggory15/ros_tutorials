/* *************************************************************************\
   Copyright 2020 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
 * Software License Agreement (BSD License)
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Institute of Industrial and Control Engineering,
 *     (IOC), Universitat Politecnica de Catalunya or BarcelonaTech nor
 *     the names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Leopold Palomo-Avellaneda
   Desc:   tablecalibrator to find a camera wrt a camera
*/


#include <tablesens/cameraposfinder.h>
#include <math.h>
#include <eigen_conversions/eigen_msg.h>

cameraPosFinder::cameraPosFinder(ros::NodeHandle *nh) {
        iteration = 0;
        maxIter = 0;

        capturing = false;
        configured = false;
        node = nh;
        name = node->getNamespace();

        serviceConf = node->advertiseService("configureFromService",
                                           &cameraPosFinder::configureFromService, this);
    }
bool cameraPosFinder::configureFromService(tablesens::configureFromService::Request &req,
                                           tablesens::configureFromService::Response &resp){

    markersFixRefs.clear();
    resp.success = true;
    return true;
}

bool cameraPosFinder::configureFromParams(){

    ROS_INFO_STREAM_NAMED(name, "Starting tablecalibrator node...");

    // we delete all previous info
    markersFixRefs.clear();
    iteration = 0;
    uint k = 0;
    configured = false;

    // here we store a vector with the markerFix list
    // this list contains the ar_marker_## number
    std::vector<int> markerFixList;

    // here we store the camera frame link
    std::string camera_frame;

    // temporally var to store names
    std::string param_name;

    // now we have a list of markers that we'll use to calibrate
    // now we need to obtain its position wrt the workd frame

    if (!node->hasParam("markerFixList"))
    {
         ROS_INFO_STREAM("No param named markerFixList");
         return configured;
    }
    else {
      node->getParam("markerFixList", markerFixList);
      if ( markerFixList.empty()){
          ROS_WARN_STREAM_NAMED(name, "Parameter vector has not correct size. ");
          return configured;
      }
    }
    //poolT.resize(markerFixList.size());

    while (k<markerFixList.size()){
        // we need position of markerFixList[k] items
        param_name = "markerFixPos_" + std::to_string(markerFixList[k]);
        std::vector<double> tempVect;
        if (!node->hasParam(param_name))
        {
             ROS_INFO_STREAM("No param named " + param_name);
             return configured;
        }
        else {
          node->getParam(param_name, tempVect);
          if ( tempVect.size()==7){ // we expect 7 elements

              markerPosRef newRef;

              Eigen::Quaterniond q;
              q = Eigen::Quaterniond(tempVect[6], tempVect[3],tempVect[4], tempVect[5]);
              q.normalize();
              Eigen::Affine3d aq = Eigen::Affine3d(q);
              Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(tempVect[0],tempVect[1],tempVect[2])));

              newRef.fixRef = (t*aq);

              //   std::map<uint,markerPosRef> markersFixRefs;
              markersFixRefs.insert( std::pair<uint,markerPosRef>(uint(markerFixList[k]),newRef) );

          }else{
              ROS_WARN_STREAM_NAMED(name, "Parameter vector has not correct size. ");
              return configured;
          }
       }
        ++k;
    }

    // If no error we have loaded the fixed position of the markers wrt the fixed frame
    for (std::map<uint,markerPosRef>::iterator iter=markersFixRefs.begin(); iter!=markersFixRefs.end(); ++iter)
    {
      ROS_INFO_STREAM("#######################################################");
      ROS_INFO_STREAM("We have loaded the fixed marker " << (*iter).first);
      ROS_INFO_STREAM("with the Transform wrt fixed world of:");
      ROS_INFO_STREAM(std::endl << (*iter).second.fixRef.matrix());
    }
    configured = true;
    return configured;
}
// prepare the class
bool cameraPosFinder::captureMarkersStart(uint64_t elem){
    iteration = 0;
    capturing = true;
    maxIter = elem;
    if(maxIter > 0){
       // we subscribe to the topic of ar_pose_marker
        ROS_INFO_STREAM("Subscribing to the ar_pose_marker topic");

        this->obtainMarkers = node->subscribe("/aruco_marker_publisher/markers", 1000, &cameraPosFinder::calculate_Pose, this);
    }
    ros::Rate loop_rate(10);
    ROS_INFO_STREAM("Starting the loop");
    while (node->ok() && iteration < maxIter ){
         loop_rate.sleep();
         ros::spinOnce();
    }
    capturing = false;
    return true;
}
bool cameraPosFinder::captureMarkersStop(){
    capturing = false;
    return true;
}

//void cameraPosFinder::callback_number(const std_msgs::Int64& msg) {
//        counter += msg.data;
//        std_msgs::Int64 new_msg;
//        new_msg.data = counter;
//        pub.publish(new_msg);
//    }

//bool cameraPosFinder::callback_reset_counter(std_srvs::SetBool::Request &req,
//                                std_srvs::SetBool::Response &res)
//    {
//        if (req.data) {
//            counter = 0;
//            res.success = true;
//            res.message = "Counter has been successfully reset";
//        }
//        else {
//            res.success = false;
//            res.message = "Counter has not been reset";
//        }

//        return true;
//    }


void cameraPosFinder::calculate_Pose(const aruco_msgs::MarkerArray &msg)
{
    // first we check if we have arrived to limit
    if(iteration > maxIter){
        ROS_INFO_STREAM("End capturing poses");
        capturing = false;
        obtainMarkers.shutdown();
    }
    else{
        Eigen::Affine3d markerTCamera, markerTCamera2, worldTCamera;

        //ROS_INFO_STREAM("We have detected : " << msg.markers.size() << " markers");

        // let we calculate the weidth of every marker. To do that, we need the
        // total distance of each marker to camera

        double distMarker = 0.;


        for( uint i = 0; i < msg.markers.size();i++)
        {
            //ROS_INFO_STREAM("Marker " << i << " has the id: " << msg.markers.at(i).id );
            auto it = markersFixRefs.find(msg.markers.at(i).id); // let we check if we have it

            if(it == markersFixRefs.end()){
                //ROS_INFO_STREAM("Marker id: " << msg.markers.at(i).id << " is not in the Fix List");
            }
           else{
                //ROS_INFO_STREAM("Marker id = " << msg.markers.at(i).id << " in the Fix List");

                 // now we can ponderate every marker wrt its distance to the camera
                distMarker = sqrt(pow(msg.markers.at(i).pose.pose.position.x,2) +
                                  pow(msg.markers.at(i).pose.pose.position.y,2) +
                                  pow(msg.markers.at(i).pose.pose.position.z,2));

                Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(msg.markers.at(i).pose.pose.position.x,
                                                                       msg.markers.at(i).pose.pose.position.y,
                                                                       msg.markers.at(i).pose.pose.position.z)));
                Eigen::Quaterniond q;
                q = Eigen::Quaterniond(msg.markers.at(i).pose.pose.orientation.w,
                                       msg.markers.at(i).pose.pose.orientation.x,
                                       msg.markers.at(i).pose.pose.orientation.y,
                                       msg.markers.at(i).pose.pose.orientation.z);
                q.normalize();
                Eigen::Affine3d aq = Eigen::Affine3d(q);

                markerTCamera = t*aq;
                tf::poseMsgToEigen(msg.markers.at(i).pose.pose, markerTCamera2);

                worldTCamera = it->second.fixRef * markerTCamera.inverse();
                worldTCamera.rotation().normalized();

                // add the new transform
                it->second.poolT.addtrans(worldTCamera, distMarker);
                q = worldTCamera.rotation();
                //ROS_INFO_STREAM("Added trans " << std::endl << worldTCamera.matrix());
                //ROS_INFO_STREAM(std::endl << "Just put (x, y, z, qx, qy, qx, qw): "
                //                          << worldTCamera.translation().x() << " " << worldTCamera.translation().y()  << " " << worldTCamera.translation().z()  << " "
                //                << q.x() << " " << q.y() << " " << q.z() << " " << q.w());
            }
        }
        iteration++;
        ROS_INFO_STREAM("We are in the iteration " << iteration << " of " << maxIter << std::endl);
        if(iteration > maxIter){
            ROS_INFO_STREAM("End capturing poses");
            capturing = false;
        }
   }
}

void cameraPosFinder::calculateFinalCameraPosition(){
    // we need to wait till capturing is end

    while(capturing){
        ROS_INFO_STREAM("We are still capturing ...");
        ROS_INFO_STREAM("We wait ...");
        ros::Duration(0.5).sleep();
    }

    // If no error we have loaded the fixed position of the markers wrt the fixed frame
    double avgx, avgy, avgz, avgqw, avgqx, avgqy, avgqz, weight;

    // here we store the inverse of the final distance
    // the idea is to weigh with the inverse of
    // the distance between the marker and the camera
    // giving more weight to nearby markers
    // that far markers
    double totalDistance = 0.;

    avgx = 0.;
    avgy = 0.;
    avgz = 0.;
    avgqw = 0.;
    avgqx = 0.;
    avgqy = 0.;
    avgqz = 0.;
    Eigen::Quaterniond rot;

    for (std::map<uint,markerPosRef>::iterator iter=markersFixRefs.begin(); iter!=markersFixRefs.end(); ++iter)
    {
//      ROS_INFO_STREAM("We have found this camera position for the Marker: " << (*iter).first << " with it Transform wrt fixed world");
//      ROS_INFO_STREAM(std::endl << (*iter).second.poolT.aritMean.matrix());
//      ROS_INFO_STREAM("at this distance : " << (*iter).second.poolT.avgDistance);

      totalDistance += 1./(*iter).second.poolT.avgDistance;

      rot = (*iter).second.poolT.aritMean.rotation();
//      ROS_INFO_STREAM(std::endl << "Its propose is to put (x, y, z, qx, qy, qx, qw): "
//                                << (*iter).second.poolT.aritMean.translation().x() << " " << (*iter).second.poolT.aritMean.translation().y()  << " " << (*iter).second.poolT.aritMean.translation().z()  << " "
//                      << rot.x() << " " << rot.y() << " " << rot.z() << " " << rot.w());
    }
    // now we have the total distante weighted

    for (std::map<uint,markerPosRef>::iterator iter=markersFixRefs.begin(); iter!=markersFixRefs.end(); ++iter)
    {

      weight = (1./(*iter).second.poolT.avgDistance)/totalDistance;

      avgx += (*iter).second.poolT.aritMean.translation().x() * weight;
      avgy += (*iter).second.poolT.aritMean.translation().y() * weight;
      avgz += (*iter).second.poolT.aritMean.translation().z() * weight;

      rot  = (*iter).second.poolT.aritMean.rotation();
      avgqw += rot.w() * weight;
      avgqx += rot.x() * weight;
      avgqy += rot.y() * weight;
      avgqz += rot.z() * weight;

    }
    // Now we build the new avgTrans
    Eigen::Affine3d tc(Eigen::Translation3d(Eigen::Vector3d(avgx, avgy, avgz)));
    Eigen::Quaterniond qc;
    qc = Eigen::Quaterniond(avgqw, avgqx, avgqy, avgqz);
    qc.normalize();
    Eigen::Affine3d worldTCamera = tc*qc;
    ROS_INFO_STREAM(std::endl
                 << "###########################################################" << std::endl
                 << "###########################################################" << std::endl
                 << "We have finally obtain Transform Camera wrt fixed world" << std::endl
                 << worldTCamera.matrix() << std::endl
                 << "Just put (x, y, z, qx, qy, qx, qw): " << std::endl
                 << avgx << " " << avgy << " " << avgz << " "
                 << qc.x() << " " << qc.y() << " " << qc.z() << " " << qc.w() << std::endl
                 << "###########################################################" << std::endl
                 << "###########################################################");
}
void cameraPosFinder::findCamera(unsigned int iterations)
{
   configureFromParams();
   captureMarkersStart(iterations);
   calculateFinalCameraPosition();
   ros::shutdown();
}



