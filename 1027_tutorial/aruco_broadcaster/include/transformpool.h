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
   Desc:   cameraPosFinder class to find a camera position wrt a fixed frame
*/


#ifndef TRANSFORMPOOL_H
#define TRANSFORMPOOL_H
// C++
#include <vector>
#include <iostream>

// Eigen stuff
#include <Eigen/Geometry>
#include <Eigen/Dense>


// transformPool is a class that store a set of Eigen::Affine3d
// and operates with them

class transformPool{

    private:
        // where we store the trans;
        std::vector<Eigen::Affine3d> transStore;
        Eigen::Matrix4d M; // where we store sum of qi * qi^T
        bool initiate; // we have to take care of the first case.

    public:
        // here we put the average distance of the marker to the camera
        // it will be use to ponderate wrt other markers
        double avgDistance;

        // here we store the Arithmetic mean
        Eigen::Affine3d aritMean;

        // default contructors
        transformPool(){
          Eigen::Affine3d t(Eigen::Translation3d(0., 0., 0.));
          Eigen::Quaterniond q;
          q = Eigen::Quaterniond(0., 0., 0., 0.);
          Eigen::Affine3d aq = Eigen::Affine3d(q);
          aritMean = t*aq;
          avgDistance = 0.;
          M = Eigen::Matrix4d::Zero(); // init to zero
          initiate = false;
        }

        transformPool& operator = (const transformPool &b)
        {
          avgDistance = b.avgDistance;
          aritMean = b.aritMean;
          transStore = b.transStore;
          return *this;
        }
        // this method add a new Tranform to the pool
        // and update the aritMean
        // and add a new distance vector from the marker to the
        // camera to calculate the avg

        void addtrans(Eigen::Affine3d &newTrans, const double &newDist){
            Eigen::Quaterniond rot;


            if(!initiate){
                // first case. We use it
                avgDistance = newDist;
                transStore.push_back(newTrans);
                // to access to quaternion we need first to have the rotation
                rot = newTrans.rotation();
                Eigen::Vector4d q0(rot.x(), rot.y(), rot.z(), rot.w());
                M = q0 * q0.transpose();
                aritMean = newTrans;

                initiate = true;

            }
            else{

                uint64_t elems; // tmp var just to have a clean code
                double avgx, avgy, avgz, avgqw, avgqx, avgqy, avgqz;
                // temporally vars
                double tnewqx, tnewqy, tnewqz, tnewqw;

                Eigen::Quaterniond newrot;

                avgx = aritMean.translation().x();
                avgy = aritMean.translation().y();
                avgz = aritMean.translation().z();

                rot  = aritMean.rotation();
                avgqw = rot.w();
                avgqx = rot.x();
                avgqy = rot.y();
                avgqz = rot.z();

                transStore.push_back(newTrans);
                // we update the new element size
                elems = transStore.size();

                // Average n+1 = new average with the element n+1
                // element n+1 = the new element added
                // A_{n+1} = A_n + (element_{n+1}-A_{n})/(n+1)

                // here we put the new average of the x, y and z values
                // and the quaternion part qw, qx, qy ,qz
                avgx = avgx + (newTrans.translation().x() - avgx)/elems;
                avgy = avgy + (newTrans.translation().y() - avgy)/elems;
                avgz = avgz + (newTrans.translation().z() - avgz)/elems;

                // to get access to quaternion values we need to access them
                newrot = newTrans.rotation();

                // we need to check if we can added or invert it
                if(!checkQuaternionClose(rot,newrot)){
                    //we new to make inverse
                    tnewqw = -newrot.w();
                    tnewqx = -newrot.x();
                    tnewqy = -newrot.y();
                    tnewqz = -newrot.z();
                }else {
                    tnewqw = newrot.w();
                    tnewqx = newrot.x();
                    tnewqy = newrot.y();
                    tnewqz = newrot.z();
                }

                /// Calculate average with a traditional method
                ///
                ///
                /*
                avgqw = avgqw + (tnewqw - avgqw)/elems;
                avgqx = avgqx + (tnewqx - avgqx)/elems;
                avgqy = avgqy + (tnewqy - avgqy)/elems;
                avgqz = avgqz + (tnewqz - avgqz)/elems;
                */
                // Calculate average using method proposed here
                // http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872_2007014421.pdf
                Eigen::Vector4d qi(tnewqx, tnewqy, tnewqz, tnewqw);
                qi.normalize(); //should be normalized but ...
                M = M + qi * qi.transpose(); // e have to take care i i goes to infinite.

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(M);
                if (eigensolver.info() != Eigen::Success){
                    std::abort(); //we have to do something
                }
                Eigen::Vector4d _qi = eigensolver.eigenvectors().col(3); //we take the biggest one that contains the average

                // No we build the quaternion
                Eigen::Quaterniond qc;
                qc = Eigen::Quaterniond(_qi(3), _qi(0), _qi(1), _qi(2));
                //qc.normalize(); we don't need it. It's normalized

                // Now we build the new avgTrans
                Eigen::Affine3d tc(Eigen::Translation3d(Eigen::Vector3d(avgx, avgy, avgz)));
                aritMean = tc*qc;

                avgDistance = avgDistance + (newDist - avgDistance)/elems;
            } // end of else (initiate)
        }
        bool checkQuaternionClose(Eigen::Quaterniond &q1, Eigen::Quaterniond q2){
            double dot = q1.dot(q2);
            if(dot < 0.0){
                return false;
            }else {
              return true;
            }
        }
        ~transformPool(){}

};

#endif // TRANSFORMPOOL_H
