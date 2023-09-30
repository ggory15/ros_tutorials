
#include "ros/ros.h"
#include "add_two_ints_server_py/AddTwoInts.h"

bool add(add_two_ints_server_py::AddTwoInts::Request  &req,
         add_two_ints_server_py::AddTwoInts::Response &res )
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("  sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

// %Tag(SERVICE_SERVER)%
  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
// %EndTag(SERVICE_SERVER)%

  ros::spin();

  return 0;
}
