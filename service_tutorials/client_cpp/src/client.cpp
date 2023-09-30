#include "ros/ros.h"
#include "add_two_ints_server_py/AddTwoInts.h"
#include <cstdlib>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<add_two_ints_server_py::AddTwoInts>("add_two_ints");
  add_two_ints_server_py::AddTwoInts srv;
  srv.request.a = 1;
  srv.request.b = 3;

  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
