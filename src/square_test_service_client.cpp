#include "ros/ros.h"
#include "std_srvs/Empty.h"
// Import the service message used by the service /perform_square

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_move_Robot_in_square_client"); // Initialise a ROS node
  ros::NodeHandle nh;

  // Create the connection to the service /perform_square
  ros::ServiceClient perform_square_service_client = nh.serviceClient<std_srvs::Empty>("/perform_square");
  std_srvs::Empty srv; // Create an object of type Empty

  if (perform_square_service_client.call(srv))
  {
    ROS_INFO("Service successfully called. Moving BB8 in a square.");
  }
  else
  {
    ROS_ERROR("Failed to call service /perform_square");
    return 1;
  }

  return 0;
}