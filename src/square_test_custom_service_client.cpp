#include "ros/ros.h"
#include "square_test_mobile_robot/square_test_service.h"
// Import the service message used by the service /move_bb8_in_square_custom

int main(int argc, char **argv) {
  ros::init(argc, argv,
            "service_move_bb8_in_square_client"); // Initialise a ROS node
  ros::NodeHandle nh;

  // Create the connection to the service /move_bb8_in_square_custom
  ros::ServiceClient perform_square_service_client =
      nh.serviceClient<square_test_mobile_robot::square_test_service>(
          "/move_bb8_in_square_custom");
  square_test_mobile_robot::square_test_service
      srv; // Create an object of type Empty
  srv.request.radius = 3.0;
  srv.request.repetitions = 2;

  if (perform_square_service_client.call(srv)) {
    ROS_INFO("Service successfully called. Moving BB8 in a square.");
  } else {
    ROS_ERROR("Failed to call service /move_bb8_in_square_custom");
    return 1;
  }

  return 0;
}