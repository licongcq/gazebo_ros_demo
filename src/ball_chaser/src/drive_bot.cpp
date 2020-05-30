#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

class DriveToTargetService {
 public:
  DriveToTargetService(ros::NodeHandle& node_handle);

 private:
  // Execute whenever a drive_bot service is requested. This function publish
  // the requested linear x and angular velocities to the robot wheel joints.
  // After publishing the requested velocities, a message feedback is returned
  // with the requested wheel velocities
  bool HandleDriveRequest(ball_chaser::DriveToTarget::Request& req,
                          ball_chaser::DriveToTarget::Response& res);

  // ROS::Publisher motor commands;
  ros::Publisher motor_command_publisher_;

  // Registered ROS service. Note this is after motor_command_publisher_ so we
  // are sure that the motor_command_publisher_ is always available to service.
  ros::ServiceServer service_;
};

DriveToTargetService::DriveToTargetService(ros::NodeHandle& node_handle) {
  // Inform ROS master that we will be publishing a message of type
  // geometry_msgs::Twist on the robot actuation topic with a publishing queue
  // size of 10.
  motor_command_publisher_ =
      node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Define a drive /ball_chaser/command_robot service with a
  // handle_drive_request callback function
  service_ = node_handle.advertiseService(
      "ball_chaser/command_robot", &DriveToTargetService::HandleDriveRequest,
      this);

  ROS_INFO("DriveToTarget: Ready");
}

bool DriveToTargetService::HandleDriveRequest(
    ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res) {
  // ROS_INFO("DriveToTarget: Received linear_x:%1.2f, angular_z:%1.2f",
  //          (float)req.linear_x, (float)req.angular_z);

  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;
  // Set wheel velocities.
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;
  // Publish angles to drive the robot
  motor_command_publisher_.publish(motor_command);

  // Return a response message
  res.msg_feedback = "DriveToTarget: Requested linear_x:" +
      std::to_string(req.linear_x) + " angular_z: " + 
      std::to_string(req.angular_z);
  // ROS_INFO_STREAM(res.msg_feedback);

  return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Setup the service and start processing requests.
    DriveToTargetService drive_to_target_service(n);

    ros::spin();

    return 0;
}
