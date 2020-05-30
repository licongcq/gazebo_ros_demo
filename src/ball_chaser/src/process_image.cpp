#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // ROS_INFO_STREAM("ProcessImage: Moving the robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the DriveToTarget service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service DriveToTarget");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;

    if (img.height <= 0 || img.width <= 0 || img.step <= 0) {
      return;
    }

    // We'll calculate the average X coordinates of all white pixels to get an
    // estimation of the center of the ball.
    int white_pixel_count = 0;
    int white_pixels_sum_x = 0;

    // Loop through the image to find white pixels & calculate their average
    // X coordinates.
    for (int i = 0; i < img.height; i++) {
      for (int j = 0; j < img.width; j++) {
        int pixel_color_sum = 0;
        for (int k = 0; k < img.step / img.width; k++) {
          pixel_color_sum +=
              img.data[i * img.step  + j * img.step / img.width + k];
        }
        if (pixel_color_sum == white_pixel * img.step / img.width) {
          white_pixel_count++;
          white_pixels_sum_x += j;
        }
      }
    }

    if (white_pixel_count > 0) {
      const float white_pixels_avg_x =
          (float)white_pixels_sum_x / white_pixel_count;
      if (white_pixels_avg_x < img.width / 3.0) {
        // Ball is on the left side.
        drive_robot(0, 0.2);
      } else if (white_pixels_avg_x < img.width / 3.0 * 2) {
        // Ball is roughly in the front. Still slowly turn the robot to aim at
        // the ball.
        drive_robot(0.2,
            (img.width / 2.0 - white_pixels_avg_x) / img.width / 2.0 );
      } else {
        // Ball is on the right.
        drive_robot(0, -0.2);
      }
    } else {
      // Can't see the ball, stop.
      drive_robot(0, 0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>(
        "/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside
    // the process_image_callback function
    ros::Subscriber sub1 = n.subscribe(
        "/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
