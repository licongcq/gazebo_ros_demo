#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// Used to publish processed image with crosshair
ros::Publisher processed_image_publisher;

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

std::tuple<int, int> clip_to_bound(int x, int y, int width, int height) {
  return std::make_tuple(
      std::min(std::max(x, 0), width - 1),
      std::min(std::max(y, 0), height - 1));
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    constexpr int white_pixel = 255;

    if (img.height <= 0 || img.width <= 0 || img.step <= 0) {
      return;
    }

    // We'll calculate the average X coordinates of all white pixels to get an
    // estimation of the center of the ball.
    int white_pixel_count = 0;
    int white_pixels_sum_x = 0;
    int white_pixels_sum_y = 0;

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
          white_pixels_sum_y += i;
          white_pixels_sum_x += j;
        }
      }
    }

    // Make a copy of raw image input so we can start drawing the HUD
    sensor_msgs::Image processed_image = img;
    // The data field in img is readonly. So we make a mutable copy of it
    auto processed_data = img.data;
    constexpr int hud_color_red = 200;

    if (white_pixel_count > 0) {
      const float white_pixels_avg_x =
          (float)white_pixels_sum_x / white_pixel_count;
      const float white_pixels_avg_y =
          (float)white_pixels_sum_y / white_pixel_count;

      // Draw a 3-pixels-wide cross
      const int center_x = (int)round(white_pixels_avg_x);
      const int center_y = (int)round(white_pixels_avg_y);

      for (int i = 0; i < img.height; i++) {
        for (int offset = -1; offset <= 1; offset++) {
          int safe_i, safe_j;
          std::tie(safe_j, safe_i) =
              clip_to_bound(center_x + offset, i, img.width, img.height);
          // This is sort of a hack because the pixel format on data might be
          // different. Works with Gazebo's camera plugin though.
          processed_data[safe_i * img.step  + safe_j * img.step / img.width] =
              hud_color_red;
        }
      }

      for (int j = 0; j < img.width; j++) {
        for (int offset = -1; offset <= 1; offset++) {
          int safe_i, safe_j;
          std::tie(safe_j, safe_i) =
              clip_to_bound(j, center_y + offset, img.width, img.height);
          processed_data[safe_i * img.step  + safe_j * img.step / img.width] =
              hud_color_red;
        }
      }

      // Drive the robot
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

    processed_image.data = processed_data;
    processed_image_publisher.publish(processed_image);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>(
        "/ball_chaser/command_robot");

    // Subscribe to the raw image topic
    ros::Subscriber sub1 = n.subscribe(
        "/camera/rgb/image_raw", 10, process_image_callback);

    // Will be used to publish processed image with crosshair
    processed_image_publisher =
        n.advertise<sensor_msgs::Image>("/camera/rgb/image_processed", 10);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
