#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "math_funcs.hpp"

/**
 * Publishes a randomly occupied 2D Occupancy Grid.
 */
int main(int argc, char** argv)
{
  // Initialize Node
  ros::init(argc, argv, "occ_grid_generator");
  ros::NodeHandle nh;

  // Create occupancy grid publisher
  ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("occ_grid", 0);
  ros::Rate loop_rate(0.5);  // Hz
  // Initialize grid_msg
  nav_msgs::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = "map";
  grid_msg.info.map_load_time = ros::Time::now();
  // Origin of the map [m, m, rad]. Real-world pose of the cell (0,0) in the map.
  grid_msg.info.origin.orientation.w = 1.0;  // Assume 0,0;

  // Retrieve parameters
  double res, occupied;
  int width, height;
  nh.getParam("occ_grid_generator_node/resolution", res);  // meters/cell (float32)
  nh.getParam("occ_grid_generator_node/width", width);     // cells (uint32)
  nh.getParam("occ_grid_generator_node/height", height);   // cells (uint32)
  nh.getParam("occ_grid_generator_node/percent_occupied", occupied);
  grid_msg.info.resolution = res;
  grid_msg.info.width = width;
  grid_msg.info.height = height;

  // Insert actual map data:
  // The map data, in row-major order (cycle through each row),
  // starting with (0,0). Occupancy probabilities are in the range [0,100].  Unknown is -1.
  for (unsigned int ii = 0; ii < (grid_msg.info.width * grid_msg.info.height); ii++)
  {
    // Randomly pick number 0-1, if <= occupied, mark cell as occupied
    if (randomDouble(0., 1.) <= occupied)
    {
      grid_msg.data.push_back(100);
    }
    else
    {
      grid_msg.data.push_back(0);
    }
  }

  while (ros::ok())
  {
    // Update grid_msg
    grid_msg.header.stamp = ros::Time::now();
    // Publish grid_msg
    grid_pub.publish(grid_msg);
    // ROS_INFO("Published updated OccupancyGrid.");

    // Sleep if necessary
    ros::spinOnce;
    loop_rate.sleep();
  }

  return 0;
}