#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

/**
 *  @brief Initializes a Marker msg for a LINE_LIST.
 *
 *  @details
 *   Sets several marker msg fields once.
 *
 *  @param line_msg Pointer to existing marker object.
 *
 */
void init_line(visualization_msgs::Marker* line_msg)
{
  line_msg->header.frame_id = "map";
  line_msg->id = 0;
  line_msg->ns = "tree";
  line_msg->type = visualization_msgs::Marker::LINE_LIST;
  line_msg->action = visualization_msgs::Marker::ADD;
  line_msg->pose.orientation.w = 1.0;
  line_msg->scale.x = 0.05;  // in meters (width of segments)
}

/**
 *  @brief Publishes a Marker msg with two points.
 *
 *  @details
 *   Publishes a visualization_msgs/Marker to view in RViz.
 *
 *  @param line_msg Pointer to existing marker object.
 *  @param line_pub Pointer to existing marker Publisher.
 *  @param x1 x-position of first marker.
 *  @param y1 y-position of first marker.
 *  @param x2 x-position of second marker.
 *  @param y2 y-position of second marker.
 *
 */
void pub_line(visualization_msgs::Marker* line_msg, ros::Publisher* line_pub, double x1, double y1, double x2,
              double y2)
{
  // Update line_msg header
  line_msg->header.stamp = ros::Time::now();

  // Build msg
  geometry_msgs::Point p1, p2;
  std_msgs::ColorRGBA c1, c2;

  p1.x = x1;
  p1.y = y1;
  p1.z = 1.0;

  p2.x = x2;
  p2.y = y2;
  p2.z = 1.0;

  c1.r = 0.0;  // 1.0=255
  c1.g = 1.0;
  c1.b = 0.0;
  c1.a = 0.5;  // alpha

  c2.r = 0.0;  // 1.0=255
  c2.g = 1.0;
  c2.b = 0.0;
  c2.a = 0.5;  // alpha

  line_msg->points.push_back(p1);
  line_msg->points.push_back(p2);

  line_msg->colors.push_back(c1);
  line_msg->colors.push_back(c2);

  // Publish line_msg
  line_pub->publish(*line_msg);
}