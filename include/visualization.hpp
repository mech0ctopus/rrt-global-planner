#pragma once

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

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

void init_marker(visualization_msgs::Marker* marker_msg)
{
  marker_msg->header.frame_id = "map";
  marker_msg->id = 0;
  marker_msg->type = visualization_msgs::Marker::SPHERE;
  marker_msg->action = visualization_msgs::Marker::ADD;
  // Build msg
  marker_msg->pose.orientation.w = 1.0;
  marker_msg->scale.x = 0.4;  // in meters
  marker_msg->scale.y = 0.4;  // in meters
  marker_msg->scale.z = 0.4;  // in meters
  marker_msg->color.r = 1.0;  // 1.0=255
  marker_msg->color.g = 0.0;
  marker_msg->color.b = 0.0;
  marker_msg->color.a = 1.0;  // alpha
}

void pub_marker(visualization_msgs::Marker* marker_msg, ros::Publisher* marker_pub, double x1, double y1)
{
  // Update marker_msg header
  marker_msg->header.stamp = ros::Time::now();
  marker_msg->pose.position.x = x1;
  marker_msg->pose.position.y = y1;
  marker_msg->pose.position.z = 1.0;
  // Publish marker_msg
  marker_pub->publish(*marker_msg);
}
