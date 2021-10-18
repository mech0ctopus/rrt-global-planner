#pragma once

#include <math.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>

#include "math_funcs.hpp"

#define TWO_M_PI 2 * M_PI
#define M_PI_10 M_PI / 10.

/**
 *  @brief Calculates Euclidean distance between two 2D points.
 *
 *  @details
 *   Calculates Euclidean distance between two 2D points.
 *
 *  @param distance L2 Norm.
 *  @param point1 First point.
 *  @param point2 Second point.
 *  @return L2 Norm.
 *
 */
double getDistance(const geometry_msgs::Point point1, const geometry_msgs::Point point2)
{
  double distance = sqrt(pow(point2.y - point1.y, 2) + pow(point2.x - point1.x, 2));
  return distance;
}

/**
 *  @brief Checks whether a point is free on global 2D costmap.
 *
 *  @details
 *   Checks point in global 2D costmap to confirm that the robot
 *   can be placed in any orientation and will only occupy free space.
 *
 *  @param point Two points representing a line.
 *  @param costmap_ros  Pointer to ROS wrapper for global 2D costmap.
 *  @param robot_radius_max Circumscribed and padded robot radius.
 *  @return                 Whether point is in free space.
 *
 */
bool inFreeSpace(const geometry_msgs::Point point, const costmap_2d::Costmap2DROS* costmap_ros,
                 const double robot_radius_max)
{
  bool result{ 1 };
  double theta{ 0 };
  double robot_radius_ii{ robot_radius_max };
  double robot_radius_step(0.05);  // Assume 5cm step.
  costmap_2d::Costmap2D* costmap_;
  geometry_msgs::Point point_to_check;
  unsigned int mx, my;
  std::vector<costmap_2d::MapLocation> map_polygon, polygon_cells;

  costmap_ = costmap_ros->getCostmap();

  // Build inflated/circumscribed robot footprint
  while (theta <= TWO_M_PI)
  {
    costmap_2d::MapLocation map_loc;

    // Try to convert footprint to map coordinates
    if (!costmap_->worldToMap(point.x + robot_radius_max * cos(theta), point.y + robot_radius_max * sin(theta),
                              map_loc.x, map_loc.y))
    {
      // ROS_INFO("Footprint point is outside of map bounds.");
      return false;
    }

    map_polygon.push_back(map_loc);

    theta += M_PI_10;
  }

  // Get the all map cells within inflated/circumscribed robot footprint
  costmap_->convexFillCells(map_polygon, polygon_cells);

  // For each cell in polygon_cells, check the cost against the threshold.
  for (unsigned int i = 0; i < polygon_cells.size(); ++i)
  {
    if (costmap_->getCost(polygon_cells[i].x, polygon_cells[i].y) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      result = 0;
      break;
    }
  }

  return result;
}

/**
 *  @brief Checks whether an edge is free on global 2D costmap.
 *
 *  @details
 *   Discretizes edge, then checks several points along edge on global 2D costmap
 *   to confirm that the robot can be placed in any orientation and
 *   will only occupy free space.
 *
 *  @param edge Two points representing a line.
 *  @param costmap_ros  Pointer to ROS wrapper for global 2D costmap.
 *  @param robot_radius Circumscribed and padded robot radius.
 *  @return             Whether edge is in free space.
 *
 */
bool edgeInFreeSpace(const std::vector<geometry_msgs::Point> edge, const costmap_2d::Costmap2DROS* costmap_ros,
                     const double robot_radius)
{
  bool result{ 1 };

  // Discretize edge into an array of points
  double dist = getDistance(edge[0], edge[1]);
  // Get num of points. radius acts as resolution.
  double num_points = dist / robot_radius;
  geometry_msgs::Point edge_pt_ii{};
  for (double ii = 0.; ii <= num_points; ii++)
  {
    edge_pt_ii.x = edge[0].x + ii * (edge[1].x - edge[0].x) / num_points;
    edge_pt_ii.y = edge[0].y + ii * (edge[1].y - edge[0].y) / num_points;

    if (!inFreeSpace(edge_pt_ii, costmap_ros, robot_radius))
    {
      result = 0;
      break;
    }
  }

  return result;
}

/**
 *  @brief Picks a random state on global 2D costmap.
 *
 *  @details
 *   Picks a random point on global 2D costmap at which the robot can be placed
 *   in any orientation and will only occupy free space.
 *
 *  @param costmap_ros  Pointer to ROS wrapper for global 2D costmap.
 *  @param robot_radius Circumscribed and padded robot radius.
 *  @return             Random point on global costmap where robot is in free space.
 *
 */
geometry_msgs::Point getRandomState(costmap_2d::Costmap2DROS* costmap_ros, const double robot_radius)
{
  geometry_msgs::Point randomState{};
  randomState.z = 0.;  // Assume z=0 for now.
  costmap_2d::Costmap2D* costmap_;
  costmap_ = costmap_ros->getCostmap();

  // Keep picking points until you find one in free space
  bool pointIsFree{ 0 };

  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();

  while (!pointIsFree)
  {
    randomState.x = randomDouble(origin_x, origin_x + costmap_->getSizeInMetersX());
    randomState.y = randomDouble(origin_y, origin_y + costmap_->getSizeInMetersY());
    pointIsFree = inFreeSpace(randomState, costmap_ros, robot_radius);
  }
  return randomState;
}