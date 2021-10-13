#pragma once

#include <math.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>

#include "math_funcs.hpp"

#define TWO_M_PI 2 * M_PI
#define M_PI_10 M_PI / 10.

struct idx
{
  unsigned int x, y;
};

/**
 * Calculates Euclidean distance between two 2D points.
 *
 * @param[out] distance L2 Norm.
 * @param[in] point1 First point.
 * @param[in] point2 Second point.
 */
double getDistance(const geometry_msgs::Point point1, const geometry_msgs::Point point2)
{
  double distance = sqrt(pow(point2.y - point1.y, 2) + pow(point2.x - point1.x, 2));
  return distance;
}

/**
 * Determines if Point is in free spaced X_space.
 * Assumes input point is the /map frame.
 *
 * @param[out] whether point is in free space.
 */
bool inFreeSpace(const geometry_msgs::Point point, const costmap_2d::Costmap2DROS* costmap_ros)
{
  bool result{ 0 };
  unsigned char threshold{ 252 };
  unsigned int mx, my;

  // Set mx, my (map coordinates) from world
  costmap_ros->getCostmap()->worldToMap(point.x, point.y, mx, my);

  if (costmap_ros->getCostmap()->getCost(mx, my) <= threshold)
  {
    result = 1;
  }

  return result;
}

// Checks if robot path along edge is in free space.
// Checks at robot centerpoint and around circumscribed radius.
bool edgeInFreeSpace(const std::vector<geometry_msgs::Point> edge, const costmap_2d::Costmap2DROS* costmap_ros,
                     const double robot_radius)
{
  bool result{ 1 };

  // Discretize edge into an array of points
  double dist = getDistance(edge[0], edge[1]);
  // Get num of points. radius acts as resolution.
  double num_points = dist / robot_radius;
  std::vector<geometry_msgs::Point> edge_points;
  geometry_msgs::Point edge_pt_ii{};
  for (double ii = 0.; ii <= num_points; ii++)
  {
    edge_pt_ii.x = edge[0].x + ii * (edge[1].x - edge[0].x) / num_points;
    edge_pt_ii.y = edge[0].y + ii * (edge[1].y - edge[0].y) / num_points;
    edge_points.push_back(edge_pt_ii);
  }
  // Check each point in array using inFreeSpace
  bool edgePointIsFree{ 0 }, circPointIsFree{ 0 };
  geometry_msgs::Point circ_point{};  // Point on robot's circumscribed radius
  circ_point.z = 0;
  for (auto edge_point : edge_points)
  {
    // Check centerpoint of robot
    edgePointIsFree = inFreeSpace(edge_point, costmap_ros);
    if (edgePointIsFree)
    {
      // pass
    }
    else
    {
      result = 0;
      break;
    }
    // Check points around circumscribed radius
    double theta{ 0 };
    while (theta <= TWO_M_PI)
    {
      circ_point.x = edge_point.x + robot_radius * cos(theta);
      circ_point.y = edge_point.y + robot_radius * sin(theta);
      circPointIsFree = inFreeSpace(circ_point, costmap_ros);
      if (circPointIsFree)
      {
        // pass
      }
      else
      {
        result = 0;
        break;
      }
      theta += M_PI_10;
    }
  }
  return result;
}

/**
 * Randomly picks state from X_space.
 *
 * @param[out] state Random state from space.
 */
geometry_msgs::Point getRandomState(costmap_2d::Costmap2DROS* costmap_ros)
{
  // Choose random number from 0 to max. [inclusive]
  geometry_msgs::Point randomState{};
  randomState.z = 0.;  // Assume z=0 for now.
  // Keep picking points until you find one in free space
  bool pointIsFree{ 0 };

  double max_w = costmap_ros->getCostmap()->getSizeInCellsX() * costmap_ros->getCostmap()->getResolution();
  double max_h = costmap_ros->getCostmap()->getSizeInCellsY() * costmap_ros->getCostmap()->getResolution();
  while (!pointIsFree)
  {
    randomState.x = randomDouble(-max_w / 2, max_w / 2);
    randomState.y = randomDouble(-max_h / 2, max_h / 2);
    pointIsFree = inFreeSpace(randomState, costmap_ros);
  }

  return randomState;
}