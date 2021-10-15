#include <pluginlib/class_list_macros.h>
#include "rrt.hpp"

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::RRTGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

// Default Constructor
namespace global_planner
{
RRTGlobalPlanner::RRTGlobalPlanner()
{
}

RRTGlobalPlanner::RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

void RRTGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  ROS_INFO("Initializing RRTGlobalPlanner.");
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  footprint = costmap_ros_->getRobotFootprint();
  robot_radius = getRobotRadius(footprint);

  ros::NodeHandle nh("~/"+name);
  // Retrieve RRT parameters (or set to default values)
  nh.param("goal_tol", goal_tol, 0.05);
  nh.param("K_in", K_in, 4000);
  nh.param("d", d, 0.2);
  nh.param("viz_tree", viz_tree, false);

  plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);

  if (viz_tree)
  {
    tree_pub_ = nh.advertise<visualization_msgs::Marker>("tree", 1);
  }
}

bool RRTGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan)
{
  ROS_INFO("Generating Global Plan with RRT.");
  rrt T_out = generateRRT(start, goal, this->costmap_ros_, this->robot_radius, this->goal_tol, this->K_in, this->d);

  if (viz_tree)
  {
    visualization_msgs::Marker tree_msg;
    init_line(&tree_msg);
    // Publish all edges
    for (auto edge : T_out.edges)
    {
      pub_line(&tree_msg, &tree_pub_, edge.at(0).x, edge.at(0).y, edge.at(1).x, edge.at(1).y);
    }
  }

  if (T_out.success)
  {
    // Get Global path (clears, then sets plan)
    getGlobalPath(&T_out, &plan, start, goal);
    publishPlan(plan);
    return true;
  }
  else
  {
    ROS_INFO("Failed to find Global Plan with RRT.");
    return false;
  }
}

void RRTGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  // create a message for the plan
  nav_msgs::Path rviz_path;
  rviz_path.poses.resize(plan.size());

  if (plan.empty())
  {
    // still set a valid frame so visualization won't hit transform issues
    rviz_path.header.frame_id = "map";
    rviz_path.header.stamp = ros::Time::now();
  }
  else
  {
    rviz_path.header.frame_id = plan[0].header.frame_id;
    rviz_path.header.stamp = plan[0].header.stamp;
  }

  // Extract the plan in world co-ordinates, we assume the plan is all in the same frame
  for (unsigned int i = 0; i < plan.size(); i++)
  {
    rviz_path.poses[i] = plan[i];
  }

  plan_pub_.publish(rviz_path);
}
};  // namespace global_planner