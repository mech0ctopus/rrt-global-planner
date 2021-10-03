#include <pluginlib/class_list_macros.h>
#include "rrt.hpp"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::RRTGlobalPlanner, 
                       nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace global_planner {
  RRTGlobalPlanner::RRTGlobalPlanner (){
}

RRTGlobalPlanner::RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  initialize(name, costmap_ros);
}

void RRTGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    costmap_ros_ = costmap_ros;
    ROS_INFO("Initializing RRTGlobalPlanner.");
    // Retrieve RRT parameters
    nh.getParam("/rrt/goal_tol", goal_tol);
    nh.getParam("/rrt/K_in", K_in); 
    nh.getParam("/rrt/d", d);
    // Retrieve move_base parameters
    nh.getParam("/move_base/global_costmap/global_frame", global_costmap_frame);
    nh.getParam("/move_base/global_costmap/origin_x", global_costmap_origin_x);
    nh.getParam("/move_base/global_costmap/origin_y", global_costmap_origin_y);
    ROS_INFO("Retrieved parameters for RRTGlobalPlanner.");
}

bool RRTGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal,  
                                std::vector<geometry_msgs::PoseStamped>& plan ){

    ROS_INFO("Generating RRT.");
    rrt T_out=generateRRT(start.pose.position, goal.pose.position, 
                          this->costmap_ros_, this->goal_tol, 
                          this->K_in, this->d);
    ROS_INFO("Setting Global Path from RRT.");
    // Get Global path
    plan=getGlobalPath(&T_out);

    return true;
  }
};