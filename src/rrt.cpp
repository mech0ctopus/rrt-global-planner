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

    // Retrieve RRT parameters
    nh.getParam("rrt_node/goal_tol", goal_tol);
    nh.getParam("rrt_node/K_in", K_in); 
    nh.getParam("rrt_node/d", d);
}

bool RRTGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal,  
                                std::vector<geometry_msgs::PoseStamped>& plan ){

    rrt T_out=generateRRT(start.pose.position, goal.pose.position, 
                          this->costmap_ros_, this->goal_tol, 
                          this->K_in, this->d);
    // Get Global path
    plan=getGlobalPath(&T_out);

    return true;
  }
};