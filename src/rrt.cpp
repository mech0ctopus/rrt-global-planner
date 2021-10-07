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
    costmap_ = costmap_ros_->getCostmap();
    // footprint_ = costmap_ros_->getRobotFootprint();

    ros::NodeHandle nh;
    plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/DWAPlannerROS/global_plan", 1);
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
    rrt T_out=generateRRT(start, goal, 
                          this->costmap_ros_, this->goal_tol, 
                          this->K_in, this->d);

    // Get Global path (clears, then sets plan)
    getGlobalPath(&T_out, &plan, start, goal);
    ROS_INFO("Found Global Path from RRT");

    publishPlan(plan);

    return true;
  }

  void RRTGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
    //create a message for the plan 
    nav_msgs::Path rviz_path;
    rviz_path.poses.resize(plan.size());
    
    if(plan.empty()) {
      //still set a valid frame so visualization won't hit transform issues
      rviz_path.header.frame_id = "map";
      rviz_path.header.stamp = ros::Time::now();
    } else { 
      rviz_path.header.frame_id = plan[0].header.frame_id;
      rviz_path.header.stamp = plan[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the plan is all in the same frame
    for(unsigned int i=0; i < plan.size(); i++){
      rviz_path.poses[i] = plan[i];
    }

    plan_pub_.publish(rviz_path);
  }
};