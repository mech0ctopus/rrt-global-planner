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

    ros::NodeHandle nh;

    ROS_INFO("Initializing RRTGlobalPlanner.");
    // Retrieve RRT parameters (or set to default values)
    nh.param("/rrt/goal_tol", goal_tol, 0.05);
    nh.param("/rrt/K_in", K_in, 4000); 
    nh.param("/rrt/d", d, 0.2);
    nh.param("/rrt/viz_tree", viz_tree, false);

    plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/RRTGlobalPlanner/global_plan", 1);
    
    if (viz_tree){
      tree_pub_ = nh.advertise<visualization_msgs::Marker>("/move_base/RRTGlobalPlanner/tree", 1);
    }
}

bool RRTGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
                                const geometry_msgs::PoseStamped& goal,  
                                std::vector<geometry_msgs::PoseStamped>& plan ){
    ROS_INFO("Generating RRT.");
    rrt T_out=generateRRT(start, goal, 
                          this->costmap_ros_, this->goal_tol, 
                          this->K_in, this->d);

    if (viz_tree){
        //Clear previous tree markers and reset
        visualization_msgs::Marker tree_msg;
        init_line(&tree_msg);
        // Publish all edges
        for (auto edge: T_out.edges){
            pub_line(&tree_msg, &tree_pub_,
                     edge.at(0).x, edge.at(0).y,
                     edge.at(1).x, edge.at(1).y);
        }
    }

    if (T_out.success){
      // Get Global path (clears, then sets plan)
      getGlobalPath(&T_out, &plan, start, goal);
      publishPlan(plan);
      return true;
    } else {
      ROS_INFO("Failed to find Global Path from RRT.");
      return false;
    }
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