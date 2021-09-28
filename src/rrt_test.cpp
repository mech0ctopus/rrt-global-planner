#include <vector>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/buffer.h>

#include "2d_space.hpp"
#include "rrt.hpp"
#include "visualization.hpp"

int main(int argc, char **argv){
    // Initialize random seed
    srand(1221);

    // Initialize Node
    ros::init(argc, argv, "rrt_visualizer");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.5); //Hz

    // Create RRT publishers
    ros::Publisher line_pub=nh.advertise<visualization_msgs::Marker>("line", 0);
    //ros::Publisher path_pub=nh.advertise<nav_msgs::Path>("global_path", 0);
    ros::Publisher start_marker_pub=nh.advertise<visualization_msgs::Marker>("start_marker", 0);
    ros::Publisher goal_marker_pub=nh.advertise<visualization_msgs::Marker>("goal_marker", 0);

    // Initialize visualizations
    visualization_msgs::Marker line_msg;
    visualization_msgs::Marker start_marker_msg, goal_marker_msg;
    init_line(&line_msg);
    init_marker(&start_marker_msg);
    init_marker(&goal_marker_msg);

    // Retrieve RRT parameters
    double x0_x, x0_y, xf_x, xf_y;
    double goal_tol, d;
    int K_in;
    nh.getParam("rrt_node/x_initial", x0_x);
    nh.getParam("rrt_node/y_initial", x0_y);
    nh.getParam("rrt_node/x_goal", xf_x);
    nh.getParam("rrt_node/y_goal", xf_y);

    // Populate initial and final/goal states
    geometry_msgs::Point x0{}, xf{};
    x0.x=x0_x;
    x0.y=x0_y;
    x0.z=1.;
    xf.x=xf_x;
    xf.y=xf_y;
    xf.z=1.;

    nh.getParam("rrt_node/goal_tol", goal_tol);
    nh.getParam("rrt_node/K_in", K_in); 
    nh.getParam("rrt_node/d", d);

    // Generate RRT
    tf2_ros::Buffer tfBuffer;
    costmap_2d::Costmap2DROS costmap_ros("global_costmap", tfBuffer);

    rrt T_out=generateRRT(x0, xf, &costmap_ros, goal_tol, K_in, d);
    // Get Global path
    std::vector<geometry_msgs::PoseStamped> global_path=getGlobalPath(&T_out);

    // Publish for visualization
    while (ros::ok()){
        // Publish start/end points
        pub_marker(&start_marker_msg, &start_marker_pub,
                   x0.x, x0.y);
        pub_marker(&goal_marker_msg, &goal_marker_pub,
                   xf.x, xf.y);

        // Publish all edges (after RRT is built)
        for (auto edge: T_out.edges){
            pub_line(&line_msg, &line_pub,
                     edge.at(0).x, edge.at(0).y,
                     edge.at(1).x, edge.at(1).y);
        }
       
        // Publish global path
        // global_path.header.stamp=ros::Time::now();
        // path_pub.publish(global_path);

        // ROS_INFO("Published updated RRT vertices.");

        // Sleep if necessary
        ros::spinOnce;
        loop_rate.sleep();
    }

    return 0;
}