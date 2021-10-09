#include <algorithm>
#include <angles/angles.h>
#include <math.h>
#include <vector>
#include <stdlib.h>

#include <ros/ros.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include "2d_space.hpp"
#include "math_funcs.hpp"

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace global_planner {
    class RRTGlobalPlanner : public nav_core::BaseGlobalPlanner {
        public:
            RRTGlobalPlanner();
            RRTGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            bool makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan
                          );
            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            ~RRTGlobalPlanner(){}
        private:
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D* costmap_;
            double goal_tol, d;
            int K_in;
            std::string global_costmap_frame;
            std::string global_costmap_origin_x, global_costmap_origin_y;
            ros::Publisher plan_pub_;
    };  
};

#endif

/**
 * Performs basic Rapidly-exploring Random Tree (RRT)
 * algorithm per original paper.
 * 
 * http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
 * https://comrob.fel.cvut.cz/papers/romoco09rrt.pdf (helpful)
 */

struct tree_node
{
    int parent_id{};
    geometry_msgs::Point vertex{};
};

/**
 * RRT Data Structure.
 *
 * @param[in] x_init Initial State.
 */
class rrt
{
public:
    geometry_msgs::Point x_initial{};
    // Vertices and Edges in RRT
    std::vector<tree_node> tree_nodes{};
    std::vector<std::vector<geometry_msgs::Point>> edges{};
    // Free state space
    costmap_2d::Costmap2DROS* X_space;
    // Whether tree reached goal
    bool success{0};

    //Constructor for RRT
    rrt(geometry_msgs::Point x_init, costmap_2d::Costmap2DROS* costmap_ros){
        x_initial=x_init;

        //Populate X_space
        this->X_space=costmap_ros;

        //Add initial point to vertices
        tree_node initial_node;
        initial_node.parent_id=0;
        initial_node.vertex=x_init;
        add_vertex(initial_node);
    }

    //Adds node to node vector
    void add_vertex(const tree_node new_node){
        this->tree_nodes.push_back(new_node);
    }

    //Adds edge to list of edges
    void add_edge(geometry_msgs::Point point1, geometry_msgs::Point point2){
        std::vector<geometry_msgs::Point> edge{};
        edge.push_back(point1);
        edge.push_back(point2);
        this->edges.push_back(edge);
    }

    ~rrt(){};
};

/**
 * Finds closest pose to point1 in X_space.
 
 * @param[in] point1 Point in X_space.
 * @param[in] T Existing RRT.
 * @param[out] nearest_neighbor Closest pose to pose1.
 */
tree_node getNearestNeighbor(const geometry_msgs::Point point1, 
                             const rrt* T){
    geometry_msgs::Point nearest_neighbor{};
    tree_node nearest_neighbor_node{};
    int parent_id{};
    double nearest_distance{HUGE_VAL};
    double current_distance{HUGE_VAL};

    //For each vertex (a tree_node)
    for (int ii=0; ii<T->tree_nodes.size(); ii++){
        //Make sure it's not the same point
        if (point1.x!=T->tree_nodes.at(ii).vertex.x && point1.y!=T->tree_nodes.at(ii).vertex.y){
            //Get the closest existing vertex
            current_distance=getDistance(point1, T->tree_nodes.at(ii).vertex);
            if (current_distance<nearest_distance){
                nearest_distance=current_distance;
                nearest_neighbor=T->tree_nodes.at(ii).vertex;
                parent_id=ii;
            }  
        }
    }

    nearest_neighbor_node.vertex=nearest_neighbor;
    nearest_neighbor_node.vertex.z=0.; //Assume planar for now
    nearest_neighbor_node.parent_id=parent_id;

    return nearest_neighbor_node;
}

/**
 * Extend pose_near towards point_rand.
 */
tree_node extendTree(const tree_node point_near, 
                     const geometry_msgs::Point point_rand,
                     const double d){
    tree_node point_new{};
    point_new.vertex.z=0.; //Assume z=0 for now

    double theta=atan2(point_rand.y-point_near.vertex.y,
                       point_rand.x-point_near.vertex.x);
    point_new.vertex.x=point_near.vertex.x+d*cos(theta);
    point_new.vertex.y=point_near.vertex.y+d*sin(theta);

    point_new.parent_id=point_near.parent_id;

    return point_new;
}

/**
 * Generates RRT.
 *
 * @param[out] T RRT.
 * @param[in] x_init Initial State within X_space.
 * @param[in] x_final Final State within X_space.
 * @param[in] goal_tol Cartesian Goal Tolerance.
 * @param[in] K Number of vertices.
 * @param[in] d Distance to extend tree per step.
 */
rrt generateRRT(geometry_msgs::PoseStamped x_init, //In map frame
                geometry_msgs::PoseStamped x_final, //In map frame
                costmap_2d::Costmap2DROS* costmap_ros,
                double goal_tol,
                int K,
                double d){
    //Initialize RRT with x_init
    rrt T(x_init.pose.position, costmap_ros);
    //Initialize local variables
    geometry_msgs::Point x_rand; //in map frame
    tree_node x_near, x_new; //in map frame

    // Build Tree
    for (int k = 1; k <= K; k++)
    {
        bool edgeIsFree{0};
        std::vector<geometry_msgs::Point> edge{};

        //Get random configuration
        x_rand=getRandomState(T.X_space);
        //Get nearest existing neighbor to random pose
        x_near=getNearestNeighbor(x_rand, &T);
        //Extend x_near toward x_rand
        x_new=extendTree(x_near, x_rand, d);

        // Check if x_new and x_near can connect
        edge.push_back(x_new.vertex);
        edge.push_back(x_near.vertex);
        
        // Check if edge is in free space
        edgeIsFree=edgeInFreeSpace(edge, T.X_space);
        if (edgeIsFree){
            T.add_vertex(x_new);
            T.add_edge(x_near.vertex, x_new.vertex);
        } else {
            continue;
        };

        ROS_INFO("Processed %i/%i RRT vertices.", k, K);

        if (getDistance(x_new.vertex, x_final.pose.position)<=goal_tol){
            ROS_INFO("Found solution with %i/%i RRT vertices.", k, K); 
            T.success=1;
            break;          
        }

    }

    return T;
}

//Returns Global path from start to goal from RRT
bool getGlobalPath(const rrt* tree,
                   std::vector<geometry_msgs::PoseStamped>* plan,
                   const geometry_msgs::PoseStamped& start, 
                   const geometry_msgs::PoseStamped& goal){
    geometry_msgs::PoseStamped pose_stamped; 
    pose_stamped.header.frame_id="map";
    int prev_id;
    tf2::Quaternion quat_tf;
    geometry_msgs::Quaternion quat_msg;

    plan->clear();

    //Add last vertex (closest to goal)
    int current_id=tree->tree_nodes.size()-1;
    //Set goal orientation for last vertex
    pose_stamped.pose.orientation=goal.pose.orientation;

    //Work our way back to x_initial, building plan
    while (current_id != 0){
        // Retrieve pose of current ID
        pose_stamped.pose.position=tree->tree_nodes.at(current_id).vertex;
        // Add pose to plan
        plan->push_back(pose_stamped);
        // Identify next vertex in path (parent node), store previous ID
        prev_id=current_id;
        current_id=tree->tree_nodes.at(current_id).parent_id;

        // Set orientation for next iteration
        double dy, dx, yaw;
        dy=tree->tree_nodes.at(prev_id).vertex.y-
           tree->tree_nodes.at(current_id).vertex.y;
        dx=tree->tree_nodes.at(prev_id).vertex.x-
           tree->tree_nodes.at(current_id).vertex.x;
        // Get yaw from atan2 using current point and prev. point.
        yaw=atan2(dy,dx);
        // Convert RPY to quat
        quat_tf.setRPY(0, 0, yaw);
        // Convert Quat TF to msg
        quat_msg=tf2::toMsg(quat_tf);
        // set orientation.
        pose_stamped.pose.orientation=quat_msg;
    }

    // Add x_initial
    pose_stamped.pose.position=tree->tree_nodes.at(0).vertex;
    pose_stamped.pose.orientation=start.pose.orientation;
    plan->push_back(pose_stamped);

    // Reverse so that x_initial is first and goal is last.
    std::reverse(plan->begin(), plan->end());

    return true;
}