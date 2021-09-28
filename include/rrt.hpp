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
#include <nav_msgs/Path.h>
#include <nav_core/base_global_planner.h>

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
        private:
            costmap_2d::Costmap2DROS* costmap_ros_;
            double goal_tol, d;
            int K_in;
            ros::NodeHandle nh;
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
    //~rrt();

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
    point_new.vertex.z=1.; //Assume z=1 for now

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
rrt generateRRT(geometry_msgs::Point x_init,
                geometry_msgs::Point x_final,
                costmap_2d::Costmap2DROS* costmap_ros,
                double goal_tol,
                int K,
                double d){
    //Initialize RRT with x_init
    rrt T(x_init, costmap_ros);
    //Initialize local variables
    geometry_msgs::Point x_rand;
    tree_node x_near, x_new;

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
        edgeIsFree=edgeInFreeSpace(edge, T.X_space);
        if (edgeIsFree){
            T.add_vertex(x_new);
            T.add_edge(x_near.vertex, x_new.vertex);
        } else {
            k--;
            continue;
        };

        std::cout << "Processed " << k << "/" << K << " RRT vertices. \n";

        if (getDistance(x_new.vertex, x_final)<=goal_tol){
            std::cout << "Successfully found Path with RRT. \n";  
            break;          
        }

    }

    std::cout << "Finished Processing. Returning RRT. \n";

    return T;
}

//Returns Global path from start to goal from RRT
std::vector<geometry_msgs::PoseStamped> getGlobalPath(const rrt* tree){
    std::vector<geometry_msgs::PoseStamped> global_path;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id="map";

    //Add last vertex (closest to goal)
    int current_id=tree->tree_nodes.size()-1;

    //Work our way back to x_initial, building global_path
    while (current_id != 0){
        // Retrieve pose of current ID
        pose_stamped.pose.position=tree->tree_nodes.at(current_id).vertex;
        // Add pose to global path
        global_path.push_back(pose_stamped);
        // Identify next vertex in path (parent node)
        current_id=tree->tree_nodes.at(current_id).parent_id;
    }

    // Add x_initial
    pose_stamped.pose.position=tree->tree_nodes.at(0).vertex;
    global_path.push_back(pose_stamped);

    return global_path;
}