#pragma once

#include <math.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>

#include "math_funcs.hpp"

struct idx{
    unsigned int x, y;
};

/**
 * Calculates Euclidean distance between two 2D points.
 *
 * @param[out] distance L2 Norm.
 * @param[in] point1 First point.
 * @param[in] point2 Second point.
 */
double getDistance(const geometry_msgs::Point point1, 
                   const geometry_msgs::Point point2){
    double distance=sqrt(pow(point2.y-point1.y,2)+
                         pow(point2.x-point1.x,2));
    return distance;
}

/**
 * Determines if Point is in free spaced X_space.
 *
 * Return costmap index corresponding with input XY pair (from Point).
 * 
 * */
idx get_idx(const geometry_msgs::Point point, 
            const costmap_2d::Costmap2DROS* costmap_ros){
    //Assumes Map is at (0,0)
    ROS_INFO("Point.x: %f m.",point.x);
    ROS_INFO("Point.y: %f m.",point.y);

   ROS_INFO("max width (cells): %i.", costmap_ros->getCostmap()->getSizeInCellsX());  //384
    ROS_INFO("max height (cells): %i.",costmap_ros->getCostmap()->getSizeInCellsY()); //384

    //TODO: Convert input point from /map from to costmap_frame. Maybe this should be in rrt.hpp

    //Identify which row/col indices corresponding with point
    idx idx_out{};
    const double res{costmap_ros->getCostmap()->getResolution()}; //0.05
    idx_out.x=(unsigned int)(costmap_ros->getCostmap()->getSizeInCellsX()/2)+
              (unsigned int)floor(point.x/res);
    idx_out.y=(unsigned int)(costmap_ros->getCostmap()->getSizeInCellsY()/2)+
              (unsigned int)floor(point.y/res);

    // idx_out.x=floor((unsigned int)(((point.x)/max_w)*costmap_ros->getCostmap()->getSizeInCellsX()));
    // idx_out.y=floor((unsigned int)(((point.y)/max_h)*costmap_ros->getCostmap()->getSizeInCellsY()));

    return idx_out;
}

/**
 * Determines if Point is in free spaced X_space.
 * Assumes input point is the /map frame.
 *
 * @param[out] whether point is in free space.
 */
bool inFreeSpace(const geometry_msgs::Point point, 
                 const costmap_2d::Costmap2DROS* costmap_ros){
    bool result{0};
    unsigned char threshold{127};
    //Get grid indices of Point x/y values
    idx idx_out=get_idx(point, costmap_ros);

    ROS_INFO("Grid XY idx: %i, %i.",idx_out.x,idx_out.y);
    //Check if point in costmap_ros is unoccupied
    ROS_INFO("Cost at idx: %u.",costmap_ros->getCostmap()->getCost(idx_out.x,idx_out.y));

    //getCost returns unsigned char
    if (costmap_ros->getCostmap()->getCost(idx_out.x,idx_out.y)<=threshold) {
        ROS_INFO("The point is free!.");
        result=1;
    }
    ROS_INFO("Returning inFreeSpace.");

    return result;
}

bool edgeInFreeSpace(const std::vector<geometry_msgs::Point> edge, 
                     const costmap_2d::Costmap2DROS* costmap_ros){
    bool result{1};
    double edge_res{0.01}; //Resolution of discrete points on edge

    //Discretize edge into an array of points
    double dist=getDistance(edge[0],
                            edge[1]);
    //Get num of points
    double num_points=dist/edge_res;
    std::vector<geometry_msgs::Point> edge_points;
    geometry_msgs::Point edge_pt_ii{};
    for (double ii = 0.; ii <= num_points; ii++){
        edge_pt_ii.x=edge[0].x+ii*(edge[1].x-edge[0].x)/num_points;
        edge_pt_ii.y=edge[0].y+ii*(edge[1].y-edge[0].y)/num_points;
        edge_points.push_back(edge_pt_ii);
    }
    //Check each point in array using inFreeSpace
    bool edgePointIsFree{0};
    for (auto edge_point:edge_points){
        ROS_INFO("edge_point.x:%f m.",edge_point.x);
        ROS_INFO("edge_point.y:%f m.",edge_point.y);
        edgePointIsFree=inFreeSpace(edge_point, costmap_ros);
        if (edgePointIsFree){
            continue; //skip to next iteration
        } else {
            result=0;

            break;
        }
    }
    ROS_INFO("Returning edgeInFreeSpace.");
    return result;
}

/**
 * Randomly picks state from X_space.
 *
 * @param[out] state Random state from space.
 */
geometry_msgs::Point getRandomState(costmap_2d::Costmap2DROS* costmap_ros){
    //Choose random number from 0 to max. [inclusive]
    geometry_msgs::Point randomState{};
    randomState.z=1.; //Assume z=1 for now.
    //Keep picking points until you find one in free space
    bool pointIsFree{0};

    double max_w=costmap_ros->getCostmap()->getSizeInCellsX()*
                 costmap_ros->getCostmap()->getResolution();
    double max_h=costmap_ros->getCostmap()->getSizeInCellsY()*
                 costmap_ros->getCostmap()->getResolution();
    while (!pointIsFree){
        randomState.x=randomDouble(-max_w/2,max_w/2);
        randomState.y=randomDouble(-max_h/2,max_h/2);
       ROS_INFO("Checking new random state.");
        pointIsFree=inFreeSpace(randomState, costmap_ros);
    }

    return randomState;
}