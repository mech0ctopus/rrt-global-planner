#pragma once

#include <math.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>

#include "math_funcs.hpp"

struct idx{
    int x, y;
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
 * Return occupancy grid index corresponding with input XY pair (from Point).
 * 
 * info.width and info.height are in "cells"
 * */
idx get_idx(const geometry_msgs::Point point, 
            const costmap_2d::Costmap2DROS* costmap_ros){
    //Assumes Map is at (0,0)
    // cout << "Point.x: " << point.x << " m. \n";
    // cout << "Point.y: " << point.y << " m. \n";

    double max_w=costmap_ros->getCostmap()->getSizeInCellsX()*
                 costmap_ros->getCostmap()->getResolution();
    double max_h=costmap_ros->getCostmap()->getSizeInCellsY()*
                 costmap_ros->getCostmap()->getResolution();

    //Get row and col IDs
    double row_id=point.x/max_w;
    double col_id=point.y/max_h;

    // cout << "Row_id: " << (int)row_id << " . \n";
    // cout << "Col_id: " << (int)col_id << " . \n";

    idx idx_out{};
    idx_out.x=floor((int)(row_id*costmap_ros->getCostmap()->getSizeInCellsX()));
    idx_out.y=floor((int)(col_id*costmap_ros->getCostmap()->getSizeInCellsY()));

    return idx_out;
}

/**
 * Determines if Point is in free spaced X_space.
 *
 * @param[out] whether point is in free space.
 */
bool inFreeSpace(const geometry_msgs::Point point, 
                 const costmap_2d::Costmap2DROS* costmap_ros){
    bool result{0};
    int threshold{127};
    //Get grid indices of Point x/y values
    idx idx_out=get_idx(point, costmap_ros);

    //Check if point in costmap_ros is unoccupied
    if (costmap_ros->getCostmap()->getCost(idx_out.x,idx_out.y)<=threshold) {
        result=1;
    }

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
        // cout << "edge_point.x: " << edge_point.x << " m. \n";
        // cout << "edge_point.y: " << edge_point.y << " m. \n";
        edgePointIsFree=inFreeSpace(edge_point, costmap_ros);
        if (edgePointIsFree){
            continue; //skip to next iteration
        } else {
            result=0;

            break;
        }
    }

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
        randomState.x=randomDouble(0.,max_w);
        randomState.y=randomDouble(0.,max_h);
        pointIsFree=inFreeSpace(randomState, costmap_ros);
    }

    return randomState;
}