#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h> //rand

/* 
 * Generates a random Path for visualization in RViz.
 *
 * param[in] Pointer to existing Path object.
 *
 */
void getPath(nav_msgs::Path* path_ptr){
	//Update time stamp
	path_ptr->poses.clear();
	path_ptr->header.stamp=ros::Time::now();
	path_ptr->header.frame_id="map";
	//Define pose
	geometry_msgs::PoseStamped waypoint{};
	waypoint.header.frame_id="map";	
	//Add waypoints
	int max_ii;
	max_ii=rand()%20+2; //max_ii in 2-20;
	for (size_t ii=0; ii<max_ii; ii++){
		waypoint.pose.position.x=rand()%10+1; //x in 1-10;
		waypoint.pose.position.y=rand()%20+1; //y in 1-20;
		waypoint.pose.position.z=1.;
		path_ptr->poses.push_back(waypoint);
	}
}

/* 
 * Publishes a Path for visualization in RViz.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_pub_node");
	ros::NodeHandle nh;

	ros::Publisher path_pub=nh.advertise<nav_msgs::Path>("path_viz", 0);
	ros::Rate loop_rate(0.5); //Hz

	//Create path message
	nav_msgs::Path path_msg{};

	while (ros::ok()){
		//Update path_msg
		getPath(&path_msg);

		//Publish path_msg
		path_pub.publish(path_msg);
		ROS_INFO("Published updated Path.");

		//Sleep if necessary
		ros::spinOnce;
		loop_rate.sleep();
	}

	return 0;
}