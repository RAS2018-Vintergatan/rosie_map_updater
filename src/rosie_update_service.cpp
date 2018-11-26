#include "ros/ros.h"
#include <rosie_map_updater/NewGrid.h>
#include <rosie_map_updater/GetGrid.h>
#include <nav_msgs/OccupancyGrid.h>

#include <cstdlib>

nav_msgs::OccupancyGrid newGrid; // no rerun
bool updated = 0;

bool gridCallback(rosie_map_updater::NewGrid::Request &req,rosie_map_updater::NewGrid::Response &res){
	newGrid = req.newGrid;
	updated = 1;
	return true;
}

bool getterCallback(rosie_map_updater::GetGrid::Request &req, rosie_map_updater::GetGrid::Response &res){
	if(req.question == 1){
		res.newGrid = newGrid;
		res.answer = updated;
		updated = 0;
	}

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosie_rrt_server");

  ros::NodeHandle n;

  //ros::ServiceClient client = n.serviceClient<arduino_servo_control::SetServoAngles>("SetServoAngles");
  ros::ServiceServer GridSercie = n.advertiseService<rosie_map_updater::NewGrid::Request, rosie_map_updater::NewGrid::Response>("update_grid", gridCallback);
	ros::ServiceServer GetterService = n.advertiseService<rosie_map_updater::GetGrid::Request, rosie_map_updater::GetGrid::Response>("get_updated_grid", getterCallback);
  //ros::Subscriber sub = n.subscribe("MoveGates", 1000, GateCallback);
  ROS_INFO("Ready to move the grippers.");
  ros::spin();

  return 0;
}
