#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include "map_mux/MapRequest.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include<iostream>


void map_request_cb(const map_mux::MapRequest::ConstPtr& _map_request)
{
    std::string fleet_name = map_request.fleet_name;
    std::string robot_name = map_request.robot_name;
    int map_number = map_request.map_number;
    ROS_INFO("Requested map_number: [%d]", map_number);

    //TODO support lift_number int the future
    switch_map(map_number);
}

void switch_map(int map_number)
{
  
}

void read_yaml()
{

}

void read_map_cb(const nav_msgs::OccupancyGrid& map)
{
  
}

void start()
{ 
  YAML::Node config = YAML::LoadFile("config.yaml");
  ros::init(argc, argv, "map_mux");
  ros::NodeHandle node;
  std::string yaml_file_name = "config.yaml";

  std::string map_file;
  if (node.getParam("map_file", map_file))
    yaml_file_name = map_file;

  std::cout<<yaml_file_name<<std::endl;
  YAML::Node yaml_config = YAML::LoadFile(yaml_file_name);

  //vector of ros subscribers depending on number of levels/maps
  vector<ros::Subscriber> maps_sub;

  for(const auto levels : yaml_config)
  {
    std::string map_topic_name;
    std::string topic_prefix = "/multimap_server/maps"
    std::string attrib_1 = "maps";
    std::string attrib_2 = "routes";

    map_path = levels[attrib_1][attrib_2];

    map_topic_name += topic_prefix + "/" + levels.first.as<std:string>() + "/routes/map";


    ros::Subscriber map_sub;
    map_sub = node.subscribe(map_topic_name, 100, read_map_cb)
  }

  ros::Subscriber map_request_sub = node.subscribe("map_request", 100, map_request_cb);
  
  ros::spin();
}

int main()
{ 

  start();
  return 0;

}