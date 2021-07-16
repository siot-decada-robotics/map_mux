#include "ros/ros.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include "map_mux/MapRequest.h"
#include "yaml-cpp/yaml.h"
#include <vector>
#include<iostream>

class MapMux
{
public:
    MapMux(ros::NodeHandle *node_ptr)
    {
      node = node_ptr;
    }

    void start()
    { 
      std::cout<<"initiating map mux."<<std::endl;
      //negative so it will not equal to any map number
      current_map_number = -1;

      yaml_file_path = "/home/kai/multi_map_ws/install/map_mux/share/map_mux/config/example_environment.yaml";
      if (node->getParam("map_file", map_file))
      { 
        yaml_file_path = map_file;
        std::cout<<"input path to yaml file: "<<yaml_file_path<<std::endl;
      }
      else
      {
        std::cout<<"no yaml file path specified. Defaulting to use"<<yaml_file_path<<std::endl;
      }
      
      yaml_config = YAML::LoadFile(yaml_file_path);
      
      //vector of ros subscribers depending on number of levels/maps

      for(const auto levels : yaml_config)
      {
        std::string map_topic_name;
        std::string topic_prefix = "multimap_server/maps";
        std::string attrib_1 = "maps";
        std::string attrib_2 = "routes";
        
        // map_path = levels.second[attrib_1].second[attrib_2];

        map_topic_name += topic_prefix + "/" + levels.first.as<std::string>() + "/routes/map";

        ros::Subscriber map_sub;
        map_sub = node->subscribe(map_topic_name, 100, &MapMux::read_map_cb, this);
        vector_map_sub.push_back(map_sub);
      }
    
      map_request_sub = node->subscribe("map_request", 100, &MapMux::map_request_cb, this);
      map_pub = node->advertise<nav_msgs::OccupancyGrid>("map", 10);
    
    }

    void map_request_cb(const map_mux::MapRequest::ConstPtr& map_request)
    {   
        //do not publish again if the current map is the requested map
        if (map_request->map_number == current_map_number)
            std::cout<<"Already publishing desired map of map_number: "<<current_map_number<<std::endl;
        else
        { 
          std::cout<<"publishing new map"<<std::endl;
          current_map_number = map_request->map_number;
          map_pub.publish(vector_maps[map_request->map_number]);
        }
            
    }

    void read_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
    {   
        nav_msgs::OccupancyGrid map;
        map.header = map_msg->header;
        map.info = map_msg->info;
        map.data = map_msg->data;
        vector_maps.push_back(map);
        std::cout<<"size of vector maps is now: "<<vector_maps.size()<<std::endl;
    }



private:
    //ros node handler
    ros::NodeHandle* node;
    
    //subcribers
    ros::Subscriber map_request_sub;
    std::vector<ros::Subscriber> vector_map_sub;

    //publishers
    ros::Publisher map_pub;
    std::vector<nav_msgs::OccupancyGrid> vector_maps;

    //private attributes
    std::string map_file;
    int current_map_number;

    //config file parameters
    std::string yaml_file_path;
    YAML::Node yaml_config;
    std::string map_path;

};

int main(int argc, char** argv)
{   
    std::cout<<"starting script"<<std::endl;
    ros::init(argc, argv,"map_mux");
    ros::NodeHandle node;
    MapMux mux = MapMux(&node);
    std::cout<<"starting node"<<std::endl;
    mux.start();
    ros::spin();
    

}