#include "ros/ros.h"
#include "ros/package.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_mux/MapRequest.h"
#include "map_server/image_loader.h"
#include "nav_msgs/GetMap.h"

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

    void start(int argc, char** argv)
    { 
  
      //negative so it will not equal to any map number
      current_map_number = -1;

      std::string default_yaml_file_path = "./example_environment.yaml";
      
      if (argc == 2)
      { 
        yaml_file_path = std::string(argv[1]);
        ROS_INFO("input path to yaml file: %s ",yaml_file_path.c_str());
        yaml_config = YAML::LoadFile(yaml_file_path);
      }
      else
      { 
        ROS_ERROR("no yaml file path specified.");
        exit(0);
      }
       
      //add the path of the maps from the yaml file into the map_paths vector
      for(const auto level : yaml_config)
      {
        //TODO: MapRequest to support the use of level_name instead of map number
        std::string level_name = level.first.as<std::string>();
        level_names.push_back(level_name);
        std::cout<<"level name: "<<level_name<<std::endl;

        //path to the yaml file of the map

        std::string package_name = level.second["maps_package"].as<std::string>();
        
        std::string yaml_map_path = level.second["maps"]["path"].as<std::string>();
        yaml_map_path = ros::package::getPath(package_name) + "/" + yaml_map_path;
        map_paths.push_back(yaml_map_path);
        std::cout<<"yaml file path to map "<<level_name<<": "<<yaml_map_path<<std::endl;

        //path to the pgm file relative to yaml file
        std::string pgm_map_path;
        double origin[3];
        double resolution;
        int negate;
        double occ_th, free_th;
        MapMode mode = TRINARY;

        YAML::Node yaml_file_node = YAML::LoadFile(yaml_map_path);
        
        
        pgm_map_path = ros::package::getPath(package_name) + "/maps" + "/" + level_name + "/" + yaml_file_node["image"].as<std::string>();
        std::cout<<"pgm file path to map "<<level_name<<": "<<pgm_map_path<<std::endl;

        std::vector<double> vector_origin = yaml_file_node["origin"].as<std::vector<double>>();

        origin[0] = vector_origin[0];
        origin[1] = vector_origin[1];
        origin[2] = vector_origin[2];

        resolution = yaml_file_node["resolution"].as<double>();

        negate = yaml_file_node["negate"].as<int>();

        occ_th = yaml_file_node["occupied_thresh"].as<double>();

        free_th = yaml_file_node["free_thresh"].as<double>();

        std::cout<<"pgm_map_path: "<<pgm_map_path<<std::endl;
        std::cout<<"origin: "<<origin[0]<<", "<<origin[1]<<", "<<origin[2]<<std::endl;
        std::cout<<"resolution: "<<resolution<<std::endl;
        std::cout<<"negate: "<<negate<<std::endl;
        std::cout<<"occ_th: "<<occ_th<<std::endl;
        std::cout<<"free_th: "<<free_th<<std::endl;

        nav_msgs::GetMap::Response map_response;

        map_server::loadMapFromFile(&map_response, pgm_map_path.c_str(), resolution, negate, occ_th, free_th, origin, mode);
        vector_maps.push_back(map_response.map);
      }
    
      map_pub = node->advertise<nav_msgs::OccupancyGrid>("map", 10);
      map_request_sub = node->subscribe("map_request", 100, &MapMux::map_request_cb, this);
    }

    void map_request_cb(const map_mux::MapRequest::ConstPtr& map_request)
    { 
      bool map_exist = false;

      std::string req_level_name = map_request->level_name;
      if (req_level_name.compare(current_level) == 0)
      {
        map_exist = true;
        std::cout<<"Already publishing desired map of map_number: "<<current_map_number<<std::endl;
      }
          

      else //publish map
      { 
        for(int i = 0; i < level_names.size(); i ++)
        {
          if (req_level_name.compare(level_names[i]) == 0)
          {
            current_level = req_level_name;
            std::cout<<"Publishing map of "<<current_level<<std::endl;
            map_pub.publish(vector_maps[i]);
            map_exist = true;
            break;
          }
        }
      }
      if (map_exist == false)
        std::cout<<"map of name: "<<req_level_name<<" does not exist!"<<std::endl;
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

    void get_map()
    {
      
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
    std::vector<std::string> map_paths;
    std::vector<std::string> level_names;

    //private attributes
    std::string current_level;
    int current_map_number;

    //config file parameters
    std::string yaml_file_path;
    YAML::Node yaml_config;
    

};

int main(int argc, char** argv)
{   
    ros::init(argc, argv,"map_mux");
    ros::NodeHandle node;
    MapMux mux = MapMux(&node);
    mux.start(argc, argv);
    ros::spin();
    return 0;
}