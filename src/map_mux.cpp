#include "ros/ros.h"
#include "ros/package.h"
#include "ros/console.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_mux/MapRequest.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/LoadMap.h"
#include "map_server/image_loader.h"
#include <std_srvs/Empty.h>

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
      map_request_sub = node->subscribe("/map_request", 100, &MapMux::map_request_cb, this);

      map_change_svc = node->serviceClient<nav_msgs::LoadMap>("/tb3_0/change_map");

    }

    void map_request_cb(const map_mux::MapRequest::ConstPtr& map_request)
    { 
      bool map_exist = false;
      std::string map_name = "/maps/" + map_request->level_name;
      bool use_prefix = true;

      if (node->getParam("/map_mux/use_prefix", use_prefix)){
        std::string map_change_topic = "/" + map_request->robot_name + "/change_map";
        map_change_svc = node->serviceClient<nav_msgs::LoadMap>(map_change_topic);
      }
      else
      {
        map_change_svc = node->serviceClient<nav_msgs::LoadMap>("/change_map");
      }

      std::string map_url = "";
      if (node->getParam(map_name, map_url)){
        ROS_INFO("Changing to : %s", map_name.c_str());
        nav_msgs::LoadMap srv;
        srv.request.map_url = map_url;

        if (map_change_svc.call(srv))
        {
          node->setParam("/" + map_request->robot_name + "_free_fleet_client_node/level_name", map_request->level_name);
          ROS_INFO("Response: %s", srv.response.result);
        }
        else{
          ROS_INFO("Error at the map_svc Call back, no such service");
        }
      }
      else{
        ROS_INFO("No such param for map_name");
      }

      // if (map_request->level_name == "MBC_L9")
      // {
      //   ROS_INFO("Changing to MBC_L9");

      //   nav_msgs::LoadMap srv;
      //   srv.request.map_url = "/media/abilash/Ubuntu_Data1/ROS/adhoc_ws/src/adhoc_simulations/maps/mbc_L9.yaml";



      //   if (map_change_svc.call(srv))
      //   {
      //     node->setParam("/tb3_0_free_fleet_client_node/level_name", "L9");
      //     ROS_INFO("Response: %s", srv.response.result);
      //   }
      // }
      // else if (map_request->level_name == "MBC_L10")
      // {
      //   ROS_INFO("Changing to MBC_L10");

      //   nav_msgs::LoadMap srv;
      //   srv.request.map_url = "/media/abilash/Ubuntu_Data1/ROS/adhoc_ws/src/adhoc_simulations/maps/mbc_L10.yaml";

      //   if (map_change_svc.call(srv))
      //   {
      //     node->setParam("/tb3_0_free_fleet_client_node/level_name", "L10");
      //     ROS_INFO("Response: %s", srv.response.result);
      //   }
      // }

      std_srvs::Empty emptymsg;
      ros::service::call("/tb3_0/move_base/clear_costmaps",emptymsg);

    }

    void get_map()
    {
      
    }



private:
    //ros node handler
    ros::NodeHandle* node;
    
    //subcribers
    ros::Subscriber map_request_sub;
    ros::ServiceClient map_change_svc;

    //publishers
    ros::Publisher map_pub;

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