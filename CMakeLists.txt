cmake_minimum_required(VERSION 2.8.3)
project(map_mux)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            nav_msgs
            tf2
            roslib
            message_generation
        )

find_package(Bullet REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

#FOR CUSTOM MESSAGE
#=============================================================================
add_message_files(
  FILES
  MapRequest.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
#=============================================================================


find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
        map_server_image_loader
    CATKIN_DEPENDS
        rospy
        roscpp
        nav_msgs
        tf2
        message_runtime
)

include_directories(
    include
    ${BULLET_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${SDL_INCLUDE_DIR}
    ${SDL_IMAGE_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
)

add_library(map_server_image_loader src/image_loader.cpp)
add_dependencies(map_server_image_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server_image_loader
    ${BULLET_LIBRARIES}
    ${catkin_LIBRARIES}
    ${SDL_LIBRARY}
    ${SDL_IMAGE_LIBRARIES}
)


add_executable(map_mux src/map_mux.cpp)
##add_dependencies(map_mux ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} multimap_server_msgs_generate_messages_cpp )
target_link_libraries(map_mux
    map_server_image_loader
    ${YAMLCPP_LIBRARIES}
    ${catkin_LIBRARIES}
)


## Install executables and/or libraries
install(TARGETS map_mux map_server_image_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

##Install Python scripts
  install(
    PROGRAMS 
      scripts/send_map_request.py
    DESTINATION lib/${PROJECT_NAME}
  )

## Install launch, config, map directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

