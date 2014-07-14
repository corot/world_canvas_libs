/*
 * test_annotation_collection.cpp
 *
 *  Created on: Jul 8, 2014
 *      Author: jorge
 */

#include <ros/ros.h>

#include <yocs_msgs/Wall.h>
#include <yocs_msgs/Column.h>
#include <nav_msgs/OccupancyGrid.h>

#include <world_canvas_client_cpp/annotation_collection.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_annotation_collection");

  // Parameters
  ros::NodeHandle nh("~");

  std::string world_id;
  std::string topic_name;
  std::string topic_type;
  std::string default_tn("annotations");
  std::string default_tt;
  std::string default_wi("INVALID_UUID");
  bool        pub_as_list;
  std::vector<std::string> ids;
  std::vector<std::string> names;
  std::vector<std::string> types;
  std::vector<std::string> keywords;
  std::vector<std::string> relationships;

  nh.param("world_id",      world_id, default_wi);
  nh.param("topic_name",    topic_name, default_tn);
  nh.param("topic_type",    topic_type, default_tt);
  nh.param("pub_as_list",   pub_as_list, false);
  nh.param("ids",           ids, ids);
  nh.param("names",         names, names);
  nh.param("types",         types, types);
  nh.param("keywords",      keywords, keywords);
  nh.param("relationships", relationships, relationships);

  // Prepare the annotation collection
  FilterCriteria filter(world_id, ids, names, types, keywords, relationships);
  AnnotationCollection ac(filter);
  ac.loadData();
  ROS_INFO("Annotation collection ready!");

  // Publish annotations' visual markers on client side
  ac.publishMarkers("annotation_markers");

  // Request server to publish the annotations
  ac.publish(topic_name, true, pub_as_list, topic_type);

  std::vector<yocs_msgs::Wall> walls;
  std::vector<yocs_msgs::Column> columns;
  std::vector<nav_msgs::OccupancyGrid> maps;

  ROS_INFO("Done    %u   %u   %u",
           ac.getData("yocs_msgs/Wall", walls),
           ac.getData("yocs_msgs/Column", columns),
           ac.getData("nav_msgs/OccupancyGrid", maps));
  ROS_INFO("Done    %lu   %lu   %lu", walls.size(), columns.size(), maps.size());
  ros::Publisher wp = nh.advertise<yocs_msgs::Wall> ("walls_on_client", 1, true);
  ros::Publisher cp = nh.advertise<yocs_msgs::Column> ("columns_on_client", 1, true);
  ros::Publisher mp = nh.advertise<nav_msgs::OccupancyGrid> ("maps_on_client", 1, true);

  for (unsigned int i = 0; i < walls.size(); i++)
    wp.publish(walls[i]);
  for (unsigned int i = 0; i < columns.size(); i++)
    cp.publish(columns[i]);
  for (unsigned int i = 0; i < maps.size(); i++)
    mp.publish(maps[i]);

  ros::spin();

  return 0;
}
