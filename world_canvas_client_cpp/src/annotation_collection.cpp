/*
 * annotation_collection.cpp
 *
 *  Created on: May 7, 2014
 *      Author: jorge
 */

#include <ros/ros.h>
#include <world_canvas_msgs/GetAnnotations.h>
#include <world_canvas_msgs/GetAnnotationsData.h>
#include <world_canvas_msgs/PubAnnotationsData.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "world_canvas_client_cpp/annotation_collection.hpp"


AnnotationCollection::AnnotationCollection(const std::string& world)
  : filter(FilterCriteria(world))
{
}

AnnotationCollection::AnnotationCollection(const FilterCriteria& criteria)
  : filter(criteria)
{
  // Filter parameters provided, so don't wait more to retrieve annotations!
  this->filterBy(criteria);
}

AnnotationCollection::~AnnotationCollection()
{
}


bool AnnotationCollection::filterBy(const FilterCriteria& criteria)
{
  this->filter = criteria;

  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<world_canvas_msgs::GetAnnotations>("get_annotations");

  ROS_INFO("Waiting for get_annotations service...");
  if (client.waitForExistence(ros::Duration(5.0)) == false)
  {
    ROS_ERROR("get_annotations service not available after 5s");
    return false;
  }

  ROS_INFO("Getting annotations for world %s and additional filter criteria",
           this->filter.getWorld().c_str());
  world_canvas_msgs::GetAnnotations srv;
  srv.request.world         = this->filter.getWorld();
  srv.request.ids           = this->filter.getUuids();
  srv.request.names         = this->filter.getNames();
  srv.request.types         = this->filter.getTypes();
  srv.request.keywords      = this->filter.getKeywords();
  srv.request.relationships = this->filter.getRelationships();
  if (client.call(srv))
  {
    if (srv.response.result == true)
    {
      if (srv.response.annotations.size() > 0)
      {
        ROS_INFO("%lu annotations found", srv.response.annotations.size());
      }
      else
      {
        ROS_INFO("No annotations found for world %s with the given search criteria",
                 this->filter.getWorld().c_str());
      }
      this->annotations = srv.response.annotations;
      return true;
    }
    else
    {
      ROS_ERROR("Server reported an error: %s", srv.response.message.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call get_annotations service");
    return false;
  }
}

bool AnnotationCollection::load()
{
  // Retrieve annotations with current filter parameters
  return this->filterBy(this->filter);
}

bool AnnotationCollection::loadData()
{
  if (this->annotations.size() == 0)
  {
    ROS_ERROR("No annotations retrieved. Nothing to load!");
    return false;
  }

  ros::NodeHandle nh;
  ros::ServiceClient client =
      nh.serviceClient<world_canvas_msgs::GetAnnotationsData>("get_annotations_data");

  ROS_INFO("Waiting for get_annotations_data service...");
  if (client.waitForExistence(ros::Duration(5.0)) == false)
  {
    ROS_ERROR("get_annotations_data service not available after 5s");
    return false;
  }

  // Request from server the data for annotations previously retrieved; note that we send data
  // uuids, that identify the data associated to the annotation instead of the annotation itself
  ROS_INFO("Loading data for the %lu retrieved annotations", this->annotations.size());
  world_canvas_msgs::GetAnnotationsData srv;
  srv.request.annotation_ids = this->getAnnotsDataIDs();
  if (client.call(srv))
  {
    if (srv.response.result == true)
    {
      if (srv.response.data.size() > 0)
      {
        ROS_INFO("%lu annotations data found", srv.response.data.size());
      }
      else
      {
        ROS_INFO("No data found for the %lu retrieved annotations", this->annotations.size());
      }
      this->annots_data = srv.response.data;
      return true;
    }
    else
    {
      ROS_ERROR("Server reported an error: %s", srv.response.message.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to call get_annotations_data service");
    return false;
  }
}

bool AnnotationCollection::publishMarkers(const std::string& topic)
{
  if (this->annotations.size() == 0)
  {
    ROS_ERROR("No annotations retrieved. Nothing to publish!");
    return false;
  }

  // Advertise a topic for retrieved annotations' visualization markers
  markers_pub = nh.advertise <visualization_msgs::MarkerArray> (topic, 1, true);

  // Process retrieved data to build markers lists
  visualization_msgs::MarkerArray markers_array;
  for (unsigned int i = 0; i < this->annotations.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.id     = i;
    marker.header = annotations[i].pose.header;
    marker.type   = annotations[i].shape;
    marker.ns     = annotations[i].type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose   = annotations[i].pose.pose.pose;
    marker.scale  = annotations[i].size;
    marker.color  = annotations[i].color;

    markers_array.markers.push_back(marker);
  }

  markers_pub.publish(markers_array);
  return true;
}

bool AnnotationCollection::publish(const std::string& topic_name, bool by_server, bool as_list,
                                   const std::string& topic_type)
{
  if (this->annotations.size() == 0)
  {
    ROS_ERROR("No annotations retrieved. Nothing to publish!");
    return false;
  }

  std::string common_tt = topic_type;

  if (common_tt.empty() == true)
  {
    if (as_list == true)
    {
      ROS_ERROR("Topic type argument is mandatory if as_list is true");
      return false;
    }
    else
    {
      // Take annotations type and verify that it's the same within the
      // collection (as we will publish all of them in the same topic)
      for (unsigned int i = 0; i < this->annotations.size(); i++)
      {
        if ((common_tt.empty() == false) && (common_tt != annotations[i].type))
        {
          ROS_ERROR("Cannot publish annotations of different types (%s, %s)",
                    common_tt.c_str(), annotations[i].type.c_str());
          return false;
        }
        common_tt = annotations[i].type;
      }
    }
  }

  if (by_server == true)
  {
    ros::NodeHandle nh;
    ros::ServiceClient client =
        nh.serviceClient<world_canvas_msgs::PubAnnotationsData>("pub_annotations_data");
    ROS_INFO("Waiting for pub_annotations_data service...");
    if (client.waitForExistence(ros::Duration(5.0)) == false)
    {
      ROS_ERROR("pub_annotations_data service not available after 5s");
      return false;
    }

    // Request server to publish the annotations previously retrieved; note that we send the data
    // uuids, that identify the data associated to the annotation instead of the annotation itself
    ROS_INFO("Requesting server to publish annotations");
    world_canvas_msgs::PubAnnotationsData srv;
    srv.request.annotation_ids = this->getAnnotsDataIDs();
    srv.request.topic_name = topic_name;
    srv.request.topic_type = common_tt;
    srv.request.pub_as_list = as_list;
    if (client.call(srv))
    {
      if (srv.response.result == true)
      {
        return true;
      }
      else
      {
        ROS_ERROR("Server reported an error: %s", srv.response.message.c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("Failed to call pub_annotations_data service");
      return false;
    }
  }
  else
  {
    // TODO: we cannot publish here without the messages class, as we did with Python. Maybe I can
    // use templates, as the user of this class knows the message class. Or I can even make this a
    // template class, assuming annotations collections have a uniform type.
    // See https://github.com/corot/world_canvas/issues/5 for details
    ROS_ERROR("Publish by client not implemented!");
    return false;
  }
}

std::vector<UniqueIDmsg> AnnotationCollection::getAnnotationIDs()
{
  std::vector<UniqueIDmsg> uuids(annotations.size());
  for (unsigned int i = 0; i < annotations.size(); i++)
  {
    uuids[i] = annotations[i].id;
  }
  return uuids;
}

std::vector<UniqueIDmsg> AnnotationCollection::getAnnotsDataIDs()
{
  std::vector<UniqueIDmsg> uuids(annotations.size());
  for (unsigned int i = 0; i < annotations.size(); i++)
  {
    uuids[i] = annotations[i].data_id;
  }
  return uuids;
}
