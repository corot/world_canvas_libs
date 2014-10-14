/*
 * annotation_collection.hpp
 *
 *  Created on: May 7, 2014
 *      Author: jorge
 */

#ifndef ANNOTATION_COLLECTION_HPP_
#define ANNOTATION_COLLECTION_HPP_

#include <ros/serialization.h>
#include <ros/serialized_message.h>

#include <visualization_msgs/Marker.h>

#include <world_canvas_msgs/Annotation.h>
#include <world_canvas_msgs/AnnotationData.h>

#include "world_canvas_client_cpp/unique_id.hpp"
#include "world_canvas_client_cpp/filter_criteria.hpp"
#include "world_canvas_client_cpp/world_canvas_client.hpp"

namespace wcf
{

/**
 * Manages a collection of annotations and its associated data, initially empty.
 * Annotations and data are retrieved from the world canvas server, filtered by
 * the described parameters.
 * This class can also publish the retrieved annotations and RViz visualization
 * markers, mostly for debug purposes.
 */
class AnnotationCollection : public WorldCanvasClient
{
protected:
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  ros::Publisher markers_pub;
  std::string srv_namespace;

  FilterCriteria filter;

  std::vector<world_canvas_msgs::Annotation>     annotations;
  std::vector<world_canvas_msgs::AnnotationData> annots_data;

  /**
   * Return data IDs for the current annotations collection. This method is private
   * because is only necessary for loading annotations data from server, operation
   * that must be transparent for the user.
   *
   * @returns Vector of unique IDs.
   */
  std::vector<UniqueIDmsg> getAnnotsDataIDs();

  /**
   * Create a service client of the template type and wait until the service is available.
   *
   * @param service_name: ROS service name to get, without namespace.
   * @param timeout: Timeout to wait for the service to come up.
   * @returns: The service handle.
   * @throws: ROS exception on timeout.
   */
  template <typename T>
  ros::ServiceClient getServiceHandle(const std::string& service_name, double timeout = 5.0)
  {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<T>(srv_namespace + service_name);
    ROS_INFO("Waiting for '%s' service...", service_name.c_str());
    if (client.waitForExistence(ros::Duration(timeout)) == false)
    {
      ROS_ERROR("'%s' service not available after %.2f s", service_name.c_str(), timeout);
      throw ros::Exception(service_name + " service not available");
    }

    return client;
  }

public:
  /**
   * Initializes the collection of annotations and associated data, initially empty.
   *
   * @param world: Annotations in this collection belong to this world.
   * @param srv_namespace: World canvas handles can be found under this namespace.
   */
  AnnotationCollection(const std::string& world, const std::string& srv_namespace = "");

  /**
   * Initializes the collection of annotations and associated data, initially empty.
   *
   * @param criteria: Annotations filter criteria to pass to the server (must contain
   * at least a valid world name).
   * @param srv_namespace: World canvas handles can be found under this namespace.
   */
  AnnotationCollection(const FilterCriteria& criteria, const std::string& srv_namespace = "");

  virtual ~AnnotationCollection();

  /**
   * Reload annotations collection, filtered by new selection criteria.
   *
   * @param criteria: Annotations filter criteria to pass to the server.
   * @returns True on success, False otherwise.
   */
  bool filterBy(const FilterCriteria& criteria);

  /**
   * Load annotations collection, filtered by current selection criteria.
   *
   * @returns True on success, False otherwise.
   */
  bool load();

  /**
   * Load associated data for the current annotations collection.
   *
   * @returns True on success, False otherwise.
   */
  bool loadData();

  /**
   * Publish RViz visualization markers for the current collection of annotations.
   *
   * @param topic: Where we must publish annotations markers.
   * @returns True on success, False otherwise.
   */
  bool publishMarkers(const std::string& topic);

  /**
   * Publish RViz visualization marker for the given annotation.
   *
   * @param topic  Where we must publish annotation marker.
   * @param marker_id Id to add to the visual marker.
   * @param ann Annotation to publish a marker for.
   * @param deleteExisting Delete all markers previously published.
   * @returns True on success, False otherwise.
   */
  bool publishMarker(const std::string& topic, int marker_id,
                     const world_canvas_msgs::Annotation& ann, bool deleteExisting = true);

  /**
   * Create a RViz visualization marker for the given annotation.
   *
   * @param id Id to add to the visual marker.
   * @param ann Annotation to create a marker for.
   * @returns The created marker.
   */
  visualization_msgs::Marker makeMarker(int id, const world_canvas_msgs::Annotation& ann);

  /**
   * Create a RViz visualization marker to add a label above the given marker.
   *
   * @param marker Marker to create a label for.
   * @returns The created marker.
   */
  visualization_msgs::Marker makeLabel(const visualization_msgs::Marker& marker);

  /**
   * Publish the current collection of annotations, by this client or by the server.
   * As we use just one topic, all annotations must be of the same type (function will
   * return with error otherwise).
   *
   * @param topic_name: Where we must publish annotations data.
   * @param by_server:  Request the server to publish the annotations instead of this client.
   * @param as_list:    If true, annotations will be packed in a list before publishing,
   *                    so topic_type must be an array of currently loaded annotations.
   * @param topic_type: The message type to publish annotations data.
   *                    Mandatory if as_list is true; ignored otherwise.
   * @returns True on success, False otherwise.
   */
  bool publish(const std::string& topic_name, bool by_server = false, bool as_list = false,
               const std::string& topic_type = "");

  /**
   * Return the annotation with the given unique id.
   *
   * @param id Target annotation id.
   * @returns Searched annotation.
   */
  const world_canvas_msgs::Annotation& getAnnotation(const UniqueIDmsg& id);

  /**
   * Return the annotations with the given name. Can be more than one, as we don't enforce
   * uniqueness on annotation names.
   *
   * @param name Target annotations name.
   * @returns Searched annotations.
   */
  std::vector<world_canvas_msgs::Annotation> getAnnotations(const std::string& name);

  /**
   * Return unique IDs for the current annotations collection.
   *
   * @returns Vector of unique IDs.
   */
  std::vector<UniqueIDmsg> getAnnotationIDs();

  /**
   * Return the current annotations collection.
   *
   * @returns Vector of annotations.
   */
  const std::vector<world_canvas_msgs::Annotation>& getAnnotations() { return annotations; }

  /**
   * Return the data for annotations of the template type.
   *
   * @param data Vector of annotations data.
   */
  template <typename T>
  unsigned int getData(std::vector<T>& data)
  {
    unsigned int count = 0;
    std::string type = ros::message_traits::DataType<T>::value();

    for (unsigned int i = 0; i < annots_data.size(); ++i)
    {
      // Check id this data corresponds to an annotation of the requested type. TODO: but we should
      // a list of tuples containing anns + data, so we don't need to relay on data.type
      if (annots_data[i].type != type)
        continue;

      // Deserialize the ROS message contained on data field; we must clone the serialized data
      // because SerializedMessage requires a boost::shared_array as input that will try to destroy
      // the underlying array once out of scope. Also we must skip 4 bytes on message_start pointer
      // because the first 4 bytes on the buffer contains the (already known) serialized data size.
      T object;
      uint32_t serial_size = annots_data[i].data.size();
      boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
      memcpy(buffer.get(), &annots_data[i].data[0], serial_size);
      ros::SerializedMessage sm(buffer, serial_size);
      sm.type_info = &typeid(object);
      sm.message_start += 4;
      try
      {
        ROS_DEBUG("Deserializing object '%s' of type '%s'",
                  uuid::toHexString(annots_data[i].id).c_str(), type.c_str());
        ros::serialization::deserializeMessage(sm, object);
        data.push_back(object);
        count++;
      }
      catch (ros::serialization::StreamOverrunException& e)
      {
        ROS_ERROR("Deserialization failed on object %d: %s", i, e.what());
      }
    }
    return count;
  }

};

} // namespace wcf

#endif /* ANNOTATION_COLLECTION_HPP_ */
