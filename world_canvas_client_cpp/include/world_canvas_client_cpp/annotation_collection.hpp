/*
 * annotation_collection.hpp
 *
 *  Created on: May 7, 2014
 *      Author: jorge
 */

#ifndef ANNOTATION_COLLECTION_HPP_
#define ANNOTATION_COLLECTION_HPP_

#include <world_canvas_msgs/Annotation.h>
#include <world_canvas_msgs/AnnotationData.h>

#include "world_canvas_client_cpp/filter_criteria.hpp"

/**
 * Manages a collection of annotations and its associated data, initially empty.
 * Annotations and data are retrieved from the world canvas server, filtered by
 * the described parameters.
 * This class can also publish the retrieved annotations and RViz visualization
 * markers, mostly for debug purposes.
 */
class AnnotationCollection
{
private:
  ros::NodeHandle nh;
  ros::Publisher markers_pub;

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

public:
  /**
   * Initializes the collection of annotations and associated data, initially empty.
   *
   * @param world_id: Annotations in this collection belong to this world.
   */
  AnnotationCollection(const std::string& world_id);

  /**
   * Initializes the collection of annotations and associated data, initially empty.
   *
   * @param criteria: Annotations filter criteria to pass to the server (must contain
   * at least a valid world_id uuid).
   */
  AnnotationCollection(const FilterCriteria& criteria);

  virtual ~AnnotationCollection();

  /**
   * Reload annotations collection, filtered by new selection criteria.
   *
   * @param criteria: Annotations filter criteria to pass to the server.
   * @returns True on success, False otherwise.
   */
  bool filterBy(const FilterCriteria& criteria);

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
   * Return unique IDs for the current annotations collection.
   *
   * @returns Vector of unique IDs.
   */
  std::vector<UniqueIDmsg> getAnnotationIDs();
};

#endif /* ANNOTATION_COLLECTION_HPP_ */
