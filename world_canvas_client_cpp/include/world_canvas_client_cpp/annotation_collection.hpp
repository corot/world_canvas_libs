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
  FilterCriteria filter;

  std::vector<world_canvas_msgs::Annotation>     annotations;
  std::vector<world_canvas_msgs::AnnotationData> annots_data;

public:
  /**
   * Initializes the collection of annotations and associated data, initially empty.
   *
   * @param world_id: Annotations in this collection belong to this world
   */
  AnnotationCollection(const std::string& world_id);

  /**
   * Initializes the collection of annotations and associated data, initially empty.
   *
   * @param filter: Annotations filter criteria to pass to the server (must include world_id)
   */
  AnnotationCollection(const FilterCriteria& filter);

  virtual ~AnnotationCollection();

  /**
   * Reload annotations collection, filtered by new selection criteria.
   *
   * @param filter:   Annotations filter criteria to pass to the server
   * @returns True on success, False otherwise.
   */
  bool filterBy(const FilterCriteria& filter);
};

#endif /* ANNOTATION_COLLECTION_HPP_ */
