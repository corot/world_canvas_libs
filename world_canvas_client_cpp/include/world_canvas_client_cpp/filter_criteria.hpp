/*
 * filter_criteria.hpp
 *
 *  Created on: Jul 6, 2014
 *      Author: jorge
 */

#ifndef FILTER_CRITERIA_HPP_
#define FILTER_CRITERIA_HPP_

#include <string>
#include <vector>

#include <uuid_msgs/UniqueID.h>

typedef uuid_msgs::UniqueID UniqueIDmsg;

/**
 * Annotations filter criteria to pass to the world canvas server. The only
 * mandatory criteria is the world_id.
 * Empty vectors are ignored; the non-empty are concatenated with logic ANDs.
 * Within the lists, elements are concatenated with logic ORs, so for example
 * an annotation only needs to contain one of the keywords to be retrieved.
 *
 * TODO: world_name is by now a hex string of the uuid; we must implement a
 * friendly world naming system.
 */
class FilterCriteria
{
private:
  UniqueIDmsg              world_id;
  std::string              world_name;
  std::vector<UniqueIDmsg> uuids;
  std::vector<std::string> names;
  std::vector<std::string> types;
  std::vector<std::string> keywords;
  std::vector<UniqueIDmsg> relationships;

public:
  /**
   * Creates an empty filter criteria set.
   */
  FilterCriteria(const std::string& world_id);

  /**
   * Creates a filter criteria set at one blow.
   *
   * @param id:       Filter annotations by their uuid
   * @param name:     Filter annotations by their name
   * @param type:     Filter annotations by their type
   * @param keyword:  Filter annotations by their keywords
   * @param related:  Filter annotations by their relationships
   */
  FilterCriteria(const std::string & world_id,
                 const std::vector<std::string>& uuids,
                 const std::vector<std::string>& names,
                 const std::vector<std::string>& types,
                 const std::vector<std::string>& keywords,
                 const std::vector<std::string>& relationships);

  virtual ~FilterCriteria();

  bool nullFilter();

  void setWorldId(const std::string& world_id);
  void setUuids(const std::vector<std::string>& uuids);
  void setNames(const std::vector<std::string>& names);
  void setTypes(const std::vector<std::string>& types);
  void setKeywords(const std::vector<std::string>& keywords);
  void setRelationships(const std::vector<std::string>& relationships);

  UniqueIDmsg getWorldId() const { return world_id; }
  std::string getWorldName() const { return world_name; }
  std::vector<UniqueIDmsg> getUuids() const { return uuids; }
  std::vector<std::string> getNames() const { return names; }
  std::vector<std::string> getTypes() const { return types; }
  std::vector<std::string> getKeywords() const { return keywords; }
  std::vector<UniqueIDmsg> getRelationships() const { return relationships; }
};

#endif /* FILTER_CRITERIA_HPP_ */
