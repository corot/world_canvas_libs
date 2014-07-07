/*
 * annotation_collection.cpp
 *
 *  Created on: May 7, 2014
 *      Author: jorge
 */

#include <ros/ros.h>
#include <world_canvas_msgs/GetAnnotations.h>
//#include <geometry_msgs*
//  from rospy_message_converter import message_converter
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "world_canvas_client_cpp/annotation_collection.hpp"

AnnotationCollection::AnnotationCollection(const std::string& world_id)
  : filter(FilterCriteria(world_id))
{
}

AnnotationCollection::AnnotationCollection(const FilterCriteria& filter)
  : filter(filter)
{

//  if world_id is not None:
//      # Filter parameters provided, so don't wait more to retrieve annotations!
//      self.filterBy(world_id, id, name, type, keyword, related)
//
//  return
}

bool AnnotationCollection::filterBy(const FilterCriteria& filter)
{
  this->filter = filter;

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<world_canvas_msgs::GetAnnotations>("get_annotations");
  world_canvas_msgs::GetAnnotations srv;
  srv.request.world_id      = filter.getWorldId();
  srv.request.ids           = filter.getUuids();
  srv.request.names         = filter.getNames();
  srv.request.types         = filter.getTypes();
  srv.request.keywords      = filter.getKeywords();
  srv.request.relationships = filter.getRelationships();

  ROS_INFO("Waiting for get_annotations service...");
//  rospy.wait_for_service('get_annotations')

  ROS_INFO("Getting annotations for world %s and additional filter criteria",
           filter.getWorldName().c_str());
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
                 filter.getWorldName().c_str());
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


//  get_anns_srv = rospy.ServiceProxy('get_annotations', world_canvas_msgs.srv.GetAnnotations)
//  response = get_anns_srv(unique_id.toMsg(uuid.UUID('urn:uuid:' + world_id)),
//                         [unique_id.toMsg(uuid.UUID('urn:uuid:' + annot_id)) for annot_id in id],
//                          name, type, keyword, [], [], [], related)  # 3 filters unimplemented
}

AnnotationCollection::~AnnotationCollection()
{
  // TODO Auto-generated destructor stub
}


//def loadData(self):
//    '''
//    @returns True on success, False otherwise.
//
//    Load associated data for the current annotations collection.
//    '''
//    if self.annotations is None:
//        ROS_ERROR('No annotations retrieved. Nothing to load!')
//        return False
//
//    ROS_INFO("Waiting for get_annotations_data service...")
//    rospy.wait_for_service('get_annotations_data')
//
//    ROS_INFO('Loading data for the %d retrieved annotations', len(self.annotations))
//    get_data_srv = rospy.ServiceProxy('get_annotations_data', world_canvas_msgs.srv.GetAnnotationsData)
//    response = get_data_srv([a.data_id for a in self.annotations])
//
//    if response.result:
//        if len(response.data) > 0:
//            ROS_INFO('%d annotations data retrieved', len(response.data))
//            self.annots_data = response.data
//        else:
//            ROS_WARN('No data found for the %d retrieved annotations', len(self.annotations))
//    else:
//        ROS_ERROR('Server reported an error: ', response.message)
//
//    return response.result
//
//def publishMarkers(self, topic):
//    '''
//    @param topic: Where we must publish annotations markers.
//    @returns True on success, False otherwise.
//
//    Publish RViz visualization markers for the current collection of annotations.
//    '''
//    if self.annotations is None:
//        ROS_ERROR('No annotations retrieved. Nothing to publish!')
//        return False
//
//    # Advertise a topic for retrieved annotations' visualization markers
//    markers_pub = rospy.Publisher(topic, MarkerArray, latch=True, queue_size=5)
//
//    # Process retrieved data to build markers lists
//    markers_list = MarkerArray()
//
//    marker_id = 1
//    for a in self.annotations:
//        marker = Marker()
//        marker.id = marker_id
//        marker.header = a.pose.header
//        marker.type = a.shape
//        marker.ns = a.type
//        marker.action = Marker.ADD
//        marker.lifetime = rospy.Duration.from_sec(0)
//        marker.pose = copy.deepcopy(a.pose.pose.pose)
//        marker.scale = a.size
//        marker.color = a.color
//
//        markers_list.markers.append(marker)
//
//        marker_id = marker_id + 1
//
//    markers_pub.publish(markers_list)
//    return True
//
//def publish(self, topic_name, topic_type=None, by_server=False, as_list=False):
//    '''
//    @param topic_name: Where we must publish annotations data.
//    @param topic_type: The message type to publish annotations data.
//                       Mandatory if pub_as_list is true; ignored otherwise.
//    @param by_server:  Request the server to publish the annotations instead of this client.
//    @param as_list:    If true, annotations will be packed in a list before publishing,
//                       so topic_type must be an array of currently loaded annotations.
//    @returns True on success, False otherwise.
//
//    Publish the current collection of annotations, by this client or by the server.
//    As we use just one topic, all annotations must be of the same type (function will return
//    with error otherwise.
//    '''
//    if self.annotations is None:
//        ROS_ERROR('No annotations retrieved. Nothing to publish!')
//        return False
//
//    if topic_type is None:
//        if as_list:
//            ROS_ERROR("Topic type argument is mandatory if as_list is true")
//            return False
//        else:
//            # Take annotations type and verify that it's the same within the
//            # collection (as we will publish all of them in the same topic)
//            for a in self.annotations:
//                if topic_type is not None and topic_type != a.type:
//                    ROS_ERROR("Cannot publish annotations of different types (%s, %s)" % (topic_type, a.type))
//                    return False
//                topic_type = a.type
//
//    if by_server:
//        ROS_INFO("Waiting for pub_annotations_data service...")
//        rospy.wait_for_service('pub_annotations_data')
//
//        # Request server to publish the annotations previously retrieved
//        ROS_INFO('Requesting server to publish annotations')
//        pub_data_srv = rospy.ServiceProxy('pub_annotations_data', world_canvas_msgs.srv.PubAnnotationsData)
//        response = pub_data_srv([a.data_id for a in self.annotations], topic_name, topic_type, as_list)
//        if not response.result:
//            ROS_ERROR('Server reported an error: %s' % response.message)
//        return response.result
//    else:
//        # Advertise a topic to publish retrieved annotations
//        topic_class = roslib.message.get_message_class(topic_type)
//        if topic_class is None:
//            # This happens if the topic type is wrong or not known (i.e. absent from ROS_PACKAGE_PATH)
//            ROS_ERROR("Topic type %s definition not found" % topic_type)
//            return False
//
//        objects_pub = rospy.Publisher(topic_name, topic_class, latch=True, queue_size=5)
//
//        # Process retrieved data to build annotations lists
//        objects_list = list()
//
//        for d in self.annots_data:
//            object = pickle.loads(d.data)
//            objects_list.append(object)
//
//        # Publish resulting list
//        if as_list:
//            objects_pub.publish(objects_list)
//        else:
//            # if as_list is false, publish objects one by one
//            for object in objects_list:
//                objects_pub.publish(object)
//
//    return True
