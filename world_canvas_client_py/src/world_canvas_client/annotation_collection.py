import rospy
import roslib
import copy
import uuid
import unique_id
import cPickle as pickle
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker, MarkerArray

class AnnotationCollection:

    def __init__(self, world_id = None, id = [], type = [], keyword = [], related = []):
        """
        @param world_id: Annotations in this collection belong to this world
        @param id:       Filter annotations by their uuid
        @param type:     Filter annotations by their type
        @param keyword:  Filter annotations by their keywords
        @param related:  Filter annotations by their relationships

        Creates a collection of annotations and its associated data, initially empty.
        Annotations and data are retrieved from the world canvas server, filtered by
        the described parameters.
        This class can also publish the retrieved annotations and RViz visualization
        markers, mostly for debug purposes.
        """
        self.annotations = None
        self.annots_data = None
        
        if world_id is not None:
            # Filter parameters provided, so don't wait more to retrieve annotations!
            self.filterBy(world_id, id, type, keyword, related)
            
        return
    
    def filterBy(self, world_id, id = [], type = [], keyword = [], related = []):
        rospy.loginfo("Waiting for get_annotations service...")
        rospy.wait_for_service('get_annotations')
    
        rospy.loginfo('Getting annotations with map uuid %s and additional filter criteria', world_id)
        get_anns_srv = rospy.ServiceProxy('get_annotations', world_canvas_msgs.srv.GetAnnotations)
        response = get_anns_srv(unique_id.toMsg(uuid.UUID('urn:uuid:' + world_id)),
                               [unique_id.toMsg(uuid.UUID('urn:uuid:' + annot_id)) for annot_id in id],
                                type, keyword, [], [], [], related)  # 3 filters unimplemented

        if response.result:
            if len(response.annotations) > 0:
                rospy.loginfo('%d annotations found', len(response.annotations))
                self.annotations = response.annotations
            else:
                rospy.loginfo('No annotations found for map %s with the given search criteria', world_id)
        else:
            rospy.logerror('Server reported an error: ', response.message)

        return response.result

    def loadData(self):    
        if self.annotations is None:
            rospy.logerror('No annotations retrieved. Nothing to load!')
            return False
            
        rospy.loginfo("Waiting for get_annotations_data service...")
        rospy.wait_for_service('get_annotations_data')
    
        rospy.loginfo('Loading data for the %d retrieved annotations', len(self.annotations))
        get_data_srv = rospy.ServiceProxy('get_annotations_data', world_canvas_msgs.srv.GetAnnotationsData)
        response = get_data_srv([a.id for a in self.annotations])
    
        if response.result:
            if len(response.data) > 0:
                rospy.loginfo('%d annotations data retrieved', len(response.data))
                self.annots_data = response.data
            else:
                rospy.logwarn('No data found for the %d retrieved annotations', len(self.annotations))
        else:
            rospy.logerror('Server reported an error: ', response.message)

        return response.result
    
    def publishMarkers(self, topic):
        if self.annotations is None:
            rospy.logerror('No annotations retrieved. Nothing to publish!')
            return False
            
        # Advertise a topic for retrieved annotations' visualization markers
        markers_pub = rospy.Publisher(topic, MarkerArray, latch = True)
    
        # Process retrieved data to build markers lists
        markers_list = MarkerArray()    
    
        marker_id = 1
        for a in self.annotations:
            marker = Marker()
            marker.id = marker_id
            marker.header = a.pose.header
            marker.type = a.shape
            marker.ns = a.type
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration.from_sec(0)
            marker.pose = copy.deepcopy(a.pose.pose.pose)
    #        marker.pose.position.z += marker.pose.position.z/2.0
            marker.scale = a.size
            marker.color = a.color
    
            markers_list.markers.append(marker)
    
            marker_id = marker_id + 1
    
        markers_pub.publish(markers_list)
        return True

    def publish(self, topic_name, topic_type, by_server = False, as_list = False):
        if self.annotations is None:
            rospy.logerror('No annotations retrieved. Nothing to publish!')
            return False
            
        if by_server:
            rospy.loginfo("Waiting for pub_annotations_data service...")
            rospy.wait_for_service('pub_annotations_data')

            # Request server to publish the annotations previously retrieved
            rospy.loginfo('Requesting server to publish annotations')
            pub_data_srv = rospy.ServiceProxy('pub_annotations_data', world_canvas_msgs.srv.PubAnnotationsData)
            response = pub_data_srv([a.id for a in self.annotations], topic_name, topic_type, as_list)
            if not response.result:
                rospy.logerror('Server reported an error: ', response.message)
            return response.result
        else:
            # Advertise a topic to publish retrieved annotations
            topic_class = roslib.message.get_message_class(topic_type)
            objects_pub = rospy.Publisher(topic_name, topic_class, latch = True)

            # Process retrieved data to build annotations lists
            objects_list = list()

            for d in self.annots_data:
                object = pickle.loads(d.data)
                objects_list.append(object)

            # Publish resulting list
            if as_list:
                objects_pub.publish(objects_list)
            else:
                # if as_list is false, publish objects one by one
                for object in objects_list:
                    objects_pub.publish(object)
        
        return True
