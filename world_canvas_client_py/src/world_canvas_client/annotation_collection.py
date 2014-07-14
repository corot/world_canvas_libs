#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jorge Santos

import genpy
import rospy
import roslib
import copy
import uuid
import unique_id
import cStringIO as StringIO
import world_canvas_msgs.msg
import world_canvas_msgs.srv

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from visualization_msgs.msg import Marker, MarkerArray

class AnnotationCollection:

    def __init__(self, world_id=None, id=[], name=[], type=[], keyword=[], related=[]):
        '''
        @param world_id: Annotations in this collection belong to this world
        @param id:       Filter annotations by their uuid
        @param name:     Filter annotations by their name
        @param type:     Filter annotations by their type
        @param keyword:  Filter annotations by their keywords
        @param related:  Filter annotations by their relationships

        Creates a collection of annotations and its associated data, initially empty.
        Annotations and data are retrieved from the world canvas server, filtered by
        the described parameters.
        This class can also publish the retrieved annotations and RViz visualization
        markers, mostly for debug purposes.
        '''
        self.annotations = None
        self.annots_data = None
        
        if world_id is not None:
            # Filter parameters provided, so don't wait more to retrieve annotations!
            self.filterBy(world_id, id, name, type, keyword, related)
            
        return
    
    def filterBy(self, world_id, id=[], name=[], type=[], keyword=[], related=[]):
        '''
        @param world_id: Annotations in this collection belong to this world
        @param id:       Filter annotations by their uuid
        @param name:     Filter annotations by their name
        @param type:     Filter annotations by their type
        @param keyword:  Filter annotations by their keywords
        @param related:  Filter annotations by their relationships
        @returns True on success, False otherwise.
        
        Reload annotations collection, filtered by new selection criteria.
        '''
        rospy.loginfo("Waiting for get_annotations service...")
        rospy.wait_for_service('get_annotations')
    
        rospy.loginfo('Getting annotations with map uuid %s and additional filter criteria', world_id)
        get_anns_srv = rospy.ServiceProxy('get_annotations', world_canvas_msgs.srv.GetAnnotations)
        response = get_anns_srv(unique_id.toMsg(uuid.UUID('urn:uuid:' + world_id)),
                               [unique_id.toMsg(uuid.UUID('urn:uuid:' + annot_id)) for annot_id in id],
                                name, type, keyword,
                               [unique_id.toMsg(uuid.UUID('urn:uuid:' + relat_id)) for relat_id in related])

        if response.result:
            if len(response.annotations) > 0:
                rospy.loginfo('%d annotations found', len(response.annotations))
                self.annotations = response.annotations
            else:
                rospy.loginfo('No annotations found for map %s with the given search criteria', world_id)
                self.annotations = None
        else:
            rospy.logerr('Server reported an error: ', response.message)

        return response.result

    def loadData(self):    
        '''
        @returns True on success, False otherwise.
        
        Load associated data for the current annotations collection.
        '''
        if self.annotations is None:
            rospy.logerr('No annotations retrieved. Nothing to load!')
            return False
            
        rospy.loginfo("Waiting for get_annotations_data service...")
        rospy.wait_for_service('get_annotations_data')
    
        rospy.loginfo('Loading data for the %d retrieved annotations', len(self.annotations))
        get_data_srv = rospy.ServiceProxy('get_annotations_data', world_canvas_msgs.srv.GetAnnotationsData)
        response = get_data_srv([a.data_id for a in self.annotations])
    
        if response.result:
            if len(response.data) > 0:
                rospy.loginfo('%d annotations data retrieved', len(response.data))
                self.annots_data = response.data
            else:
                rospy.logwarn('No data found for the %d retrieved annotations', len(self.annotations))
                self.annots_data = None
        else:
            rospy.logerr('Server reported an error: ', response.message)

        return response.result
    
    def getData(self, type):
        result = list()
        
        for ad in self.annots_data: 
            buffer = StringIO.StringIO()
            buffer.write(ad.data)
        
            msg_queue = list()
            try:
                rospy.msg.deserialize_messages(buffer, msg_queue, type)
            except genpy.DeserializationError as e:
                # rospy.logerr('Deserialization failed: %s' % str(e))
                # TODO/WARN: this is ok, as I'm assuming that deserialize will always fail with messages of
                # different types, so I use it as filter. It works by now BUT I'm not 100% sure about that!
                # The TODO is that I should organize both annotations and data in a unified list. Once I have
                # it, make a version of this method with default param type=None and use annotation.type to
                # deserialize each object
                continue
            if len(msg_queue) > 0:
                result.append(msg_queue.pop())
            if len(msg_queue) > 0:
                # This should be impossible
                rospy.logwarn('More than one object deserialized (%d)!' % len(msg_queue) + 1)

        return result;

    def publishMarkers(self, topic):
        '''
        @param topic: Where we must publish annotations markers.
        @returns True on success, False otherwise.
        
        Publish RViz visualization markers for the current collection of annotations.
        '''
        if self.annotations is None:
            rospy.logerr('No annotations retrieved. Nothing to publish!')
            return False
            
        # Advertise a topic for retrieved annotations' visualization markers
        markers_pub = rospy.Publisher(topic, MarkerArray, latch=True, queue_size=5)
    
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
            marker.scale = a.size
            marker.color = a.color
    
            markers_list.markers.append(marker)
    
            marker_id = marker_id + 1
    
        markers_pub.publish(markers_list)
        return True

    def publish(self, topic_name, topic_type=None, by_server=False, as_list=False):
        '''
        @param topic_name: Where we must publish annotations data.
        @param topic_type: The message type to publish annotations data.
                           Mandatory if as_list is true; ignored otherwise.
        @param by_server:  Request the server to publish the annotations instead of this client.
        @param as_list:    If true, annotations will be packed in a list before publishing,
                           so topic_type must be an array of currently loaded annotations.
        @returns True on success, False otherwise.
        
        Publish the current collection of annotations, by this client or by the server.
        As we use just one topic, all annotations must be of the same type (function will return
        with error otherwise).
        '''
        if self.annotations is None:
            rospy.logerr('No annotations retrieved. Nothing to publish!')
            return False

        if by_server:
            rospy.loginfo("Waiting for pub_annotations_data service...")
            rospy.wait_for_service('pub_annotations_data')

            # Request server to publish the annotations previously retrieved
            rospy.loginfo('Requesting server to publish annotations')
            pub_data_srv = rospy.ServiceProxy('pub_annotations_data', world_canvas_msgs.srv.PubAnnotationsData)
            response = pub_data_srv([a.data_id for a in self.annotations], topic_name, topic_type, as_list)
            if not response.result:
                rospy.logerr('Server reported an error: %s' % response.message)
            return response.result
        else:
            # Take annotations message type and verify that it's the same within the collection,
            # as we will publish all the elements with the same topic (as a list or one by one)
            for a in self.annotations:
                if 'msg_type' not in locals():
                    msg_type = a.type
                elif msg_type != a.type:
                    rospy.logerr("Cannot publish annotations of different types (%s, %s)" % (msg_type, a.type))
                    return False

            if 'msg_type' not in locals():
                rospy.logerr('Annotations message type not found? impossible! (we already checked at method start)')
                return False
    
            # Keep the class of the messages to be published; we need it later when deserializing them
            msg_class = roslib.message.get_message_class(msg_type)
            if msg_class is None:
                # This could happen if the message type is wrong or not known for the server (i.e. its
                # package is not on ROS_PACKAGE_PATH). Both cases are really weird in the client side.
                rospy.logerr('Topic type %s definition not found' % topic_type)
                return False
            
            # Advertise a topic with message type topic_type if we will publish results as a list (note that we
            # ignore list's type) or use current annotations type otherwise (we have verified that it's unique) 
            if as_list:
                if topic_type is None:
                    rospy.logerr("Topic type argument is mandatory if as_list is true")
                    return False
                topic_class = roslib.message.get_message_class(topic_type)
                if topic_class is None:
                    # Same comment as in "msg_class is None" applies here
                    rospy.logerr('Topic type %s definition not found' % topic_type)
                    return False
            else:
                topic_class = msg_class
            
            # Advertise a topic to publish retrieved annotations
            objects_pub = rospy.Publisher(topic_name, topic_class, latch=True, queue_size=5)

            # Process retrieved data to build annotations lists
            objects_list = self.getData(msg_class)

            # Publish resulting list
            if as_list:
                objects_pub.publish(objects_list)
            else:
                # if as_list is false, publish objects one by one
                for object in objects_list:
                    objects_pub.publish(object)
        
        return True
