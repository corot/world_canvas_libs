#!/usr/bin/env python

import rospy

import world_canvas_client

from yocs_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('test_annotation_collection')
    topic_name  = rospy.get_param('~topic_name', 'annotations')
    topic_type  = rospy.get_param('~topic_type', None)
    pub_as_list = rospy.get_param('~pub_as_list', False)
    world    = rospy.get_param('~world')
    ids      = rospy.get_param('~ids', [])
    names    = rospy.get_param('~names', [])
    types    = rospy.get_param('~types', [])
    keywords = rospy.get_param('~keywords', [])
    related  = rospy.get_param('~relationships', [])

    ac = world_canvas_client.AnnotationCollection(world, ids, names, types, keywords, related)
    ac.loadData()
    walls = ac.getData(yocs_msgs.msg.Wall)

    # Publish annotations' visual markers on client side
    ac.publishMarkers('annotation_markers')

    # Publish annotations on client side
    ac.publish(topic_name + '_client', topic_type, False, pub_as_list)

    # Request server to also publish the same annotations
    ac.publish(topic_name,             topic_type, True,  pub_as_list)

    rospy.loginfo("Done")
    rospy.spin()
