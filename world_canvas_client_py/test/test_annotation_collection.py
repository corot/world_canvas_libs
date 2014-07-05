#!/usr/bin/env python

import roslib; roslib.load_manifest('world_canvas_client_py')
import rospy

import world_canvas_client

if __name__ == '__main__':
    rospy.init_node('objects_loader')
    topic_name  = rospy.get_param('~topic_name', 'annotations')
    topic_type  = rospy.get_param('~topic_type', None)
    pub_as_list = rospy.get_param('~pub_as_list', False)
    world_id = rospy.get_param('~world_id')
    ids      = rospy.get_param('~ids', [])
    names    = rospy.get_param('~names', [])
    types    = rospy.get_param('~types', [])
    keywords = rospy.get_param('~keywords', [])
    related  = rospy.get_param('~relationships', [])

    ac = world_canvas_client.AnnotationCollection(world_id, ids, names, types, keywords, related)
    ac.loadData()

    # Publish annotations' visual markers on client side
    ac.publishMarkers('annotation_markers')

    # Publish annotations on client side
    ac.publish(topic_name + '_client', topic_type, False, pub_as_list)

    # Request server to also publish the same annotations
    ac.publish(topic_name,             topic_type, True,  pub_as_list)

    rospy.loginfo("Done")
    rospy.spin()
