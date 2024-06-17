#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def handle_turtle_pose(msg, turtlename):
    br = tf2_ros.TransformBroadcaster()         # broadcaster
    t = geometry_msgs.msg.TransformStamped()    # transfrom

    t.header.stamp = rospy.Time.now()   # assign timestamp
    t.header.frame_id = "world"         # assign parent frame of link
    t.child_frame_id = turtlename       # assign child frame
    t.transform.translation.x = msg.x   # x translation
    t.transform.translation.y = msg.y   # y translation
    t.transform.translation.z = 0.0     # z translation
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta) # transform to quaternion
    t.transform.rotation.x = q[0]   
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename,
                     turtlesim.msg.Pose,
                     handle_turtle_pose,
                     turtlename)
    rospy.spin()