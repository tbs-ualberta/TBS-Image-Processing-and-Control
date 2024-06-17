#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # For turtlesim
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    turtle_name = rospy.get_param('turtle', 'turtle2')
    spawner(4, 2, 0, turtle_name)

    turtle_vel = rospy.Publisher('%s/cmd_vel' % turtle_name, geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            
            # compute transform from turtle1 to turtle2 (basic)
            # trans = tfBuffer.lookup_transform('turtle2',            # transform from this frame
            #                                   'carrot1',            # to this frame
            #                                   rospy.Time.now(),     # for the frame at this time
            #                                   rospy.Duration(1.0))  # timeout

            # go to where turtle was 5 seconds ago (advanced)
            past = rospy.Time.now() - rospy.Duration(5.0)
            trans = tfBuffer.lookup_transform_full(
                target_frame=turtle_name,       # From this frame,
                target_time=rospy.Time.now(),   # at this time,
                source_frame='carrot1',         # to this frame,
                source_time=past,               # at this time.
                fixed_frame='world',            # frame that does not change over time
                timeout=rospy.Duration(1.0)     # timeout
            )

            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        msg = geometry_msgs.msg.Twist()

        msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

        turtle_vel.publish(msg)

        rate.sleep()