#!/usr/bin/python

#
# Similar to static_transform_broadcaster, this node constantly publishes
# static odometry information (Odometry msg and tf). This can be used
# with fake_localization to evaluate planning algorithms without running
# an actual robot with odometry or localization
#
# Author: Armin Hornung
# License: BSD

import rospy

from tf2_ros.transform_broadcaster import TransformBroadcaster
from transforms3d.euler import euler2quat
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3
)
from std_msgs.msg import Header



def publishOdom():
    rospy.init_node('fake_odom')
    base_frame_id = rospy.get_param("~base_frame_id", "base_link")
    odom_frame_id = rospy.get_param("~odom_frame_id", "odom")
    publish_frequency = rospy.get_param("~publish_frequency", 10.0)
    pub = rospy.Publisher('odom', Odometry)
    tf_pub = TransformBroadcaster()

    #TODO: static pose could be made configurable (cmd.line or parameters)
    quat = euler2quat(0, 0, 0)
    quat = Quaternion(quat[1], quat[2], quat[3], quat[0])

    odom = Odometry()
    odom.header.frame_id = odom_frame_id
    odom.pose.pose = Pose(Point(0, 0, 0), quat)

    rospy.loginfo("Publishing static odometry from \"%s\" to \"%s\"", odom_frame_id, base_frame_id)
    r = rospy.Rate(publish_frequency)
    while not rospy.is_shutdown():
        odom.header.stamp = rospy.Time.now()
        pub.publish(odom)
        t_stamped = TransformStamped(
            Header(stamp=odom.header.stamp, frame_id=odom_frame_id),
            base_frame_id,
            Transform(Vector3(0, 0, 0), quat)
        )
        tf_pub.sendTransform(t_stamped)
        r.sleep()


if __name__ == '__main__':
    try:
        publishOdom()
    except rospy.ROSInterruptException:
        pass
