#!/usr/bin/env python3.8
from typing import Any
import rospy
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose
import tf


def main():
    rospy.init_node("pub_test_custom")
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        top_name = input("topic name : ")
        msg_type = input("msg type : ")
        if top_name == '1':
            rospy.Publisher('dwa_status', Bool, queue_size=1).publish(True)
            continue
        if msg_type == 'string':
            pub = rospy.Publisher(top_name, String, queue_size=1)
            data = input("data : ")
        elif msg_type == 'bool':
            pub = rospy.Publisher(top_name, Bool, queue_size=1)
            data = input("data : ")
            if data == 't':
                data = True
            else:
                data = False
        elif msg_type == 'int':
            pub = rospy.Publisher(top_name, Int32, queue_size=1)
            data = int(input("data : "))
        else:
            trans = 0
            while trans == 0:
                try:
                    trans, rot = listener.lookupTransform('link_base', 'Aruco_L515', rospy.Time(0))
                except Exception as e:
                    print(e)
            print(trans)
            pub = rospy.Publisher(top_name, Pose, queue_size=1)
            data = Pose()
            data.position.x = trans[0]
            data.position.y = trans[1]
            data.position.z = trans[2]
        pub.publish(data)


if __name__ == '__main__':
    main()