#!/usr/bin/env python3.8
import rospy
from std_msgs.msg import String

# status = "zero"

# def status_callback(msg):
#     global status
#     status = msg

# def main():
#     rospy.init_node("control")
#     rospy.Subscriber('status_sub', String, status_callback)
#     status_pub = rospy.Publisher('status_pub', String, queue_size=1)
#     global status
#     print("control")

#     while not rospy.is_shutdown():
#         if status == "zero":
#             status = "grab"
#             print(1)
#             status_pub.publish(status)

#         if status == "grab_fin":
#             print("Point plz")

#         if status == "dwa_fin":
#             status = "release"
#             status_pub.publish(status)

#         if status == "release_fin":
#             print("Good Job")


class Control:
    def __init__(self, status = "zero") -> None:
        self.pub = rospy.Publisher("status_pub", String, queue_size=1)
        self.status = status
    
    def pub_status(self, msg):
        self.status = msg
        self.pub.publish(msg)


def main():
    rospy.init_node("control")
    control = Control()
    rospy.Subscriber("status_sub", String, control.pub_status)
    while control.status == "zero":
        control.pub.publish("grab")
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
