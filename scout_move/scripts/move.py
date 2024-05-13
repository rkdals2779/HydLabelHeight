#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move():
    rospy.init_node('scout_move', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    msg = Twist()

    print("input speed within 0.0 ~ 0.22")
    speed = input("Input your speed: ")
    
    print("input distance within 0.0 ~ 2.84")
    distance = input("Type your distance: ")
    
    print("direction foward:1, backward:0")
    isForward = int(input("Forward?: ")) # True or False(1 or 0)

    if(isForward):
        print(f"forward")
        msg.linear.x =  abs(float(speed))
    else:
        print(f"msg.linear.x = {-abs(float(speed))}")
        msg.linear.x = -abs(float(speed))
        
    msg.linear.y  = msg.linear.z  = 0
    msg.angular.x = msg.angular.y = msg.angular.z = 0

    while not rospy.is_shutdown():
        
        distance = float(distance)
        speed = float(speed)
        duration = distance / speed
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        pub.publish(msg)
        rospy.sleep(0.001)
        
        while(rospy.Time.now() < time2end):
            pass    
        
        msg.linear.x = 0
        pub.publish(msg)

if __name__ == '__main__':
    try:
        move()
        rospy.spin()
    except rospy.ROSInterruptException: pass
