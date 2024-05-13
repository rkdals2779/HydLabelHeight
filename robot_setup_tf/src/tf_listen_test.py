#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Pose


def main():
	rospy.init_node('send_current_xyz')
	listener = tf.TransformListener()
	pub = rospy.Publisher('current_pose', Pose, queue_size=1)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0)) 
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		current_pose = Pose()
		current_pose.position.x = trans[0]
		current_pose.position.y = trans[1]
		current_pose.position.z = trans[2]
		current_pose.orientation.x = rot[0]
		current_pose.orientation.y = rot[1]
		current_pose.orientation.z = rot[2]
		current_pose.orientation.w = rot[3]
		# rospy.loginfo("pos x : {} y : {} z : {}\n ori x : {} y : {} z : {} w : {}".format(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]))
		pub.publish(current_pose)
		# rate.sleep()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
