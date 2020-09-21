#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from geometry_msgs.msg import Twist
def talker():
    #publish geometry_msgs/Twist to /mobile_base/commands/velocity
    velocity_publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    #node name straight_line
    rospy.init_node('straight_line', anonymous=True)
    vel_msg = Twist()
    #These are constant for a straight line
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0 
    #Move in a circle
    while not rospy.is_shutdown():
	
        t0 = rospy.Time.now().to_sec()
	time.sleep(3)
	#sets velocity
    	vel_msg.linear.x = -0.2
	vel_msg.angular.z = 0.2
	#publishes messages
	for i in range(0,900):
        	velocity_publisher.publish(vel_msg)
		time.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
