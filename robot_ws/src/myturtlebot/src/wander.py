#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReadScan:
    
    range_value = 1 # anything to start

    def query_range(self):
        return self.range_value

    def scan_callback(self, msg):
        import math
        self.range_value = min(msg.ranges)
        if math.isnan(self.range_value):
            self.range_value = 0
        rospy.loginfo(self.range_value)

range_sensor = ReadScan()
scan_sub = rospy.Subscriber('scan', LaserScan, range_sensor.scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now()
driving_forward = True
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
        scan_value = range_sensor.query_range()
        if (scan_value < 0.8):# or rospy.Time.now() > state_change_time):
            driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(0.5)
    else: # we're not driving_forward
        if rospy.Time.now() > state_change_time:
            driving_forward = True # we're done spinning, time to go forward!
            state_change_time = rospy.Time.now() + rospy.Duration(30)

    twist = Twist()
    if driving_forward:
        twist.linear.x = 0.5
        twist.angular.z = 0
    else:
        twist.linear.x = 0
        twist.angular.z = 0.1
    
    cmd_vel_pub.publish(twist)
    
    rate.sleep()
