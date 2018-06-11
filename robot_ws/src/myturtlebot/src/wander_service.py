#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from myturtlebot.srv import *

class ReadScan:
    
    range_value = 1 # anything to start
    driving_forward = True

    def query_range(self):
        return self.range_value

    def scan_callback(self, msg):
        import math
        self.range_value = min(msg.ranges)
        if math.isnan(self.range_value):
            self.range_value = 0
        #rospy.loginfo(self.range_value)
    
    def service_callback(self, rqst):
        rospy.loginfo("Requesting to change status? "+str(rqst.forwardrqst))
        self.driving_forward = rqst.forwardrqst
        ans = TurtlebotStatusResponse()
        ans.range = self.range_value
        ans.forwardresp = self.driving_forward
        return ans
        
range_sensor = ReadScan()

scan_sub = rospy.Subscriber('scan', LaserScan, range_sensor.scan_callback)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

status_srv = rospy.Service('turtlebot_status', TurtlebotStatus, range_sensor.service_callback)

rospy.init_node('wander_service')
state_change_time = rospy.Time.now()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if range_sensor.driving_forward:
        scan_value = range_sensor.query_range()
        if (scan_value < 0.8) or (rospy.Time.now() > state_change_time):
            range_sensor.driving_forward = False
            state_change_time = rospy.Time.now() + rospy.Duration(0.5)
    else: # we're not driving_forward
        if rospy.Time.now() > state_change_time:
            range_sensor.driving_forward = True # we're done spinning, time to go forward!
            state_change_time = rospy.Time.now() + rospy.Duration(5)

    twist = Twist()
    if range_sensor.driving_forward:
        twist.linear.x = 0.5
        twist.angular.z = 0
    else:
        twist.linear.x = 0
        twist.angular.z = 0.1
    
    cmd_vel_pub.publish(twist)
    
    rate.sleep()
