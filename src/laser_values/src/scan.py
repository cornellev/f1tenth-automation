#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class LidarBasedSteering:
    def __init__(self):
        rospy.init_node('lidar_based_steering')

        self.max_steering_angle = rospy.get_param('~max_steering_angle', 1.0)
        self.max_speed = rospy.get_param('~max_speed', 10.0)

        self.drive_pub = rospy.Publisher('/rand_drive_topic', AckermannDriveStamped, queue_size=10)

        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

    def lidar_callback(self, msg):
        left_scan = min(msg.ranges[:540]) # left half
        right_scan = min(msg.ranges[540:]) # right half
        #min to find the closest to wall 
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()

        if left_scan < right_scan:
            drive_msg.drive.steering_angle = -self.max_steering_angle
        else:
            drive_msg.drive.steering_angle = self.max_steering_angle

        drive_msg.drive.speed = 1.0

        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        LidarBasedSteering()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# def callback(msg):
#     print(msg.ranges[180])
#     print(msg.ranges[90])
#     print(msg.ranges[270])

# rospy.init_node('random_walk')
# sub = rospy.Subscriber('/scan', LaserScan, callback)
# rospy.spin()
