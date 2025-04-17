
# ----- Part 6: Conversion Node (LaserScan) -----
# File: lidar_converter.py

#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

class LidarConverter:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('lidar_converter', anonymous=True)
        
        # Subscribe to the raw lidar data
        self.lidar_sub = rospy.Subscriber('simulated_lidar', Float32MultiArray, self.lidar_callback)
        
        # Publish converted LaserScan message
        self.scan_pub = rospy.Publisher('simulated_lidar_scan', LaserScan, queue_size=10)
        
        # LaserScan parameters
        self.frame_id = "base_laser"
        self.angle_min = 0.0
        self.angle_max = 2 * 3.14159  # 2π radians (360 degrees)
        self.angle_increment = 2 * 3.14159 / 360  # 1-degree resolution
        self.time_increment = 0.0
        self.scan_time = 0.1  # 10 Hz
        self.range_min = 0.0
        self.range_max = 10.0
        
        rospy.loginfo("Lidar converter initialized")
    
    def lidar_callback(self, msg):
        # Convert Float32MultiArray to LaserScan
        if not msg.data:
            return
        
        scan = LaserScan()
        
        # Set header
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.frame_id
        
        # Set scan parameters
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = self.time_increment
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        
        # Set range data
        scan.ranges = msg.data
        
        # Publish the scan
        self.scan_pub.publish(scan)

if __name__ == '__main__':
    try:
        converter = LidarConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass