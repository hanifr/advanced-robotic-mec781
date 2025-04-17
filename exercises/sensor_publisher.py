# ROS Publisher-Subscriber Exercise
# Advanced Robotics with ROS - Module 1
# 
# This exercise guides students through creating a basic ROS publisher and subscriber system
# that simulates robot sensor data processing.

# ----- Part 1: Publisher Node -----
# File: sensor_publisher.py

#!/usr/bin/env python

import rospy
import random
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class SensorPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('sensor_publisher', anonymous=True)
        
        # Create publishers for simulated sensors
        self.lidar_pub = rospy.Publisher('simulated_lidar', Float32MultiArray, queue_size=10)
        self.imu_pub = rospy.Publisher('simulated_imu', Float32MultiArray, queue_size=10)
        
        # Subscribe to velocity commands to simulate robot motion affecting sensor readings
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # Set the publishing rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Initialize simulated robot state
        self.position = [0.0, 0.0]  # x, y in meters
        self.orientation = 0.0  # theta in radians
        self.linear_velocity = 0.0  # m/s
        self.angular_velocity = 0.0  # rad/s
        
        # Initialize simulated environment with random obstacles
        # [x, y, radius] for each obstacle
        self.obstacles = []
        for _ in range(5):
            x = random.uniform(-10, 10)
            y = random.uniform(-10, 10)
            radius = random.uniform(0.5, 2.0)
            self.obstacles.append([x, y, radius])
        
        rospy.loginfo("Sensor publisher initialized with %d obstacles", len(self.obstacles))
    
    def cmd_vel_callback(self, msg):
        # Store the commanded velocities
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
    
    def update_robot_state(self):
        # Simple motion model to update robot position and orientation
        dt = 0.1  # seconds (matches the 10 Hz rate)
        
        # Update orientation
        self.orientation += self.angular_velocity * dt
        self.orientation = self.normalize_angle(self.orientation)
        
        # Update position
        self.position[0] += self.linear_velocity * math.cos(self.orientation) * dt
        self.position[1] += self.linear_velocity * math.sin(self.orientation) * dt
    
    def normalize_angle(self, angle):
        # Normalize angle to [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def simulate_lidar(self):
        # Simulate a 2D lidar with 360 degrees FOV and 1-degree resolution
        num_rays = 360
        max_range = 10.0  # meters
        
        lidar_data = Float32MultiArray()
        ranges = []
        
        for i in range(num_rays):
            # Calculate ray angle
            ray_angle = self.normalize_angle(self.orientation + math.radians(i))
            
            # Start with maximum range
            ray_range = max_range
            
            # Check intersection with each obstacle
            for obs_x, obs_y, obs_radius in self.obstacles:
                # Vector from robot to obstacle center
                dx = obs_x - self.position[0]
                dy = obs_y - self.position[1]
                
                # Direction vector of the ray
                ray_dx = math.cos(ray_angle)
                ray_dy = math.sin(ray_angle)
                
                # Project obstacle vector onto ray direction
                proj = dx * ray_dx + dy * ray_dy
                
                # Skip if obstacle is behind the robot
                if proj < 0:
                    continue
                
                # Find closest point on ray to obstacle center
                closest_x = self.position[0] + proj * ray_dx
                closest_y = self.position[1] + proj * ray_dy
                
                # Distance from closest point to obstacle center
                dist_to_center = math.sqrt((closest_x - obs_x)**2 + (closest_y - obs_y)**2)
                
                # If ray intersects obstacle
                if dist_to_center <= obs_radius:
                    # Calculate intersection points using Pythagorean theorem
                    half_chord = math.sqrt(obs_radius**2 - dist_to_center**2)
                    
                    # Distance to first intersection
                    intersection_dist = proj - half_chord
                    
                    # Update ray range if this intersection is closer
                    if 0 <= intersection_dist < ray_range:
                        ray_range = intersection_dist
            
            # Add some noise to the measurement
            ray_range += random.gauss(0, 0.05)  # 5cm std dev noise
            ray_range = max(0.0, min(ray_range, max_range))  # Clamp to valid range
            
            ranges.append(ray_range)
        
        lidar_data.data = ranges
        return lidar_data
    
    def simulate_imu(self):
        # Simulate IMU data: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
        imu_data = Float32MultiArray()
        
        # Simple acceleration model based on velocity changes
        # In a real robot, this would be much more complex
        accel_x = self.linear_velocity * math.cos(self.orientation)
        accel_y = self.linear_velocity * math.sin(self.orientation)
        accel_z = 9.81  # Gravity
        
        # Angular velocities
        gyro_x = 0.0  # Assuming planar motion
        gyro_y = 0.0  # Assuming planar motion
        gyro_z = self.angular_velocity
        
        # Add noise
        accel_x += random.gauss(0, 0.1)
        accel_y += random.gauss(0, 0.1)
        accel_z += random.gauss(0, 0.1)
        gyro_x += random.gauss(0, 0.01)
        gyro_y += random.gauss(0, 0.01)
        gyro_z += random.gauss(0, 0.01)
        
        imu_data.data = [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]
        return imu_data
    
    def run(self):
        # Main publishing loop
        while not rospy.is_shutdown():
            # Update robot state based on velocities
            self.update_robot_state()
            
            # Simulate and publish lidar data
            lidar_data = self.simulate_lidar()
            self.lidar_pub.publish(lidar_data)
            
            # Simulate and publish IMU data
            imu_data = self.simulate_imu()
            self.imu_pub.publish(imu_data)
            
            # Log current robot state occasionally
            if random.random() < 0.02:  # ~2% chance each iteration
                rospy.loginfo("Robot position: (%.2f, %.2f), orientation: %.2f rad",
                             self.position[0], self.position[1], self.orientation)
            
            # Sleep to maintain the publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = SensorPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass

