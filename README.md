# ROS Introduction in Google Colab
## Advanced Robotics with ROS - Module 1


## Getting Started with ROS Using Docker - Step-by-Step Guide
## Here's a practical guide to setting up ROS with Docker on your own laptop, following the activities we discussed for Module 1 of the Advanced Robotics with ROS course.

## Prerequisites

## Docker installed on your system

### Install Docker for Windows
### Install Docker for macOS
### Install Docker for Linux


 Basic familiarity with terminal/command prompt

## Step 1: Pull the ROS Docker Image
## Open a terminal and run:
```bash
docker pull ros:noetic-ros-base-focal
```

# This will download the ROS Noetic image based on Ubuntu Focal.
## Step 2: Create a ROS Workspace
## Create a directory on your host machine to store your ROS files:
```bash
# For Linux/macOS
mkdir -p ~/ros_ws/src

# For Windows (PowerShell)
mkdir -p $HOME/ros_ws/src
```

## Step 3: Initialize Your Workspace
## Run a Docker container to initialize the workspace:
```bash
# For Linux/macOS
docker run --rm -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "cd /ros_ws && mkdir -p src && catkin_init_workspace src"

# For Windows (PowerShell)
docker run --rm -v ${HOME}/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "cd /ros_ws && mkdir -p src && catkin_init_workspace src"
```

## Step 4: Create a ROS Package
## Create a package for our demo:
```bash
# For Linux/macOS
docker run --rm -v ~/ros_ws:/ros_ws -w /ros_ws/src ros:noetic-ros-base-focal bash -c "catkin_create_pkg ros_intro_demo std_msgs rospy"

# For Windows (PowerShell)
docker run --rm -v ${HOME}/ros_ws:/ros_ws -w /ros_ws/src ros:noetic-ros-base-focal bash -c "catkin_create_pkg ros_intro_demo std_msgs rospy"
```

## Step 5: Build the Workspace
## Now build the workspace:
```bash
# For Linux/macOS
docker run --rm -v ~/ros_ws:/ros_ws -w /ros_ws ros:noetic-ros-base-focal bash -c "catkin_make"

# For Windows (PowerShell)
docker run --rm -v ${HOME}/ros_ws:/ros_ws -w /ros_ws ros:noetic-ros-base-focal bash -c "catkin_make"
```

## Step 6: Create Directories for Scripts and Launch Files
```bash
# For Linux/macOS
mkdir -p ~/ros_ws/src/ros_intro_demo/scripts
mkdir -p ~/ros_ws/src/ros_intro_demo/launch

# For Windows (PowerShell)
mkdir -p $HOME/ros_ws/src/ros_intro_demo/scripts
mkdir -p $HOME/ros_ws/src/ros_intro_demo/launch
```
## Step 7: Create Publisher and Subscriber Nodes
## Create these two files in your workspace:
# File: ~/ros_ws/src/ros_intro_demo/scripts/sensor_publisher.py

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import random
import math

class SensorPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('sensor_publisher', anonymous=True)
        
        # Create publishers for simulated sensors
        self.temperature_pub = rospy.Publisher('temperature', Float32, queue_size=10)
        self.distance_pub = rospy.Publisher('distance', Float32, queue_size=10)
        
        # Set the publishing rate (1 Hz)
        self.rate = rospy.Rate(1)
        
        # Initialize sensor values
        self.temperature = 25.0  # starting temperature in Celsius
        self.distance = 1.0  # starting distance in meters
        
        rospy.loginfo("Sensor publisher initialized")
    
    def update_sensor_values(self):
        # Simulate temperature changes (random walk)
        self.temperature += random.uniform(-0.5, 0.5)
        
        # Simulate distance changes (sine wave)
        self.distance = 1.0 + 0.5 * math.sin(rospy.get_time() * 0.5)
    
    def run(self):
        rospy.loginfo("Starting to publish sensor data...")
        
        while not rospy.is_shutdown():
            # Update sensor values
            self.update_sensor_values()
            
            # Publish sensor values
            self.temperature_pub.publish(Float32(self.temperature))
            self.distance_pub.publish(Float32(self.distance))
            
            # Log values occasionally
            rospy.loginfo(f"Published: Temperature={self.temperature:.2f}°C, Distance={self.distance:.2f}m")
            
            # Sleep to maintain the publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = SensorPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
```
# File: ~/ros_ws/src/ros_intro_demo/scripts/sensor_processor.py
```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, String

class SensorProcessor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('sensor_processor', anonymous=True)
        
        # Subscribe to sensor topics
        self.temp_sub = rospy.Subscriber('temperature', Float32, self.temperature_callback)
        self.dist_sub = rospy.Subscriber('distance', Float32, self.distance_callback)
        
        # Publisher for status messages
        self.status_pub = rospy.Publisher('robot_status', String, queue_size=10)
        
        # Store the latest sensor readings
        self.temperature = None
        self.distance = None
        
        # Set the processing rate (1 Hz)
        self.rate = rospy.Rate(1)
        
        rospy.loginfo("Sensor processor initialized")
    
    def temperature_callback(self, msg):
        # Store the temperature data
        self.temperature = msg.data
        rospy.loginfo(f"Received temperature: {self.temperature:.2f}°C")
    
    def distance_callback(self, msg):
        # Store the distance data
        self.distance = msg.data
        rospy.loginfo(f"Received distance: {self.distance:.2f}m")
    
    def process_data(self):
        # Check if we have received data
        if self.temperature is None or self.distance is None:
            return
        
        # Process the data and determine robot status
        status = ""
        
        # Temperature-based status
        if self.temperature > 30.0:
            status += "WARNING: Temperature too high! "
        elif self.temperature < 20.0:
            status += "WARNING: Temperature too low! "
        else:
            status += "Temperature normal. "
        
        # Distance-based status
        if self.distance < 0.7:
            status += "ALERT: Obstacle very close! "
        elif self.distance < 1.2:
            status += "Caution: Obstacle nearby. "
        else:
            status += "Path clear. "
        
        # Publish status
        self.status_pub.publish(String(status))
        rospy.loginfo(f"Published status: {status}")
    
    def run(self):
        rospy.loginfo("Starting to process sensor data...")
        
        while not rospy.is_shutdown():
            # Process the data
            self.process_data()
            
            # Sleep to maintain the processing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        processor = SensorProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass
```

## Step 8: Create a Launch File
## Create this file in your workspace:
# File: ~/ros_ws/src/ros_intro_demo/launch/sensors.launch
```python
<launch>
  <!-- Start the sensor publisher node -->
  <node name="sensor_publisher" pkg="ros_intro_demo" type="sensor_publisher.py" output="screen">
    <!-- Optional parameters could be set here -->
  </node>
  
  <!-- Start the sensor processor node -->
  <node name="sensor_processor" pkg="ros_intro_demo" type="sensor_processor.py" output="screen">
    <!-- Optional parameters could be set here -->
  </node>
</launch>
```
## Step 9: Make the Scripts Executable
```bash
# For Linux/macOS
chmod +x ~/ros_ws/src/ros_intro_demo/scripts/sensor_publisher.py
chmod +x ~/ros_ws/src/ros_intro_demo/scripts/sensor_processor.py

# For Windows (PowerShell)
# Windows doesn't typically need to change file permissions
```
## Step 10: Build the Workspace Again
```bash
# For Linux/macOS
docker run --rm -v ~/ros_ws:/ros_ws -w /ros_ws ros:noetic-ros-base-focal bash -c "catkin_make"

# For Windows (PowerShell)
docker run --rm -v ${HOME}/ros_ws:/ros_ws -w /ros_ws ros:noetic-ros-base-focal bash -c "catkin_make"
```

## Step 11: Run the ROS Master
## Start a Docker container with the ROS master:
```bash
# For Linux/macOS
docker run -it --rm --name ros_master -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && roscore"

# For Windows (PowerShell)
docker run -it --rm --name ros_master -v ${HOME}/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && roscore"
```

## Leave this terminal open.
## Step 12: Run the Publisher Node
## Open a new terminal and run:
```bash
# For Linux/macOS
docker run -it --rm --network container:ros_master -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && rosrun ros_intro_demo sensor_publisher.py"

# For Windows (PowerShell)
docker run -it --rm --network container:ros_master -v ${HOME}/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && rosrun ros_intro_demo sensor_publisher.py"
```

## Step 13: Run the Processor Node
## Open a third terminal and run:
```bash
# For Linux/macOS
docker run -it --rm --network container:ros_master -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && rosrun ros_intro_demo sensor_processor.py"

# For Windows (PowerShell)
docker run -it --rm --network container:ros_master -v ${HOME}/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && rosrun ros_intro_demo sensor_processor.py"
```

## Step 14: Using the Launch File Instead
## Instead of steps 12 and 13, you can use the launch file to start both nodes at once. Open a new terminal and run:
```bash
# For Linux/macOS
docker run -it --rm --network container:ros_master -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && roslaunch ros_intro_demo sensors.launch"

# For Windows (PowerShell)
docker run -it --rm --network container:ros_master -v ${HOME}/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && roslaunch ros_intro_demo sensors.launch"
```

## Step 15: Exploring ROS Tools
## With your nodes running, you can explore different ROS tools in separate terminals:
## List Active Nodes
```bash
docker run --rm --network container:ros_master ros:noetic-ros-base-focal rosnode list
```
## See Messages on a Topic
```bash
docker run --rm --network container:ros_master ros:noetic-ros-base-focal rostopic echo /temperature
```
## Record Data with rosbag
```bash
# For Linux/macOS
docker run --rm --network container:ros_master -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "cd /ros_ws && rosbag record -O sensors.bag /temperature /distance /robot_status"

# For Windows (PowerShell)
docker run --rm --network container:ros_master -v ${HOME}/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "cd /ros_ws && rosbag record -O sensors.bag /temperature /distance /robot_status"
```
## Inspect the Bag File
```bash
# For Linux/macOS
docker run --rm -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "cd /ros_ws && rosbag info sensors.bag"

# For Windows (PowerShell)
docker run --rm -v ${HOME}/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "cd /ros_ws && rosbag info sensors.bag"
```

### Quick Start
## Start ROS Master:
```bash
docker run -it --rm --name ros_master -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && roscore"
```
## Run a ROS Node:
```bash
docker run -it --rm --network container:ros_master -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && rosrun PACKAGE_NAME NODE_NAME"
```
## Launch Multiple Nodes:
```bash
docker run -it --rm --network container:ros_master -v ~/ros_ws:/ros_ws ros:noetic-ros-base-focal bash -c "source /ros_ws/devel/setup.bash && roslaunch PACKAGE_NAME LAUNCH_FILE"
```
## Use ROS Tools:
```bash
docker run --rm --network container:ros_master ros:noetic-ros-base-focal ROS_COMMAND
```