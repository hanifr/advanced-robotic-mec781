Exercise Components

Publisher Node (sensor_publisher.py)

Simulates a mobile robot with LIDAR and IMU sensors
Creates a virtual environment with random obstacles
Publishes simulated sensor data to ROS topics
Responds to velocity commands to update robot position


Subscriber Node (sensor_processor.py)

Subscribes to the simulated sensor data
Processes LIDAR data for obstacle detection
Implements a basic obstacle avoidance algorithm
Analyzes IMU data for unusual motion
Toggles between autonomous and manual control


Launch File (sensor_simulation.launch)

Configures the complete system startup
Includes RViz for visualization
Sets necessary parameters


Visualization Converter (lidar_converter.py)

Converts custom sensor data to standard ROS message types
Enables visualization in RViz


Student Tasks

Specific implementation activities
Analysis tasks to understand the system
Extension opportunities for advanced students
Reporting requirements



Teaching Notes
This exercise reinforces key ROS concepts from Module 1:

Node communication through publishers and subscribers
Message passing and data processing
Launch file configuration
Sensor data analysis
Basic autonomous behavior

Students will gain hands-on experience with:

ROS node development
Python programming for robotics
Sensor data processing algorithms
Autonomous navigation concepts
System integration