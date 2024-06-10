# Final Stereo Rig Software Package

## Custom_guyi

This ROS2 package controls the GUI that displays on the Jetson Orin screens on boot. The node subscribes to the following topics:
- **debayer/image_raw/rgb** - Camera
- **bar30/depth** - Depth sensor
- **imagenex831l/sonar_health** - Sonar
- **imu/data** - IMU

For the IMU and sonar, the GUI displays "active" or "not active" depending on whether the topics have a publisher. Similarly, the depth sensor will display "not active" if there is no publisher and the depth value if there is. The image from the image topic displays directly on the screen. The gui determines if the orin is connected by pinging the ip address of the other and waiting for a response. If there is no response the orin connection is not active, otherwise it is active.

## Flir_camera_driver

This ROS2 package connects to the camera and publishes a Bayer image of the scene. This package is maintained by the [ros-drivers](https://github.com/ros-drivers/flir_camera_driver/tree/humble-devel) on GitHub.

## Imagenex831l_ros2

This ROS2 package connects to the sonar and publishes both `raw_range` and `processed_range` custom messages. Originally created by Alberto Quattrini Li for ROS1 Melodic in Python 2, the old package can be found [here](https://github.com/quattrinili/imagenex831l).

This package has been converted to ROS2. The parameters for the node can be found in `/cfg/sonar.yaml`. These parameters are automatically loaded from this file when the launch file is called. Additionally, a `sonar_health` topic was created to publish the connection status of the sonar (active if the sonar is properly connected and not active if there is no connection). The node publishes the sonar's range at a rate of 1/frequency, which can be set in the YAML file.

## Microstrain_inertial

This ROS2 package connects to a Microstrain IMU and is a slight modification of their ROS2 package, which can be found [here](https://github.com/LORD-MicroStrain/microstrain_inertial/tree/ros2).

This package contains a few differences from the original package. Firstly, when the `c7_launch` file is called, RViz does not launch at the same time. Secondly, no namespace is defined, which allows the `ros2_bringup` package to declare a namespace for the system.

## ms5837_bar_ros

This ROS2 package connects to the BlueRobotics Bar30 depth sensor and publishes the data from the sensor. This node was originally created by tasada038, and the most up-to-date package can be found [here](https://github.com/tasada038/ms5837_bar_ros).

## ros2_bringup

This ROS2 package serves two major purposes: 
1. Launch all the required packages for the stereo rig.
2. Launch a debayer node to successfully convert the images produced from the `flir_camera_driver` node to both regular RGB images and compressed RGB images.

The package consists of two launch files, one for Jetson 1 and one for Jetson 2. The node requires five parameters:
- **namespace**
- **camera_type**
- **serial**
- **sonar**
- **cam_topic**

The `camera_type` and `serial` parameters are fed into the `flir_camera_driver` package and are required for the camera to properly connect to the rest of the system. The `sonar` parameter indicates whether the system should launch the sonar node. The `cam_topic` parameter specifies which topic the GUI should subscribe to in order to display the image. The `namespace` parameter declares the namespace for the system.

The `ros2_bringup` package also starts the rosbag and records all the necessary topics.

### Instructions
To launch:
1. Build all required packages:
   - **custom_guyi**
   - **flir_camera_driver**
   - **imagenex831l_ros2**
   - **microstrain_inertial**
   - **ms5837_bar_ros**
   - **ros2_bringup**
2. Source `install/setup.bash`
3. Run the appropriate launch command based on the device

#### Jetson_1:

``ros2 launch ros2_bringup stereorig_1.launch.py serial:="'21502646'" sonar:='false' namespace:='jetson_1' cam_topic:="debayer/image_raw/rgb" camera_type:="blackfly_s"``

#### Jetson_2:

``ros2 launch ros2_bringup stereorig_2.launch.py serial:="'21387972'" sonar:='true'  namespace:='jetson_2' camera_type:="blackfly_s" cam_topic:="debayer/image_raw/rgb"``

## Connecting to and Pulling bag from Orin

Due to the sonar, the jetsons have a pre-set ip address of 192.168.0.100 (jetson_2) and 192.168.0.150 (jetson_1). Therefore, the easiest way to connect to the orins is to change your computer's ip address to contain the same subnet (ex: 192.168.0.250). I have provided in instructions to both ssh to the orins and scp to pull the bags below.

### ssh to orins
1. Change IP adress of host computer
2. connect via ethernet to orin
3. ping orin to confirm connection
4. ssh -p 22 afrl@_ip adress of orin_
5. input password for afrl user

### scp to retrieve bags
1. Change IP adress of host computer
2. connect via ethernet to orin
3. ping orin to confirm connection
4. scp -P 22 afrl@_ip adress of orin_:_path to bag file on orin_ _path to desired directory for bag_
5. input password for afrl user
