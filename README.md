# Final Stereo Rig Software Package

## Custom_guyi

This ROS2 package controls the GUI that displays on the Jetson Orin screens on boot. The node subscribes to the following topics:
- **debayer/image_raw/rgb** - Camera
- **bar30/depth** - Depth sensor
- **imagenex831l/sonar_health** - Sonar
- **imu/data** - IMU

For the IMU and sonar, the GUI displays "active" or "not active" depending on whether the topics have a publisher. Similarly, the depth sensor will display "not active" if there is no publisher and the depth value if there is. The image from the image topic displays directly on the screen.

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
