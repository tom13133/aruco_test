# ROS package to accomplish target center estimation.

This package is to estimate the target center from ***camera*** messages.  

## Dependencies
1. Eigen3

## Content
1. Target center from Camera (sensor_msgs::ImageConstPtr)

## 1. Target center from Camera (sensor_msgs::ImageConstPtr)
In this module, we use ***ArUco*** marker for detection.

### (a) Setup
Before we start running the estimation module, there is two configuration files needed to be specfied.  

1. path:  
Their path are  
```
~/aruco_test/config/camera_config.yaml
```

2. format:  
**camera_config.yaml** are used for launching **CameraProcessor_node**.  

**camera_config.yaml**
> topic: "/usb_cam/image_raw"
> intrinsic_parameter:  
>   fx: 660.475770  
>   fy: 660.431370  
>   cx: 286.997907  
>   cy: 235.907360  
> distortion_parameter:  
>   k: [0.088377, -0.202261, 0]  
>   p: [-0.000519, -0.003010]  
> edge_length: 0.4  

**topic** specifies the topic name of images
**intrinsic_parameter** is the intrinsic parameters of camera.  
**distortion_parameter** is the distortion parameters of camera.   
**edge_length** is the edge length of ArUco marker.  


### (b) Getting Started.
1. Launch the node  
```
roslaunch target_processing camera.launch
```

2. Play the bag, then start processing.  
After bag is finished, one output file **camera_data_raw.csv** would be generated in ```~/aruco_test/data/```. It contains the ArUco marker id and target center with timestamp:  
(**List order**: time_stamp, id, x, y, z)   

* Notice that if the image message is **compressed** type, we neet to republish image by using package **image_transport**, which can be specified in ***camera.launch***.  
