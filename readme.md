# NSL-11100AV ROS1
--- NSL-1110AA ROS1 demo ---

1. Build env
 - Ubuntu18.04
 - ROS1 Melodic
 - OPENCV 4.5.4
 
 
2. Build NSL-1110AA demo
```
$ cd nsd1110_driver
$ catkin_make
$ source ./devel/setup.bash
```
 
3. Start commands
```
$ rosrun roboscan_nsl1110 roboscan_publish_node
$ roslaunch roboscan_nsl1110 camera.launch
```

# NSL-1110AV View
 
![pointcloud](https://github.com/user-attachments/assets/9a46060c-65d9-43a0-92e1-30acf0476286)

# OPENCV View

![imshow](https://github.com/user-attachments/assets/ac2e7f6e-08a6-491b-a085-2062012ebd0c)

# Set parameters
```
$ rqt
 (reconfigure)
```
![rqt](https://github.com/user-attachments/assets/257d3154-760b-4042-9399-fd617464887f)

```

//
```

 



 
 
 
