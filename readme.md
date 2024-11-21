# NSL-11100AA ROS1
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

# NSL-1110AA View

 ![aaa](https://user-images.githubusercontent.com/106071093/226831747-71e4c269-0fa9-483a-b781-78ac131eaf6b.png)

# OPENCV View

  ![phase2](https://github.com/nano-roboscan/NSL-3130AA-ROS2/assets/106071093/4a134327-213f-4558-9fa7-47de543866c9)

# Set parameters
```
$ rqt
 (reconfigure)
```

![bbbb](https://user-images.githubusercontent.com/106071093/226831796-d487fc42-5ae4-40c4-b5f9-e4f18af08d7c.png)


```

//
```

 



 
 
 
