#환경설정 및 실행방법
'''
~/.bashrc 파일에 추가합니다. 
	alias cw='cd ~/catkin_ws'
	alias cs='cd ~/catkin_ws/src'
	alias cm='cd ~/catkin_ws && catkin_make'
	source /opt/ros/melodic/setup.bash
	source ~/catkin_ws/devel/setup.bash
~/catkin_ws에서 "cm"을 실행합니다. 
~/catkin_ws/.catkin 파일 내의 경로를 /home/hsoh/catkin_ws/devel/lib/roboscan_pkg으로 변경합니다. 
roscore 및 rosrun roboscan_pkg roboscan_publish_node 를 실행합니다
'''
