# teknofest_industrial_tech_AGV_robot
Teknofest Digital Techs competition codes

This project for Teknofest 2022 digital industrial tech competition which is orhanised by Turkey Ministry of Industry. 

There are mainly 4 nodes that deal with path planning, localization, control and GUI. For hardware selections, NVIDIA Jetson tx2 is used as main computer and odrive version 6.2 is used for driving brushless DC motors. The robot uses differential drive model to control itself, for the localization, rplidar, wheeltec IMU,  odrive encodeers are used. there is a lenovo 720 pixel camera to track lane on the competetion area. it is note that GUI code must be in another host.

competetion rules cover that there 4 four points on the area, two of them are load point and others are unload points. The robot have to carry them from one to another
properly. In addition, according to report of the competition, before each cross point there are 2 qr to localize robot itself. 

for installation;

sudo apt install ros-melodic-serial

pip install paho-mqtt

cd /home/($username)/($catkin_workspace)/src

git clone https://github.com/Jawad-RoboLearn/IMU_Wheeltec.git

git clone https://github.com/Slamtec/rplidar_ros.git

git clone https://github.com/ros-drivers/rosserial.git

git clone https://github.com/ros-perception/slam_gmapping.git

cd /home/($username)/($catkin_workspace)

rosdep install --from-paths src --ignore-src -r -y

catkin_make

then run "roslaunch teknofest_industrial_tech init.launch" 

This project referenced reealy useful robot project is made by James Bruton.
