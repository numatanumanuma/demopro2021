# demopro2021

ros melodic

~~~
git clone --recursive git@github.com:numatanumanuma/demopro2021.git
~~~

~~~
sudo apt-get install ros-melodic-sound-play
~~~

~~~
sudo apt-get install ros-melodic-rosserial
sudo apt-get install ros-melodic-rosserial-arduino
~~~

~~~
cd ~/catkin_ws/src
git clone https://github.com/openspur/yp-spur.git

sudo apt-get install ros-melodic-urg-node
~~~

## Usage

~~~
$ roslaunch human_disinfector human_disinfector.launch
$ roslaunch human_disinfector ypspur_ros.launch
$ roslaunch human_disinfector media_player.launch
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
~~~

