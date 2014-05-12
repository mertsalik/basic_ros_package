basic_ros_package
=================

Package for motion planning workouts.

The package contains implementations of [Probabilistic Road Map (PRM)](http://www.isi.edu/robots/CS561/Lectures/Probabilisticpath.pdf) and [Rapidly exploring Random Tree (RRT)](http://en.wikipedia.org/wiki/Rapidly_exploring_random_tree) motion planning algorithms on a custom ROS map using rosbuild. 


setup
=================


 - Install [Box2D](https://code.google.com/p/box2d/downloads/list)
 - in Build folder, run :

 > `cmake ..`
 
 > `sudo make`
 
 > `sudo make install`

 - for CUSTOM MAP, merge opt/ folder with yours.

build
==================

 - In Box2DHelper folder run:

 > `make clean && make`

 - In BLG456EAssignment3 folder run :

 > `make clean && make`


run 
==================

 - In BLG456EAssignment3 folder run:

 > `roslaunch BLG456EAssignment3 startup.launch`
