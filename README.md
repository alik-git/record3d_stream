# record3d_stream
A simple ros package that allows use of the Record3D iphone app as a ROS compatible sensor.

## Installation

Just clone this repo in your workspace folder, typically in the `~/catkin_ws/src` folder. Then follow the standard commands for building ROS pacakges. i.e.
```bash
cd ~/catkin_ws
catkin_make
source ./devel/setup.bash
```

You need to install the record3d python package for whatever python installation ROS is using. So essentially

```bash
/usr/bin/python3 -m pip install record3d
```

## Usage 

You can launch the script using the launch file, so use

```bash
roslaunch record3d_stream record3d_launch.launch 
```

and if an iphone is connected it should detect it and start publishing to the ROS topics `/record3d/depth_image_raw` and 
`/record3d/rgb_image_raw` 

You can also launch the script using rosrun, so thats 
```bash
rosrun record3d_stream record3d_ros.py 
```
