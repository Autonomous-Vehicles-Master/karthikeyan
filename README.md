# Virtual road environments in Vehicle test tracks

This package is to demonstrate the possibiity to reproduce the Real world road environment into test-tracks using Lanelets



### Dependancies

ROS (Assuming Already installed)

Python2 (Assuming Already installed)

Other dependancies can be installed using commands

```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install libboost-dev libeigen3-dev libgeographic-dev libpugixml-dev libpython-dev libboost-python-dev python-catkin-tools
```

### Installation / Building

```
cd
mkdir catkin_ws && cd catkin_ws && mkdir src 
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
cd src
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
git clone https://github.com/fzi-forschungszentrum-informatik/lanelet2.git
git clone https://github.com/coincar-sim/lanelet2_interface_ros.git
git clone https://github.com/KIT-MRT/rosinterface_handler.git
git clone https://github.com/coincar-sim/util_rviz.git
git clone https://github.com/gareth-cross/rviz_satellite.git
git clone https://github.com/Autonomous-Vehicles-Master/karthikeyan.git
cd ..
catkin build
echo “source ~/catkin_ws/devel/setup.bash” >> ~/.bashrc

```

### Launching files
Two methods of simulating the virtual test tracks are done

```
roslaunch virtual_tracks display_with_transform.launch
```

```
roslaunch virtual_tracks display_without_transform.launch
```

To Visualize Path planning / path prediction

```
roslaunch virtual_tracks lanelet_vehicles.launch
```

To Visualize Navigation with dynamic reconfiguration of target GPS,

```
roslaunch virtual_tracks navigation.launch
```
