# RM Aerial Simulation

This Repo utilised the work done by [Intelligent Quads](https://github.com/Intelligent-Quads/iq_tutorials)

This Repo is specifically designed to work with the Ardupilot control system, and utilizes the ardupilot gazebo plugin to allow the ardupilot control software to interface and control the model drone in gazebo. 

## Dependencies [Ubuntu 20.04]

Take a look at these tutorials to setup ardupilot, gazebo and the ardupilot gazebo plugin. 

[Installing Ardupilot and MAVProxy](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot_20_04.md)

[Installing QGroundControl](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_qgc.md)

[Installing Gazebo and ArduPilot Plugin](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_gazebo_arduplugin.md)

[Install ROS and Setup Workspace](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_ros_20_04.md)

Installing x-term is recommended as it allows the ardupilot sitl interface to run in a terminal that will cleanly close when closing you sitl instance
```
sudo apt install xterm
```

## Drone Simulations 

This repo contains a couple different gazebo worlds containing various ardupilot drone configurations. 

- `rmuc.world` - a gazebo world containing the DJI 2021 RoboMaster Arena

### Running Drone Simulations 

To launch aerial simulation in RMUC world, run the following
```
roslaunch aerial rmuc.launch
``` 
In another terminal, launch the ardupilot instance by running 
```
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console
``` 
If sim_vehicle.py is not found, try
``` 
cd ~/ardupilot/Tools/autotest && python3 sim_vehicle.py -v ArduCopter -f gazebo-iris --console
``` 
For more information, take a look at the corresponding tutorials [here](https://github.com/Intelligent-Quads/iq_tutorials)

## Others

### drone1-12
This repo hosts a few helpful gazebo models you can use to build upon. Included in this repo are 12 drones which contain the ardupilot plugin. Each ardupilot plugin is staggered such that you can simulate multiple unique aircraft using ardupilot's SITL. For more information please see the iq_tutorial on [ardupilot drone swarming](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/swarming_ardupilot.md)

### drone_with_lidar

Example drone that has a 2d lidar attached. The lidar plugin publishes a `sensor_msgs/LaserScan` ROS msg containing return data.

topics
```
/spur/laser/scan
```

![drone_with_lidar](docs/imgs/drone_with_lidar.png)

### drone_with_sonar

Example drone with 4 sonars attached in each direction. Each sonar publishes a `sensor_msgs/Range` ROS msg containing range data.

topics 
```
/drone1/sensor/sonar/back
/drone1/sensor/sonar/front
/drone1/sensor/sonar/left
/drone1/sensor/sonar/right
```
![drone_with_sonar](docs/imgs/drone_with_sonar.png)

### drone_with_camera 

Example drone with a forward facing camera. The camera published a `sensor_msgs/Image` ROS msg which can be used to view or do image processing on. 

![drone_with_camera](docs/imgs/drone_with_camera.png)
