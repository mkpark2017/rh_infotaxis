# rh-infotaxis

### 1. Build package
```
$ mkdir -p ~/catkin_ws/rh_infotaxis/src
$ cd ~/catkin_ws/rh_infotaxis/src
$ git clone $this_package
$ cd ..
$ catkin build
$ echo "source ~/catkin_ws/rh_infotaxis/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### 2. Run node
```
$ roslaunch rh_infotaxis rh_infotaxis.launch --screen
```

### 3. Run matlab HILS_ENV
```
$ roscd rh_infotaxis
$ cd Env
$ matlab
```
* Note, you need to change roscore url in matlab code
* hils_env.m (only matlab is required)
* hils_env_with_gazebo.m (matlab and px4 sitl are required)

