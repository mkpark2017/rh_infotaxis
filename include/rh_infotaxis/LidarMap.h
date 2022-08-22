
#ifndef LIDAR_MAP_H_
#define LIDAR_MAP_H_

#include <vector>
#include <cmath> //M_PI, round
#include <iostream> //cout
#include <sensor_msgs/LaserScan.h>

#include "rh_infotaxis/UavClass.h"

using std::vector;
using std::cout;
using std::endl;

class LidarMap
{
public:
    void initialization(double grid_x, double grid_y, double grid_z, double resolution);
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void lidar_uav_pose_update(UavClass sensing_uav);
    vector<vector<int8_t>>  grid_map;
    double resolution;
    bool trigger;
    bool updated;

private:
    double uav_x;
    double uav_y;
    double uav_z;
};

#endif
