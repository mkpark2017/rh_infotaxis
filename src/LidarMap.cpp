#include "rh_infotaxis/LidarMap.h"

void LidarMap::initialization(double grid_x, double grid_y, double grid_z, double grid_resolution)
{
    for(int i=0; i<grid_y; i++)
    {
        vector<int8_t> grid_map_1d;
        for(int j=0; j<grid_x; j++)
        {
            grid_map_1d.push_back(50);
        }
        grid_map.push_back(grid_map_1d);
    }

    resolution = grid_resolution;
    updated = true;
    trigger = false;
}

void LidarMap::lidar_uav_pose_update(UavClass sensing_uav)
{
    uav_x = (sensing_uav.x/resolution)-1;
    uav_y = (sensing_uav.y/resolution)-1;
    uav_z = (sensing_uav.z/resolution)-1;
}

void LidarMap::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    updated = false;

    if(trigger && uav_z < round(30/resolution -1))
    {
        float a_min = (msg->angle_min) / 180 * M_PI;
        float a_max = (msg->angle_max) / 180 * M_PI;
        float a_gap = (msg->angle_increment) / 180 * M_PI;
        //float r_min = msg->range_min;
        float r_max = msg->range_max;
        int laser_num = (a_max-a_min)/a_gap;

        double ranges[laser_num];

        int map_nx = grid_map.size();
        int map_ny = grid_map[0].size();

        for(int i=0; i<laser_num; i++)
        {
            ranges[i] = (msg->ranges[i])/resolution; //do i have to round it?
            double laser_angle = a_min + a_gap*i;
	    int grid_x, grid_y;
            for(int j=0; j<=(int)ranges[i]; j++)
            {
                grid_x = round(uav_x + cos(laser_angle)*(j+0.1));
                grid_y = round(uav_y + sin(laser_angle)*(j+0.1));
                //cout << "grid_x: " << grid_x << " grid_y: " << grid_y << endl;
                if(grid_x <0 || grid_y < 0 || grid_x >= map_nx || grid_y >= map_ny)
                    break;

                grid_map[grid_y][grid_x] = 0;

                if(j==(int)ranges[i])
                {
                    if(ranges[i]*resolution < r_max)
                    {
                        if(grid_y>=0 && grid_y< map_ny && grid_x>=0 && grid_x< map_nx)
                            grid_map[grid_y][grid_x] = 100;
                    }
                }
            }

        }
        updated = true;
    }
    else if(trigger)
    {
        cout << "laser_num: " << msg->ranges.size() << endl;
        if(msg->ranges.size() > 0)
        {
            grid_map[uav_y][uav_x] = 100;
        }
        else
            grid_map[uav_y][uav_x] = 0;
        updated = true;
    }
}
