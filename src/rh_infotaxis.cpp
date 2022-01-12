#include "rh_infotaxis/ParticleFilter.h"
#include "rh_infotaxis/EnvClass.h"
#include "rh_infotaxis/UavClass.h"
#include "rh_infotaxis/LidarMap.h"
#include "rh_infotaxis/DecisionMaker.h"
/*--------------------------*/
//#include <stdr_msgs/AddCO2Source.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
/*--------------------------*/
#include <ros/ros.h> //ALWAYS need to include this
#include <iostream> // cout
#include <string> // string
#include <chrono> // high_resolution_clock, std::chrono::duration
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>


using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration;

EnvClass env;
ParticleFilter pf;
UavClass current_uav;
UavClass sensing_uav;
UavClass next_goal;
void currentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_uav.x = msg->pose.position.x;
    current_uav.y = msg->pose.position.y;
    current_uav.z = msg->pose.position.z;
}

void gasMeasureCallback(const sensor_msgs::ChannelFloat32::ConstPtr& msg)
{
    current_uav.sensor_value = msg->values[0];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    //-----------------------------------domain setup-------------------------------------
    double source[8], domain[5];
    n.param("source_x",   source[0], 204.0);
    n.param("source_y",   source[1], 210.0);
    n.param("source_z",   source[2], 11.0);
    n.param("source_u",   source[3], 3.0);
    n.param("source_q",   source[4], 7.2*1000*1000/60/60); //  7.2kg/h = 2000mg/s
    n.param("source_phi", source[5], 290*M_PI/180);
    n.param("source_d",   source[6], 10.0);
    n.param("source_tau", source[7], 1000.0);

    n.param("nx",         domain[0], 270.0);
    n.param("ny",         domain[1], 270.0);
    n.param("nz",         domain[2], 45.0);
    n.param("dt",         domain[3], 1.0);
    n.param("env_sig",    domain[4], 2.0);
    env.initialization(source, domain);
    //----------------------------------particle filter setup---------------------------
    int num_particle = 1000;
    pf.initialization(num_particle, env);
    //---------------------------------------gas sensing----------------------------------
    double sen_param[8];
    n.param("sensing_sig_multiple",         sen_param[0], 0.1);
    n.param("sensing_horizontal_range",     sen_param[1], 3.0);
    n.param("sensing_vertical_range",       sen_param[2], 1.0);
    n.param("sensing_time",                 sen_param[3], 0.1); // seconds, 0=just one sample
    n.param("binary_threshold_upate_ratio", sen_param[4], 0.8);
    n.param("uav_decision_number",          sen_param[5], 6.0); // 4=2d (horizontal), 6=3d
    n.param("uav_distance_horizontal",      sen_param[6], 9.0);
    n.param("uav_distance_vertical",        sen_param[7], 9.0);
    sensing_uav.initialization(sen_param);

    double sensing_conc = -1.0;
    int sensing_iter = 0;
    double sensing_pose[3] = {-1.0, -1.0, -1.0};
    auto sensing_start = high_resolution_clock::now();
    auto sensing_now = high_resolution_clock::now();
    duration<double> sensing_duration;
    ros::Subscriber gas_sub = n.subscribe("/gas_measurement", 1000, gasMeasureCallback);
    //---------------------------------------current pose sub------------------------------
    ros::Subscriber current_pose_sub = n.subscribe("/mavros/local_position/pose", 1000, currentPoseCallback);
    //----------------------------------------lidar scan sub---------------------------------------
    double grid_resolution;
    n.param("grid_map_resolution", grid_resolution, 1.0);
    int32_t grid_x = domain[0]/grid_resolution;
    int32_t grid_y = domain[1]/grid_resolution;
    int32_t grid_z = domain[2]/grid_resolution;
    LidarMap lidar_map; 
    lidar_map.initialization(grid_x, grid_y, grid_z, grid_resolution);
    ros::Subscriber lidar_map_sub = n.subscribe("/lidar_2d", 1000, &LidarMap::callback, &lidar_map);
    //-----------------------------------------lidar map pub----------------------------------------
    ros::Publisher map_2d_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_2d", 1000);
    nav_msgs::OccupancyGrid map_2d_msg;
    map_2d_msg.info.resolution = grid_resolution; // 1m/cell
    map_2d_msg.info.width = grid_x;
    map_2d_msg.info.height = grid_y;
    //----------------------------------------next goal pub---------------------------------------
    ros::Publisher next_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);
    geometry_msgs::PoseStamped next_goal_msg;
    double goal_ep;
    int total_rh;
    n.param("goal_epsilon",            goal_ep, 0.1);
    n.param("receding_horizon_length", total_rh, 3);

    next_goal.x = 6.0; //temp
    next_goal.y = 6.0; //temp
    next_goal.z = 12.0; //temp
    next_goal_msg.pose.position.x = next_goal.x; //temp
    next_goal_msg.pose.position.y = next_goal.y; //temp
    next_goal_msg.pose.position.z = next_goal.z; //temp
    next_goal_msg.pose.orientation.w = 1.0;
    next_goal_msg.header.stamp = ros::Time::now();
    next_goal_msg.header.frame_id = "map";
    //----------------------next goal list (now just uav pose) pub------------------------------------
    ros::Publisher next_goal_list_pub = n.advertise<sensor_msgs::PointCloud2>("/next_goal_pose", 1000);
    sensor_msgs::PointCloud2 next_goal_list_msg;

    //-----------------------------------path---------------------------------------
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/trajectory", 1000);
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";

    //---------------------------particle filter publish--------------------------------------
    ros::Publisher pf_pub = n.advertise<sensor_msgs::PointCloud2>("/particle_filter", 1000);
    sensor_msgs::PointCloud2 pf_msg;


    ros::Rate loop_rate(100); // 10Hz
    ros::Rate map_rate(100); // 10Hz

    vector<int8_t> current_map;

    int iteration = 0;
    double time_sum = 0;

    while(ros::ok())
    {
        double del_x_sq = pow(next_goal.x-current_uav.x,2);
        double del_y_sq = pow(next_goal.y-current_uav.y,2);
        double del_z_sq = pow(next_goal.z-current_uav.z,2);
        double goal_dist = sqrt(del_x_sq + del_y_sq + del_z_sq);
        // -------------------------Trigger to get measurement-----------------------
        //cout << "goal_dist: " << goal_dist << endl;
        if(goal_dist < goal_ep) // reaching goal
        {
            if(sensing_conc == -1.0)
            {
                sensing_pose[0] = 0.0, sensing_pose[1] = 0.0, sensing_pose[2] = 0.0;
                sensing_conc = 0.0;
                sensing_iter = 0;
                sensing_start = high_resolution_clock::now();
                //cout << "-------------------------------------------------" << endl;
            }
            sensing_now = high_resolution_clock::now();
            sensing_duration = sensing_now - sensing_start;
            // start pf update + decision making when "actual time > user defined sensing time"
            // if user defined sensing time is shorter than real sensing time, just get single sample
            if(sensing_duration.count() < sensing_uav.sen_t || sensing_iter == 0)
            {
                sensing_pose[0] += current_uav.x;
                sensing_pose[1] += current_uav.y;
                sensing_pose[2] += current_uav.z;
                sensing_conc    += current_uav.sensor_value; //string to double
                sensing_iter    += 1;
                //cout << "sensing_done" << endl;
            }
            else
            {
                //cout << "sensing duration: " << sensing_duration.count() << endl;
                //cout << "sensing num: " << sensing_iter << endl;
                sensing_uav.x            = sensing_pose[0]/sensing_iter; // taking average
                sensing_uav.y            = sensing_pose[1]/sensing_iter;
                sensing_uav.z            = sensing_pose[2]/sensing_iter;
                //cout << "Current sensing location: [";
                //cout << sensing_uav.x << ", ";
                //cout << sensing_uav.y << ", ";
                //cout << sensing_uav.z << "]" << endl;

                geometry_msgs::PoseStamped new_location;
                new_location.pose.position.x = sensing_uav.x;
                new_location.pose.position.y = sensing_uav.y;
                new_location.pose.position.z = sensing_uav.z;
                path_msg.poses.push_back(new_location);
                path_pub.publish(path_msg);


                double sen_val = sensing_conc/sensing_iter;
                sensing_uav.sensor_value = sen_val;

                if(sensing_uav.sensor_value > sensing_uav.bi_thre)
                    sensing_uav.bi_thre_update();

                //------------particle filter update-----------------------
                bool resampling_on = true;
                int8_t sensor_model = 1; // 1: gaussian, 2: binary
                pf.weight_update(sensing_uav, sensor_model, sen_val, resampling_on);

                //--------------lidar map update-----------------------
                //cout << "Start lidar update" << endl;
                lidar_map.lidar_uav_pose_update(sensing_uav);
                lidar_map.trigger = true; //then start map update
                while(!lidar_map.updated && ros::ok())
                {
                    //cout << "NOW MAP UPDATING" << endl;
                    ros::spinOnce(); // run LidarMap::Callback function
                    next_goal_pub.publish(next_goal_msg);
                    map_rate.sleep();
                }
//cout << "Map Updated" << endl;
                lidar_map.trigger = false;
                
                //------------decision making (new goal)---------------
                
                auto process_start = high_resolution_clock::now();
                DecisionMaker decision(env, lidar_map, total_rh); 
                int current_rh = 1;
//cout << "Start Decision" << endl;
                for(int i=0; i<2; i++)
                    decision.RHI_BR(sensing_uav, pf, current_rh);

//cout << "Done Decision" << endl;
                //next_goal = decision.new_goal;
                /*
		cout << "num rh decision at main: " << decision.rh_decision.size() << endl;
        	for(int a =0; a<decision.rh_decision.size() ; a++)
	        {
		    cout << decision.rh_entropy[a] << " " << endl;
	            for(int b =0; b<decision.rh_decision[a].size(); b++)
	            {
		        cout << static_cast<int16_t>(decision.rh_decision[a][b]) << " ";
		        //cout << decision.rh_entropy[a][b] << " ";
		    }
		    cout << endl;
	            for(int b =0; b<decision.rh_decision[a].size(); b++)
	            {
		        cout << static_cast<int16_t>(decision.max_decision[b]) << " ";
		        //cout << decision.rh_entropy[a][b] << " ";
		    }
		    cout << endl;
		}
                */
                pcl::PointCloud<pcl::PointXYZRGB> next_goal_list_XYZRGB;
                pcl::PCLPointCloud2 next_goal_obj;

                pcl::PointXYZRGB new_next_goal;
                uint32_t color = 3;
                new_next_goal.x = sensing_uav.x;
                new_next_goal.y = sensing_uav.y;
                new_next_goal.z = sensing_uav.z;
                new_next_goal.rgb = color;
                next_goal_list_XYZRGB.push_back(new_next_goal);
                //convert pointXYG to pointCloud2
                pcl::toPCLPointCloud2(next_goal_list_XYZRGB, next_goal_obj);
                //convert pointCloud2 to ROS message
                pcl_conversions::fromPCL(next_goal_obj, next_goal_list_msg);
                next_goal_list_msg.header.frame_id = "map";
                next_goal_list_pub.publish(next_goal_list_msg);

                //-------------------------------------pf-------------------------------------
                pcl::PointCloud<pcl::PointXYZRGB> pf_XYZRGB;
                pcl::PCLPointCloud2 pf_obj;

                pcl::PointXYZRGB pf_single;
                color = 10;
                for(int samp=0; samp<pf.n_p; samp++)
                {
                    pf_single.x = pf.X[samp];
                    pf_single.y = pf.Y[samp];
                    pf_single.z = pf.Z[samp];
                    pf_single.rgb = color;
                    pf_XYZRGB.push_back(pf_single);
                }
                //convert pointXYG to pointCloud2
                pcl::toPCLPointCloud2(pf_XYZRGB, pf_obj);
                //convert pointCloud2 to ROS message
                pcl_conversions::fromPCL(pf_obj, pf_msg);
                pf_msg.header.frame_id = "map";
                pf_pub.publish(pf_msg);




//cout << "a" << endl;
//cout << decision.rh_decision.size() << endl;
//cout << decision.max_decision.size() << endl;
                next_goal.x = sensing_uav.x + sensing_uav.xnew[decision.max_decision[0]];
                next_goal.y = sensing_uav.y + sensing_uav.ynew[decision.max_decision[0]];
                next_goal.z = sensing_uav.z + sensing_uav.znew[decision.max_decision[0]];
//cout<< "b" << endl;
                decision.max_entropy = -10000.0;
                next_goal_msg.pose.position.x = next_goal.x;
                next_goal_msg.pose.position.y = next_goal.y;
                next_goal_msg.pose.position.z = next_goal.z;
                next_goal_msg.header.stamp = ros::Time::now();

                //------------------initialize------------------
                sensing_conc = -1.0;

                iteration += 1;
                cout << "Iteration: " << iteration << endl;

                auto process_done = high_resolution_clock::now();
                duration<double> process_duration = process_done - process_start;
                time_sum += process_duration.count();
                double time_avg = time_sum/iteration;

                cout << "Processing average time: " << time_avg << endl;
            } // finish sensing + pf update + decision making

        } // goal reached

        //-----------------------always publish target pose------------------------
        next_goal_pub.publish(next_goal_msg);
        //--------------------------------------------------------------------------
        //ROS_INFO("Sensor_data_obtaining_ready");
        //cout<<lidar_map.updated << endl;
        if(lidar_map.updated)
        {
            current_map.clear();
            for(int i=0; i<lidar_map.grid_map.size(); i++)
            {
                for(int j=0; j<lidar_map.grid_map[i].size(); j++)
                {
                    current_map.push_back(lidar_map.grid_map[i][j]);
                }
            }
            //cout << current_map[0] << endl;
            lidar_map.updated = false;
        }
        //cout << current_map[0] << endl;
        map_2d_msg.data = current_map;
        map_2d_pub.publish(map_2d_msg);

        if(pf.vector_std(pf.Wpnorm) < 3)
        {
            double xmean, ymean, zmean;
            for(int i=0; i<pf.n_p; i++)
            {
                xmean += pf.Wpnorm[i]*pf.X[i];
                zmean += pf.Wpnorm[i]*pf.Y[i];
                ymean += pf.Wpnorm[i]*pf.Z[i];
            }
            if(sqrt(pow(source[0]-xmean,2)+pow(source[1]-ymean,2)+pow(source[3]-zmean,2)) < 6)
                cout << "SUCCESS" << endl;
            else
                cout << "FAILED" << endl;

            ros::shutdown();
            //initialization
        }
        ros::spinOnce();
        //cout << "uav_x_now: " << uav.x << endl;;
        /*
        for (size_t i=0; i< pf.X.size(); i++)
        {
            cout << pf.X[i] << ";  ";
        }
        cout << endl;

        cout << env.source_x << endl;
        */
        loop_rate.sleep();
    }
    return 0;
}
