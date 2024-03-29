close all
clear all
clc

rosinit('http://192.168.0.113:11311');

%rosinit('http://192.168.35.51:11311');

addpath('Functions')
load('transient_data_3d_one_hour_CO_gas_11th_map_new.mat')
[ny, nx] = size(map);
nz = 45;
map_nan = map;
map_nan(map_nan ~= 0) = NaN;
g = transient_conc_3d{1}/1000;

[map_pub, map_msg] = rospublisher('/map_truth', 'nav_msgs/OccupancyGrid');
for i= 1:nx
    map_msg.Data = [map_msg.Data; 100*map(i,:)'];
end
map_msg.Info.Resolution = 1;
map_msg.Info.Width = nx;
map_msg.Info.Height = ny;


%gas_sen_service_server = rossvcserver('/gas_sen_svc', 'mavros_msgs/ParamPull', @sensorMeasureCallback);
%current_pose = [6 15 12];
[current_pose_pub, current_pose_msg] = rospublisher('/mavros/local_position/pose', 'geometry_msgs/PoseStamped');
current_pose_msg.Pose.Position.X = 0;
current_pose_msg.Pose.Position.Y = 0;
current_pose_msg.Pose.Position.Z = 0;
send(current_pose_pub, current_pose_msg);

[lidar_pub, lidar_msg] = rospublisher('/lidar_2d', 'sensor_msgs/LaserScan');
lidar_msg.AngleMin = 0;
lidar_msg.AngleMax = 360;
lidar_msg.AngleIncrement = 1;
lidar_num = (lidar_msg.AngleMax-lidar_msg.AngleMin)/lidar_msg.AngleIncrement;
lidar_msg.RangeMax = 16;
lidar_length = lidar_msg.RangeMax;
lidar_msg.Header.FrameId = 'map';

[gas_pub, gas_msg] = rospublisher('/gas_measurement', 'sensor_msgs/ChannelFloat32');


local_pose_sub = rossubscriber('/mavros/local_position/pose', 'geometry_msgs/PoseStamped');
while(1)
    send(map_pub, map_msg);
    
    local_pose_msg = receive(local_pose_sub, 10);
    local_pose(1) = local_pose_msg.Pose.Position.X;
    local_pose(2) = local_pose_msg.Pose.Position.Y;
    local_pose(3) = local_pose_msg.Pose.Position.Z;


    gas_msg.Values(1) = g(round(local_pose(2)/3),round(local_pose(1)/3),round(local_pose(3)/3));
    send(gas_pub, gas_msg);
    if current_pose_msg.Pose.Position.Z < 30
        for jj = 1:lidar_num
            lidar_msg.Ranges(jj) = lidar_length;
            for ii = 1:lidar_length
                jjj = 2*pi/lidar_num*(jj-1);
                x_r(jj) = round(local_pose(1)+cos(jjj)*(ii+0.1));
                y_r(jj) = round(local_pose(2)+sin(jjj)*(ii+0.1));
                %if x_r(jj)<1 || y_r(jj)<1 || x_r(jj)>nx || y_r(jj)>ny
                %    lidar_msg.Ranges(jj) = ii-1;
                %    break;
                %else
                if x_r(jj) < 1 || y_r(jj) < 1 || y_r(jj) > ny || x_r(jj) > nx
                    break;
                end
                if isnan(map_nan(y_r(jj),x_r(jj)))
                    lidar_msg.Ranges(jj) = ii;
                    break;
                end
                %end
            end
        end
    else
        lidar_msg.Ranges = [];        
        if isnan( map_nan( round(local_pose(2)), round(local_pose(1)) ) )
            lidar_msg.Ranges(1) = local_pose(3) - 30;
        end
    end
    send(lidar_pub, lidar_msg);
    
    pause(0.01);
    
end
