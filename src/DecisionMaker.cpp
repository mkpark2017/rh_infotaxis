#include "rh_infotaxis/DecisionMaker.h"

DecisionMaker::DecisionMaker(EnvClass env, LidarMap lidarmap, int rh_length)
{
    agent_env = env;
    agent_map = lidarmap;
    total_rh = rh_length;
    max_entropy = -10000.0;
}

bool DecisionMaker::check_overlap(int8_t i, int8_t j)
{
    bool overlap = false;
    if(i == 0 && j==2)
        overlap = true;
    else if(i==2 && j==0)
        overlap = true;
    else if(i==1 && j==3)
        overlap = true;
    else if(i==3 && j==1)
        overlap = true;
    else if(i==4 && j==5)
        overlap = true;
    else if(i==5 && j==4)
        overlap = true;

    return overlap;
}

bool DecisionMaker::check_building_block(UavClass uav, UavClass current)
{
    int uav_grid_x = round(uav.x/agent_map.resolution)-1; //location 3 = grid 0
    int uav_grid_y = round(uav.y/agent_map.resolution)-1;
    int current_grid_x = round(current.x/agent_map.resolution)-1;
    int current_grid_y = round(current.y/agent_map.resolution)-1;
    int current_grid_z = round(current.z/agent_map.resolution)-1;
    bool building_block = false;

    int min_x = uav_grid_x, max_x = current_grid_x;
    if(uav_grid_x > current_grid_x)
    {
        max_x = uav_grid_x;
        min_x = current_grid_x;
    }
    int min_y = uav_grid_y, max_y = current_grid_y;
    if(uav_grid_y > current_grid_y)
    {
        max_y = uav_grid_y;
        min_y = current_grid_y;
    }

    if(current_grid_z < 30)
    {
        for(int k=min_x; k<=max_x; k++)
        {
            for(int l=min_y; l<=max_y; l++)
            {
                if(agent_map.grid_map[l][k] == 100)
                {
                    building_block = true;
                }
            }
        }
    }
    return building_block;
}


void DecisionMaker::RHI_BR(UavClass uav, ParticleFilter pf, int current_rh, vector<int8_t> ind_1d, vector<double> entropy_1d)
{
    if(current_rh <= total_rh)
    {
        for(int i=0; i<uav.num_decision; i++)
        {
            ParticleFilter temp_pf = pf;
            vector<int8_t> ind_1d_temp = ind_1d;
            vector<double> entropy_1d_temp = entropy_1d;
            UavClass current = uav;
            current.x += current.xnew[i];
            current.y += current.ynew[i];
            current.z += current.znew[i];
            
            if(current_rh > 1)
                if(check_overlap(i, ind_1d.back()))
                    continue;

            if(current.x<0 || current.x>agent_env.nx || current.y < 0 || current.y > agent_env.ny)
                continue;
            else if(current.z < 3 || current.z > agent_env.nz)
                continue;
            else if(check_building_block(uav, current))
                continue;
            else if(agent_map.grid_map[current.y][current.y] == 100 && current.z <= 30)
                continue;
            else
            {
                int8_t sensor_model = 2; //1: Gaussian 2: binary sensor model
                bool reampling_on = false;
                double current_entropy = 0;
                for(int j=0; j<temp_pf.n_p; j++)
                {
                    if(temp_pf.Wpnorm[j] != 0 )
                        current_entropy -= temp_pf.Wpnorm[j]*log2(temp_pf.Wpnorm[j]);
                }

                vector<double> Wp_sum;
                vector<double> future_entropy;
                double Wp_sum_sum = 0;
                for(int sen_val=0; sen_val<=1; sen_val++)
                {
                    double new_entropy = 0;
                    temp_pf.weight_update(current, sensor_model, sen_val, reampling_on);
                    Wp_sum.push_back(temp_pf.Wp_sum);
                    Wp_sum_sum += temp_pf.Wp_sum;
                    
                    for(int j=0; j<temp_pf.n_p; j++)
                    {
                        if(temp_pf.Wpnorm[j] != 0)
                            new_entropy -= temp_pf.Wpnorm[j]*log2(temp_pf.Wpnorm[j]);
                    }
                    future_entropy.push_back(new_entropy);

                }
                double new_entropy = 0;
                for(int sen_val=0; sen_val<=1; sen_val++)
                {

                    Wp_sum[sen_val] = Wp_sum[sen_val]/Wp_sum_sum;
                    future_entropy[sen_val] = Wp_sum[sen_val] * future_entropy[sen_val];
                    new_entropy += future_entropy[sen_val];
                }
                double entropy = current_entropy - new_entropy;

                ind_1d_temp.push_back(i);
                entropy_1d_temp.push_back(entropy);
                RHI_BR(current, pf, current_rh+1, ind_1d_temp, entropy_1d_temp);
            }
        }
    }
    else
    {
        rh_decision.push_back(ind_1d);
        double entropy_sum = 0;

        for(int i=0; i<entropy_1d.size(); i++)
        {
            entropy_sum += pow(0.9,i)*entropy_1d[i];
        }

        rh_entropy.push_back(entropy_sum);
        if(max_entropy < entropy_sum)
        {
            max_decision = ind_1d;
            max_entropy = entropy_sum;
        }

    }

}

