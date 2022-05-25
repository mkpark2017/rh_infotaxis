#ifndef DECISION_MAKER_H_
#define DECISION_MAKER_H_

#include <iostream>
#include <numeric>

#include "rh_infotaxis/EnvClass.h"
#include "rh_infotaxis/UavClass.h"
#include "rh_infotaxis/ParticleFilter.h"
#include "rh_infotaxis/LidarMap.h"

using std::cout;
using std::endl;
using std::vector;

class DecisionMaker
{
public:
    EnvClass agent_env;
    LidarMap agent_map;
    int total_rh;
    DecisionMaker(EnvClass env, LidarMap lidarmap, int total_rh);

    vector<vector<int8_t>> rh_decision;
    vector<int8_t> max_decision;

    vector<double> rh_entropy;
    vector<vector<double>> rh_entropy_list;

    void RHI_BR(UavClass uav, ParticleFilter pf, int current_rh, vector<int8_t> temp_ind = {}, vector<double> temp_entropy = {});
    bool check_overlap(int8_t i, int8_t j);
    bool check_building_block(UavClass uav, UavClass current);

    double max_entropy;
};

#endif
