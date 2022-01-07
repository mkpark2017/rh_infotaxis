#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

//some generically useful stuff to include...
#include <vector>
#include <cmath>  // M_PI, erfc
#include <random>
#include <iostream> // cout endl
#include <algorithm>
#include <string> // for string sensor model

#include "rh_infotaxis/EnvClass.h"
#include "rh_infotaxis/UavClass.h"

using std::vector;

class ParticleFilter
{
public:
    void initialization(int num_particles, EnvClass env); // initialization

    void weight_update(UavClass uav, int8_t sen_model, double sen_val, bool resamp_on);

    int n_p;
    vector<double> X;
    vector<double> Y;
    vector<double> Z;

    vector<double> Q;
    vector<double> Phi;
    vector<double> D;
    vector<double> Tau;

    vector<double> Wpnorm;
    double Wp_sum;

    double vector_std(vector<double> vec);
private:
    vector<double> isotropic_plume(UavClass uav);
    vector<double> prior(vector<double> likelihood);
    void resampling(UavClass uav, vector<double> likelihood, int8_t sen_model, double sen_val);


    vector<double> gaussian_sensor_model(UavClass uav, double sen_val);
    vector<double> binary_sensor_model(UavClass uav, double sen_val);

    EnvClass pf_env;
    //void sensor_model();
    //void dispersion_model();
    //void resampling();
};

#endif
