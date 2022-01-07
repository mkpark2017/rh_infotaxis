#ifndef ENV_CLASS_H_
#define ENV_CLASS_H_

class EnvClass
{
public:
    void initialization(double source[], double domain[]);

    double source_x;
    double source_y;
    double source_z;

    double source_u;
    double source_q;
    double source_phi;
    double source_d;
    double source_tau;

    //--------------------grid--------------------
    double nx;
    double ny;
    double nz;

    double dt;
    double env_sig;

private:
    //void sensor_model();
    //void dispersion_model();
    //void resampling();
};

#endif
