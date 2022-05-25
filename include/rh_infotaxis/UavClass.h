#ifndef UAV_CLASS_H_
#define UAV_CLASS_H_

#include <vector>
#include <cmath>

using std::vector;

class UavClass
{
public:
    double x;
    double y;
    double z;
    double sensor_value;
    double sensor_sig_m;
    double sen_h_range; // sensor horizontal range
    double sen_v_range; // sensor vertical range
    double sen_t;

    double bi_thre;
    double bi_thre_ratio;

    int num_decision;
    double move_h;
    double move_v;

    double xnew[7]; // 3d + stay
    double ynew[7];
    double znew[7];

    void initialization(double sen_param[]);
    void bi_thre_update();

};

#endif
