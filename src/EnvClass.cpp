#include "rh_infotaxis/EnvClass.h"

void EnvClass::initialization(double source[], double domain[]) //constructor
{
    source_x = source[0];
    source_y = source[1];
    source_z = source[2];

    source_u = source[3];
    source_q = source[4];
    source_phi = source[5];
    source_d = source[6];
    source_tau = source[7];

    //--------------------grid--------------------
    nx = domain[0];
    ny = domain[1];
    nz = domain[2];

    dt = domain[3];
    env_sig = domain[4];

}
