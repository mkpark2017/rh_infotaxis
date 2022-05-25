#include "rh_infotaxis/UavClass.h"

void UavClass::bi_thre_update()
{
    bi_thre = sensor_value * bi_thre_ratio + bi_thre * (1-bi_thre_ratio);
}

void UavClass::initialization(double sen_param[])
{
    sensor_sig_m  = sen_param[0];
    sen_h_range   = sen_param[1];
    sen_v_range   = sen_param[2];
    sen_t         = sen_param[3]; // seconds to milliseconds

    bi_thre_ratio = sen_param[4];

    num_decision  = int(sen_param[5]);
    move_h        = sen_param[6];
    move_v        = sen_param[7];

    bi_thre = 0.0;

    for(int i=0; i<4; i++)
    {
        xnew[i] = cos(i * M_PI/2) * move_h;
        ynew[i] = sin(i * M_PI/2) * move_h;
        znew[i] = 0;
    }
    xnew[4] = 0,           xnew[5] = 0,          xnew[6] = 0;
    ynew[4] = 0,           ynew[5] = 0,          ynew[6] = 0;
    znew[4] = -1.0*move_v, znew[5] = 1.0*move_v, znew[6] = 0;
}
