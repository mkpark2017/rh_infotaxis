#include "rh_infotaxis/ParticleFilter.h"


void ParticleFilter::initialization(int num_particles, EnvClass env) //constructor
{
    pf_env = env;

    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<double> rand_data(0.0, 1.0);

    n_p = num_particles;
    for(int i=0; i<n_p; i++)
    {
        
        X.push_back( rand_data(gen)*pf_env.nx );
        Y.push_back( rand_data(gen)*pf_env.ny );
        Z.push_back( rand_data(gen)*pf_env.nz );
        
        Q.push_back( rand_data(gen)*pf_env.source_q );
        //Phi.push_back( rand_data(gen)*pf_env.source_phi );
        Phi.push_back( pf_env.source_phi );
        D.push_back( rand_data(gen)*pf_env.source_d );
        Tau.push_back( rand_data(gen)*pf_env.source_tau );

        Wpnorm.push_back( 1.0/n_p );
    }

    Wp_sum = 0;

}

vector<double> ParticleFilter::isotropic_plume(UavClass uav)
{
    double dist, y_n, lambda_plume, rate;
    vector<double> exp_conc;
    for(int i=0; i<n_p; i++)
    {
        dist = sqrt( pow(uav.x-X[i],2) + pow(uav.y-Y[i],2) + pow(uav.z-Z[i],2) );
        y_n = ( -(uav.x-X[i])*sin(Phi[i]) + (uav.y-Y[i])*cos(Phi[i]) );
        lambda_plume = sqrt( D[i]*Tau[i] / (1+pow(pf_env.source_u,2)*Tau[i]/4/D[i]) );
        rate = Q[i]/(4*M_PI*D[i]*dist) * exp(-y_n*pf_env.source_u/(2*D[i]) - dist/lambda_plume);
        double sensor_space = (uav.sen_h_range*uav.sen_h_range) * uav.sen_v_range;
        //horizontal size = 3m X 3m, vertical size = 1m
        exp_conc.push_back(rate * pf_env.dt * sensor_space);

        //std::cout << exp_conc[i] << "  ";
    }
    //std::cout << std::endl;
    return exp_conc;
}

vector<double> ParticleFilter::prior(vector<double> likelihood)
{
    bool keep[n_p] = {0,};
    for(int i=0; i<n_p; i++)
    {
        if(X[i]>=0 && X[i]<=pf_env.nx && Y[i]>=0 && Y[i]<=pf_env.ny && Z[i]>=0 && Z[i]<=pf_env.nz)
            if(Q[i]>=0.01*pf_env.source_q && D[i]>=0 && Tau[i]>=0)
                keep[i] = 1; // Keep true
        if (keep[i] != 1) // if not keep
            likelihood[i] = 0;
    }
    return likelihood;
}

double ParticleFilter::vector_std(vector<double> vector) //standard deviation
{
    double mean = 0;
    double var = 0;
    for(int i=0; i<vector.size(); i++)
        mean += vector[i]/vector.size();

    for(int i=0; i<vector.size(); i++)
        var += (vector[i]*vector[i] - mean) / vector.size();

    double std = sqrt(var);
    return std;
}


vector<double> ParticleFilter::gaussian_sensor_model(UavClass uav, double sen_val)
{
    vector<double> exp_conc;
    exp_conc = isotropic_plume(uav);

    double pdetSig[n_p], ga_val[n_p];
    vector<double> Likelihood;

    for(int i=0; i<n_p; i++)
    {
        pdetSig[i] = sqrt( pow(exp_conc[i]*uav.sensor_sig_m,2) + pow(pf_env.env_sig,2) );
        if(pdetSig[i] < 1e-100)
	    pdetSig[i] = 1e-100;

        ga_val[i] = (sen_val - exp_conc[i]) / pdetSig[i];

        Likelihood.push_back(1/(sqrt(2*M_PI) * pdetSig[i]) * exp(-pow(ga_val[i],2) / 2));
        if(Likelihood[i] < 1e-100)
	    Likelihood[i] = 1e-100;
        /*
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "Wp_origin: " << Wpnorm[i] << std::endl;
        std::cout << "Exp_conc: " << exp_conc[i] << std::endl;
        std::cout << "likelihood: " << Likelihood[i] << std::endl;
        */
    }
    return Likelihood;
}

vector<double> ParticleFilter::binary_sensor_model(UavClass uav, double sen_val)
{
    //std::cout << "sen_val: " << sen_val << std::endl;
    vector<double> exp_conc;
    exp_conc = isotropic_plume(uav);

    double pdetSig[n_p], ga_val[n_p];
    vector<double> Likelihood;

    for(int i=0; i<n_p; i++)
    {
        pdetSig[i] = sqrt( pow(exp_conc[i]*uav.sensor_sig_m,2) + pow(pf_env.env_sig,2) );
        if(pdetSig[i] < 1e-100)
	    pdetSig[i] = 1e-100;

        ga_val[i] = (uav.bi_thre - exp_conc[i]) / pdetSig[i];
        double cdf_val = erfc(-ga_val[i]/sqrt(2))/2;
        Likelihood.push_back(sen_val + pow(-1,sen_val)*cdf_val); // if val=0 -> cdf, val=1 -> 1-cdf   
        if(Likelihood[i] < 1e-100)
	    Likelihood[i] = 1e-100;
    }

    return Likelihood;
}


void ParticleFilter::resampling(UavClass uav, vector<double> Likelihood, int8_t sen_model, double sen_val)
{
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<double> rand_data(0.0, 1.0);

    double random_select[n_p];
    random_select[0] = rand_data(gen)/n_p;


    vector<double> sum_weight;
    int num_sector = 10; // resampling sector
    for(int i=0; i<num_sector; i++)
    {
        vector<double> sum_weight_temp;
        sum_weight_temp.push_back(Wpnorm[i*(n_p/num_sector)]);
        for(int j=1; j<(n_p/num_sector); j++)
            sum_weight_temp.push_back(sum_weight_temp[j-1] + Wpnorm[i*(n_p/num_sector)+j]);

        for(int j=0; j<(n_p/num_sector); j++)
            sum_weight_temp[j] = sum_weight_temp[j]/sum_weight_temp.back() / num_sector;

        if(i==0)
        {
            for(int j=0; j<(n_p/num_sector); j++)
                sum_weight.push_back(sum_weight_temp[j]);
        }
        else
        {
            //std::cout << sum_weight[i*(n_p/num_sector)-1] << std::endl;
            for(int j=0; j<(n_p/num_sector); j++)
                sum_weight.push_back(sum_weight[i*(n_p/num_sector)-1] + sum_weight_temp[j]);
        }
    }

    //std::cout << "sum_weight: " << sum_weight[n_p-1] << std::endl;


    for(int i=1; i<n_p; i++)
    {
        //sum_weight[i] = sum_weight[i-1] + Wpnorm[i];
        random_select[i] = double(i)/n_p + rand_data(gen)/n_p;
        //std::cout << "WP: " << Wpnorm[i] << std::endl;
        //std::cout << "SUM: " << sum_weight[i] << std::endl;
        //std::cout << random_select[i] << std::endl;
    }
    int j =0;
    std::vector<int> indx;
    for(int i=0; i<n_p; i++)
    {
        while(sum_weight[j] < random_select[i])
        {
            j += 1;
        }
        indx.push_back(j);

        //std::cout << "indx: " << indx[i] << std::endl;
    }

    //----------------------------------MCMC------------------------------------
    double mm=5.0;
    double A=pow(4.0/(mm+2.0), 1.0/(mm+4.0) );
    double hopt = A* pow(n_p, -1.0/(mm+4.0) );

    for(int i=0; i<n_p; i++)
    {
        X[i] = X[indx[i]];
        Y[i] = Y[indx[i]];
        Z[i] = Z[indx[i]];

        Q[i]   = Q[indx[i]];
        Phi[i] = Phi[indx[i]];
        D[i]   = D[indx[i]];
        Tau[i] = Tau[indx[i]];
    }

    std::vector<int> indx_unique;
    indx_unique = indx;
    indx_unique.erase(unique(indx_unique.begin(), indx_unique.end() ), indx_unique.end() );

    int num_while_iter = 1;
    double l_keep = indx_unique.size();
    //while(l_keep < n_p*0.5)
    //{
        double stdX, stdY, stdZ, stdQ, stdPhi, stdD, stdTau;
        double dkX, dkY, dkZ, dkQ, dkPhi, dkD, dkTau;
        double oX[n_p], oY[n_p], oZ[n_p], oQ[n_p], oPhi[n_p], oD[n_p], oTau[n_p]; //old samples
        //------------------------------------------STD-----------------------------------------

        stdX = vector_std(X);
        dkX  = stdX/double(num_while_iter) + 0.5;
        stdY = vector_std(Y);
        dkY  = stdY/double(num_while_iter) + 0.5;
        stdZ = vector_std(Z);
        dkZ  = stdZ/double(num_while_iter) + 0.5;

        stdQ   = vector_std(Q);
        dkQ    = stdQ  / double(num_while_iter) + 0.5;
        stdPhi = vector_std(Phi);
        dkPhi  = stdPhi/ double(num_while_iter) + 0.5;
        stdD   = vector_std(D);
        dkD    = stdD  / double(num_while_iter) + 0.5;
        stdTau = vector_std(Tau);
        dkTau  = stdTau/ double(num_while_iter) + 0.5;

        for(int i=0; i<n_p; i++)
        {
            oX[i] = X[i];
            oY[i] = Y[i];
            oZ[i] = Z[i];

            oQ[i]   = Q[i];
            oPhi[i] = Phi[i];
            oD[i]   = D[i];
            oTau[i] = Tau[i];

            X[i] = oX[i] + hopt * dkX * rand_data(gen);
            Y[i] = oY[i] + hopt * dkY * rand_data(gen);
            Z[i] = oZ[i] + hopt * dkZ * rand_data(gen);

            Q[i]   = oQ[i]   + hopt * dkQ   * rand_data(gen);
            Phi[i] = oPhi[i] + hopt * dkPhi * rand_data(gen);
            D[i]   = oD[i]   + hopt * dkD   * rand_data(gen);
            Tau[i] = oTau[i] + hopt * dkTau * rand_data(gen);
        }

        vector<double> Likelihood_new;
        if(sen_model == 1) //gaussian
            Likelihood_new = gaussian_sensor_model(uav, sen_val);
        else if(sen_model == 2) //binary
            Likelihood_new = binary_sensor_model(uav, sen_val);
        else
            std::cout << "WARNING: Undefined sensor model!!!!!!!!!!!" << std::endl;

        Likelihood_new = prior(Likelihood_new);

        bool keep[n_p] = {0,};
        vector<double> check_unique;
        for(int i=0; i<n_p; i++) // keep old samples
        {
            //if new likelihood is not enough higer than old one
            if( Likelihood_new[i]/Likelihood[indx[i]] < rand_data(gen) )
            {
                X[i] = oX[i];
                Y[i] = oY[i];
                Z[i] = oZ[i];

                Q[i]   = oQ[i];
                Phi[i] = oPhi[i];
                D[i]   = oD[i];
                Tau[i] = oTau[i];
            }
            check_unique.push_back(X[i]+Y[i]+Z[i]);
        }
        check_unique.erase(unique(check_unique.begin(), check_unique.end() ), check_unique.end() );
        l_keep = check_unique.size();
        num_while_iter += 1;
        if(num_while_iter > 100)
            l_keep = n_p;
    //}
}


void ParticleFilter::weight_update(UavClass uav, int8_t sen_model, double sen_val, bool resamp_on)
{
    vector<double> Likelihood;

    if(sen_model == 1) //gaussian
        Likelihood = gaussian_sensor_model(uav, sen_val);
    else if(sen_model == 2) //binary
        Likelihood = binary_sensor_model(uav, sen_val);
    else
        std::cout << "WARNING: Undefined sensor model!!!!!!!!!!!" << std::endl;

    Likelihood = prior(Likelihood); // check particles out of domain

    for(int i=0; i<n_p; i++)
        Wpnorm[i] = Wpnorm[i]*Likelihood[i];

    double Wp_sum_temp = 0, Neff_inv = 0;
    for(int i=0; i<n_p; i++)
        Wp_sum_temp += Wpnorm[i];

    for(int i=0; i<n_p; i++)
    {
        Wpnorm[i] = Wpnorm[i]/Wp_sum_temp;
        //std::cout << i << " Wp_new: " << Wpnorm[i] << std::endl;

        Neff_inv += pow(Wpnorm[i],2);
    }
    //std::cout << "Neff :" << 1.0/Neff_inv << std::endl;

    if(1.0/Neff_inv < 0.5*n_p && resamp_on)
    {
        resampling(uav, Likelihood, sen_model, sen_val);
        for(int i=0; i<n_p; i++)
            Wpnorm[i] = 1.0/n_p;
    }
    //std::cout << "Wp sum: " << Wp_sum_temp << std::endl;
    Wp_sum = Wp_sum_temp;

}

