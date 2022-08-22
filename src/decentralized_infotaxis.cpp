#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/ParamGet.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath> // M_SQRT1_2 M_PI
#include <math.h> // exp()
#include <iostream> // for std::cout
#include <random> // random
#include <chrono>
#include <ctime> // for std::time()

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h> //for PointCloud2Modifier

using namespace std; 


float g_p_x, g_p_y, g_p_z;
float s_x, s_y, s_z;

const int num_gas = 5;
float gas_value[num_gas];

const int num_agent = 5;
float neighbor_g[num_agent];
float neighbor_x[num_agent];
float neighbor_y[num_agent];
float neighbor_z[num_agent];

float neig_x[num_agent];
float neig_y[num_agent];
float neig_z[num_agent];

float temp_n_g[num_agent];
float temp_n_x[num_agent];
float temp_n_y[num_agent];
float temp_n_z[num_agent];

float wind_dir, wind_vel;

double gas_s_v = 0;

bool trig[num_agent];
bool bayes_trig[num_agent];

void pose_global_callback(const sensor_msgs::NavSatFix::ConstPtr& g_pose_msg)
{
  g_p_x = g_pose_msg->longitude;
  g_p_y = g_pose_msg->latitude;
  g_p_z = g_pose_msg->altitude;
}

void neig_callback1(const sensor_msgs::NavSatFix::ConstPtr& g_pose_msg)
{
  neig_x[0] = g_pose_msg->longitude;
  neig_y[0] = g_pose_msg->latitude;
  neig_z[0] = g_pose_msg->altitude;
}
void neig_callback2(const sensor_msgs::NavSatFix::ConstPtr& g_pose_msg)
{
  neig_x[1] = g_pose_msg->longitude;
  neig_y[1] = g_pose_msg->latitude;
  neig_z[1] = g_pose_msg->altitude;
}
void neig_callback3(const sensor_msgs::NavSatFix::ConstPtr& g_pose_msg)
{
  neig_x[2] = g_pose_msg->longitude;
  neig_y[2] = g_pose_msg->latitude;
  neig_z[2] = g_pose_msg->altitude;
}
void neig_callback4(const sensor_msgs::NavSatFix::ConstPtr& g_pose_msg)
{
  neig_x[3] = g_pose_msg->longitude;
  neig_y[3] = g_pose_msg->latitude;
  neig_z[3] = g_pose_msg->altitude;
}
void neig_callback5(const sensor_msgs::NavSatFix::ConstPtr& g_pose_msg)
{
  neig_x[4] = g_pose_msg->longitude;
  neig_y[4] = g_pose_msg->latitude;
  neig_z[4] = g_pose_msg->altitude;
}

void source_callback(const sensor_msgs::NavSatFix::ConstPtr& source_msg)
{
  s_x = source_msg->longitude;
  s_y = source_msg->latitude;
  s_z = source_msg->altitude;
}

void sensor_callback0(const sensor_msgs::Range::ConstPtr& gas_msg)
{
  gas_value[0] = (gas_msg->range - 10.0)/(874.0-17.0)*30.0;
}
void sensor_callback1(const sensor_msgs::Range::ConstPtr& gas_msg)
{
  gas_value[1] = (gas_msg->range - 10.0)/(800.0-17.0)*30.0/2.0;
}
void sensor_callback2(const sensor_msgs::Range::ConstPtr& gas_msg)
{
  gas_value[2] = (gas_msg->range - 10.0)/(968.0-17.0)*30.0;
}
void sensor_callback3(const sensor_msgs::Range::ConstPtr& gas_msg)
{
  gas_value[3] = (gas_msg->range - 10.0)/(873.0-47.0)*30.0;
}
void sensor_callback4(const sensor_msgs::Range::ConstPtr& gas_msg)
{
  gas_value[4] = (gas_msg->range - 10.0)/(886.0-47.0)*30.0;
}

void wind_callback(const sensor_msgs::Range::ConstPtr& wind_msg)
{
  wind_dir = wind_msg->min_range + 90.0;
  wind_vel = wind_msg->range;
}

void neighbor_callback1(const geometry_msgs::PoseStamped::ConstPtr& neigh_msg1)
{
  neighbor_g[0] = neigh_msg1->pose.orientation.w;
  neighbor_x[0] = neigh_msg1->pose.position.x;
  neighbor_y[0] = neigh_msg1->pose.position.y;
  neighbor_z[0] = neigh_msg1->pose.position.z;  
}
void neighbor_callback2(const geometry_msgs::PoseStamped::ConstPtr& neigh_msg2)
{
  neighbor_g[1] = neigh_msg2->pose.orientation.w;
  neighbor_x[1] = neigh_msg2->pose.position.x;
  neighbor_y[1] = neigh_msg2->pose.position.y;
  neighbor_z[1] = neigh_msg2->pose.position.z;

}
void neighbor_callback3(const geometry_msgs::PoseStamped::ConstPtr& neigh_msg3)
{
  neighbor_g[2] = neigh_msg3->pose.orientation.w;
  neighbor_x[2] = neigh_msg3->pose.position.x;
  neighbor_y[2] = neigh_msg3->pose.position.y;
  neighbor_z[2] = neigh_msg3->pose.position.z;
}
void neighbor_callback4(const geometry_msgs::PoseStamped::ConstPtr& neigh_msg4)
{
  neighbor_g[3] = neigh_msg4->pose.orientation.w;
  neighbor_x[3] = neigh_msg4->pose.position.x;
  neighbor_y[3] = neigh_msg4->pose.position.y;
  neighbor_z[3] = neigh_msg4->pose.position.z;
}
void neighbor_callback5(const geometry_msgs::PoseStamped::ConstPtr& neigh_msg5)
{
  neighbor_g[4] = neigh_msg5->pose.orientation.w;
  neighbor_x[4] = neigh_msg5->pose.position.x;
  neighbor_y[4] = neigh_msg5->pose.position.y;
  neighbor_z[4] = neigh_msg5->pose.position.z;
}

double mean_concentration(float x_a, float y_a, float z_a, float x_s, float y_s, float q_s, float u_s, float p_s, float d_s, float t_s)
{
  double lamda = sqrt((d_s*t_s)/( 1.0+( pow(u_s,2.0)*t_s )/( 4.0*d_s ) ) );
  double module_dist = sqrt( pow(x_s-x_a,2.0) + pow(y_s-y_a,2.0) + pow(0.0-z_a,2.0) );
  return q_s/(module_dist*4.0*M_PI*d_s) *exp( (-1.0*module_dist/lamda) + (-1.0*(y_a-y_s)*u_s*sin(p_s)/(2.0*d_s)) + (-1.0*(x_a-x_s)*u_s*cos(p_s)/(2.0*d_s)) );
}

double mean_cal(double val[], double wpnorm[], int size)
{
  double mean_val;
  for (int i=0; i<size; i++)
  {
    mean_val += val[i]*wpnorm[i];
  }
  return mean_val;
}

double var_cal(double val[], double wpnorm[], int size)
{
  double mean_value;
  mean_value = mean_cal(val, wpnorm, size);
  double var_val;
  for (int i=0; i<size; i++)
  {
    var_val += wpnorm[i]*(val[i]-mean_value)*(val[i]-mean_value);
  }
  return var_val;
}

double *bayesian_update(double xp[], double yp[], double qp[], double dp[], double ga_new_pi[], double nx, double ny, double ext_length, float wind_vel_mean, float wind_dir_mean, int num_particle, double sen_sig_m_est, double env_sig, double source_tau, double dt)
{
  double ax_min, ax_max, ay_min, ay_max;
  ax_min = 0.0-ext_length;
  ay_min = 0.0-ext_length;
  ax_max = nx+ext_length;
  ay_max = ny+ext_length;

  int n_count = -1;
  double detrate[num_particle], detconc[num_particle]; 
  double detsig[num_particle], detsig_sq[num_particle];
  double ga_val[num_particle], ga_new[num_particle], ga_sum;

  for (int j=0; j<num_agent; j++)
  {
    if (trig[j] == true )
    {
      n_count = n_count+1;
      ga_sum = 0.0;
      for (int i=0; i<num_particle; i++) 
      {
        if (xp[i]>=ax_min && xp[i]<ax_max && yp[i]>=ay_min && yp[i]<ay_max && qp[i]>=0.0) 
        {
          detrate[i] = mean_concentration(neighbor_x[j],neighbor_y[j],neighbor_z[j],xp[i],yp[i],qp[i],wind_vel_mean,wind_dir_mean,dp[i],source_tau);
          detconc[i] = detrate[i]*dt;
          detsig[i] = detconc[i]*sen_sig_m_est + env_sig;
          detsig_sq[i] = detsig[i]*detsig[i];
          if (detsig[i] < pow(10.0,-300.0) )
            detsig[i] = pow(10.0,-300.0);
          if (detsig_sq[i] < pow(10.0,-300.0) )
            detsig_sq[i] = pow(10.0,-300.0);

          ga_val[i] = (neighbor_g[j] - detconc[i])/detsig[i];
          ga_new[i] = 1.0/sqrt(2.0*M_PI*detsig_sq[i])*exp(-pow(ga_val[i],2.0)/2.0);
          ga_new_pi[i] = ga_new_pi[i]*ga_new[i];
          ga_sum = ga_sum + ga_new_pi[i];
        } // if the particles are in the search domain and real values
        else
        {
          ga_new_pi[i] = 0.0;
        }
      } // for potential source terms in particle filter
      for (int i=0; i<num_particle; i++)
      {
        ga_new_pi[i] = ga_new_pi[i]/ga_sum; 
        //cout<< endl << "ga_pi = " << ga_new_pi[i] << endl << endl;
      }
    } // if receive neighbor data
  } // for multiple agents data
  cout << "Counted neighbor = " << n_count << endl;
  return ga_new_pi;
}

double normalCDF(double x, double m, double s)
{
   return 0.5 * erfc(-(x-m) * (s * M_SQRT1_2) );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "decentralized_infotaxis");
  ros::NodeHandle nh;

//--------------------------------Set environment once-----------------------------------

  ros::Subscriber neig_1_sub = nh.subscribe<geometry_msgs::PoseStamped>("/UAV1/current_measure", 10,neighbor_callback1);
  ros::Subscriber neig_2_sub = nh.subscribe<geometry_msgs::PoseStamped>("/UAV2/current_measure", 10,neighbor_callback2);
  ros::Subscriber neig_3_sub = nh.subscribe<geometry_msgs::PoseStamped>("/UAV3/current_measure", 10,neighbor_callback3);
  ros::Subscriber neig_4_sub = nh.subscribe<geometry_msgs::PoseStamped>("/UAV4/current_measure", 10,neighbor_callback4);
  ros::Subscriber neig_5_sub = nh.subscribe<geometry_msgs::PoseStamped>("/UAV5/current_measure", 10,neighbor_callback5);


  ros::Publisher mode_pub = nh.advertise<sensor_msgs::Range>("sensing_mode", 10);
  sensor_msgs::Range mode_msg;
  ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_position/global", 10);
  mavros_msgs::GlobalPositionTarget target_msg;
  ros::Publisher current_m_pub = nh.advertise<geometry_msgs::PoseStamped>("current_measure", 1);
  geometry_msgs::PoseStamped current_m_msg;
  ros::Publisher pf_pub = nh.advertise<sensor_msgs::PointCloud2>("particle_filter_points", 1);


  ros::Subscriber g_p_sub = nh.subscribe("mavros/global_position/global", 10, pose_global_callback);

  ros::Subscriber n_p_sub1 = nh.subscribe("/UAV1/mavros/global_position/global", 10, neig_callback1);
  ros::Subscriber n_p_sub2 = nh.subscribe("/UAV2/mavros/global_position/global", 10, neig_callback2);
  ros::Subscriber n_p_sub3 = nh.subscribe("/UAV3/mavros/global_position/global", 10, neig_callback3);
  ros::Subscriber n_p_sub4 = nh.subscribe("/UAV4/mavros/global_position/global", 10, neig_callback4);
  ros::Subscriber n_p_sub5 = nh.subscribe("/UAV5/mavros/global_position/global", 10, neig_callback5);

  float i_g_x;
  float i_g_y;
  float i_g_z;
  ros::Subscriber s_p_sub = nh.subscribe("/Source/mavros/global_position/global", 10, source_callback);
  ros::Subscriber gas_self_0_sub = nh.subscribe("gas_sensor_value_0", 1, sensor_callback0);
  ros::Subscriber gas_self_1_sub = nh.subscribe("gas_sensor_value_1", 1, sensor_callback1);
  ros::Subscriber gas_self_2_sub = nh.subscribe("gas_sensor_value_2", 1, sensor_callback2);
  ros::Subscriber gas_self_3_sub = nh.subscribe("gas_sensor_value_3", 1, sensor_callback3);
  ros::Subscriber gas_self_4_sub = nh.subscribe("gas_sensor_value_4", 1, sensor_callback4);

  ros::Subscriber wind_sub = nh.subscribe("/wind_data", 10, wind_callback);
  float source_x;
  float source_y;
  double source_Q, source_D, source_tau, building_size;

  nh.param("source_Q", source_Q, 1.3*10.0);
  nh.param("source_D", source_D, 3.0*pow(10.0,-3.0));
  nh.param("source_tau", source_tau, 1200.0);
  nh.param("building_size", building_size, 1.0);

  double nx, ny, fixed_h, ext_length, move_frac, dt;
  double sen_sig_m_est, env_sig;
  nh.param("search_nx", nx, 20.0);
  nh.param("search_ny", ny, 20.0);
  nh.param("uav_altitude", fixed_h, 2.0);
  nh.param("extra_length", ext_length, 0.5);
  nh.param("adaptive_move_frac", move_frac, 10.0);
  nh.param("sensing_time", dt, 2.0);
  nh.param("sensor_sig_multip_est", sen_sig_m_est, 0.1);
  nh.param("env_sig", env_sig, 1.0);

  double poten_sig;
  nh.param("potential_field_sigma", poten_sig, 1.0);

  float ax_min, ax_max, ay_min, ay_max;
  ax_min = 0-ext_length;
  ay_min = 0-ext_length;
  ax_max = nx+ext_length;
  ay_max = ny+ext_length;

  float xx = 0.0, yy = 0.0, zz = 0.0;
  float x_tgt, y_tgt, z_tgt;
  x_tgt = 0.0;
  y_tgt = 0.0;
  z_tgt = fixed_h;

  int max_step;  
  nh.param("max_steps", max_step, 100);
  int num_particle;
  nh.param("number_of_particles", num_particle, 1000);

  double xp[num_particle], yp[num_particle], qp[num_particle];
  double dp[num_particle], wp[num_particle], wpnorm[num_particle];

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine mersenne (seed);
  std::uniform_real_distribution<double> particles(0.0, 1.0);

  
  for(int i=0; i<num_particle; i++)
  {
    xp[i] = particles(mersenne) * nx;
    yp[i] = particles(mersenne) * ny;
    qp[i] = particles(mersenne) * 2.0 * source_Q;
    dp[i] = particles(mersenne) * 10.0;
    wp[i] = 1.0;
    wpnorm[i] = 1.0/num_particle;
  }

  for(int i=0; i<num_agent; i++)
  {
    temp_n_x[i] = 0.0;
    temp_n_y[i] = 0.0;
    temp_n_z[i] = 0.0;
    temp_n_g[i] = 0.0;
  }

  float sensing_time = 2.0;

  int success_fail;
  int iter = 0.0;

  ros::Rate loop_rate(10);

  //--------------------------------Strat loop-----------------------------------
  float c_x, c_y, c_z;
  float err_x, err_y, err_z, dist_err;
  double mean_xp=0.0, mean_yp=0.0, mean_qp=0.0, mean_dp=0.0;
  double var_x, var_y, var_q, var_d;
  float spread_p;
  float xnew[5], xnext[5], ynew[5], ynext[5], znext[5];
  float gas_self_value[num_gas];

  float wind_dir_mean, wind_vel_mean;
  double timer, timer_init;
  int sensing_count;
  double sp_x, sp_y, sp_z;

  double gas_n[num_gas];

  double cum_wp[num_particle], takes[num_particle], sum_wp;
  int indx[num_particle];

  int mm =4.0;
  double aa, hopt;

  double rxp[num_particle], ryp[num_particle], rqp[num_particle], rdp[num_particle];

  double ga_new_pi[num_particle], na_new_pi[num_particle];
  double *ga_pi;
  double *na_pi;
  double n_eff, n_eff_inv;

  const int poten_map_x = (int)(nx*2);
  const int poten_map_y = (int)(ny*2);
  double map[poten_map_x][poten_map_y];
  for (int i=0; i<poten_map_x; i++)
  {
    for (int j=0; j<poten_map_y; j++)
    {
      map[i][j] = 1;
    }
  }

  var_x = var_cal(xp, wpnorm, num_particle);
  var_y = var_cal(yp, wpnorm, num_particle);
  bool start_ex = false;
  bool init_x = false, init_y = false, init_z = false;
  bool init_s_x = false, init_s_y = false, init_s_z = false;
  while (ros::ok())
  {

    if ( (-0.0001>g_p_x || g_p_x>0.0001 ) && init_x == false)
    {
      i_g_x = g_p_x;
      init_x = true;
    }
    if ( (-0.0001>g_p_y || g_p_y>0.0001 ) && init_y == false)
    {
      i_g_y = g_p_y;
      init_y = true;
    }
    if ( (-0.0001>g_p_z || g_p_z>0.0001 ) && init_z == false)
    {
      i_g_z = g_p_z;
      init_z = true;
    }
    if ( (-0.0001>s_x || s_x>0.0001 ) && init_s_x == false)
    {
      init_s_x = true;
    }
    if ( (-0.0001>s_y || s_y>0.0001 ) && init_s_y == false)
    {
      init_s_y = true;
    }
    if ( (-0.0001>s_z || s_z>0.0001 ) && init_s_z == false)
    {
      init_s_z = true;
    }
    if (init_x == true && init_y == true && init_z == true && init_s_x == true && init_s_y == true && init_s_z == true)
    {
      start_ex = true;
    }
//cout<< "iter = " << iter << endl;
//cout<< "max_step = " << max_step << endl;
    if (start_ex == true && iter<max_step)
    {
      source_x = (s_x - i_g_x)*pow(10.0,5.0);
      source_y = (s_y - i_g_y)*pow(10.0,5.0);

      c_x = g_p_x;
      c_y = g_p_y;
      c_z = g_p_z;

      target_msg.longitude = x_tgt*pow(10.0,-5.0)+i_g_x;
      target_msg.latitude = y_tgt*pow(10.0,-5.0)+i_g_y;
      target_msg.altitude = z_tgt+i_g_z;
      target_msg.yaw = 0.0;
      target_msg.header.stamp = ros::Time::now();
      setpoint_pub.publish(target_msg);

      err_x = x_tgt - (c_x - i_g_x)*pow(10.0,5.0);
      err_y = y_tgt - (c_y - i_g_y)*pow(10.0,5.0);
      err_z = z_tgt - (c_z - i_g_z);
      dist_err = sqrt(err_x*err_x + err_y*err_y + err_z*err_z);
      cout<< "tgt_x: "<< x_tgt << "  tgt_y: " << y_tgt << "  tgt_z: " << z_tgt << "  dist_err = " << dist_err << endl;

	current_m_msg.pose.position.x = xx;
	current_m_msg.pose.position.y = yy;
	current_m_msg.pose.position.z = zz;
        current_m_msg.pose.orientation.w = gas_s_v;
	current_m_msg.header.stamp = ros::Time::now();
	current_m_pub.publish(current_m_msg);

      if (dist_err <= 0.2)
      {
        // Intializing the variable for every time steps 
        for (int i=0; i<num_agent; i++)
        { trig[i] = false; }

        spread_p = sqrt(var_x+var_y);

        ynew[0] = 1.0+spread_p/move_frac;
        ynew[1] = -(1.0+spread_p/move_frac);
        ynew[2] = 0;
        ynew[3] = 0;
        xnew[0] = 0;
        xnew[1] = 0;
        xnew[2] = 1.0+spread_p/move_frac;
        xnew[3] = -(1.0+spread_p/move_frac);

        gas_self_value[0] = 0.0; gas_self_value[1] = 0.0; gas_self_value[2] = 0.0;
        gas_self_value[3] = 0.0; gas_self_value[4] = 0.0;
        wind_dir_mean = 0.0;
        wind_vel_mean = 0.0;
        sp_x = 0.0;
        sp_y = 0.0;
        sp_z = 0.0;

        timer_init = std::time(0);
        timer = std::time(0);
        sensing_count = 0.0;

        // Wait for sensing
        while (timer < timer_init+dt) // ROS thread is pause for some seconds 
        {
          mode_msg.range = 1.0;
          mode_msg.header.stamp = ros::Time::now();
          mode_pub.publish(mode_msg);

          target_msg.header.stamp = ros::Time::now();
          setpoint_pub.publish(target_msg);

  	  current_m_msg.header.stamp = ros::Time::now();
	  current_m_pub.publish(current_m_msg);

          sp_x = sp_x + g_p_x;
          sp_y = sp_y + g_p_y;
          sp_z = sp_z + g_p_z;

          for (int i=0; i<num_gas; i++)
          { gas_self_value[i] = gas_self_value[i] + gas_value[i]; }

          wind_dir_mean = wind_dir_mean + wind_dir;
          wind_vel_mean = wind_vel_mean + wind_vel;

          timer = std::time(0);

        //ros::spinOnce();
        //loop_rate.sleep();
          usleep(100000); //0.1s
          sensing_count = sensing_count + 1.0;
        }
cout << endl << "sensing_count = " << sensing_count;
        mode_msg.range = 0.0;
        mode_msg.header.stamp = ros::Time::now();
        mode_pub.publish(mode_msg);

        target_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(target_msg);

        // Calculate own current data and publish
        xx = (sp_x/sensing_count - i_g_x)*pow(10.0,5.0);
        yy = (sp_y/sensing_count - i_g_y)*pow(10.0,5.0);
        zz = sp_z/sensing_count - i_g_z;
        gas_s_v = 0.0;
        for (int i=0.0; i<num_gas; i++)
        {
          gas_self_value[i] = gas_self_value[i]/sensing_count;
          gas_s_v = gas_s_v + gas_self_value[i]/num_gas;
        }
	current_m_msg.pose.position.x = xx;
	current_m_msg.pose.position.y = yy;
	current_m_msg.pose.position.z = zz;
        current_m_msg.pose.orientation.w = gas_s_v;
	current_m_msg.header.stamp = ros::Time::now();
	current_m_pub.publish(current_m_msg);

        wind_dir_mean = wind_dir_mean/sensing_count;
        wind_vel_mean = wind_vel_mean/sensing_count;
        cout << endl << "zz = " << zz << endl;

        int num_resol = 20;
        double gauss_x[num_resol];
        if (gas_s_v <= 0.0)
        {
          for (int i=0; i<num_resol; i++)
          {
            gauss_x[i] = (1.0*i/num_resol)*3.0;
          }
        } 
        else
        {
          for (int i=0; i<num_resol; i++)
          {
            gauss_x[i] = (gas_s_v*i/num_resol)*3.0;
          }
        }

        ros::spinOnce();
        loop_rate.sleep();

        target_msg.header.stamp = ros::Time::now();
        setpoint_pub.publish(target_msg);

        // Receiving the current measurements
        ros::spinOnce();
        loop_rate.sleep();

        // Particle filter update
        for (int i=0; i<num_particle; i++)
        { ga_new_pi[i] = 1.0; }
        for (int i=0; i<num_agent; i++)
        {
          if(neighbor_x[i]!=temp_n_x[i] || neighbor_y[i]!=temp_n_y[i] || neighbor_z[i]!=temp_n_z[i])
          {
            trig[i] = true;
            temp_n_x[i] = neighbor_x[i]; //save current changed neighbor pose
            temp_n_y[i] = neighbor_y[i];
            temp_n_z[i] = neighbor_z[i];
          }
          cout << "trig = " << trig[i] << endl; //check neighbor data received
        }
        cout << endl;

        ga_pi = bayesian_update(xp, yp, qp, dp, ga_new_pi, nx, ny, ext_length, wind_vel_mean, wind_dir_mean, num_particle, sen_sig_m_est, env_sig, source_tau, dt); //using all neighbors' data in function
 
        for (int i=0; i<num_particle; i++)
        { wpnorm[i] = wpnorm[i]*ga_new_pi[i]; }
        sum_wp = 0;
        for (int i=0; i<num_particle; i++)
        { sum_wp += wpnorm[i]; }

        for (int i=0; i<num_particle; i++)
        { wpnorm[i] = wpnorm[i]/sum_wp; }

        // Resampling
        n_eff_inv = 0;
        for (int i=0; i<num_particle; i++)
        { n_eff_inv += (wpnorm[i]*wpnorm[i]); }
        n_eff = 1.0/n_eff_inv;
cout << "e_ff =" << n_eff << endl;
        if (n_eff < num_particle*0.5)
        {
          target_msg.header.stamp = ros::Time::now();
          setpoint_pub.publish(target_msg);

          // resample stratified
          cum_wp[0] = wpnorm[0];
          takes[0] = particles(mersenne)/num_particle;
          for (int i=1; i<num_particle; i++)
          {
            cum_wp[i] = cum_wp[i-1]+wpnorm[i];
            takes[i] = (1.0/num_particle)*i + particles(mersenne)/num_particle;
          }
          int j = 0;
          double alpha;
          double mcrand;
          for(int i=0; i<num_particle; i++)
          {
//cout << "cum_wp = " << cum_wp[i] << "	takes = " << takes[i] << "	i = " << i <<endl;
            while (cum_wp[j] < takes[i])
            {
              j = j+1;
            }
            indx[i] = j;
//cout<< "indx = " << indx[i] << endl;
            xp[i] = xp[indx[i]];
            yp[i] = yp[indx[i]];
            qp[i] = qp[indx[i]];
            dp[i] = dp[indx[i]];
            wpnorm[i] = 1.0/num_particle;
          }

          var_x = var_cal(xp, wpnorm, num_particle);
          var_y = var_cal(yp, wpnorm, num_particle);
          var_q = var_cal(qp, wpnorm, num_particle);
          var_d = var_cal(dp, wpnorm, num_particle);

          aa = pow(4.0/(mm+2.0),1/(mm+4.0));
          hopt = aa*pow(num_particle,-1.0/(mm+4.0));
          for (int j=0; j<3; j++)
          {
            for (int i=0; i<num_particle; i++)
            {
              rxp[i] = xp[i] + ( hopt*sqrt(var_x)*particles(mersenne) );
              ryp[i] = yp[i] + ( hopt*sqrt(var_y)*particles(mersenne) );
              rqp[i] = qp[i] + ( hopt*sqrt(var_q)*particles(mersenne) );
              rdp[i] = dp[i] + ( hopt*sqrt(var_d)*particles(mersenne) );
              na_new_pi[i] = 1.0;
            }
            na_pi = bayesian_update(rxp, ryp, rqp, rdp, na_new_pi, nx, ny, ext_length, wind_vel_mean, wind_dir_mean, num_particle, sen_sig_m_est, env_sig, source_tau, dt);
            for (int i=0; i<num_particle; i++)
            {
              alpha = na_pi[i]/ga_pi[indx[i]];
              mcrand = particles(mersenne);
              if (alpha > mcrand)
              {
                xp[i] = rxp[i];
                yp[i] = ryp[i];
                qp[i] = rqp[i];
                dp[i] = rdp[i];
                wpnorm[i] = 1.0/num_particle;
              } // if selected by mcmc
            } // for particle filter
          } //for 3times
        } // if satisfies resampling
        mean_xp = mean_cal(xp, wpnorm, num_particle);
        mean_yp = mean_cal(yp, wpnorm, num_particle);
        mean_qp = mean_cal(qp, wpnorm, num_particle);
        mean_dp = mean_cal(dp, wpnorm, num_particle);

        double var[4];
        bool obstacles[4];
        double dist_ob[num_agent+1], dist_ob_x[num_agent+1], dist_ob_y[num_agent+1];
        for (int i=0; i<num_agent+1; i++)
        {
          dist_ob_x[i] = 100000;
          dist_ob_y[i] = 100000;
        }
        dist_ob_x[0] = source_x - xx;
        dist_ob_y[0] = source_y - yy;
        dist_ob[0] = sqrt(pow(dist_ob_x[0],2) + pow(dist_ob_y[0],2) );

        for (int i=0; i<num_agent; i++)
        {
          if (trig[i] == true )
          {
            dist_ob_x[i+1] = neighbor_x[i] - xx;
            dist_ob_y[i+1] = neighbor_y[i] - yy;
            dist_ob[i+1] = sqrt(pow(dist_ob_x[i+1],2) + pow(dist_ob_y[i+1],2) );
            if (dist_ob[i+1] < 0.01 )
            { dist_ob[i+1] = 100000; }
          }
        }
        double dist_throu;
        double deter_x_next_ob;

        double pdetrate, pdetconc;
        double ga_val_temp, g_d_s[num_particle][num_resol], gds_temp[num_resol];
        double zu_tot[num_particle], zu_sum[num_resol], gds_sum;
        double zwpnorm[num_particle], wpnorm_temp[num_particle];

        double entropy_next, entropy_now, entropy_delta, val;
        int ind = 0;

        for (int kk=0; kk<4; kk++)
        {
          xnext[kk] = xx + xnew[kk];
          ynext[kk] = yy + ynew[kk];
          znext[kk] = fixed_h;
          obstacles[kk] = false;

          target_msg.header.stamp = ros::Time::now();
          setpoint_pub.publish(target_msg);

          for (int i=0; i<num_agent+1; i++)
          {
            if (dist_ob[i] < ynew[0]+building_size/2)
            {
              deter_x_next_ob = abs(xnew[kk]*dist_ob_y[i]-ynew[kk]*dist_ob_x[i+1]);
              dist_throu = deter_x_next_ob/sqrt(pow(xnew[kk],2)+pow(ynew[kk],2));
              if (dist_throu <= building_size/2)
              {
                double dZero = 0.0;
                var[kk] = -1.0/dZero;
                obstacles[kk] = true;
                cout << "conflicts to obstacles: ";
                cout << " x = " << xnext[kk];
                cout << " y = " << ynext[kk] << endl;
                cout << "Utility function[" << kk << "] = " << var[kk] << endl;
                continue;
              }
            }
          }
          if (xnext[kk]<ax_min || ynext[kk]<ay_min || xnext[kk]>ax_max || ynext[kk]>ay_max)
          {
            double dZero = 0.0;
            var[kk] = -1.0/dZero;
            cout << "out of area: " << " x = " << xnext[kk] << " y = " << ynext[kk] << endl;
            cout << "Utility function[" << kk << "] = " << var[kk] << endl;
            continue;
          }
          if (obstacles[kk] == false)
          {
            entropy_delta = 0;
            for (int i=0; i<num_particle; i++)
            {
              pdetrate = mean_concentration(xnext[kk],ynext[kk],znext[kk],xp[i],yp[i],qp[i],wind_vel_mean,wind_dir_mean,dp[i],source_tau);
              pdetconc = pdetrate*dt;
              for (int j=0; j<num_resol; j++)
              { gds_temp[j] = normalCDF(gauss_x[j], pdetconc, pdetconc*sen_sig_m_est + env_sig); }
	      if (gds_temp[num_resol-1] == 0)
              {
                for (int j=0; j<num_resol; j++)
                { gds_temp[j] = 1.0/(double)num_resol; }
              }
              else
              {
                for (int j=0; j<num_resol; j++)
                { gds_temp[j] = gds_temp[j]/gds_temp[num_resol-1]; }
              }
              for (int j=1; j<num_resol; j++)
              { g_d_s[i][j] = gds_temp[j] - gds_temp[j-1]; }
              g_d_s[i][0] = gds_temp[0];
//cout << gds_temp[num_resol-1] << endl;
            }
            for (int j=0; j<num_resol; j++)
            {
              zu_sum[j] = 0;
              for (int i=0; i<num_particle; i++)
              {
                zu_tot[i] = g_d_s[i][j]*wpnorm[i];
                zu_sum[j] += zu_tot[i];
              }
//cout << "sum_wp[" << j << "] = " << sum_wp << endl;
//cout << "zu_sum[" << j << "] = " << zu_sum[j] << endl;
              for (int i=0; i<num_particle; i++)
              {
                if (zu_sum[j] == 0)
                { zwpnorm[i] = 1; }
                else
                { zwpnorm[i] = zu_tot[i]/zu_sum[j]; }

                if(zwpnorm[i] == 0)
                { zwpnorm[i] = 1; }

                if(wpnorm[i] == 0)
                { wpnorm_temp[i] = 1; }
                else
                { wpnorm_temp[i] = wpnorm[i]; }

                entropy_next = -zu_sum[j]*zwpnorm[i]*log2(zwpnorm[i]);
                entropy_now = -wpnorm_temp[i]*log2(wpnorm_temp[i]);
                entropy_delta += -(entropy_next - entropy_now) ;

                if (isnan(entropy_delta))
                {
               /*   for (int j=0; j<num_resol; j++)
                  {
                    cout <<endl <<"gds_temp["<<j<< "] = " <<gds_temp[j] << endl;
                    cout << "gauss_x[" << j <<"] = " <<gauss_x[j] << endl <<endl;
                  }*/
                  cout << "entropy_next = " << entropy_next << endl;
                  cout << "zwpnorm["<<i <<"][" << j << "] = " << zwpnorm[i] << endl;
                  cout << "entropy_now = " << entropy_now << endl;
                 
                  break;
                }
              }
            }
            int x_poten = (int)round(xnext[kk]);
            int y_poten = (int)round(ynext[kk]);
            if (x_poten < 0)
            { x_poten = 0; }
            if (y_poten < 0)
            { y_poten = 0; }
            var[kk] = entropy_delta/map[x_poten][y_poten];
            cout << "Utility function[" << kk << "] = " << var[kk] << endl;

            if (kk == 1)
            {
              ind = 1;
              val = var[1];
            }
            else
            {
              if(val<var[kk])
              {
                ind = kk;
                val = var[kk];
              }
            }
          } // if there is in search area
        } // for calculate utility function 4 directions
        x_tgt = xnext[ind];
        y_tgt = ynext[ind];
        z_tgt = znext[ind];

        for (int px=0; px<poten_map_x; px++)
        {
          for (int py=0; py<poten_map_y; py++)
          {
            map[px][py] = map[px][py] + (1/(poten_sig*sqrt(2*M_PI))*exp(-(pow(px/2-round(xx),2)+pow(py/2-round(yy),2) )/(2*poten_sig*poten_sig) ) )*0.1;
          }
        }
      } // if reaching setpoint
      else
      {
        for (int i=0; i<num_agent; i++)
        {
          if (-0.0001>neig_x[i] || neig_y[i]>0.0001 )
          {
            double dist_x = (g_p_x - neig_x[i])*pow(10.0,5.0);
            double dist_y = (g_p_y - neig_y[i])*pow(10.0,5.0);
            double dist_xy = sqrt(dist_x*dist_x + dist_y*dist_y);
            if (dist_xy >0.01 && dist_xy < 2)
            {
              target_msg.longitude = g_p_x;
              target_msg.latitude = g_p_y;
              target_msg.altitude = g_p_z;
              target_msg.yaw = 0.0;
              target_msg.header.stamp = ros::Time::now();
              setpoint_pub.publish(target_msg);
              cout << "****** Emergency stop ********** " << endl;
     // cout<< "tgt_x: "<< g_p_x << "  tgt_y: " << g_p_y << "  tgt_z: " << g_p_z << endl;
            }
          }
        }

        ros::spinOnce();
        loop_rate.sleep();
      }

      //declare message and sizes
      sensor_msgs::PointCloud2 cloud;
      cloud.header.stamp = ros::Time::now();
      cloud.header.frame_id = "map";
      cloud.width  = num_particle;
      cloud.height = 1;
      cloud.is_bigendian = false;
      cloud.is_dense = false; // there may be invalid points

      //for fields setup
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2FieldsByString(2,"xyz","rgb");
      modifier.resize(num_particle);

      //iterators
      sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
      sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud, "r");
      //sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud, "g");
      //sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud, "b");

      for(int i=0; i<num_particle; i++)
      {
        *out_x = xp[i];
        *out_y = yp[i];
        *out_z = qp[i];

        *out_r = wpnorm[i];

        
        //increment
        ++out_x;
        ++out_y;
        ++out_z;
        ++out_r;
      }
      pf_pub.publish(cloud);
    } // if start the experiment
    else
    {
      current_m_msg.pose.position.x = xx;
      current_m_msg.pose.position.y = yy;
      current_m_msg.pose.position.z = zz;
      current_m_msg.pose.orientation.w = gas_s_v;
      current_m_msg.header.stamp = ros::Time::now();
      current_m_pub.publish(current_m_msg);

      cout << "start_ex = " << start_ex << endl;


      ros::spinOnce();
      loop_rate.sleep();
    }
  }


  return 0;
}
