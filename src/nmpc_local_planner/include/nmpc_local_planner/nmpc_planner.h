#ifndef NMPC_PLANNER_H_
#define NMPC_PLANNER_H_

#include <time.h>
#include <math.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include <boost/bind.hpp>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <functional>
#include <vector>
#include <nmpc_local_planner/obstacle_update.h>
#include <nmpc_local_planner/NmpcLocalPlannerReconfigureConfig.h>

#include <casadi/casadi.hpp>


namespace nmpc_local_planner
{

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using namespace std;
using namespace casadi;

typedef struct _ROBOT_STATE
{
    double X;
    double Y;
    double Theta;
    double Speed;

}ROBOT_STATE;

typedef struct _OBSTACLE_STRUCT
{
    double X;
    double Y;
    double R1;
    double R2;
    double Theta;
}OBSTACLES;


typedef struct _WEIGHTING
{
    double x;           // Weighting matrix for X-coordinate
    double y;           // Weighting matrix for Y-coordinate
    double theta;         // Weighting factor for heading
    double speed;    // Weighting matrix for velocity
    double final_x;     // Weighting matrix for X-coordinate at final point
    double final_y;     // Weighting matrix for Y-coordinate at final point
    double final_theta;
    double final_speed;
    double acc;
    double steer;    // Weighting matrix for steering angle
    double steer_acc;

    double left_side_pnt;
    double right_side_pnt;
    double obs_pnt;

}WEIGHTS;


typedef struct _NMPC_STRUCT
{

    /* Model parameters */
    int N_x;     // Number of state variables
    int N_u;     // Number of control variables
    int N_xu;    // Number of optimization variables per time intervall
    // int N_coll;  // Number of collocation points

    /* Optimization parameters */
    int N_dt;           // Number of time intervals
    double delta_time;                // Length of the time intervals in [s]

}NMPC_CONFIG;

class NMPC_planner
{
    public:
        NMPC_planner();

        void reconfigure(NmpcLocalPlannerReconfigureConfig& config);

        bool configOCP();
        void loadROSParam(ros::NodeHandle& nh);
        void calculateOCP(vector<geometry_msgs::PoseStamped>& robot_plan, std::vector<geometry_msgs::PoseStamped> &leftSidewalk_plan, std::vector<geometry_msgs::PoseStamped> &rightSidwalk_plan, 
                        geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& robot_vel, costmap_2d::Costmap2D& costmap, const double &plan_length);

        void insertSegmentObs(b_segment seg, double inflation_rate);
        void insertCircleObs(circle circ);
        bool checkObstacleFootprintIntersection(const std::vector<geometry_msgs::PoseStamped> &plan, 
                                            const std::vector<geometry_msgs::Point> &_footprint_spec, const std::vector<b_segment> &seg_obs_, const std::vector<circle> &circ_obs_);
        void visualize(std::vector<geometry_msgs::PoseStamped> &ref_plan, std::vector<geometry_msgs::PoseStamped> &sol_plan);

        double _speed, _steer;
        NMPC_CONFIG _nmpc;

        double _dt;
        WEIGHTS weights, default_weights;
        double side_weights;
        
        // NLP variable bounds and initial guess
        std::vector<double> v_min,v_max,v_init;

        // int nx, nu, ns;
        int nobs;

        std::vector<ROBOT_STATE> reference_traj, nmpc_traj;

        std::vector<double> nlp_params;   
        std::vector<double> path_coeff_val,ct_coeff_val,lt_coeff_val,rt_coeff_val,obs_val,w_cost_val,wobs_val,wref_val,x_tf_val,w_tf_val,offset_val;           
        std::vector<OBSTACLES> _obstacles_vec;

        Function solver;
        std::vector<double> lbg, ubg;
        
        bool use_c_code, use_irk, generate_c;
        int print_level;
        string linear_solver, nlp_solver_name;

        bool obstaclePresent;
    
    protected:
    
        void updateObstacleWithCostmap(costmap_2d::Costmap2D& costmap);

    private:

        void generateSpeedProfile(double& path_length, double& max_speed, int& num_pts);
        // double find_t_given_S(double& S_given, double& t_0, double& t1);

        double acc_lim_x_, acc_lim_y_, acc_lim_theta_, speed_lim_, steer_lim_;
                
        double _xICR, _body_width;
        int _num_trajecotry_points;
        std::vector<double> _model_param;
        boost::mutex configure_mutex_;
        double _seg_inflation_rate;

};
}

#endif