#include "nmpc_local_planner/nmpc_planner.h"
#include "nmpc_local_planner/ref_path_planner.h"
#include <base_local_planner/goal_functions.h>
#include <tf2/utils.h>
#include <ros/package.h>

namespace nmpc_local_planner
{
    // ====================================
    // NMPC class definition implementation.
    // ====================================

    // NMPC_STRUCT* NMPC_planner::_nmpc;

    NMPC_planner::NMPC_planner() 
    {
        // _nmpc = (NMPC_STRUCT*)calloc(1,sizeof(NMPC_STRUCT));
    }

    void NMPC_planner::reconfigure(NmpcLocalPlannerReconfigureConfig& config)
    {
        boost::mutex::scoped_lock l(configure_mutex_);
        // acc_lim_x_ = config.acc_lim_x;
        
        default_weights.x = config.weight_x;
        default_weights.y = config.weight_y;
        default_weights.theta = config.weight_theta;  
        default_weights.speed = config.weight_speed;
        default_weights.final_x = config.weight_final_x;
        default_weights.final_y = config.weight_final_y;
        default_weights.final_theta = config.weight_final_theta;
        default_weights.final_speed = config.weight_final_speed;
        default_weights.acc = config.weight_acc;
        default_weights.steer = config.weight_steer;
        default_weights.left_side_pnt =  config.weight_left_side;
        default_weights.right_side_pnt =  config.weight_right_side;
        default_weights.obs_pnt = config.weight_obs;

    }

    void NMPC_planner::visualize(std::vector<geometry_msgs::PoseStamped> &ref_plan, std::vector<geometry_msgs::PoseStamped> &sol_plan)
    {
        geometry_msgs::PoseStamped nmpc_pose,ref_pose;

        for(int i = 0; i < _nmpc.N_dt; i++)
        {
            
            tf2::Quaternion q;

            ref_pose.header.frame_id = "base_link";
            ref_pose.pose.position.x = reference_traj[i].X;
            ref_pose.pose.position.y = reference_traj[i].Y;
            q.setRPY( 0, 0, reference_traj[i].Theta);                  

            tf2::convert(q,ref_pose.pose.orientation);
            ref_plan.push_back(ref_pose);  

            ROS_DEBUG("ref---x:%.3f,y:%.3f,theta:%.3f",
                reference_traj[i].X,reference_traj[i].Y,
                reference_traj[i].Theta);       

            nmpc_pose.header.frame_id = "base_link";

            nmpc_pose.pose.position.x = nmpc_traj[i].X;
            nmpc_pose.pose.position.y = nmpc_traj[i].Y;
            q.setRPY( 0, 0, nmpc_traj[i].Theta );

            tf2::convert(q,nmpc_pose.pose.orientation);
            sol_plan.push_back(nmpc_pose);
            ROS_DEBUG("nmpc---x:%.3f,y:%.3f,theta:%.3f",
                nmpc_traj[i].X,nmpc_traj[i].Y,
                nmpc_traj[i].Theta);
        } 
    }


    void NMPC_planner::updateObstacleWithCostmap(costmap_2d::Costmap2D& costmap)
    {
        // Add costmap obstacles if desired
        // if (_params.include_costmap_obstacles)
        // {
            // Eigen::Vector2d robot_orient = _robot_pose.orientationUnitVec();

            for (unsigned int i = 0; i < costmap.getSizeInCellsX() - 1; ++i)
            {
                for (unsigned int j = 0; j < costmap.getSizeInCellsY() - 1; ++j)
                {
                    if (costmap.getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
                    {
                        Eigen::Vector2d obs;
                        costmap.mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));
                        // ROS_INFO("Obs position %f %f",obs.coeffRef(0), obs.coeffRef(1));
                        // check if obstacle is interesting (e.g. not far behind the robot)
                        // Eigen::Vector2d obs_dir = obs - _robot_pose.position();
                        // if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > _params.costmap_obstacles_behind_robot_dist) continue;

                        // _obstacles.push_back(ObstaclePtr(new PointObstacle(obs)));
                    }
                }
            }
        // }
    }
    void NMPC_planner::loadROSParam(ros::NodeHandle& nh)
    {
        nh.param("xICR",_xICR,0.00248); 
        nh.param("width",_body_width,0.5);
        nh.param("acc_lim_x",acc_lim_x_,2.0);
        nh.param("acc_lim_y",acc_lim_y_,2.0);    
        nh.param("speed_lim",speed_lim_,1.0); 
        nh.param("steer_lim",steer_lim_,3.14);

        nh.param("num_trajecotry_points", _num_trajecotry_points, 30);            
        nh.param("seg_inflation_rate", _seg_inflation_rate, 0.1);

        nh.param("weight_x",default_weights.x, 20.0);            
        nh.param("weight_y",default_weights.y , 20.0);            
        nh.param("weight_theta",default_weights.theta, 0.0);            
        nh.param("weight_speed",default_weights.speed, 1.0);    
        nh.param("weight_final_x",default_weights.final_x, 2.0);            
        nh.param("weight_final_y",default_weights.final_y, 5.0);            
        nh.param("weight_final_theta",default_weights.final_theta, 0.0);    
        nh.param("weight_final_speed",default_weights.final_speed, 5.0);   
        nh.param("weight_acc",default_weights.acc, 1.0); 
        nh.param("weight_steer",default_weights.steer, 1.0);            
        nh.param("weight_steer_acc",default_weights.steer_acc, 1.0);   

        nh.param("weight_left_side",default_weights.left_side_pnt, 1.0);   
        nh.param("weight_right_side",default_weights.right_side_pnt, 1.0);   
        nh.param("weight_obs",default_weights.obs_pnt, 1.0);   

        // nh.param("weight_sides",side_weights, 1.0);   

        nh.param("time_interval",_dt, 0.1);      

        nh.param("use_c_code", use_c_code, true);
        nh.param("use_irk", use_irk, false);
        nh.param("generate_c", generate_c, false);
        nh.param("print_level", print_level, 0);
        nh.param<string>("linear_solver", linear_solver, "mumps");
        nh.param<string>("nlp_solver_name", nlp_solver_name, "nmpc_cartesian_dyn_whizzy");

    }

    bool NMPC_planner::configOCP()
    {
        // model parameters
        // _nmpc.xICR = _xICR;
        _model_param.clear();
        _model_param.push_back(_xICR);
        _model_param.push_back(_body_width);
        // Optimization parameters 
        // Model parameters
        _nmpc.N_x     = 4;    // Number of state variables
        _nmpc.N_u     = 2;    // Number of control variables
        _nmpc.N_xu = (_nmpc.N_x + _nmpc.N_u);    // Number of optimization variables per time intervall
        _nmpc.N_dt = _num_trajecotry_points; 		   // Number of time intervals  //12
        _nmpc.delta_time = _dt;  // Length of the time intervals in [s]

        // Weighting parameters
        weights = default_weights;

        obstaclePresent = false;


        int nx = _nmpc.N_x;
        int nu = _nmpc.N_u;
        int ns = _nmpc.N_dt; // Number of shooting nodes
        nobs = 10;

        lbg.clear();
        ubg.clear();
        nlp_params.reserve((nx+nu)*ns+100);
        _speed = 0.0;
        _steer= 0.0;
        _obstacles_vec.reserve(10);
        _obstacles_vec.clear();

        reference_traj.clear();
        nmpc_traj.clear();


        // Set options
        Dict opts;
        opts["ipopt.tol"] = 1e-5;
        opts["ipopt.max_iter"] = 100;
        opts["ipopt.print_level"] = print_level;
        opts["ipopt.sb"] = "yes";
        opts["ipopt.acceptable_tol"] = 1e-5;
        opts["ipopt.acceptable_iter"] = 0;
        opts["ipopt.linear_solver"] = linear_solver;
        // opts["ipopt.hessian_approximation"] = "limited-memory";
        opts["ipopt.warm_start_init_point"] = "yes";
        opts["ipopt.warm_start_bound_push"] = 1e-6;
        opts["ipopt.warm_start_mult_bound_push"] = 1e-6;
        opts["print_time"] = 0;        

        string package_name = ros::package::getPath("nmpc_local_planner");
        string nlp_solver_name = "nmpc_cartesian_dyn_whizzy";  
        solver = nlpsol("solver", "ipopt", package_name+"/gen_c/"+nlp_solver_name+".so",opts);


        // Bounds and initial guess for the control
        vector<double> u_min =  { -acc_lim_x_, -steer_lim_ };
        vector<double> u_max  = { acc_lim_x_, steer_lim_ };
        vector<double> u_init = {  0.0, 0.0  };

        // Bounds and initial guess for the state
        vector<double> x0_min = {0, 0, 0, 0};
        vector<double> x0_max = {0, 0, 0, 0};
        vector<double> x_min  = {-inf, -inf, -inf, 0};
        vector<double> x_max  = { inf,  inf, inf, speed_lim_};
        vector<double> xf_min = { -inf, -inf, -inf, 0};
        vector<double> xf_max = { inf,  inf, inf, speed_lim_};
        vector<double> x_init = {0, 0, 0, 0};

        double side_safe_dist = 0.5;

        
        for(int k=0; k<ns; ++k){
            if(k==0){
            v_min.insert(v_min.end(), x0_min.begin(), x0_min.end());
            v_max.insert(v_max.end(), x0_max.begin(), x0_max.end());
            } else {
            v_min.insert(v_min.end(), x_min.begin(), x_min.end());
            v_max.insert(v_max.end(), x_max.begin(), x_max.end());
            }
            v_init.insert(v_init.end(), x_init.begin(), x_init.end());
            v_init.insert(v_init.end(), u_init.begin(), u_init.end());

            v_min.insert(v_min.end(), u_min.begin(), u_min.end());
            v_max.insert(v_max.end(), u_max.begin(), u_max.end());
        }

        v_min.insert(v_min.end(), xf_min.begin(), xf_min.end());
        v_max.insert(v_max.end(), xf_max.begin(), xf_max.end());
        v_init.insert(v_init.end(), x_init.begin(), x_init.end());
        // bounds for continuity
        lbg.assign(nx*ns,0);
        ubg.assign(nx*ns,0);   
        // bounds for sidewalk
        // lbg.insert(lbg.end(), ns, -inf);
        // ubg.insert(ubg.end(), ns, -0.5);
        // lbg.insert(lbg.end(), ns, 0.5);
        // ubg.insert(ubg.end(), ns, inf);

        obs_val.assign(50,nobs);
        wobs_val.assign(nobs,0);

        return true;
    }

    void NMPC_planner::calculateOCP(vector<geometry_msgs::PoseStamped>& robot_plan, std::vector<geometry_msgs::PoseStamped> &leftSidewalk_plan, std::vector<geometry_msgs::PoseStamped> &rightSidwalk_plan, geometry_msgs::PoseStamped& global_pose, geometry_msgs::Twist& robot_vel, costmap_2d::Costmap2D& costmap, const double &plan_length)
    {    
        int nx = _nmpc.N_x;
        int nu = _nmpc.N_u;
        int ns = _nmpc.N_dt; // Number of shooting nodes

        boost::mutex::scoped_lock l(configure_mutex_);

        nlp_params.clear();

        int N = robot_plan.size(); // Number of waypoints
        int NL = leftSidewalk_plan.size();
        int NR = rightSidwalk_plan.size();
        bool path_inv = false;
        double left_offset_val = 0;
        double right_offset_val = 0;
        VectorXd x_veh(N);
        VectorXd y_veh(N);
  

        for(int i =0 ; i < N; i++)
        {
            x_veh[i] = robot_plan[i].pose.position.x;
            y_veh[i] = robot_plan[i].pose.position.y; 
        }
        auto coeffs = polyfit(x_veh, y_veh, 3);         
        ct_coeff_val = {coeffs[0], coeffs[1], coeffs[2], coeffs[3]};

        if (!leftSidewalk_plan.empty())
        {
            VectorXd x_veh_left(NL);
            VectorXd y_veh_left(NL);
            for(int i =0 ; i < NL; i++)
            {
                x_veh_left[i] = leftSidewalk_plan[i].pose.position.x;
                y_veh_left[i] = leftSidewalk_plan[i].pose.position.y; 
            }
            auto coeffs = polyfit(x_veh_left, y_veh_left, 3); 
            lt_coeff_val = {coeffs[0], coeffs[1], coeffs[2], coeffs[3]};
            if (x_veh_left[0]>0.1) left_offset_val = x_veh_left[0];
            weights.left_side_pnt = default_weights.left_side_pnt;
        }
        else
        {
            weights.left_side_pnt = 0;
            lt_coeff_val = {0.0, 0.0, 0, 0}; // {0.5,0.2,0,0}
        }
         
        if (!rightSidwalk_plan.empty())
        {
            VectorXd x_veh_right(NR);
            VectorXd y_veh_right(NR);    
            for(int i =0 ; i < NR; i++)
            {
                x_veh_right[i] = rightSidwalk_plan[i].pose.position.x;
                y_veh_right[i] = rightSidwalk_plan[i].pose.position.y; 
            }    
            auto coeffs = polyfit(x_veh_right, y_veh_right, 3); 
            rt_coeff_val = {coeffs[0], coeffs[1], coeffs[2], coeffs[3]};    
            if (x_veh_right[0]>0.1) right_offset_val = x_veh_right[0];
            weights.right_side_pnt = default_weights.right_side_pnt;
        }
        else
        {
            weights.right_side_pnt = 0;
            rt_coeff_val = {0.0, 0.0, 0, 0}; // {-0.5,0.1,0,0}   
        }

        path_coeff_val.clear();
        path_coeff_val.insert(path_coeff_val.end(),ct_coeff_val.begin(),ct_coeff_val.end());
        path_coeff_val.insert(path_coeff_val.end(),lt_coeff_val.begin(),lt_coeff_val.end());
        path_coeff_val.insert(path_coeff_val.end(),rt_coeff_val.begin(),rt_coeff_val.end());

        double x_offset_val = 0;
        if (x_veh[0]>0.1) x_offset_val = x_veh[0];

        
        if (obstaclePresent)
        {
            weights.y = 1;
            weights.steer = 1;
            weights.obs_pnt = default_weights.obs_pnt;
        }
        else
        {
            weights.y = default_weights.y;
            weights.steer = default_weights.steer;
            weights.obs_pnt = 0;
        }




        offset_val = {x_offset_val,left_offset_val,right_offset_val};
        // w_y, w_v, w_acc, w_steer
        wref_val = {weights.y,weights.speed,weights.acc,weights.steer,weights.left_side_pnt,weights.right_side_pnt}; // {10,5,5,5};
        // x_obs, y_obs, r1, r2, theta_obs
        // ROS_INFO("weighting values %f, %f, %f, %f",weights.y,weights.speed,weights.acc,weights.steer);
        int obs_cnt = 0;
        for (auto obs_it = _obstacles_vec.begin(); obs_it != _obstacles_vec.end(); obs_it++)
        {
            if (obs_cnt>=10) break;

            wobs_val[obs_cnt] = weights.obs_pnt;
            obs_val[obs_cnt*5+0] = obs_it->X; obs_val[obs_cnt*5+1] = obs_it->Y;
            obs_val[obs_cnt*5+2] = obs_it->R1; obs_val[obs_cnt*5+3] = obs_it->R2;
            obs_val[obs_cnt*5+4] = obs_it->Theta;
            obs_cnt++;
        }

 

        double cruise_speed,max_speed;
        cruise_speed = plan_length/(_nmpc.delta_time * ns);

        x_tf_val = {x_veh[N-1],y_veh[N-1],0,cruise_speed};
        w_tf_val = {weights.final_x,weights.final_y,weights.final_theta,weights.final_speed};
        w_cost_val.clear();
        w_cost_val.insert(w_cost_val.end(),wref_val.begin(),wref_val.end());
        w_cost_val.insert(w_cost_val.end(),wobs_val.begin(),wobs_val.end());
        w_cost_val.insert(w_cost_val.end(),w_tf_val.begin(),w_tf_val.end());

        // TODO: set reference, weightings and poly coefficients
        nlp_params.insert(nlp_params.end(),_model_param.begin(),_model_param.end()); // 2
        nlp_params.push_back(_nmpc.delta_time);
        nlp_params.push_back(cruise_speed); // 1
        nlp_params.insert(nlp_params.end(),path_coeff_val.begin(),path_coeff_val.end()); // 12
        nlp_params.insert(nlp_params.end(),offset_val.begin(),offset_val.end()); // 3
        nlp_params.insert(nlp_params.end(),x_tf_val.begin(),x_tf_val.end()); // 4
        nlp_params.insert(nlp_params.end(),obs_val.begin(),obs_val.end()); // 50
        nlp_params.insert(nlp_params.end(),w_cost_val.begin(),w_cost_val.end()); // 20

        for(int i=0; i<=ns; ++i){
            v_init.at(3+i*(nx+nu)) = robot_vel.linear.x;
        }
        v_min.at(3)=robot_vel.linear.x;
        v_max.at(3)=robot_vel.linear.x;

        // configure arguments for nlp
        map<string, DM> arg, res;

        // Bounds and initial guess
        arg["lbx"] = v_min;
        arg["ubx"] = v_max;
        arg["lbg"] = lbg;
        arg["ubg"] = ubg;
        arg["x0"] = v_init;
        arg["p"] = nlp_params;

        // Solve the problem
        
        // ros::Time = ros::Time::now();

        res = solver(arg);
        // ros::Duration elapsed_time = ros::Time::now() - begin;
        // ROS_INFO("ipopt calc time %.3f",elapsed_time.toSec());

        // Optimal solution of the NLP
        vector<double> V_opt(res.at("x"));

        // Get the optimal state trajectory
        v_init = V_opt;
        reference_traj.clear();
        nmpc_traj.clear();
        
        // save trajectories for visualizatioin
        ROBOT_STATE ref_pt, nmpc_pt;
        for(int i=0; i<=ns; ++i){
            double poly_t = V_opt.at(i*(nx+nu));
            ref_pt.X = poly_t;
            ref_pt.Y = polyeval(coeffs, poly_t);
            ref_pt.Theta = theta(poly_t, coeffs);
            reference_traj.push_back(ref_pt);

            nmpc_pt.X = V_opt.at(i*(nx+nu));
            nmpc_pt.Y = V_opt.at(1+i*(nx+nu));
            nmpc_pt.Theta = V_opt.at(2+i*(nx+nu));
            nmpc_traj.push_back(nmpc_pt);
        }

        // assign control signals
        _speed = V_opt.at((nx+nu) + nx-1);
        _steer = V_opt.at(nx+1);       

        if (_speed > 1.2*speed_lim_)
            _speed = 1.2*speed_lim_;

        if (_steer > steer_lim_)
            _steer = steer_lim_;
        if (_steer < -steer_lim_)
            _steer = -steer_lim_;

        // clear obstacles
        _obstacles_vec.clear();
        wobs_val.assign(nobs,0);


    }

    void NMPC_planner::insertSegmentObs(b_segment seg, double inflation_rate)
    {
        OBSTACLES seg_obs;
        double p1_x, p1_y, p2_x, p2_y;

        p1_x = bg::get<0, 0>(seg); p1_y = bg::get<0, 1>(seg); 
        p2_x = bg::get<1, 0>(seg); p2_y = bg::get<1, 1>(seg);

        seg_obs.X = (p1_x + p2_x)/2.0; // <0, 0> p1.x, <1, 0> p2.x
        seg_obs.Y = (p1_y + p2_y)/2.0; // <0, 1> p1.y, <1, 1> p2.y
        seg_obs.R1 = sqrt(pow(p2_x - p1_x, 2) + pow(p2_y - p1_y, 2));
        seg_obs.R2 = inflation_rate * seg_obs.R1;
        seg_obs.Theta = atan2(p2_y - p1_y,p2_x - p1_x);

        _obstacles_vec.push_back(seg_obs);
    }

    void NMPC_planner::insertCircleObs(circle circ)
    {
        OBSTACLES circ_obs;
        b_point circle_center = circ.center;
        double radius = circ.radius;

        circ_obs.X = bg::get<0>(circle_center); // <0, 0> p1.x, <1, 0> p2.x
        circ_obs.Y = bg::get<1>(circle_center); // <0, 1> p1.y, <1, 1> p2.y
        circ_obs.R1 = radius;
        circ_obs.R2 = radius;
        circ_obs.Theta = 0.0;
        
        _obstacles_vec.push_back(circ_obs);
    }

    bool NMPC_planner::checkObstacleFootprintIntersection(const std::vector<geometry_msgs::PoseStamped> &plan, 
                                                        const std::vector<geometry_msgs::Point> &_footprint_spec, const std::vector<b_segment> &seg_obs_, const std::vector<circle> &circ_obs_)
    {
        bool obstaclePresent = false;
        if (!plan.empty())
        {
            std::vector<b_segment> seg_obs = seg_obs_;
            std::vector<circle> circ_obs = circ_obs_;
            for (auto &t_pose : plan)
            {
                tf2::Quaternion tf_quat;
                tf2::fromMsg(t_pose.pose.orientation, tf_quat); //Assume quat_msg is a quaternion ros msg

                tf2::Matrix3x3 m(tf_quat);
                double t_roll, t_pitch, t_yaw;
                m.getRPY(t_roll, t_pitch, t_yaw);
                geometry_msgs::PolygonStamped p_footprint;
                costmap_2d::transformFootprint(t_pose.pose.position.x,t_pose.pose.position.y,t_yaw,_footprint_spec,p_footprint);                 
                // transformed_plan_footprints.push_back(p_footprint);

                b_polygon transformed_poly;
                for (auto &p : costmap_2d::toPointVector(p_footprint.polygon))
                    bg::append(transformed_poly, b_point(p.x,p.y));

                for (auto seg_it = seg_obs.begin(); seg_it != seg_obs.end();)
                {
                    bool isSegIntersect = bg::intersects(*seg_it, transformed_poly);
                    
                    if(isSegIntersect)
                    {
                        ROS_INFO("seg obstacle intersected");

                        insertSegmentObs(*seg_it, _seg_inflation_rate);
                        // path_length = max(abs(NMPC_planner::_nmpc->obs_x),2.0);
                        ROS_DEBUG("segment intersected at path x: %.4f, path y:%.4f, seg_point x: %.4f y %.4f",t_pose.pose.position.x, t_pose.pose.position.y,
                        bg::get<0, 0>(*seg_it),bg::get<0, 1>(*seg_it));
                        seg_it = seg_obs.erase(seg_it);    
                        
                        ROS_INFO("seg obstacle inserted");
                        obstaclePresent = true;
                    }
                    else ++seg_it;
                }

                for (auto circ_it = circ_obs.begin(); circ_it != circ_obs.end();)
                {

                    b_point circle_center = circ_it->center;
                    b_point pred_pose(t_pose.pose.position.x,t_pose.pose.position.y);
                    double obs_dist = bg::distance(circle_center,pred_pose);
                    double radius = circ_it->radius;
                    double isCircIntersect = false;
                    if (obs_dist<radius + 0.5) isCircIntersect = true;

                    if (isCircIntersect)
                    {
                        ROS_INFO("circle obstacle intersected");
                        insertCircleObs(*circ_it);
                        // path_length = max(abs(NMPC_planner::_nmpc->obs_x),2.0);
                        ROS_DEBUG("circle intersected at path x: %.4f, path y:%.4f, circle_point x: %.4f y: %.4f",t_pose.pose.position.x, t_pose.pose.position.y, 
                        bg::get<0>(circ_it->center),bg::get<1>(circ_it->center));
                        // circ_obs.erase(circ_it);
                        ROS_INFO("circle obstacle inserted");
                        obstaclePresent = true;
                        circ_it = circ_obs.erase(circ_it);
                    }
                    else ++circ_it;
                    // }
                }
         
            }
        }        
        return obstaclePresent;
    }
}
