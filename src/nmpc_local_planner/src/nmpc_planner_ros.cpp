#include <nmpc_local_planner/nmpc_planner_ros.h>
#include "Eigen/Core"
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.hpp>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_core/parameter_magic.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64MultiArray.h>




bool manual_mode;


PLUGINLIB_EXPORT_CLASS(nmpc_local_planner::NmpcPlannerROS, nav_core::BaseLocalPlanner)

namespace nmpc_local_planner{
    // NMPC_STRUCT *Nlmpc;

    NmpcPlannerROS::NmpcPlannerROS() : _costmap_ros(NULL), tf_(NULL), initialized_(false)
                                            {}

    NmpcPlannerROS::NmpcPlannerROS(std::string name, tf2_ros::Buffer* tf,
                            costmap_2d::Costmap2DROS* costmap_ros)
        : dynamic_recfg_(NULL), _costmap_ros(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    NmpcPlannerROS::~NmpcPlannerROS() { }

    void NmpcPlannerROS::reconfigureCB(NmpcLocalPlannerReconfigureConfig &config, uint32_t level) {
        _nmpc_planner.reconfigure(config);
        // ros::NodeHandle nh("~/" + name_);
        // lock the config mutex externally
        // boost::mutex::scoped_lock lock(cfg_.configMutex());

        // create robot footprint/contour model for optimization
        // cfg_.robot_model = getRobotFootprintFrom 
        // _nmpc_planner->updateRobotModel(cfg_.robot_model);
        reached_goal_ = false;
    }    

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void NmpcPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {
            tf_ = tf;
            _costmap_ros = costmap_ros;
            name_ = name;
            manual_mode = false;

            // create node handle with name of plugin
            ros::NodeHandle nh("~/" + name);
            // special parameters
            nh.param<string>("planner/odom_topic", _planner_params.odom_topic, "odom");
            nh.param("planner/global_plan_prune_distance", _planner_params.global_plan_prune_distance, 1.0);
            nh.param("planner/xy_goal_tolerance", _planner_params.xy_goal_tolerance, 1.0);
            nh.param("planner/max_global_plan_lookahead_dist", _planner_params.max_global_plan_lookahead_dist, 3.0);            
        //     // get parameters of NmpcConfig via the nodehandle and overide the default config
        //     cfg_.loadRosParamFromNodeHandle(nh);

        //     // reserve some memory of obstacles

            // create visualization instance
            g_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = nh.advertise<nav_msgs::Path>("ref_plan", 1);
            c_plan_pub_ = nh.advertise<nav_msgs::Path>("nmpc_plan", 1);
            bl_plan_pub_ = nh.advertise<nav_msgs::Path>("sidewalk_left", 1);
            br_plan_pub_ = nh.advertise<nav_msgs::Path>("sidewalk_right", 1);
            pose_pub_ = nh.advertise<geometry_msgs::Point>("robot_pose", 100);
            nmpc_plan_map_pub_ = nh.advertise<nav_msgs::Path>("nmpc_pln_map", 1);
            obs_center_pub_ = nh.advertise<geometry_msgs::PoseArray>("obs_center", 10);
            obs_radius_pub_ = nh.advertise<std_msgs::Float64MultiArray>("obs_radius", 10);

            footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("footprint_ref",1);
            left_side_sub = nh.subscribe<sensor_msgs::PointCloud2>("/left_point_cloud",1,&NmpcPlannerROS::leftSidewalk_pts_callback, this);
            right_side_sub = nh.subscribe<sensor_msgs::PointCloud2>("/right_point_cloud",1,&NmpcPlannerROS::rightSidewalk_pts_callback, this);
            obstacles_sub_ = nh.subscribe("/raw_obstacles", 10, &NmpcPlannerROS::obstacles_callback, this);

            joy_sub_ = nh.subscribe("/joy", 1, &NmpcPlannerROS::joy_callback, this);

        //     // create robot footprint/contour model for optimization

            // create the planner instance            
            // _nmpc_planner.config_CMSC_Casadi(nh);
            _nmpc_planner.loadROSParam(nh);
             _nmpc_planner.configOCP();

            // init other variables
            _costmap = _costmap_ros->getCostmap();
            global_frame_ = _costmap_ros->getGlobalFrameID();
            robot_base_frame_ = _costmap_ros->getBaseFrameID();
            _footprint_spec = _costmap_ros->getRobotFootprint();
            
        //     // initialize a costmap to polygon converter
            
        //     // get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices

            // init the odom helper to receive the robot's velocity from odom messages
            odom_helper_.setOdomTopic(_planner_params.odom_topic);

            // setup dynamic reconfigure
            dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<NmpcLocalPlannerReconfigureConfig> >(nh);
            dynamic_reconfigure::Server<NmpcLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&NmpcPlannerROS::reconfigureCB, this, _1, _2);
            dynamic_recfg_->setCallback(cb);

        //     // validate optimization footprint and costmap footprint

        //     // setup callback for custom obstacles

        //     // setup callback for custom via-points

        //     // initialize failure detector
            reached_goal_ = false; 
        //     // set initialize flag
            initialized_ = true;
        //     l_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan", 1);

            iter = 0;
            calc_time = 0;
            aver_time = 0;

            bg::append(reachable_region.outer(), b_point(4.0, 4.0));
            bg::append(reachable_region.outer(), b_point(4.0, -4.0));
            bg::append(reachable_region.outer(), b_point(-4.0, -4.0));
            bg::append(reachable_region.outer(), b_point(-4.0, 4.0));
            reachable_region.inners().resize(1);
            bg::append(reachable_region.inners()[0], b_point(0.1, 0)); 
            bg::append(reachable_region.inners()[0], b_point(0.5, 0.5));
            bg::append(reachable_region.inners()[0], b_point(0.5, 2.5)); 
            bg::append(reachable_region.inners()[0], b_point(-1.0, 4.0));
            bg::append(reachable_region.inners()[0], b_point(-4.0, 4.0));
            bg::append(reachable_region.inners()[0], b_point(-4.0, -4.0));
            bg::append(reachable_region.inners()[0], b_point(-1.0, -4.0));
            bg::append(reachable_region.inners()[0], b_point(0.5, -2.5));
            bg::append(reachable_region.inners()[0], b_point(0.5, -0.5)); 
            bg::append(reachable_region.inners()[0], b_point(0.1, 0.0));
            // reachable_region{{0.1, 0.3},{-1,4},{4,4},{4,-4},{-1,-4},{0.1,-0.3}};
        }


    }

    bool NmpcPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

    // To-do
        global_plan_.clear();
        global_plan_ = orig_global_plan;

        reached_goal_ = false;

        return true;

    }

    bool NmpcPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        std::string dummy_message;
        geometry_msgs::PoseStamped dummy_pose;
        geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
        uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
        cmd_vel = cmd_vel_stamped.twist;
        return outcome == mbf_msgs::ExePathResult::SUCCESS;
        // if(!initialized_)
        // {
        //     ROS_ERROR("This planner has not been initialized");
        //     return false;
        // }
        // return true;
    
    }

    uint32_t NmpcPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                        const geometry_msgs::TwistStamped& velocity,
                                                        geometry_msgs::TwistStamped &cmd_vel,
                                                        std::string &message)
    {
        // check if plugin initialized
        if(!initialized_)
        {
            ROS_ERROR("nmpc_local_planner has not been initialized, please call initialize() before using this planner");
            message = "nmpc_local_planner has not been initialized";
            return mbf_msgs::ExePathResult::NOT_INITIALIZED;
        }

        static uint32_t seq = 0;
        cmd_vel.header.seq = seq++;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.header.frame_id = robot_base_frame_;
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
        reached_goal_ = false;  

        // Get robot pose
        geometry_msgs::PoseStamped robot_pose;
        if (!_costmap_ros->getRobotPose(robot_pose)) {
            return false;
        }
        geometry_msgs::TransformStamped odom_to_global_transform = tf_->lookupTransform(
            global_plan_[0].header.frame_id,  // 目标坐标系
            ros::Time(),
            robot_pose.header.frame_id,  // 源坐标系
            robot_pose.header.stamp,  // 时间戳
            robot_pose.header.frame_id,
            ros::Duration(0.5)  // 缓冲时间
        );
        geometry_msgs::PoseStamped robot_pose_map;
        tf2::doTransform(robot_pose, robot_pose_map, odom_to_global_transform);
        robot_position.x = robot_pose_map.pose.position.x;
        robot_position.y = robot_pose_map.pose.position.y;
        pose_pub_.publish(robot_position);
        //base_local_planner::publishPlan(robot_position,pose_pub_);
        
        

        // Get robot velocity (in robot base frame)
        geometry_msgs::PoseStamped robot_vel_tf;
        odom_helper_.getRobotVel(robot_vel_tf);

        robot_vel_.linear.x = robot_vel_tf.pose.position.x;
        robot_vel_.linear.y = robot_vel_tf.pose.position.y;
        robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
        // ROS_INFO("robot vel frame id: %s with %.4f and %.4f",robot_vel_tf.header.frame_id.c_str(),robot_vel_.linear.x,robot_vel_.linear.y);

        // prune global plan to remove parts of the past. robot_pose->odom_frame, global_plan_->map_frame
        pruneGlobalPlan(*tf_, robot_pose, global_plan_, _planner_params.global_plan_prune_distance);


        // Transform global plan to the frame of interest (w.r.t. the local costmap)
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        std::vector<geometry_msgs::PoseStamped> robot_plan;
        std::vector<geometry_msgs::PoseStamped> fused_plan;
        int goal_idx;
        double path_length;
        // geometry_msgs::TransformStamped tf_plan_to_global;
        geometry_msgs::TransformStamped tf_plan_to_base;
        //                              map_frame    odom_frame             odom_frame     base_link
        /*if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *_costmap, global_frame_, robot_base_frame_, _planner_params.max_global_plan_lookahead_dist, 
                                transformed_plan, robot_plan, &goal_idx, &path_length, &tf_plan_to_base))*/
        if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *_costmap, global_frame_, robot_base_frame_, _planner_params.max_global_plan_lookahead_dist, 
                                transformed_plan, robot_plan, leftsidewalk_plan_, rightsidewalk_plan_, &goal_idx, &path_length, &tf_plan_to_base)) 
        {
            ROS_WARN("Could not transform the global plan to the frame of the controller");
            message = "Could not transform the global plan to the frame of the controller";
            return mbf_msgs::ExePathResult::INTERNAL_ERROR;
        }
        // fuseLocalPlan(robot_plan,sidewalk_plan_,fused_plan);
 

        // update via-points container

        // check if global is reached
        geometry_msgs::PoseStamped base_goal;
        // consider the last point in the global_plan (map_frame) as goal and transform into (odom_frame)
        tf2::doTransform(global_plan_.back(), base_goal, tf_plan_to_base);
        // double dx = global_goal.pose.position.x - robot_pose.pose.position.x;
        // double dy = global_goal.pose.position.y - robot_pose.pose.position.y;
        bg::model::d2::point_xy<double> p1(base_goal.pose.position.x,base_goal.pose.position.y), p2(0,0);
        double goal_dist = bg::distance(p1,p2);
        // double goal_dist = base_local_planner::getGoalPositionDistance(robot_pose,global_goal.pose.position.x,global_goal.pose.position.y);
        // ROS_INFO("Goal frame id: %s, coordinates: %.4f, %.4f", global_goal.header.frame_id.c_str(), global_goal.pose.position.x,global_goal.pose.position.y);

        if(goal_dist < _planner_params.xy_goal_tolerance)
        {
            ROS_INFO("distance to goal %f", goal_dist );

            reached_goal_ = true;

            aver_time = calc_time / iter;
            ROS_INFO("nmpc average calculation time %.4f", aver_time);
            return mbf_msgs::ExePathResult::SUCCESS;
        }
        if (goal_dist < _planner_params.xy_goal_tolerance + 1.0)
            path_length *= 0.7;


        // check if enter backup mode and apply settings

        // return false if the transformed global plan is empty
        if (transformed_plan.empty())
        {
            ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
            message = "Transformed plan is empty";
            return mbf_msgs::ExePathResult::INVALID_PATH;
        }

        // check if obstacle intersected with footprint on local reference path
        _nmpc_planner.obstaclePresent = _nmpc_planner.checkObstacleFootprintIntersection(transformed_plan, _footprint_spec, seg_obs_, circ_obs_);
        
        // Get current goal point (last point of the transformed plan)
        // double goal_x, goal_y, goal_theta;
        // goal_x = transformed_plan.back().pose.position.x;
        // goal_y = transformed_plan.back().pose.position.y;
        // // overwrite goal orientation if needed
        // goal_theta = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_base);
        
        // // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
        // tf2::Quaternion q;
        // q.setRPY(0, 0, goal_theta);
        // tf2::convert(q, transformed_plan.back().pose.orientation);
        double ref_path_theta;
        ref_path_theta = estimateRefPathOrientation(transformed_plan,10);

        // b_point first(transformed_plan[0].pose.position.x,transformed_plan[0].pose.position.y);
        // if (bg::within(first, reachable_region))
        //     ROS_INFO("ref plan inside reachable region");
        // ROS_INFO("referent path orientation %.3f",ref_path_theta);
        // // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
        // if (transformed_plan.size()==1) // plan only contains the goal
        // {
        //     transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
        // }
        // transformed_plan.front() = robot_pose; // update start
        // clear currently existing obstacles

        // update obstacle container with costmap information or polygons provided by a costmap_converter plugin        
    
        // also consider custom obstacles (must be after other updates, since the container is not cleared)

        // do not allow config changes during the following optimization step

        // Solve MPC Problem
        double v_x,av_z;
        std::vector<geometry_msgs::PoseStamped> nmpc_plan,ref_plan;
        base_local_planner::publishPlan(leftsidewalk_plan_, bl_plan_pub_);
        base_local_planner::publishPlan(rightsidewalk_plan_, br_plan_pub_);


        if (ref_path_theta < M_PI/3  && ref_path_theta > -M_PI/3) 
        {
            // ros::Time begin = ros::Time::now();
            // _nmpc_planner.CalculateMPC(transformed_plan,robot_pose,robot_vel_,*_costmap, path_length);
            // ros::Duration elapsed_time = ros::Time::now() - begin;
            // ROS_INFO("nmpc calc time %.3f",elapsed_time.toSec());

            // iterate plan until a pose close the robot is found
            // prune the reference plan on reachability region

            // ROS_INFO("Solve OCP");

            ros::Time begin2 = ros::Time::now();
            _nmpc_planner.calculateOCP(transformed_plan, leftsidewalk_plan_, rightsidewalk_plan_, robot_pose,robot_vel_,*_costmap, path_length);
            
            ros::Duration elapsed_time2 = ros::Time::now() - begin2;
            calc_time += elapsed_time2.toSec();
            iter++;
            // ROS_INFO("nmpc casadi calc time %.3f",elapsed_time2.toSec());
            // std::cout<<"-----------------------"<<std::endl;
            // get the velocity command for this sampling interval

            cmd_vel.twist.linear.x = _nmpc_planner._speed;
            cmd_vel.twist.angular.z = _nmpc_planner._steer;
            // v_x = _nmpc_planner._speed;
            // av_z = _nmpc_planner._steer;

            ROS_DEBUG("control speed: %.4f", _nmpc_planner._speed);

            _nmpc_planner.visualize(ref_plan, nmpc_plan);
            base_local_planner::publishPlan(ref_plan, l_plan_pub_);
            base_local_planner::publishPlan(nmpc_plan, c_plan_pub_);


            geometry_msgs::TransformStamped base_to_global_transform = tf_->lookupTransform(
            global_plan_[0].header.frame_id,  // 目标坐标系
            ros::Time(),
            nmpc_plan[0].header.frame_id,  // 源坐标系
            nmpc_plan[0].header.stamp,  // 时间戳
            nmpc_plan[0].header.frame_id,
            ros::Duration(0.5)  // 缓冲时间
            );
            std::vector<geometry_msgs::PoseStamped> nmpc_plan_map;
            for (const auto& pose : nmpc_plan) {  // 直接遍历 nmpc_plan
                geometry_msgs::PoseStamped transformed_pose;
                tf2::doTransform(pose, transformed_pose, base_to_global_transform);
                nmpc_plan_map.push_back(transformed_pose);
            }

            base_local_planner::publishPlan(nmpc_plan_map, nmpc_plan_map_pub_);
        }

        else
        {
            ROS_INFO("Large angle error, rotate to find forwarding path");
            cmd_vel.twist.linear.x  = 0;
            if (ref_path_theta > 0.0)
                cmd_vel.twist.angular.z = 0.2;
            else
                cmd_vel.twist.angular.z = -0.2;
        }
        // if (!manual_mode)
        // {
        //     cmd_vel.twist.linear.x = v_x;
        //     cmd_vel.twist.angular.z = av_z;
        // }
        
            
        // check for divergence

        // check feasibility
        if(_nmpc_planner.checkObstacleFootprintIntersection(nmpc_plan, _footprint_spec, seg_obs_, circ_obs_))
            // TODO
            // saturate velocity, if optimization results violates the constraints


        // a feasible solution should be found, reset counter

        // store last command 

        // visualize everything
        base_local_planner::publishPlan(transformed_plan, g_plan_pub_); 



        // get obstacles (base-map-transform)
        // 定义变量
        geometry_msgs::TransformStamped base_to_global_transform = tf_->lookupTransform(
            global_plan_[0].header.frame_id,  // 目标坐标系
            ros::Time(),
            nmpc_plan[0].header.frame_id,  // 源坐标系
            nmpc_plan[0].header.stamp,  // 时间戳
            nmpc_plan[0].header.frame_id,
            ros::Duration(0.5)  // 缓冲时间
        );
        geometry_msgs::Point center_transform;
        geometry_msgs::Point center;
        double radius;
        geometry_msgs::PoseArray center_all; 
        std_msgs::Float64MultiArray radius_msg;  // 新建消息


        center_all.header.frame_id = global_plan_[0].header.frame_id;
        center_all.header.stamp = ros::Time::now();
        for (size_t i = 0; i < circ_obs_.size(); ++i) {
            center.x = boost::geometry::get<0>(circ_obs_[i].center);
            center.y = boost::geometry::get<1>(circ_obs_[i].center);
            center.z = 0.0;  

            // 使用 tf2::doTransform 将圆心转换到目标坐标系
            tf2::doTransform(center, center_transform, base_to_global_transform);

            // 提取当前障碍物的半径
            radius = circ_obs_[i].radius;

            // 存储转换后的中心点和半径
            geometry_msgs::Pose pose;
            pose.position = center_transform;  // 设置位置
            pose.orientation.w = 1.0;          // 单位四元数（方向无关）
            center_all.poses.push_back(pose);
            radius_msg.data.push_back(radius);
        }

        obs_center_pub_.publish(center_all);
        obs_radius_pub_.publish(radius_msg);

        return mbf_msgs::ExePathResult::SUCCESS;
    }

    bool NmpcPlannerROS::isGoalReached()
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        return reached_goal_; 
    }

    bool NmpcPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
    {
        if (global_plan.empty())
            return true;
        
        try
        {
            // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
            geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
            geometry_msgs::PoseStamped robot;
            tf2::doTransform(global_pose, robot, global_to_plan_transform); // global_to_plan_transform->transform odom to map
            
            double dist_thresh_sq = dist_behind_robot*dist_behind_robot; // default 1m
            
            // iterate plan until a pose close the robot is found
            std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
            std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
            while (it != global_plan.end())
            {
            double dx = robot.pose.position.x - it->pose.position.x;
            double dy = robot.pose.position.y - it->pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < dist_thresh_sq)
            {
                erase_end = it;
                break;
            }
            ++it;
            }
            if (erase_end == global_plan.end())
                return false;
            
            if (erase_end != global_plan.begin())
            global_plan.erase(global_plan.begin(), erase_end);
        }
        catch (const tf::TransformException& ex)
        {
            // ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
            return false;
        }


















        
        return true;
    }

    /*bool NmpcPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap,
                                                const std::string& global_frame, const std::string& robot_base_frame, double max_plan_length,
                                                std::vector<geometry_msgs::PoseStamped>& transformed_plan, 
                                                std::vector<geometry_msgs::PoseStamped>& robot_plan,
                                                int* current_goal_idx, double* plan_length,
                                                // geometry_msgs::TransformStamped* tf_plan_to_global,
                                                geometry_msgs::TransformStamped* tf_plan_to_base) const
    {
        // this method is a slightly modified version of base_local_planner/goal_functions.h

        const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

        transformed_plan.clear();
        // robot_plan.clear();
        try
        {
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }

            // get plan_to_global_transform from plan frame to global_frame
            // target_frame, target_time, source_frame, source_time, fixed_frame, ros::Duration timeout) const
            geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(
                global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
            // target_frame, source_frame, time
            // geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
                // robot_base_frame, global_frame, ros::Time(0));
            geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
                robot_base_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::PoseStamped robot_pose;
            tf.transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(0.5));

            // get plan_to_global_transform from plan frame to base_link_frame
            // geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
            //     robot_base_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));


            // we'll discard points on the plan that are outside the local costmap
            double dist_threshold =
                std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
            dist_threshold *= 0.85;  // just consider 85% of the costmap size to better incorporate point obstacle that are
                                    // located on the border of the local costmap
            int i                    = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist           = 1e10;

            // we need to loop to a point on the plan that is within a certain distance of the robot
            bool robot_reached = false;
            for(int j=0; j < (int)global_plan.size(); ++j)
            {
            double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
            double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
            double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

            if (robot_reached && new_sq_dist > sq_dist)
            {
                // i = j;
                break;
            }

            if (new_sq_dist < sq_dist) // find closest distance
            {
                sq_dist = new_sq_dist;
                i = j;
                if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
                robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
            }
            }

            geometry_msgs::PoseStamped newer_pose;

            *plan_length = 0;  // check cumulative Euclidean distance along the plan

            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || *plan_length <= max_plan_length))
            {
                const geometry_msgs::PoseStamped& pose = global_plan[i];
                tf2::doTransform(pose, newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                sq_dist       = x_diff * x_diff + y_diff * y_diff;

                // caclulate distance to previous pose 
                if (i > 0 && max_plan_length > 0)
                    *plan_length += std::sqrt( std::pow(global_plan[i].pose.position.x-global_plan[i - 1].pose.position.x,2) + std::pow(global_plan[i].pose.position.y-global_plan[i - 1].pose.position.y,2) );
                    // teb_local_planner::distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

                ++i;
            }

            // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
            // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
            if (transformed_plan.empty())
            {
                tf2::doTransform(global_plan.back(), newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx) *current_goal_idx = int(global_plan.size()) - 1;
            }
            else
            {
                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx) *current_goal_idx = i - 1;  // subtract 1, since i was increased once before leaving the loop
            }

            // int k = 0;
            // // transform to robot frame
            // while (k < (int)transformed_plan.size())
            // {
            //     const geometry_msgs::PoseStamped& pose = transformed_plan[k];
            //     tf2::doTransform(pose, newer_pose, plan_to_robot_transform);

            //     robot_plan.push_back(newer_pose);

            //     ++k;
            // }            

            // Return the transformation from the global plan to the global planning frame if desired
            // if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
            if (tf_plan_to_base) *tf_plan_to_base = plan_to_robot_transform;
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(),
                        global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }    
*/

    bool NmpcPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap,
                                                const std::string& global_frame, const std::string& robot_base_frame, double max_plan_length,
                                                std::vector<geometry_msgs::PoseStamped>& transformed_plan, 
                                                std::vector<geometry_msgs::PoseStamped>& robot_plan,
                                                std::vector<geometry_msgs::PoseStamped>& leftsidewalk_plan_,
                                                std::vector<geometry_msgs::PoseStamped>& rightsidewalk_plan_,
                                                int* current_goal_idx, double* plan_length,
                                                // geometry_msgs::TransformStamped* tf_plan_to_global,
                                                geometry_msgs::TransformStamped* tf_plan_to_base) const
    {
        // this method is a slightly modified version of base_local_planner/goal_functions.h
        const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

        transformed_plan.clear();
        leftsidewalk_plan_.clear();
        rightsidewalk_plan_.clear();
        // robot_plan.clear();
        try
        {
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                *current_goal_idx = 0;
                return false;
            }

            // get plan_to_global_transform from plan frame to global_frame
            // target_frame, target_time, source_frame, source_time, fixed_frame, ros::Duration timeout) const
            geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(
                global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
            // target_frame, source_frame, time
            // geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
                // robot_base_frame, global_frame, ros::Time(0));
            geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
                robot_base_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));
            // let's get the pose of the robot in the frame of the plan
            geometry_msgs::PoseStamped robot_pose;
            tf.transform(global_pose, robot_pose, plan_pose.header.frame_id, ros::Duration(0.5));

            // get plan_to_global_transform from plan frame to base_link_frame
            // geometry_msgs::TransformStamped plan_to_robot_transform = tf.lookupTransform(
            //     robot_base_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));


            // we'll discard points on the plan that are outside the local costmap
            /*double dist_threshold =
                std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
            dist_threshold *= 0.85;  // just consider 85% of the costmap size to better incorporate point obstacle that are
                                    // located on the border of the local costmap
            int i                    = 0;
            double sq_dist_threshold = dist_threshold * dist_threshold;
            double sq_dist           = 1e10;*/

            // we need to loop to a point on the plan that is within a certain distance of the robot

            geometry_msgs::PoseStamped newer_pose;

            *plan_length = 0;  // check cumulative Euclidean distance along the plan
            int i = 0;
            double sq_dist;
            // now we'll transform until points are outside of our distance threshold
            while (i < (int)global_plan.size() /*&& sq_dist <= sq_dist_threshold*/ && (max_plan_length <= 0 || *plan_length <= max_plan_length))
            {
                const geometry_msgs::PoseStamped& pose = global_plan[i];
                tf2::doTransform(pose, newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                sq_dist       = x_diff * x_diff + y_diff * y_diff;

                // caclulate distance to previous pose 
                if (i > 0 && max_plan_length > 0)
                    *plan_length += std::sqrt( std::pow(global_plan[i].pose.position.x-global_plan[i - 1].pose.position.x,2) + std::pow(global_plan[i].pose.position.y-global_plan[i - 1].pose.position.y,2) );
                    // teb_local_planner::distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

                ++i;
            }
                    // side
            geometry_msgs::PoseStamped left_plan_;
            geometry_msgs::PoseStamped right_plan_;

            for (double x = -20.0; x <= 50; x += 1)
            {
            ros::Time current_time = ros::Time::now(); 
            geometry_msgs::PoseStamped pose_left;
            geometry_msgs::PoseStamped pose_right;
            pose_left.header.stamp = current_time;   
            pose_left.header.frame_id = "map"; 
            pose_left.pose.position.x = x; 
            pose_left.pose.position.y = 3; 
            pose_left.pose.position.z = 0.0; 
            pose_left.pose.orientation.w = 1.0; 

            tf2::doTransform(pose_left, left_plan_, plan_to_robot_transform);
            leftsidewalk_plan_.push_back(left_plan_);
        

            pose_right.header.stamp = current_time;  
            pose_right.header.frame_id = "map"; 
            pose_right.pose.position.x = x; 
            pose_right.pose.position.y = -3; 
            pose_right.pose.position.z = 0.0; 
            pose_right.pose.orientation.w = 1.0;

            tf2::doTransform(pose_right, right_plan_, plan_to_robot_transform);
            rightsidewalk_plan_.push_back(right_plan_);
            }

            // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
            // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
            if (transformed_plan.empty())
            {
                tf2::doTransform(global_plan.back(), newer_pose, plan_to_robot_transform);

                transformed_plan.push_back(newer_pose);

                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx) *current_goal_idx = int(global_plan.size()) - 1;
            }
            else
            {
                // Return the index of the current goal point (inside the distance threshold)
                if (current_goal_idx) *current_goal_idx = i - 1;  // subtract 1, since i was increased once before leaving the loop
            }

            if (tf_plan_to_base) *tf_plan_to_base = plan_to_robot_transform;
        }
        catch (tf::LookupException& ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException& ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException& ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(),
                        global_plan[0].header.frame_id.c_str());

            return false;
        }

        return true;
    }



    void NmpcPlannerROS::leftSidewalk_pts_callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        sensor_msgs::PointCloud2 cloud_out;
        camera_frame_ = cloud->header.frame_id.c_str();
        if(transformPCL(*tf_, camera_frame_, robot_base_frame_, cloud, cloud_out))
        {
            pcl::fromROSMsg(cloud_out, *temp_cloud);
        }
            
        //do stuff
        leftsidewalk_plan_.clear();
        double left_path_length = 0;
        std::reverse(temp_cloud->points.begin(),temp_cloud->points.end());
        for (int i = 0; i < temp_cloud->points.size(); ++i)
        {
            geometry_msgs::PoseStamped newer_pose;
            newer_pose.header.frame_id = robot_base_frame_;
            newer_pose.pose.position.x = temp_cloud->points[i].x;
            newer_pose.pose.position.y = temp_cloud->points[i].y;
            leftsidewalk_plan_.push_back(newer_pose);
    
            left_path_length = std::sqrt( std::pow(temp_cloud->points[i].x,2) 
            + std::pow(temp_cloud->points[i].y,2) );
            // ROS_INFO("points %d at x: %f, y: %f, l %f", (int)leftsidewalk_plan_.size(), temp_cloud->points[i].x,temp_cloud->points[i].y,left_path_length);

            if (left_path_length > 4)
                break;
            // ROS_INFO("points %d at x: %f, y: %f, z %f", (int)leftsidewalk_plan_.size(), temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z);
        }

        //base_local_planner::publishPlan(leftsidewalk_plan_, bl_plan_pub_);
        
    }
    void NmpcPlannerROS::rightSidewalk_pts_callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        sensor_msgs::PointCloud2 cloud_out;
        camera_frame_ = cloud->header.frame_id.c_str();
        if(transformPCL(*tf_, camera_frame_, robot_base_frame_, cloud, cloud_out))
        {
            pcl::fromROSMsg(cloud_out, *temp_cloud);
        }
            
        //do stuff
        rightsidewalk_plan_.clear();
        double right_path_length = 0;
        for (int i = 0; i < temp_cloud->points.size(); ++i)
        {
            geometry_msgs::PoseStamped newer_pose;
            newer_pose.header.frame_id = robot_base_frame_;
            newer_pose.pose.position.x = temp_cloud->points[i].x;
            newer_pose.pose.position.y = temp_cloud->points[i].y;
            rightsidewalk_plan_.push_back(newer_pose);
            right_path_length = std::sqrt( std::pow(temp_cloud->points[i].x,2) 
            + std::pow(temp_cloud->points[i].y,2) );
            if (right_path_length > 4)
                break;
            // ROS_INFO("points %d at x: %f, y: %f, z %f", (int)rightsidewalk_plan_.size(), temp_cloud->points[i].x,temp_cloud->points[i].y,temp_cloud->points[i].z);
        }

         //base_local_planner::publishPlan(rightsidewalk_plan_, br_plan_pub_);
        
    }

    double NmpcPlannerROS::estimateRefPathOrientation(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, int moving_average_length) const
    {
        int n = (int)transformed_plan.size();

        moving_average_length = std::min(moving_average_length, n-1 ); // maybe redundant, since we have checked the vicinity of the goal before
        double accum_angles = 0.0;
        if (moving_average_length == 0)
            moving_average_length = 1;
        for (int i = 0; i < moving_average_length; ++i)
        {
            // Transform pose of the global plan to the planning frame
            tf2::Quaternion q;
            tf2::convert(transformed_plan.at(i).pose.orientation,q);
            accum_angles+=tf2::getYaw(q);
            // calculate yaw angle  
            // candidates.push_back( std::atan2(transformed_plan.at(i).pose.position.y - transformed_plan.at(i).pose.position.y,
            //     transformed_plan.at(i).pose.position.x - transformed_plan.at(i).pose.position.x ) );
            
        }
        return accum_angles/double(moving_average_length);
    }

    double NmpcPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
                int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
    {
        int n = (int)global_plan.size();
        
        // check if we are near the global goal already
        if (current_goal_idx > n-moving_average_length-2)
        {
            if (current_goal_idx >= n-1) // we've exactly reached the goal
            {
                return tf2::getYaw(local_goal.pose.orientation);
            }
            else
            {
                tf2::Quaternion global_orientation;
                tf2::convert(global_plan.back().pose.orientation, global_orientation);
                tf2::Quaternion rotation;
                tf2::convert(tf_plan_to_global.transform.rotation, rotation);
                // TODO(roesmann): avoid conversion to tf2::Quaternion
                return tf2::getYaw(rotation *  global_orientation);
            }     
        }
        
        // reduce number of poses taken into account if the desired number of poses is not available
        moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
        
        std::vector<double> candidates;
        geometry_msgs::PoseStamped tf_pose_k = local_goal;
        geometry_msgs::PoseStamped tf_pose_kp1;
        
        int range_end = current_goal_idx + moving_average_length;
        for (int i = current_goal_idx; i < range_end; ++i)
        {
            // Transform pose of the global plan to the planning frame
            tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

            // calculate yaw angle  
            candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
                tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
            
            if (i<range_end-1) 
                tf_pose_k = tf_pose_kp1;
        }

        double x=0, y=0;
        for (std::vector<double>::const_iterator it = candidates.begin(); it!=candidates.end(); ++it)
        {
            x += cos(*it);
            y += sin(*it);
        }
        if(x == 0 && y == 0)
            return 0;
        else
            return std::atan2(y, x);

    }



    bool NmpcPlannerROS::transformPCL(const tf2_ros::Buffer& tf, const std::string& camera_frame, const std::string& robot_frame, const sensor_msgs::PointCloud2ConstPtr &cloud,
                      sensor_msgs::PointCloud2& transformed_cloud) const
    {
        try
        {
            geometry_msgs::TransformStamped transform  = tf.lookupTransform(robot_frame, camera_frame, ros::Time(0));
            tf2::doTransform(*cloud, transformed_cloud, transform);
            // sensor_msgs::PointCloud2 cloud_out;
            // tf2::doTransform(*msg, cloud_out, transform);
            return true;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return false;
        }
    }

    void NmpcPlannerROS::obstacles_callback(const obstacle_detector::Obstacles::ConstPtr new_obstacles)
    {
        // register circle obstacles
        circ_obs_.clear();
        for (auto &circ : new_obstacles->circles)
        {
            circle new_circ_obs;
            b_point center(circ.center.x,circ.center.y);
            new_circ_obs.center = center;
            new_circ_obs.radius = circ.radius;

            bg::strategy::buffer::distance_symmetric<double> distance_strategy(new_circ_obs.radius);
            bg::strategy::buffer::end_round end_strategy;
            bg::strategy::buffer::join_round join_strategy;
            bg::strategy::buffer::point_circle point_strategy(60);
            bg::strategy::buffer::side_straight side_strategy;
            bg::buffer(new_circ_obs.center, new_circ_obs.circ, distance_strategy, side_strategy, join_strategy, end_strategy, point_strategy);    
            circ_obs_.push_back(new_circ_obs);
            ROS_DEBUG("num of circles: %ld", new_obstacles->circles.size());
        }
        // obstacles_.circles.clear();
        // obstacles_.circles.assign(new_obstacles->circles.begin(), new_obstacles->circles.end());
        
        // register segment obstacles
        seg_obs_.clear();
        for (auto &seg : new_obstacles->segments)
        {
            b_segment new_seg(b_point(seg.first_point.x, seg.first_point.y), b_point(seg.last_point.x, seg.last_point.y));
            seg_obs_.push_back(new_seg);
            ROS_DEBUG("num of segments: %ld", new_obstacles->segments.size());
        }
        // obstacles_.segments.clear();
        // obstacles_.segments.assign(new_obstacles->segments.begin(), new_obstacles->segments.end());

    }

    void NmpcPlannerROS::joy_callback(const sensor_msgs::Joy & joy_msg)
    {
        if (joy_msg.buttons.at(0) && !manual_mode) {
            manual_mode = true;
            ROS_INFO("Switch to manual mode");
        } else if (
        joy_msg.buttons.at(6) &&
        joy_msg.buttons.at(3) && manual_mode) {
            manual_mode = false;
            ROS_INFO("Switch to pilot mode");
        }
    }

    // bool NmpcPlannerROS::checkObstacleFootprintIntersection(std::vector<geometry_msgs::PoseStamped> &transformed_plan,std::vector<b_segment> &seg_obs_, std::vector<circle> &circ_obs_)
    // {
    //     bool obstaclePresent = false;
    //     if (!transformed_plan.empty())
    //     {
    //         std::vector<b_segment> seg_obs = seg_obs_;
    //         std::vector<circle> circ_obs = circ_obs_;
    //         for (auto &t_pose : transformed_plan)
    //         {
    //             tf2::Quaternion tf_quat;
    //             tf2::fromMsg(t_pose.pose.orientation, tf_quat); //Assume quat_msg is a quaternion ros msg

    //             tf2::Matrix3x3 m(tf_quat);
    //             double t_roll, t_pitch, t_yaw;
    //             m.getRPY(t_roll, t_pitch, t_yaw);
    //             geometry_msgs::PolygonStamped p_footprint;
    //             costmap_2d::transformFootprint(t_pose.pose.position.x,t_pose.pose.position.y,t_yaw,_footprint_spec,p_footprint);                 
    //             // transformed_plan_footprints.push_back(p_footprint);

    //             b_polygon transformed_poly;
    //             for (auto &p : costmap_2d::toPointVector(p_footprint.polygon))
    //                 bg::append(transformed_poly, b_point(p.x,p.y));

    //             for (auto seg_it = seg_obs.begin(); seg_it != seg_obs.end();)
    //             {
    //                 bool isSegIntersect = bg::intersects(*seg_it, transformed_poly);
    //                 if(isSegIntersect)
    //                 {
    //                     ROS_INFO("seg obstacle intersected");

    //                     _nmpc_planner.insertSegmentObs(*seg_it);
    //                     // path_length = max(abs(NMPC_planner::_nmpc->obs_x),2.0);
    //                     ROS_DEBUG("segment intersected at path x: %.4f, path y:%.4f, seg_point x: %.4f y %.4f",t_pose.pose.position.x, t_pose.pose.position.y,
    //                     bg::get<0, 0>(*seg_it),bg::get<0, 1>(*seg_it));
    //                     seg_it = seg_obs.erase(seg_it);    
                        
    //                     ROS_INFO("seg obstacle inserted");
    //                     obstaclePresent = true;
    //                 }
    //                 else ++seg_it;
    //             }

    //             for (auto circ_it = circ_obs.begin(); circ_it != circ_obs.end();)
    //             {

    //                 b_point circle_center = circ_it->center;
    //                 b_point pred_pose(t_pose.pose.position.x,t_pose.pose.position.y);
    //                 double obs_dist = bg::distance(circle_center,pred_pose);
    //                 double radius = circ_it->radius;
    //                 double isCircIntersect = false;
    //                 if (obs_dist<radius + 0.5) isCircIntersect = true;

    //                 if (isCircIntersect)
    //                 {
    //                     ROS_INFO("circle obstacle intersected");
    //                     _nmpc_planner.insertCircleObs(*circ_it);
    //                     // path_length = max(abs(NMPC_planner::_nmpc->obs_x),2.0);
    //                     ROS_DEBUG("circle intersected at path x: %.4f, path y:%.4f, circle_point x: %.4f y: %.4f",t_pose.pose.position.x, t_pose.pose.position.y, 
    //                     bg::get<0>(circ_it->center),bg::get<1>(circ_it->center));
    //                     // circ_obs.erase(circ_it);
    //                     ROS_INFO("circle obstacle inserted");
    //                     obstaclePresent = true;
    //                     circ_it = circ_obs.erase(circ_it);
    //                 }
    //                 else ++circ_it;
    //                 // }
    //             }
         
    //         }
    //     }        
    //     return obstaclePresent;
    // }


}