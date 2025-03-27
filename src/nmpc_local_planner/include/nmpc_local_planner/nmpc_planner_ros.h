#ifndef NMPC_PLANNER_ROS_H_
#define NMPC_PLANNER_ROS_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <mbf_msgs/ExePathResult.h>
#include <mbf_costmap_core/costmap_controller.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>

// transforms
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_listener.h>

// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <obstacle_detector/Obstacles.h>

#include <nmpc_local_planner/nmpc_planner.h>
#include "nmpc_local_planner/ref_path_planner.h"

using namespace std;
using namespace obstacle_detector;
// namespace bg = boost::geometry;

namespace nmpc_local_planner{

class NmpcPlannerROS : public nav_core::BaseLocalPlanner{
public:

    NmpcPlannerROS();
    NmpcPlannerROS(std::string name, tf2_ros::Buffer* tf,
                 costmap_2d::Costmap2DROS* costmap_ros);

    ~NmpcPlannerROS();

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity, geometry_msgs::TwistStamped &cmd_vel, std::string &message);


    bool isGoalReached();

protected:
    bool pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                    std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=1);

    /**
     * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
     *
     * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h
     * such that the index of the current goal pose is returned as well as
     * the transformation between the global plan and the planning frame.
     * @param tf A reference to a tf buffer
     * @param global_plan The plan to be transformed
     * @param global_pose The global pose of the robot
     * @param costmap A reference to the costmap being used so the window size for transforming can be computed
     * @param global_frame The frame to transform the plan to
     * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also
     * bounded by the local costmap size!]
     * @param[out] transformed_plan Populated with the transformed plan
     * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
     * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
     * @return \c true if the global plan is transformed, \c false otherwise
     */
    /*bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, const std::string& robot_frame,
                             double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<geometry_msgs::PoseStamped>& robot_plan, 
                             int* current_goal_idx = NULL, double* path_length = NULL, geometry_msgs::TransformStamped* tf_plan_to_base = NULL) const;*/
    bool transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                             const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, const std::string& robot_frame,
                             double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan, std::vector<geometry_msgs::PoseStamped>& robot_plan, 
                             std::vector<geometry_msgs::PoseStamped>& leftsidewalk_plan_,
                             std::vector<geometry_msgs::PoseStamped>& rightsidewalk_plan_,
                             int* current_goal_idx = NULL, double* path_length = NULL, 
                             geometry_msgs::TransformStamped* tf_plan_to_base = NULL) const;
                             
    bool transformPCL(const tf2_ros::Buffer& tf, const std::string& camera_frame, const std::string& robot_frame, const sensor_msgs::PointCloud2ConstPtr &cloud,
                      sensor_msgs::PointCloud2& transformed_cloud) const;


    double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
                                        int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length=3) const;
    double estimateRefPathOrientation(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, int moving_average_length) const;
            
    void leftSidewalk_pts_callback (const sensor_msgs::PointCloud2ConstPtr& cloud);
    void rightSidewalk_pts_callback (const sensor_msgs::PointCloud2ConstPtr& cloud);

    void joy_callback(const sensor_msgs::Joy & joy_msg);
    void obstacles_callback(const obstacle_detector::Obstacles::ConstPtr new_obstacles);

    // bool checkObstacleFootprintIntersection(std::vector<geometry_msgs::PoseStamped> &transformed_plan,std::vector<b_segment> &seg_obs_, std::vector<circle> &circ_obs_);

private:
    costmap_2d::Costmap2DROS* _costmap_ros;
    tf2_ros::Buffer* tf_;
    
    bool initialized_;
    bool setup_;

    void reconfigureCB(NmpcLocalPlannerReconfigureConfig& config, uint32_t level);
    // void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

    // void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);
    // // for visualisation, publishers of global and local plan
    ros::Publisher g_plan_pub_, l_plan_pub_, c_plan_pub_, bl_plan_pub_, br_plan_pub_, footprint_pub_, pose_pub_, nmpc_plan_map_pub_, obs_radius_pub_, obs_center_pub_;
    ros::Subscriber left_side_sub, right_side_sub, joy_sub_, obstacles_sub_;
    costmap_2d::Costmap2D* _costmap; ///< @brief The costmap the controller will use
    // base_local_planner::LocalPlannerUtil planner_util_;

    // geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::Twist robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)

    // NmpcConfig cfg_; //!< Config class that stores and manages all related parameters

    // // PoseSE2 robot_pose_; //!< Store current robot pose
    // // PoseSE2 robot_goal_; //!< Store current robot goal
    struct Parameters
    {
        std::string odom_topic;
        double global_plan_prune_distance;
        double xy_goal_tolerance;
        double max_global_plan_lookahead_dist;        
    } _planner_params;

    std::string global_frame_; ///< @brief The frame in which the controller will run
    std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
    std::string camera_frame_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> leftsidewalk_plan_, rightsidewalk_plan_;

    // double qx_, qxf_, qy_, qyf_, qtheta_, rvel_, rdvel_, rstr_, rdstr_;           // Weighting matrix for Y-coordinate
    double _dt, _w, _throttle, _speed, _max_speed, _xICR;

    bool reached_goal_;

    base_local_planner::OdometryHelperRos odom_helper_;

    std::vector<geometry_msgs::Point> _footprint_spec;  
    std::string name_;

    boost::shared_ptr< dynamic_reconfigure::Server<NmpcLocalPlannerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
    
    tf::TransformListener _tf_listener;

    NMPC_planner _nmpc_planner;  // declare and define on stack
    
    obstacle_detector::Obstacles obstacles_;
    std::vector<b_polygon> predict_footprints;
    std::vector<b_segment> seg_obs_;
    // std::vector<b_circle> circ_obs;
    std::vector<circle> circ_obs_;

    std::mutex obslock;


    b_polygon reachable_region;
    circle unreachable_1;
    circle unreachable_2;

    int iter;
    double calc_time;
    double aver_time;

    geometry_msgs::Point robot_position;

};
};

#endif