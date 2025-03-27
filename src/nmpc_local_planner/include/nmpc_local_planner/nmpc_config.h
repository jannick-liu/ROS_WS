#ifndef NMPC_CONFIG_H_
#define NMPC_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <nmpc_local_planner/NmpcLocalPlannerReconfigureConfig.h>


// Definitions
#define USE_ANALYTIC_JACOBI // if available for a specific edge, use analytic jacobi


namespace nmpc_local_planner
{

/**
 * @class TebConfig
 * @brief Config class for the teb_local_planner and its components.
 */
class NmpcConfig
{
public:

  std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame; //!< Global planning frame

  //! Trajectory related parameters
  struct Trajectory
  {
    double dt_ref; //!< Desired temporal resolution of the trajectory (should be in the magniture of the underlying control rate)

    int min_samples; //!< Minimum number of samples (should be always greater than 2)
    int max_samples; //!< Maximum number of samples; Warning: if too small the discretization/resolution might not be sufficient for the given robot model or obstacle avoidance does not work anymore.

    bool allow_init_with_backwards_motion; //!< If true, the underlying trajectories might be initialized with backwards motions in case the goal is behind the start within the local costmap (this is only recommended if the robot is equipped with rear sensors)
    int number_of_trajectory_setpoint;

    double global_plan_prune_distance; //!< Distance between robot and via_points of global plan which is used for pruning

  } trajectory; //!< Trajectory related parameters

  //! Robot related parameters
  struct Robot
  {
    double max_vel_x; //!< Maximum translational velocity of the robot
    double max_vel_x_backwards; //!< Maximum translational velocity of the robot for driving backwards
    double max_vel_y; //!< Maximum strafing velocity of the robot (should be zero for non-holonomic robots!)
    double max_vel_theta; //!< Maximum angular velocity of the robot
    double acc_lim_x; //!< Maximum translational acceleration of the robot
    double acc_lim_y; //!< Maximum strafing acceleration of the robot
    double acc_lim_theta; //!< Maximum angular acceleration of the robot
    double min_turning_radius; //!< Minimum turning radius of a carlike robot (diff-drive robot: zero);
    double wheelbase; //!< The distance between the drive shaft and steering axle (only required for a carlike robot with 'cmd_angle_instead_rotvel' enabled); The value might be negative for back-wheeled robots!
    bool cmd_angle_instead_rotvel; //!< Substitute the rotational velocity in the commanded velocity message by the corresponding steering angle (check 'axles_distance')
    bool is_footprint_dynamic; //<! If true, updated the footprint before checking trajectory feasibility
    bool use_proportional_saturation; //<! If true, reduce all twists components (linear x and y, and angular z) proportionally if any exceed its corresponding bounds, instead of saturating each one individually
    double transform_tolerance = 0.5; //<! Tolerance when querying the TF Tree for a transformation (seconds)
    double xICR;
  } robot; //!< Robot related parameters

  //! Goal tolerance related parameters
  struct GoalTolerance
  {
    double yaw_goal_tolerance; //!< Allowed final orientation error
    double xy_goal_tolerance; //!< Allowed final euclidean distance to the goal position
    bool free_goal_vel; //!< Allow the robot's velocity to be nonzero (usally max_vel) for planning purposes
    bool complete_global_plan; // true prevents the robot from ending the path early when it cross the end goal
  } goal_tolerance; //!< Goal tolerance related parameters

  //! Obstacle related parameters
  struct Obstacles
  {
    double min_obstacle_dist; //!< Minimum desired separation from obstacles
    double inflation_dist; //!< buffer zone around obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    double dynamic_obstacle_inflation_dist; //!< Buffer zone around predicted locations of dynamic obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    bool include_dynamic_obstacles; //!< Specify whether the movement of dynamic obstacles should be predicted by a constant velocity model (this also effects homotopy class planning); If false, all obstacles are considered to be static.
    bool include_costmap_obstacles; //!< Specify whether the obstacles in the costmap should be taken into account directly
    double costmap_obstacles_behind_robot_dist; //!< Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters)
    int obstacle_poses_affected; //!< The obstacle position is attached to the closest pose on the trajectory to reduce computational effort, but take a number of neighbors into account as well
    bool legacy_obstacle_association; //!< If true, the old association strategy is used (for each obstacle, find the nearest TEB pose), otherwise the new one (for each teb pose, find only "relevant" obstacles).
    double obstacle_association_force_inclusion_factor; //!< The non-legacy obstacle association technique tries to connect only relevant obstacles with the discretized trajectory during optimization, all obstacles within a specifed distance are forced to be included (as a multiple of min_obstacle_dist), e.g. choose 2.0 in order to consider obstacles within a radius of 2.0*min_obstacle_dist.
    double obstacle_association_cutoff_factor; //!< See obstacle_association_force_inclusion_factor, but beyond a multiple of [value]*min_obstacle_dist all obstacles are ignored during optimization. obstacle_association_force_inclusion_factor is processed first.
    std::string costmap_converter_plugin; //!< Define a plugin name of the costmap_converter package (costmap cells are converted to points/lines/polygons)
    bool costmap_converter_spin_thread; //!< If \c true, the costmap converter invokes its callback queue in a different thread
    int costmap_converter_rate; //!< The rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate)
    double obstacle_proximity_ratio_max_vel; //!< Ratio of the maximum velocities used as an upper bound when reducing the speed due to the proximity to a static obstacles
    double obstacle_proximity_lower_bound; //!< Distance to a static obstacle for which the velocity should be lower
    double obstacle_proximity_upper_bound; //!< Distance to a static obstacle for which the velocity should be higher
  } obstacles; //!< Obstacle related parameters



  //! Optimization related parameters
  struct Optimization
  {
    double weight_x;           // Weighting matrix for X-coordinate
    double weight_final_x;     // Weighting matrix for X-coordinate at final point
    double weight_y;           // Weighting matrix for Y-coordinate
    double weight_final_y;     // Weighting matrix for Y-coordinate at final point
    double weight_theta;         // Weighting factor for heading
    double weight_speed;    // Weighting matrix for velocity
    double weight_speed_ChangeRate;
    double weight_steering;    // Weighting matrix for steering angle
    double weight_steering_ChangeRate;
    double weight_Qrd;

    int N_dt;           // Number of time intervals
    double delta_time;                // Length of the time intervals in [s]
  } optim; //!< Optimization related parameters

  //! Recovery/backup related parameters
  struct Recovery
  {
    bool shrink_horizon_backup; //!< Allows the planner to shrink the horizon temporary (50%) in case of automatically detected issues.
    double shrink_horizon_min_duration; //!< Specify minimum duration for the reduced horizon in case an infeasible trajectory is detected.
    bool oscillation_recovery; //!< Try to detect and resolve oscillations between multiple solutions in the same equivalence class (robot frequently switches between left/right/forward/backwards)
    double oscillation_v_eps; //!< Threshold for the average normalized linear velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_omega_eps; //!< Threshold for the average normalized angular velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_recovery_min_duration; //!< Minumum duration [sec] for which the recovery mode is activated after an oscillation is detected.
    double oscillation_filter_duration; //!< Filter length/duration [sec] for the detection of oscillations
  } recovery; //!< Parameters related to recovery and backup strategies

  

  /**
  * @brief Construct the TebConfig using default values.
  * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
  *	     the default variables will be overwritten: \n
  *	     E.g. if \e base_local_planner is utilized as plugin for the navigation stack, the initialize() method will register a
  * 	     dynamic_reconfigure server. A subset (not all but most) of the parameters are considered for dynamic modifications.
  * 	     All parameters considered by the dynamic_reconfigure server (and their \b default values) are
  * 	     set in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg. \n
  * 	     In addition the rosparam server can be queried to get parameters e.g. defiend in a launch file.
  * 	     The plugin source (or a possible binary source) can call loadRosParamFromNodeHandle() to update the parameters.
  * 	     In \e summary, default parameters are loaded in the following order (the right one overrides the left ones): \n
  * 		<b>TebConfig Constructor defaults << dynamic_reconfigure defaults << rosparam server defaults</b>
  */
 NmpcConfig()
  {

    odom_topic = "odom";
    map_frame = "odom";

    // Trajectory

    
    trajectory.dt_ref = 0.3;
    
    trajectory.min_samples = 3;
    trajectory.max_samples = 500;
    
    trajectory.allow_init_with_backwards_motion = false;
    
    // Robot

    robot.max_vel_x = 0.4;
    robot.max_vel_x_backwards = 0.2;
    robot.max_vel_y = 0.0;
    robot.max_vel_theta = 0.3;
    robot.acc_lim_x = 0.5;
    robot.acc_lim_y = 0.5;
    robot.acc_lim_theta = 0.5;
    robot.min_turning_radius = 0;
    robot.wheelbase = 1.0;
    robot.cmd_angle_instead_rotvel = false;
    robot.is_footprint_dynamic = false;
    robot.use_proportional_saturation = false;

    // GoalTolerance

    goal_tolerance.xy_goal_tolerance = 0.2;
    goal_tolerance.yaw_goal_tolerance = 0.2;
    goal_tolerance.free_goal_vel = false;
    goal_tolerance.complete_global_plan = true;

    // Obstacles

    obstacles.min_obstacle_dist = 0.5;
    obstacles.inflation_dist = 0.6;
    obstacles.dynamic_obstacle_inflation_dist = 0.6;
    obstacles.include_dynamic_obstacles = true;
    obstacles.include_costmap_obstacles = true;
    obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    obstacles.obstacle_poses_affected = 25;
    obstacles.legacy_obstacle_association = false;
    obstacles.obstacle_association_force_inclusion_factor = 1.5;
    obstacles.obstacle_association_cutoff_factor = 5;
    obstacles.costmap_converter_plugin = "";
    obstacles.costmap_converter_spin_thread = true;
    obstacles.costmap_converter_rate = 5;
    obstacles.obstacle_proximity_ratio_max_vel = 1;
    obstacles.obstacle_proximity_lower_bound = 0;
    obstacles.obstacle_proximity_upper_bound = 0.5;

    // Optimization



    // Recovery

    recovery.shrink_horizon_backup = true;
    recovery.shrink_horizon_min_duration = 10;
    recovery.oscillation_recovery = true;
    recovery.oscillation_v_eps = 0.1;
    recovery.oscillation_omega_eps = 0.1;
    recovery.oscillation_recovery_min_duration = 10;
    recovery.oscillation_filter_duration = 10;


  }

  /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local ros::NodeHandle
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

  /**
   * @brief Reconfigure parameters from the dynamic_reconfigure config.
   * Change parameters dynamically (e.g. with <c>rosrun rqt_reconfigure rqt_reconfigure</c>).
   * A reconfigure server needs to be instantiated that calls this method in it's callback.
   * In case of the plugin \e nmpc_local_planner default values are defined
   * in \e PROJECT_SRC/cfg/NMPCPlanner.cfg.
   * @param cfg Config class autogenerated by dynamic_reconfigure according to the cfg-file mentioned above.
   */
  void reconfigure(NmpcLocalPlannerReconfigureConfig& cfg);

  /**
   * @brief Check parameters and print warnings in case of discrepancies
   *
   * Call this method whenever parameters are changed using public interfaces to inform the user
   * about some improper uses.
   */
  void checkParameters() const;

  /**
   * @brief Check if some deprecated parameters are found and print warnings
   * @param nh const reference to the local ros::NodeHandle
   */
  void checkDeprecated(const ros::NodeHandle& nh) const;

  /**
   * @brief Return the internal config mutex
   */
  boost::mutex& configMutex() {return config_mutex_;}

private:
  boost::mutex config_mutex_; //!< Mutex for config accesses and changes

};


} // namespace teb_local_planner

#endif