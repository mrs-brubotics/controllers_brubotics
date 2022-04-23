#define VERSION "1.0.0.1"

/* includes //{ */

#include <ros/ros.h>

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <mrs_uav_controllers/se3_controllerConfig.h>

// | ----------------- Load---------------- |
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>   // for the position
#include <geometry_msgs/Twist.h> //for the velocity
#include <math.h>  
#include <mrs_msgs/BacaProtocol.h>
#include <std_msgs/UInt8.h>
// | --------------------------------- |

#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>

#include <geometry_msgs/Vector3Stamped.h>

// custom publisher
#include <std_msgs/Float64.h>

//}

#define OUTPUT_ATTITUDE_RATE 0
#define OUTPUT_ATTITUDE_QUATERNION 1

namespace mrs_uav_controllers
{

namespace se3_copy_controller
{

/* //{ class Se3CopyController */

class Se3CopyController : public mrs_uav_managers::Controller {

public:
  void initialize(const ros::NodeHandle& parent_nh, const std::string name, const std::string name_space, const double uav_mass,
                  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers);
  bool activate(const mrs_msgs::AttitudeCommand::ConstPtr& last_attitude_cmd);
  void deactivate(void);

  const mrs_msgs::AttitudeCommand::ConstPtr update(const mrs_msgs::UavState::ConstPtr& uav_state, const mrs_msgs::PositionCommand::ConstPtr& control_reference);
  const mrs_msgs::ControllerStatus          getStatus();

  void switchOdometrySource(const mrs_msgs::UavState::ConstPtr& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  std::string _version_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                            mutex_drs_;
  typedef mrs_uav_controllers::se3_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>  Drs_t;
  boost::shared_ptr<Drs_t>                          drs_;
  void                                              callbackDrs(mrs_uav_controllers::se3_controllerConfig& config, uint32_t level);
  DrsConfig_t                                       drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;
  bool                          got_constraints_ = false;

  // | ---------- thrust generation and mass estimation --------- |

  double _uav_mass_;
  double uav_mass_difference_;

  // | ----------------- Load---------------- |
  double load_mass_;
  // | -------------------------------------- |

  // gains that are used and already filtered
  double kpxy_;       // position xy gain
  double kvxy_;       // velocity xy gain
  double kaxy_;       // acceleration xy gain (feed forward, =1)
  double kiwxy_;      // world xy integral gain
  double kibxy_;      // body xy integral gain
  double kiwxy_lim_;  // world xy integral limit
  double kibxy_lim_;  // body xy integral limit
  double kpz_;        // position z gain
  double kvz_;        // velocity z gain
  double kaz_;        // acceleration z gain (feed forward, =1)
  double km_;         // mass estimator gain
  double km_lim_;     // mass estimator limit
  double kqxy_;       // pitch/roll attitude gain
  double kqz_;        // yaw attitude gain

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs



  // | -----------------------------Load----------------------------- |
  ros::NodeHandle                                    nh_;
  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers;

  ros::Publisher custom_publisher_load_pose;
  ros::Publisher custom_publisher_load_vel;
  ros::Publisher custom_publisher_uav_state;
  ros::Publisher custom_publisher_load_pose_experiments;
  ros::Publisher custom_publisher_load_pose_error;
  ros::Publisher custom_publisher_load_velocity_error;
  ros::Publisher publisher_encoder_angle_1;
  ros::Publisher publisher_encoder_angle_2;
  ros::Publisher publisher_Difference_load_drone_position;
  ros::Publisher publisher_rel_load_pose_position;
  ros::Subscriber load_state_sub;
  geometry_msgs::Pose load_pose;
  geometry_msgs::Twist load_velocity;
  geometry_msgs::Vector3 load_pose_error;
  geometry_msgs::Vector3 load_velocity_error;
  void loadStatesCallback(const gazebo_msgs::LinkStatesConstPtr& loadmsg);
  Eigen::Vector3d load_lin_vel = Eigen::Vector3d::Zero(3);
  geometry_msgs::Vector3 load_pose_position;
  Eigen::Vector3d rel_load_pose_position = Eigen::Vector3d::Zero(3);
  geometry_msgs::Vector3 rel_load_pose_position_to_publish;
  Eigen::Vector3d Difference_load_drone_position = Eigen::Vector3d::Zero(3);
  geometry_msgs::Vector3 Difference_load_drone_position_to_publish;
  geometry_msgs::Vector3 sum_load_pose;
  geometry_msgs::Vector3 average_load_pose;
  geometry_msgs::Vector3 sum_drone_pose;
  geometry_msgs::Vector3 offset;
  geometry_msgs::Vector3 average_drone_pose;
  bool payload_spawned = false;
  bool remove_offset = true;
  std::string load_gains_switch;
  Eigen::Vector3d load_pose_position_offset = Eigen::Vector3d::Zero(3);
  std::string run_type;
  double cable_length;
  float encoder_angle_1;
  std_msgs::Float64 encoder_angle_1_to_publish;
  float encoder_angle_2;
  std_msgs::Float64 encoder_angle_2_to_publish;
  float encoder_velocity_1;
  float encoder_velocity_2;
  double uav_heading;
  int counter_angle = 0;

  //merge controller 1 and 2 uavs
  bool uav_id = false; // false = uav2, true = uav1
  std::string uav_name;
  std::string number_of_uav;
  
  ros::Subscriber data_payload_sub;
  std::array<uint8_t, 3> data_payload;
  void BacaCallback(const mrs_msgs::BacaProtocolConstPtr& msg);
  // | -----------------------------------------------------------------|

  // | ----------------------- gain muting ---------------------- |

  bool   gains_muted_ = false;  // the current state (may be initialized in activate())
  double _gain_mute_coefficient_;

  // | --------------------- gain filtering --------------------- |

  void filterGains(const bool mute_gains, const double dt);

  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool& updated);

  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  // | ------------ controller limits and saturations ----------- |

  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;

  std_msgs::Float64 _thrust_saturation_;

  // | ------------------ activation and output ----------------- |

  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

  // | ----------------------- output mode ---------------------- |

  int        output_mode_;  // attitude_rate / acceleration
  std::mutex mutex_output_mode_;

  // | ------------------------ profiler_ ------------------------ |

  mrs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ------------------------ integrals ----------------------- |

  Eigen::Vector2d Ib_b_;  // body error integral in the body frame
  Eigen::Vector2d Iw_w_;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;

  // | ------------------------- rampup ------------------------- |

  bool   _rampup_enabled_ = false;
  double _rampup_speed_;

  bool      rampup_active_ = false;
  double    rampup_thrust_;
  int       rampup_direction_;
  double    rampup_duration_;
  ros::Time rampup_start_time_;
  ros::Time rampup_last_time_;

  // | ----------------------custom publishers------------------ |

  ros::Publisher custom_publisher_projected_thrust_;
  ros::Publisher custom_publisher_thrust_;
  ros::Publisher pub_thrust_satlimit_;
  ros::Publisher pub_thrust_satlimit_physical_;
  ros::Publisher pub_thrust_satval_;
  ros::Publisher pub_hover_thrust_;
  ros::Publisher pub_tilt_angle_;;
  // | ----------------------------------------------------------- |

};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

void Se3CopyController::initialize(const ros::NodeHandle& parent_nh, [[maybe_unused]] const std::string name, const std::string name_space, const double uav_mass,
                               std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {

  ros::NodeHandle nh_(parent_nh, name_space);

  common_handlers_ = common_handlers;
    // | -----------------LOAD---------------- |
  _uav_mass_       = std::stod(getenv("UAV_MASS"));
  // | --------------------------------- |
  // _uav_mass_       = uav_mass; I find it easier to change mass in session file rather than in several files from mrs, what do you think ? Must put a recall in the readme of each test TODO

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "Se3CopyController");

  param_loader.loadParam("version", _version_);

  if (_version_ != VERSION) {

    ROS_ERROR("[Se3CopyController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::requestShutdown(); // Should be the same as below but cleaner
    // ros::shutdown();
  }

  param_loader.loadParam("enable_profiler", _profiler_enabled_);

  // lateral gains
  param_loader.loadParam("default_gains/horizontal/kp", kpxy_);
  param_loader.loadParam("default_gains/horizontal/kv", kvxy_);
  param_loader.loadParam("default_gains/horizontal/ka", kaxy_);

  param_loader.loadParam("default_gains/horizontal/kiw", kiwxy_);
  param_loader.loadParam("default_gains/horizontal/kib", kibxy_);

  // | ------------------------- rampup ------------------------- |

  param_loader.loadParam("rampup/enabled", _rampup_enabled_);
  param_loader.loadParam("rampup/speed", _rampup_speed_);

  // height gains
  param_loader.loadParam("default_gains/vertical/kp", kpz_);
  param_loader.loadParam("default_gains/vertical/kv", kvz_);
  param_loader.loadParam("default_gains/vertical/ka", kaz_);

  // attitude gains
  param_loader.loadParam("default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.loadParam("default_gains/vertical/attitude/kq", kqz_);

  // mass estimator
  param_loader.loadParam("default_gains/mass_estimator/km", km_);
  param_loader.loadParam("default_gains/mass_estimator/km_lim", km_lim_);

  // integrator limits
  param_loader.loadParam("default_gains/horizontal/kiw_lim", kiwxy_lim_);
  param_loader.loadParam("default_gains/horizontal/kib_lim", kibxy_lim_);

  // constraints
  param_loader.loadParam("constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  param_loader.loadParam("constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);
  if (_tilt_angle_failsafe_enabled_ && fabs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[Se3CopyController]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low");
    ros::requestShutdown();
  }

  param_loader.loadParam("constraints/thrust_saturation", _thrust_saturation_.data);

  // gain filtering
  param_loader.loadParam("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.loadParam("gains_filter/min_change_rate", _gains_filter_min_change_rate_);

  // gain muting
  param_loader.loadParam("gain_mute_coefficient", _gain_mute_coefficient_);

  // output mode
  param_loader.loadParam("output_mode", output_mode_);

  param_loader.loadParam("rotation_matrix", drs_params_.rotation_type);

  // angular rate feed forward
  param_loader.loadParam("angular_rate_feedforward/parasitic_pitch_roll", drs_params_.pitch_roll_heading_rate_compensation);
  param_loader.loadParam("angular_rate_feedforward/jerk", drs_params_.jerk_feedforward);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Se3CopyController]: could not load all parameters!");
    ros::requestShutdown();
  }

  // | ---------------- prepare stuff from params --------------- |

  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[Se3CopyController]: output mode has to be {0, 1}!");
    ros::requestShutdown();
  }

  // initialize the integrals
  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2);
  Ib_b_                = Eigen::Vector2d::Zero(2);

  // custom publisher
  custom_publisher_projected_thrust_ = nh_.advertise<std_msgs::Float64>("custom_projected_thrust",1);
  custom_publisher_thrust_           = nh_.advertise<std_msgs::Float64>("custom_thrust",1);
  pub_thrust_satlimit_physical_           = nh_.advertise<std_msgs::Float64>("thrust_satlimit_physical",1);
  pub_thrust_satlimit_           = nh_.advertise<std_msgs::Float64>("thrust_satlimit",1);
  pub_thrust_satval_           = nh_.advertise<std_msgs::Float64>("thrust_satval",1);
  pub_hover_thrust_            = nh_.advertise<std_msgs::Float64>("hover_thrust",1);
  pub_tilt_angle_            = nh_.advertise<std_msgs::Float64>("tilt_angle",1);

  // | -----------------LOAD---------------- |
  run_type = getenv("RUN_TYPE");
  if (run_type == "simulation")
  {
    custom_publisher_load_pose   = nh_.advertise<geometry_msgs::Pose>("load_pose",1);
    custom_publisher_load_vel   = nh_.advertise<geometry_msgs::Twist>("load_vel",1);
    custom_publisher_uav_state   = nh_.advertise<mrs_msgs::UavState>("uav_state",1);
  }else{
    //publisher for encoders when doing real experiment
    custom_publisher_load_pose_experiments   = nh_.advertise<geometry_msgs::Vector3>("load_pose_position",1);
    custom_publisher_load_pose_error = nh_.advertise<geometry_msgs::Vector3>("load_pose_error",1);
    custom_publisher_load_velocity_error = nh_.advertise<geometry_msgs::Vector3>("load_velocity_error",1);
    publisher_encoder_angle_1 = nh_.advertise<std_msgs::Float64>("encoder_angle_1",1);
    publisher_encoder_angle_2 = nh_.advertise<std_msgs::Float64>("encoder_angle_2",1);
    publisher_Difference_load_drone_position = nh_.advertise<geometry_msgs::Vector3>("Difference_load_drone_position",1);
    publisher_rel_load_pose_position = nh_.advertise<geometry_msgs::Vector3>("rel_load_pose_position",1);
  }
  // | --------------------------------- |


  // | --------------- dynamic reconfigure server --------------- |

  drs_params_.kpxy             = kpxy_;
  drs_params_.kvxy             = kvxy_;
  drs_params_.kaxy             = kaxy_;
  drs_params_.kiwxy            = kiwxy_;
  drs_params_.kibxy            = kibxy_;
  drs_params_.kpz              = kpz_;
  drs_params_.kvz              = kvz_;
  drs_params_.kaz              = kaz_;
  drs_params_.kqxy             = kqxy_;
  drs_params_.kqz              = kqz_;
  drs_params_.kiwxy_lim        = kiwxy_lim_;
  drs_params_.kibxy_lim        = kibxy_lim_;
  drs_params_.km               = km_;
  drs_params_.km_lim           = km_lim_;
  drs_params_.output_mode      = output_mode_;
  drs_params_.jerk_feedforward = true;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&Se3CopyController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);
  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(nh_, "Se3CopyController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[Se3CopyController]: initialized, version %s", VERSION);

  is_initialized_ = true;
}

//}

/* //{ activate() */

bool Se3CopyController::activate(const mrs_msgs::AttitudeCommand::ConstPtr& last_attitude_cmd) {

  if (last_attitude_cmd == mrs_msgs::AttitudeCommand::Ptr()) {

    ROS_WARN("[Se3CopyController]: activated without getting the last controller's command");

    return false;

  } else {

    activation_attitude_cmd_ = *last_attitude_cmd;
    uav_mass_difference_     = last_attitude_cmd->mass_difference;

    activation_attitude_cmd_.controller_enforcing_constraints = false;

    Ib_b_[0] = -last_attitude_cmd->disturbance_bx_b;
    Ib_b_[1] = -last_attitude_cmd->disturbance_by_b;

    Iw_w_[0] = -last_attitude_cmd->disturbance_wx_w;
    Iw_w_[1] = -last_attitude_cmd->disturbance_wy_w;

    ROS_INFO(
        "[Se3CopyController]: setting the mass difference and integrals from the last AttitudeCmd: mass difference: %.2f kg, Ib_b_: %.2f, %.2f N, Iw_w_: "
        "%.2f, %.2f N",
        uav_mass_difference_, Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);

    ROS_INFO("[Se3CopyController]: activated with a last controller's command, mass difference %.2f kg", uav_mass_difference_);
  }

  // rampup check
  if (_rampup_enabled_) {

    double hover_thrust = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, last_attitude_cmd->total_mass * common_handlers_->g);
    double thrust_difference = hover_thrust - last_attitude_cmd->thrust;

    if (thrust_difference > 0) {
      rampup_direction_ = 1;
    } else if (thrust_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[Se3CopyController]: activating rampup with initial thrust: %.4f, target: %.4f", last_attitude_cmd->thrust, hover_thrust);

    rampup_active_     = true;
    rampup_start_time_ = ros::Time::now();
    rampup_last_time_  = ros::Time::now();
    rampup_thrust_     = last_attitude_cmd->thrust;

    rampup_duration_ = fabs(thrust_difference) / _rampup_speed_;
  }

  first_iteration_ = true;
  gains_muted_     = true;

  ROS_INFO("[Se3CopyController]: activated");

  is_active_ = true;

  return true;
}

//}

/* //{ deactivate() */

void Se3CopyController::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[Se3CopyController]: deactivated");
}

//}

/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr Se3CopyController::update(const mrs_msgs::UavState::ConstPtr&        uav_state,
                                                                const mrs_msgs::PositionCommand::ConstPtr& control_reference) {

  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");
  //ROS_INFO("[Se3CopyController]: updating the se3_copy_controller of the controllers_brubotics package");
  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
  }

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }
  

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {

    last_update_time_ = uav_state->header.stamp;

    first_iteration_ = false;

    ROS_INFO("[Se3CopyController]: first iteration");

    return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));

  } else {

    dt                = (uav_state->header.stamp - last_update_time_).toSec();
    last_update_time_ = uav_state->header.stamp;
  }

  if (fabs(dt) <= 0.001) {

    ROS_DEBUG("[Se3CopyController]: the last odometry message came too close (%.2f s)!", dt);

    if (last_attitude_cmd_ != mrs_msgs::AttitudeCommand::Ptr()) {

      return last_attitude_cmd_;

    } else {

      return mrs_msgs::AttitudeCommand::ConstPtr(new mrs_msgs::AttitudeCommand(activation_attitude_cmd_));
    }
  }

  // | -----------------LOAD---------------- |

  custom_publisher_uav_state.publish(uav_state_); 
  uav_name = getenv("UAV_NAME");
  // to see how many UAVs there are
   
  if (run_type == "simulation"){
    number_of_uav = getenv("NUMBER_OF_UAV"); // is exported in the session.yaml
    if (number_of_uav == "2")
    {
      // to see which UAV it is
      if (uav_name == "uav1")
      {
        uav_id = true; //uav 1
      }else{
        uav_id = false; //uav 2
      }
    }
  }
  //ROS_INFO_STREAM("RUN_TYPE \n" << run_type );
  std::string slash = "/";
  //ROS_INFO_STREAM("UAV_NAME \n" << run_type  );

  if (run_type == "simulation")
  {
    // subscriber of the simulation
    load_state_sub =  nh_.subscribe("/gazebo/link_states", 1, &Se3CopyController::loadStatesCallback, this, ros::TransportHints().tcpNoDelay());
  }else{
    // subscriber of the encoder
    data_payload_sub = nh_.subscribe("/nuc4/serial/received_message", 1, &Se3CopyController::BacaCallback, this, ros::TransportHints().tcpNoDelay());
    //slash.append(uav_name.append("/serial/received_message"))
  }
  // ROS_INFO_STREAM("RUN_TYPE \n" << run_type );

  // | --------------------------------- |


  // | ----------------- get the current heading ---------------- |

  uav_heading = 0;

  try {
    uav_heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: could not calculate the UAV heading");
  }

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame
  // Rw - angular velocity reference

  Eigen::Vector3d Rp = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rv = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Ra = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rw = Eigen::Vector3d::Zero(3);

  if (control_reference->use_position_vertical || control_reference->use_position_horizontal) {

    if (control_reference->use_position_horizontal) {
      Rp[0] = control_reference->position.x;
      Rp[1] = control_reference->position.y;
    } else {
      Rv[0] = 0;
      Rv[1] = 0;
    }

    if (control_reference->use_position_vertical) {
      Rp[2] = control_reference->position.z;
    } else {
      Rv[2] = 0;
    }
  }

  if (control_reference->use_velocity_horizontal) {
    Rv[0] = control_reference->velocity.x;
    Rv[1] = control_reference->velocity.y;
  } else {
    Rv[0] = 0;
    Rv[1] = 0;
  }

  if (control_reference->use_velocity_vertical) {
    Rv[2] = control_reference->velocity.z;
  } else {
    Rv[2] = 0;
  }

  if (control_reference->use_acceleration) {
    Ra << control_reference->acceleration.x, control_reference->acceleration.y, control_reference->acceleration.z;
  } else {
    Ra << 0, 0, 0;
  }

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d Ov(uav_state->velocity.linear.x, uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // R - current uav attitude
  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state->pose.orientation);

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state->velocity.angular.x, uav_state->velocity.angular.y, uav_state->velocity.angular.z);

  // | -------------- calculate the control errors -------------- |

  // position control error
  Eigen::Vector3d Ep = Eigen::Vector3d::Zero(3);

  if (control_reference->use_position_horizontal || control_reference->use_position_vertical) {
    Ep = Op - Rp;
  }

  // velocity control error
  Eigen::Vector3d Ev = Eigen::Vector3d::Zero(3);

  if (control_reference->use_velocity_horizontal || control_reference->use_velocity_vertical ||
      control_reference->use_position_vertical) {  // even wehn use_position_vertical to provide dampening
    Ev = Ov - Rv;
  }

// | --------------------------LOAD--------------------------|

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rpl - position reference load in global frame
  // Rvl - velocity reference load in global frame
  Eigen::Vector3d Rpl = Eigen::Vector3d::Zero(3);

  // Opl - position load in global frame
  // Ovl - velocity load in global frame

  Eigen::Vector3d Opl(load_pose_position.x, load_pose_position.y, load_pose_position.z);
  Eigen::Vector3d Ovl(load_lin_vel[0], load_lin_vel[1], load_lin_vel[2]);
  Eigen::Vector3d Epl = Eigen::Vector3d::Zero(3); // Load position control error
  Eigen::Vector3d Evl = Eigen::Vector3d::Zero(3); // Load velocity control error
  cable_length = std::stod(getenv("CABLE_LENGTH")); // is changed inside session.yml to take length cable into account! stod to transform string defined in session to double.

  if (control_reference->use_position_vertical || control_reference->use_position_horizontal) {

    if (control_reference->use_position_horizontal) {
      Rpl[0] = control_reference->position.x;
      Rpl[1] = control_reference->position.y;
    } 

    if (control_reference->use_position_vertical) {
      Rpl[2] = control_reference->position.z - cable_length;
    } 
  }

  if (run_type == "simulation"){
    if(payload_spawned){
      if(remove_offset){
        Epl = Rp - Opl; //Op - Opl is super unstable!!
        load_pose_position_offset = Epl;
        remove_offset = false;
      } 
    }

    if (control_reference->use_position_horizontal || control_reference->use_position_vertical) {
      Epl = Rp - Opl - load_pose_position_offset; // remove offset because the load does not spawn perfectly under drone
      //(position relative to base frame)
    }

    if (control_reference->use_velocity_horizontal || control_reference->use_velocity_vertical ||
      control_reference->use_position_vertical) {  // even when use_position_vertical to provide dampening
      Evl = Ov - Ovl;
      //(speed relative to base frame)
    }
  }else{
    
    if(counter_angle < 500){
      counter_angle = counter_angle +1;
      load_pose_position.x = Op[0] + Difference_load_drone_position[0];
      load_pose_position.y = Op[1] + Difference_load_drone_position[1];
      load_pose_position.z = Op[2] + Difference_load_drone_position[2];
      
      sum_load_pose.x = sum_load_pose.x + load_pose_position.x;
      sum_drone_pose.x = sum_drone_pose.x + uav_state->pose.position.x;
      sum_load_pose.y  = sum_load_pose.y + load_pose_position.y;
      sum_drone_pose.y = sum_drone_pose.y + uav_state->pose.position.y;

      load_pose_position.x = uav_state->pose.position.x;
      load_pose_position.y = uav_state->pose.position.y;
      load_pose_position.z = uav_state->pose.position.z;
    }else{
      average_load_pose.x = sum_load_pose.x/500.0;
      average_drone_pose.x = sum_drone_pose.x/500.0;
      average_load_pose.y = sum_load_pose.y/500.0;
      average_drone_pose.y = sum_drone_pose.y/500.0;

      offset.x = average_load_pose.x - average_drone_pose.x;
      ROS_INFO_STREAM("offset x \n" << offset.x);

      offset.y = average_load_pose.y - average_drone_pose.y;
      ROS_INFO_STREAM("offset x \n" << offset.y);

      load_pose_position.x = Op[0] + Difference_load_drone_position[0] - offset.x;
      load_pose_position.y = Op[1] + Difference_load_drone_position[1] - offset.y;
      load_pose_position.z = Op[2] + Difference_load_drone_position[2];      
    }  

    if (control_reference->use_position_horizontal || control_reference->use_position_vertical) {
      Epl[0] = Op[0] - load_pose_position.x; // since encoder gives offset from drone position (position relative to drone)
      Epl[1] = Op[1] - load_pose_position.y;
      Epl[2] = Op[2] - load_pose_position.z;
    }
    if (control_reference->use_velocity_horizontal || control_reference->use_velocity_vertical ||
      control_reference->use_position_vertical) {  // even when use_position_vertical to provide dampening
      Evl = -Ovl; // if encoders don't move, there is no speed (speed relative to drone)
    }
    load_pose_error.x = -Epl[0];
    load_pose_error.y = -Epl[1];
    load_pose_error.z = -Epl[2];

    load_velocity_error.x = -Evl[0];
    load_velocity_error.y = -Evl[1];
    load_velocity_error.z = -Evl[2];

    custom_publisher_load_pose_experiments.publish(load_pose_position);
    
    custom_publisher_load_pose_error.publish(load_pose_error);
    custom_publisher_load_velocity_error.publish(load_velocity_error);

    // ROS_INFO_STREAM("Se3BruboticsLoadController: Load_pose_position = \n" << load_pose_position);
    // ROS_INFO_STREAM("Se3BruboticsLoadController: Epl (Rpl-load_pose)= \n" << Epl);

  }
  
  // 1e method pandolfo
  /*
  Eigen::Array3d  Klv = Eigen::Array3d::Zero(3); // Kv for the load
 
  if (control_reference->use_velocity_horizontal) {
      Klv[0] = 0.4;
      Klv[1] = 0.4;
  } else {
      Klv[0] = 0;
      Klv[1] = 0;
  }

  if (control_reference->use_velocity_vertical) {
    Klv[2] = 0;
  } 
  else {
    Klv[2] = 0;
  }
  */

  // 2e method pandolfo
  Eigen::Array3d  Kpl = Eigen::Array3d::Zero(3); 
  Eigen::Array3d  Kdl = Eigen::Array3d::Zero(3); 
  load_gains_switch = getenv("LOAD_GAIN_SWITCH");

  if (load_gains_switch == "true"){ 
    // gains activated
    if (control_reference->use_velocity_horizontal) {
      // gains activated
      Kpl[0] = 7.0; //7.0
      Kpl[1] = Kpl[0];
    } else {
      Kpl[0] = 0;
      Kpl[1] = 0;
    }

    if (control_reference->use_velocity_horizontal) {
      Kdl[0] = 0.5; //0.5
      Kdl[1] = Kdl[0];
    } else {
      Kdl[0] = 0;
      Kdl[1] = 0;
    }

    //ROS_INFO_STREAM("gains" << std::endl << "activated");
  }else{
    // gains desactivated
    if (control_reference->use_velocity_horizontal) {
      Kpl[0] = 0.0; //7.0
      Kpl[1] = Kpl[0];
    } else {
      Kpl[0] = 0;
      Kpl[1] = 0;
    }

    if (control_reference->use_velocity_horizontal) {
      Kdl[0] = 0.0; //0.5
      Kdl[1] = Kdl[0];
    } else {
      Kdl[0] = 0;
      Kdl[1] = 0;
    }

    //ROS_INFO_STREAM("gains" << std::endl << "desactivated");
  }

  if (control_reference->use_velocity_vertical) {
    Kpl[2] = 0;
  } 
  else {
    Kpl[2] = 0;
  }

  if (control_reference->use_velocity_vertical) {
    Kdl[2] = 0;
  } 
  else {
    Kdl[2] = 0;
  }
  // | ---------------------------------------------------------------------------|

  // | --------------------- load the gains --------------------- |

  filterGains(control_reference->disable_position_gains, dt);

  Eigen::Vector3d Ka = Eigen::Vector3d::Zero(3);
  Eigen::Array3d  Kp = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kv = Eigen::Array3d::Zero(3);
  Eigen::Array3d  Kq = Eigen::Array3d::Zero(3);

  {
    std::scoped_lock lock(mutex_gains_);

    if (control_reference->use_position_horizontal) {
      Kp[0] = kpxy_;
      Kp[1] = kpxy_;
    } else {
      Kp[0] = 0;
      Kp[1] = 0;
    }

    if (control_reference->use_position_vertical) {
      Kp[2] = kpz_;
    } else {
      Kp[2] = 0;
    }

    if (control_reference->use_velocity_horizontal) {
      Kv[0] = kvxy_;
      Kv[1] = kvxy_;
    } else {
      Kv[0] = 0;
      Kv[1] = 0;
    }

    // special case: if want to control z-pos but not the velocity => at least provide z dampening, therefore kvz_
    if (control_reference->use_velocity_vertical || control_reference->use_position_vertical) {
      Kv[2] = kvz_;
    } else {
      Kv[2] = 0;
    }

    if (control_reference->use_acceleration) {
      Ka << kaxy_, kaxy_, kaz_;
    } else {
      Ka << 0, 0, 0;
    }

    // Those gains are set regardless of control_reference setting,
    // because we need to control the attitude.
    Kq << kqxy_, kqxy_, kqz_;
  }

  // a print to test if the gains change so you know where to change:
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Ka_x = %f", Ka(0));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Ka_y = %f", Ka(1));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Ka_z = %f", Ka(2));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kq_x = %f", Kq(0));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kq_y = %f", Kq(1));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kq_z = %f", Kq(2));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_x = %f", Kp(0));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_y = %f", Kp(1));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_z = %f", Kp(2));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_x = %f", Kv(0));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_y = %f", Kv(1));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_z = %f", Kv(2));


// | -----------------LOAD--------------------------|

  load_mass_ = std::stod(getenv("LOAD_MASS")); // can be changed in session.yml file. To take mass load into account! stod to transform string defined in session to double
  uav_mass_difference_ = 0; 
  //ROS_INFO_STREAM("UAV_massInfo \n" << _uav_mass_ );
  double total_mass = 0;

  if (run_type == "simulation"){ 
    if(payload_spawned){
      total_mass = _uav_mass_ + load_mass_ + uav_mass_difference_ ;
      //ROS_INFO_STREAM("Mass spwaned" << std::endl << total_mass);
    }else{
      total_mass = _uav_mass_ + uav_mass_difference_ ;
      //ROS_INFO_STREAM("Mass NOT spwaned" << std::endl << total_mass);
    }
  }else{
    total_mass = _uav_mass_ + load_mass_ + uav_mass_difference_ ;  // TODONeed to solve that for hardware test as no load_mass.
  }

  //ROS_INFO_STREAM("Se3BruboticsLoadController: total mass = \n" << total_mass );

  Kp = Kp * total_mass;
  Kv = Kv * total_mass;

// |------------------------------------------------|

  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_x*m = %f", Kp(0));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_y*m = %f", Kp(1));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_z*m = %f", Kp(2));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_x*m = %f", Kv(0));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_y*m = %f", Kv(1));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_z*m = %f", Kv(2));


  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: _uav_mass_ = %f", _uav_mass_);
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: n_motors = %d", common_handlers_->motor_params.n_motors);
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: motor_params.A = %f", common_handlers_->motor_params.A);
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: motor_params.B = %f", common_handlers_->motor_params.B);

  // ROS_INFO_STREAM("[Se3CopyController]: common_handlers_->motor_params.n_motors = \n" << common_handlers_->motor_params.n_motors);
  // ROS_INFO_STREAM("[Se3CopyController]: common_handlers_->motor_params.A = \n" << common_handlers_->motor_params.A);
  // ROS_INFO_STREAM("[Se3CopyController]: common_handlers_->motor_params.B = \n" << common_handlers_->motor_params.B);


  // | --------------------------LOAD---------------------- |

  //2e method pandolfo
  Kpl = Kpl *total_mass;
  Kdl = Kdl *total_mass;
  
  // | -------------------------------------------------------|



  // | --------------- desired orientation matrix --------------- |

  // get body integral in the world frame

  Eigen::Vector2d Ib_w = Eigen::Vector2d(0, 0);

  {

    geometry_msgs::Vector3Stamped Ib_b_stamped;

    Ib_b_stamped.header.stamp    = ros::Time::now();
    Ib_b_stamped.header.frame_id = "fcu_untilted";
    Ib_b_stamped.vector.x        = Ib_b_(0);
    Ib_b_stamped.vector.y        = Ib_b_(1);
    Ib_b_stamped.vector.z        = 0;

    auto res = common_handlers_->transformer->transformSingle(uav_state_.header.frame_id, Ib_b_stamped);

    if (res) {
      Ib_w[0] = res.value().vector.x;
      Ib_w[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: could not transform the Ib_b_ to the world frame");
    }
  }

  // construct the desired force vector

  // double total_mass = _uav_mass_ + uav_mass_difference_;

  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, common_handlers_->g) + Ra);
  Eigen::Vector3d position_feedback = -Kp * Ep.array();
  Eigen::Vector3d velocity_feedback = -Kv * Ev.array();
  Eigen::Vector3d integral_feedback;

  // | ------------------------- LOAD-------------------------- |  

  if (run_type == "simulation"){ 
    for (int i = 0; i < 3; i++) // in order to set the error to 0 befor the load spawn
    {
      if(!payload_spawned)
      {
        Epl = Epl*0;
        Evl = Evl*0;
      }
      if(payload_spawned && abs(Epl[i]) < 0.05)
      {
        Epl[i] = Epl[i]*0;
      }
    }
  }

  //Eigen::Vector3d velocity_load_feedback = -Klv * load_lin_vel.array();
  Eigen::Vector3d position_load_feedback = -Kpl * Epl.array();
  Eigen::Vector3d velocity_load_feedback = -Kdl * Evl.array();

  // |---------------------------------------------------| 

  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w_[0], Ib_w[1] + Iw_w_[1], 0;
  }

// | -------------------------------se3+LOAD------------------------ | 
  Eigen::Vector3d f = position_load_feedback + velocity_load_feedback + position_feedback + velocity_feedback + integral_feedback + feed_forward;
// | -------------------------------------------------------------| 
  
  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity

  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", f[2]);

    f << 0, 0, 1;
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in spherical coordinates
  std_msgs::Float64 theta;
  theta.data = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // check for the failsafe limit
  if (!std::isfinite(theta.data)) {

    ROS_ERROR("[Se3CopyController]: NaN detected in variable 'theta', returning null");

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (_tilt_angle_failsafe_enabled_ && theta.data > _tilt_angle_failsafe_) {

    ROS_ERROR("[Se3CopyController]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", (180.0 / M_PI) * theta.data,
              (180.0 / M_PI) * _tilt_angle_failsafe_);
    ROS_INFO("[Se3CopyController]: f = [%.2f, %.2f, %.2f]", f[0], f[1], f[2]);
    ROS_INFO("[Se3CopyController]: position feedback: [%.2f, %.2f, %.2f]", position_feedback[0], position_feedback[1], position_feedback[2]);
    ROS_INFO("[Se3CopyController]: velocity feedback: [%.2f, %.2f, %.2f]", velocity_feedback[0], velocity_feedback[1], velocity_feedback[2]);
    ROS_INFO("[Se3CopyController]: integral feedback: [%.2f, %.2f, %.2f]", integral_feedback[0], integral_feedback[1], integral_feedback[2]);
    ROS_INFO("[Se3CopyController]: position_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", control_reference->position.x, control_reference->position.y,
             control_reference->position.z, control_reference->heading);
    ROS_INFO("[Se3CopyController]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", uav_state->pose.position.x, uav_state->pose.position.y,
             uav_state->pose.position.z, uav_heading);

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // saturate the angle

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
  ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: constraints.tilt = %f", constraints.tilt);
  if (fabs(constraints.tilt) > 1e-3 && theta.data > constraints.tilt) {
    ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", (theta.data / M_PI) * 180.0,
                      (constraints.tilt / M_PI) * 180.0);
    theta.data = constraints.tilt;
  }

  // reconstruct the vector
  f_norm[0] = sin(theta.data) * cos(phi);
  f_norm[1] = sin(theta.data) * sin(phi);
  f_norm[2] = cos(theta.data);

  // publish the tilt angle
  pub_tilt_angle_.publish(theta);

  // | ------------- construct the (desired) rotational matrix ------------ |

  Eigen::Matrix3d Rd;

  if (control_reference->use_orientation) {

    // fill in the desired orientation based on the desired orientation from the control command
    Rd = mrs_lib::AttitudeConverter(control_reference->orientation);

    if (control_reference->use_heading) {
      try {
        Rd = mrs_lib::AttitudeConverter(Rd).setHeading(control_reference->heading);
      }
      catch (...) {
        ROS_ERROR("[Se3CopyController]: could not set the desired heading");
      }
    }

  } else {

    Eigen::Vector3d bxd;  // desired heading vector

    if (control_reference->use_heading) {
      bxd << cos(control_reference->heading), sin(control_reference->heading), 0;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: desired heading was not specified, using current heading instead!");
      bxd << cos(uav_heading), sin(uav_heading), 0;
    }

    // fill in the desired orientation based on the state feedback
    if (drs_params.rotation_type == 0) {

      Rd.col(2) = f_norm;
      Rd.col(1) = Rd.col(2).cross(bxd);
      Rd.col(1).normalize();
      Rd.col(0) = Rd.col(1).cross(Rd.col(2));
      Rd.col(0).normalize();

    } else {

      // | ------------------------- body z ------------------------- |
      Rd.col(2) = f_norm;

      // | ------------------------- body x ------------------------- |

      // construct the oblique projection
      Eigen::Matrix3d projector_body_z_compl = (Eigen::Matrix3d::Identity(3, 3) - f_norm * f_norm.transpose());

      // create a basis of the body-z complement subspace
      Eigen::MatrixXd A = Eigen::MatrixXd(3, 2);
      A.col(0)          = projector_body_z_compl.col(0);
      A.col(1)          = projector_body_z_compl.col(1);

      // create the basis of the projection null-space complement
      Eigen::MatrixXd B = Eigen::MatrixXd(3, 2);
      B.col(0)          = Eigen::Vector3d(1, 0, 0);
      B.col(1)          = Eigen::Vector3d(0, 1, 0);

      // oblique projector to <range_basis>
      Eigen::MatrixXd Bt_A               = B.transpose() * A;
      Eigen::MatrixXd Bt_A_pseudoinverse = ((Bt_A.transpose() * Bt_A).inverse()) * Bt_A.transpose();
      Eigen::MatrixXd oblique_projector  = A * Bt_A_pseudoinverse * B.transpose();

      Rd.col(0) = oblique_projector * bxd;
      Rd.col(0).normalize();

      // | ------------------------- body y ------------------------- |

      Rd.col(1) = Rd.col(2).cross(Rd.col(0));
      Rd.col(1).normalize();
    }
  }

  // --------------------------------------------------------------
  // |                      orientation error                     |
  // --------------------------------------------------------------

  /* orientation error */

  // | -----------------LOAD-------------------------------- |
  Eigen::Matrix3d E = Eigen::Matrix3d::Zero();

  if (!control_reference->use_attitude_rate) {
    E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
  }
  // | --------------------------------------------------- | Not sure why this is needed compared to the line just below

  // Eigen::Matrix3d E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);



  Eigen::Vector3d Eq;

  // clang-format off
  Eq << (E(2, 1) - E(1, 2)) / 2.0,
        (E(0, 2) - E(2, 0)) / 2.0,
        (E(1, 0) - E(0, 1)) / 2.0;
  // clang-format on

  std_msgs::Float64 thrust_force;
  thrust_force.data = f.dot(R.col(2));
  double thrust       = 0;

  // custom publisher
  custom_publisher_projected_thrust_.publish(thrust_force);
  std_msgs::Float64 thrust_norm;
  thrust_norm.data = sqrt(f(0,0)*f(0,0)+f(1,0)*f(1,0)+f(2,0)*f(2,0)); // norm of f ( not projected on the z axis of the UAV frame)
  custom_publisher_thrust_.publish(thrust_norm);
  // print _motor_params_.A and _motor_params_.B
  // ROS_INFO_STREAM("_motor_params_.A = \n" << _motor_params_.A);
  // ROS_INFO_STREAM("_motor_params_.B = \n" << _motor_params_.B);
  // ROS_INFO_STREAM("_thrust_saturation_ = \n" << _thrust_saturation_);
  // OLD double thrust_saturation_physical = pow((_thrust_saturation_-_motor_params_.B)/_motor_params_.A, 2);
  std_msgs::Float64 thrust_saturation_physical;
  thrust_saturation_physical.data = mrs_lib::quadratic_thrust_model::thrustToForce(common_handlers_->motor_params, _thrust_saturation_.data);
  ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: thrust_saturation_physical (N) = %f", thrust_saturation_physical.data);
  // ROS_INFO_STREAM("thrust_saturation_physical = \n" << thrust_saturation_physical);
  // double hover_thrust = total_mass*_g_; use this as most correct if total_mass used in control
  std_msgs::Float64 hover_thrust;
  // Choose one of these two: replace uav_mass by total_mass to get the estimated hover thrust
  // hover_thrust.data = _uav_mass_*common_handlers_->g; 
  hover_thrust.data = total_mass*common_handlers_->g;
  // publish these so you have them in matlab
  pub_thrust_satlimit_physical_.publish(thrust_saturation_physical);
  pub_thrust_satlimit_.publish(_thrust_saturation_);
  pub_hover_thrust_.publish(hover_thrust);

  if (!control_reference->use_thrust) {
    if (thrust_force.data >= 0) {
      thrust = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, thrust_force.data);
    } else {
      ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: just so you know, the desired thrust force is negative (%.2f)", thrust_force.data);
    }
  } else {
    // the thrust is overriden from the tracker command
    thrust = control_reference->thrust;
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {

    thrust = 0;
    ROS_ERROR("[Se3CopyController]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");

  } else if (thrust > _thrust_saturation_.data) {

    thrust = _thrust_saturation_.data;
    ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: saturating thrust to %.2f", _thrust_saturation_.data);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: ---------------------------");
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", control_reference->position.x,
    //                   control_reference->position.y, control_reference->position.z, control_reference->heading);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", control_reference->velocity.x,
    //                   control_reference->velocity.y, control_reference->velocity.z, control_reference->heading_rate);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", control_reference->acceleration.x,
    //                   control_reference->acceleration.y, control_reference->acceleration.z, control_reference->heading_acceleration);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", control_reference->jerk.x, control_reference->jerk.y,
    //                   control_reference->jerk.z, control_reference->heading_jerk);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: ---------------------------");
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", uav_state->pose.position.x, uav_state->pose.position.y,
    //                   uav_state->pose.position.z, uav_heading);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", uav_state->velocity.linear.x,
    //                   uav_state->velocity.linear.y, uav_state->velocity.linear.z, uav_state->velocity.angular.z);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: ---------------------------");

  } else if (thrust < 0.0) {

    thrust = 0.0;
    ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: saturating thrust to 0");
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: ---------------------------");
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", control_reference->position.x,
    //                   control_reference->position.y, control_reference->position.z, control_reference->heading);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", control_reference->velocity.x,
    //                   control_reference->velocity.y, control_reference->velocity.z, control_reference->heading_rate);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", control_reference->acceleration.x,
    //                   control_reference->acceleration.y, control_reference->acceleration.z, control_reference->heading_acceleration);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", control_reference->jerk.x, control_reference->jerk.y,
    //                   control_reference->jerk.z, control_reference->heading_jerk);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: ---------------------------");
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", uav_state->pose.position.x, uav_state->pose.position.y,
    //                   uav_state->pose.position.z, uav_heading);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", uav_state->velocity.linear.x,
    //                   uav_state->velocity.linear.y, uav_state->velocity.linear.z, uav_state->velocity.angular.z);
    // ROS_WARN_THROTTLE(0.1, "[Se3BruboticsLoadController]: ---------------------------");
  }

  // custom publisher
  std_msgs::Float64 thrust_physical_saturated;
  thrust_physical_saturated.data = mrs_lib::quadratic_thrust_model::thrustToForce(common_handlers_->motor_params, thrust);
  //ROS_INFO_STREAM("thrust_physical_saturated = \n" << thrust_physical_saturated);
  pub_thrust_satval_.publish(thrust_physical_saturated);

  // prepare the attitude feedback
  Eigen::Vector3d q_feedback = -Kq * Eq.array();

  if (control_reference->use_attitude_rate) {
    Rw << control_reference->attitude_rate.x, control_reference->attitude_rate.y, control_reference->attitude_rate.z;
  } else if (control_reference->use_heading_rate) {

    // to fill in the feed forward yaw rate
    double desired_yaw_rate = 0;

    try {
      desired_yaw_rate = mrs_lib::AttitudeConverter(Rd).getYawRateIntrinsic(control_reference->heading_rate);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: exception caught while calculating the desired_yaw_rate feedforward");
    }

    Rw << 0, 0, desired_yaw_rate;
  }

  // feedforward angular acceleration
  Eigen::Vector3d q_feedforward = Eigen::Vector3d(0, 0, 0);

  if (drs_params.jerk_feedforward) {

    Eigen::Matrix3d I;
    I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
    Eigen::Vector3d desired_jerk = Eigen::Vector3d(control_reference->jerk.x, control_reference->jerk.y, control_reference->jerk.z);
    q_feedforward                = (I.transpose() * Rd.transpose() * desired_jerk) / (thrust_force.data / total_mass);
  }

  // angular feedback + angular rate feedforward
  Eigen::Vector3d t = q_feedback + Rw + q_feedforward;

  // compensate for the parasitic heading rate created by the desired pitch and roll rate
  Eigen::Vector3d rp_heading_rate_compensation = Eigen::Vector3d(0, 0, 0);

  if (drs_params.pitch_roll_heading_rate_compensation) {

    Eigen::Vector3d q_feedback_yawless = t;
    q_feedback_yawless(2)              = 0;  // nullyfy the effect of the original yaw feedback

    double parasitic_heading_rate = 0;

    try {
      parasitic_heading_rate = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeadingRate(q_feedback_yawless);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: exception caught while calculating the parasitic heading rate!");
    }

    try {
      rp_heading_rate_compensation(2) = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getYawRateIntrinsic(-parasitic_heading_rate);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: exception caught while calculating the parasitic heading rate compensation!");
    }
  }

  t += rp_heading_rate_compensation;

  // --------------------------------------------------------------
  // |                      update parameters                     |
  // --------------------------------------------------------------

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error
    if (control_reference->use_position_horizontal) {
      Iw_w_ -= kiwxy_ * Ep.head(2) * dt;
    } else if (control_reference->use_velocity_horizontal) {
      Iw_w_ -= kiwxy_ * Ev.head(2) * dt;
    }

    // saturate the world X
    // double world_integral_saturated = false; seems incorrect
    bool world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[0])) {
      Iw_w_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: NaN detected in variable 'Iw_w_[0]', setting it to 0!!!");
    } else if (Iw_w_[0] > kiwxy_lim_) {
      Iw_w_[0]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[0] < -kiwxy_lim_) {
      Iw_w_[0]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: SE3's world X integral is being saturated!");
    }

    // saturate the world Y
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[1])) {
      Iw_w_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: NaN detected in variable 'Iw_w_[1]', setting it to 0!!!");
    } else if (Iw_w_[1] > kiwxy_lim_) {
      Iw_w_[1]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[1] < -kiwxy_lim_) {
      Iw_w_[1]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: SE3's world Y integral is being saturated!");
    }
  }

  //}

  /* body error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the body error                  |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    Eigen::Vector2d Ep_fcu_untilted = Eigen::Vector2d(0, 0);  // position error in the untilted frame of the UAV
    Eigen::Vector2d Ev_fcu_untilted = Eigen::Vector2d(0, 0);  // velocity error in the untilted frame of the UAV

    // get the position control error in the fcu_untilted frame
    {

      geometry_msgs::Vector3Stamped Ep_stamped;

      Ep_stamped.header.stamp    = ros::Time::now();
      Ep_stamped.header.frame_id = uav_state_.header.frame_id;
      Ep_stamped.vector.x        = Ep(0);
      Ep_stamped.vector.y        = Ep(1);
      Ep_stamped.vector.z        = Ep(2);

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ep_stamped);

      if (res) {
        Ep_fcu_untilted[0] = res.value().vector.x;
        Ep_fcu_untilted[1] = res.value().vector.y;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: could not transform the position error to fcu_untilted");
      }
    }

    // get the velocity control error in the fcu_untilted frame
    {
      geometry_msgs::Vector3Stamped Ev_stamped;

      Ev_stamped.header.stamp    = ros::Time::now();
      Ev_stamped.header.frame_id = uav_state_.header.frame_id;
      Ev_stamped.vector.x        = Ev(0);
      Ev_stamped.vector.y        = Ev(1);
      Ev_stamped.vector.z        = Ev(2);

      auto res = common_handlers_->transformer->transformSingle("fcu_untilted", Ev_stamped);

      if (res) {
        Ev_fcu_untilted[0] = res.value().vector.x;
        Ev_fcu_untilted[1] = res.value().vector.x;
      } else {
        ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: could not transform the velocity error to fcu_untilted");
      }
    }

    // integrate the body error
    if (control_reference->use_position_horizontal) {
      Ib_b_ -= kibxy_ * Ep_fcu_untilted * dt;
    } else if (control_reference->use_velocity_horizontal) {
      Ib_b_ -= kibxy_ * Ev_fcu_untilted * dt;
    }

    // saturate the body
    // double body_integral_saturated = false;
    bool body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[0])) {
      Ib_b_[0] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: NaN detected in variable 'Ib_b_[0]', setting it to 0!!!");
    } else if (Ib_b_[0] > kibxy_lim_) {
      Ib_b_[0]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[0] < -kibxy_lim_) {
      Ib_b_[0]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: SE3's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[1])) {
      Ib_b_[1] = 0;
      ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: NaN detected in variable 'Ib_b_[1]', setting it to 0!!!");
    } else if (Ib_b_[1] > kibxy_lim_) {
      Ib_b_[1]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[1] < -kibxy_lim_) {
      Ib_b_[1]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: SE3's body roll integral is being saturated!");
    }
  }

  //}

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    if (control_reference->use_position_vertical && !rampup_active_) {
      uav_mass_difference_ -= km_ * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  //}

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now();

  // | ------------ compensated desired acceleration ------------ |

  double desired_x_accel = 0;
  double desired_y_accel = 0;
  double desired_z_accel = 0;

  {

    Eigen::Matrix3d des_orientation = mrs_lib::AttitudeConverter(Rd);
    Eigen::Vector3d thrust_vector   = thrust_force.data * des_orientation.col(2);

    double world_accel_x = (thrust_vector[0] / total_mass) - (Iw_w_[0] / total_mass) - (Ib_w[0] / total_mass);
    double world_accel_y = (thrust_vector[1] / total_mass) - (Iw_w_[1] / total_mass) - (Ib_w[1] / total_mass);
    double world_accel_z = (thrust_vector[2] / total_mass) - common_handlers_->g;

    geometry_msgs::Vector3Stamped world_accel;

    world_accel.header.stamp    = ros::Time::now();
    world_accel.header.frame_id = uav_state->header.frame_id;
    world_accel.vector.x        = world_accel_x;
    world_accel.vector.y        = world_accel_y;
    world_accel.vector.z        = world_accel_z;

    auto res = common_handlers_->transformer->transformSingle("fcu", world_accel);

    if (res) {

      desired_x_accel = res.value().vector.x;
      desired_y_accel = res.value().vector.y;
      desired_z_accel = res.value().vector.z;
    }
  }

  // | --------------- saturate the attitude rate --------------- |

  if (got_constraints_) {

    auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);

    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: constraints.roll_rate = %f", constraints.roll_rate);
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: constraints.pitch_rate = %f", constraints.pitch_rate);
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: constraints.yaw_rate = %f", constraints.yaw_rate);

    if (t[0] > constraints.roll_rate) {
      t[0] = constraints.roll_rate;
    } else if (t[0] < -constraints.roll_rate) {
      t[0] = -constraints.roll_rate;
    }

    if (t[1] > constraints.pitch_rate) {
      t[1] = constraints.pitch_rate;
    } else if (t[1] < -constraints.pitch_rate) {
      t[1] = -constraints.pitch_rate;
    }

    if (t[2] > constraints.yaw_rate) {
      t[2] = constraints.yaw_rate;
    } else if (t[2] < -constraints.yaw_rate) {
      t[2] = -constraints.yaw_rate;
    }
  } else {
    ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: missing dynamics constraints");
  }

  // | --------------- fill the resulting command --------------- |

  auto output_mode = mrs_lib::get_mutexed(mutex_output_mode_, output_mode_);

  // fill in the desired attitude anyway, since we know it
  output_command->attitude = mrs_lib::AttitudeConverter(Rd);

  if (output_mode == OUTPUT_ATTITUDE_RATE) {

    // output the desired attitude rate
    output_command->attitude_rate.x = t[0];
    output_command->attitude_rate.y = t[1];
    output_command->attitude_rate.z = t[2];

    output_command->mode_mask = output_command->MODE_ATTITUDE_RATE;

  } else if (output_mode == OUTPUT_ATTITUDE_QUATERNION) {

    output_command->mode_mask = output_command->MODE_ATTITUDE;

    ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: outputting desired orientation (this is not normal)");
  }

  output_command->desired_acceleration.x = desired_x_accel;
  output_command->desired_acceleration.y = desired_y_accel;
  output_command->desired_acceleration.z = desired_z_accel;

  if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

      rampup_active_         = false;
      output_command->thrust = thrust;

      ROS_INFO("[Se3CopyController]: rampup finished");

    } else {

      double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

      rampup_thrust_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

      rampup_last_time_ = ros::Time::now();

      output_command->thrust = rampup_thrust_;

      ROS_INFO_THROTTLE(0.1, "[Se3CopyController]: ramping up thrust, %.4f", output_command->thrust);
    }

  } else {
    output_command->thrust = thrust;
  }

  output_command->ramping_up = rampup_active_;

  output_command->mass_difference = uav_mass_difference_;
  output_command->total_mass      = total_mass;

  output_command->disturbance_bx_b = -Ib_b_[0];
  output_command->disturbance_by_b = -Ib_b_[1];

  output_command->disturbance_bx_w = -Ib_w[0];
  output_command->disturbance_by_w = -Ib_w[1];

  output_command->disturbance_wx_w = -Iw_w_[0];
  output_command->disturbance_wy_w = -Iw_w_[1];

  output_command->controller_enforcing_constraints = false;

  output_command->controller = "Se3CopyController";

  last_attitude_cmd_ = output_command;

  return output_command;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus Se3CopyController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void Se3CopyController::switchOdometrySource(const mrs_msgs::UavState::ConstPtr& new_uav_state) {

  ROS_INFO("[Se3CopyController]: switching the odometry source");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  // | ----- transform world disturabances to the new frame ----- |

  geometry_msgs::Vector3Stamped world_integrals;

  world_integrals.header.stamp    = ros::Time::now();
  world_integrals.header.frame_id = uav_state.header.frame_id;

  world_integrals.vector.x = Iw_w_[0];
  world_integrals.vector.y = Iw_w_[1];
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(new_uav_state->header.frame_id, world_integrals);

  if (res) {

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = res.value().vector.x;
    Iw_w_[1] = res.value().vector.y;

  } else {

    ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: could not transform world integral to the new frame");

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = 0;
    Iw_w_[1] = 0;
  }
}

//}

/* resetDisturbanceEstimators() //{ */

void Se3CopyController::resetDisturbanceEstimators(void) {

  std::scoped_lock lock(mutex_integrals_);

  Iw_w_ = Eigen::Vector2d::Zero(2);
  Ib_b_ = Eigen::Vector2d::Zero(2);
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr Se3CopyController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  got_constraints_ = true;

  ROS_INFO("[Se3CopyController]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------


// | ----------------- LOAD ----------------------------------------- | 
// | ----------------- load subscribtion callback --------------|

void Se3CopyController::loadStatesCallback(const gazebo_msgs::LinkStatesConstPtr& loadmsg) {
  int load_index = -1;
  std::vector<std::string> link_names = loadmsg->name;

  for(size_t i = 0; i < link_names.size(); i++)
  {
    if (number_of_uav == "2")
    {
      if (uav_id) // for uav1
      {
        if(link_names[i] == "bar::link_04"){
            load_index = i;
            payload_spawned = true;
        }
      }else { // for uav2
        if(link_names[i] == "bar::link_01"){
            load_index = i;
            payload_spawned = true;
        }
      }
    }else{
      if(link_names[i] == "bar::link_01")
      {
        load_index = i;
        payload_spawned = true;
      }else{
      }  
    }   
  }
  
  load_pose = loadmsg->pose[load_index];
  load_pose_position.x = load_pose.position.x;
  load_pose_position.y = load_pose.position.y;
  load_pose_position.z = load_pose.position.z;

  //ROS_INFO_STREAM("Load Pose \n" << load_pose );

  // for (int i = 0; i < 3; i++) // to set it to zero when the load hasn't spawn yet
  // {
  //     if (load_pose_position[i] > 1000)
  //     {
  //       load_pose_position[i] = 0;
  //     } else {
  //       load_pose_position[i] = load_pose_position[i];
  //     }
  // }

  if (load_pose_position.x > 1000)
      {
        load_pose_position.x = 0;
      }
  if (load_pose_position.y > 1000)
      {
        load_pose_position.y = 0;
      }
  if (load_pose_position.z > 1000)
      {
        load_pose_position.z = 0;
      }

  load_velocity = loadmsg->twist[load_index];
  

  load_lin_vel[0]= load_velocity.linear.x;
  load_lin_vel[1]= load_velocity.linear.y;
  load_lin_vel[2]= load_velocity.linear.z;
  custom_publisher_load_pose.publish(load_pose);
  custom_publisher_load_vel.publish(load_velocity);
}


void Se3CopyController::BacaCallback(const mrs_msgs::BacaProtocolConstPtr& msg) {
  int message_id;
  int payload_1;
  int payload_2;
  message_id = msg->payload[0];
  payload_1 = msg->payload[1];
  payload_2 = msg->payload[2];

  int16_t combined = payload_1 << 8;
  combined |= payload_2;


  float encoder_output = (float) combined/ 1000.0;
  if (message_id == 24)
  {
    encoder_angle_1 = encoder_output;
  }else if (message_id == 25)
  {
    encoder_angle_2 = encoder_output;
  }else if (message_id == 32)
  {
    encoder_velocity_1 = encoder_output;
  }else
  {
    encoder_velocity_2 = encoder_output;
  }

  encoder_angle_1_to_publish.data = encoder_angle_1;
  publisher_encoder_angle_1.publish(encoder_angle_1_to_publish);
  encoder_angle_2_to_publish.data = encoder_angle_2;
  publisher_encoder_angle_2.publish(encoder_angle_2_to_publish);
  //ROS_INFO_STREAM("Encoder_angle_1 \n" << encoder_angle_1_to_publish );
  //ROS_INFO_STREAM("Encoder_angle_2 \n" << encoder_angle_2_to_publish );
  

  rel_load_pose_position[0] = cable_length*sin(encoder_angle_1); // x relative to drone in drone coordinate (turns with uav_heading)
  rel_load_pose_position[1] = cable_length*sin(encoder_angle_2); // y relative to drone in drone coordinate (turns with uav_heading)
  rel_load_pose_position[2] = -sqrt(pow(cable_length,2) - (pow(rel_load_pose_position[0],2) + pow(rel_load_pose_position[1],2))); // z relative to drone in drone coordinate (turns with uav_heading)

  rel_load_pose_position_to_publish.x = rel_load_pose_position[0];
  rel_load_pose_position_to_publish.y = rel_load_pose_position[1];
  rel_load_pose_position_to_publish.z = rel_load_pose_position[2];

  publisher_rel_load_pose_position.publish(rel_load_pose_position_to_publish);
  //ROS_INFO_STREAM("rel load pose position \n" << rel_load_pose_position_to_publish );

  Difference_load_drone_position[0] = (rel_load_pose_position[0]*cos(uav_heading) - rel_load_pose_position[1]*sin(uav_heading)); // x relative to drone in absolute coordinate
  Difference_load_drone_position[1] = (-(rel_load_pose_position[0]*sin(uav_heading) + rel_load_pose_position[1]*cos(uav_heading))); // y relative to drone in absolute coordinate
  Difference_load_drone_position[2] = (rel_load_pose_position[2]); // z relative to drone in absolute coordinate

  Difference_load_drone_position_to_publish.x = Difference_load_drone_position[0];
  Difference_load_drone_position_to_publish.y = Difference_load_drone_position[1];
  Difference_load_drone_position_to_publish.z = Difference_load_drone_position[2];


  publisher_Difference_load_drone_position.publish(Difference_load_drone_position_to_publish);
  //ROS_INFO_STREAM("Difference_load_drone_position \n" << Difference_load_drone_position_to_publish );

  //load_pose.position.x = load_pose_position[0];
  //load_pose.position.y = load_pose_position[1];
  //load_pose.position.z = load_pose_position[2];
  
  //ROS_INFO_STREAM("Load Pose \n" << load_pose );
  
  load_lin_vel[0]= encoder_velocity_1*cable_length;
  load_lin_vel[1]= encoder_velocity_2*cable_length;
  load_lin_vel[2]= 0;
}
// | ------------------------------------------------------------------- | 


/* //{ callbackDrs() */

void Se3CopyController::callbackDrs(mrs_uav_controllers::se3_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_, mutex_output_mode_);

    drs_params_ = config;

    output_mode_ = config.output_mode;
  }

  ROS_INFO("[Se3CopyController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

/* filterGains() //{ */

void Se3CopyController::filterGains(const bool mute_gains, const double dt) {

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains || gains_muted_);
  double gain_coeff    = (mute_gains || gains_muted_) ? _gain_mute_coefficient_ : 1.0;

  gains_muted_ = mute_gains;

  // calculate the difference
  {
    std::scoped_lock lock(mutex_gains_, mutex_drs_params_);

    bool updated = false;

    kpxy_  = calculateGainChange(dt, kpxy_, drs_params_.kpxy * gain_coeff, bypass_filter, "kpxy", updated);
    kvxy_  = calculateGainChange(dt, kvxy_, drs_params_.kvxy * gain_coeff, bypass_filter, "kvxy", updated);
    kaxy_  = calculateGainChange(dt, kaxy_, drs_params_.kaxy * gain_coeff, bypass_filter, "kaxy", updated);
    kiwxy_ = calculateGainChange(dt, kiwxy_, drs_params_.kiwxy * gain_coeff, bypass_filter, "kiwxy", updated);
    kibxy_ = calculateGainChange(dt, kibxy_, drs_params_.kibxy * gain_coeff, bypass_filter, "kibxy", updated);
    kpz_   = calculateGainChange(dt, kpz_, drs_params_.kpz * gain_coeff, bypass_filter, "kpz", updated);
    kvz_   = calculateGainChange(dt, kvz_, drs_params_.kvz * gain_coeff, bypass_filter, "kvz", updated);
    kaz_   = calculateGainChange(dt, kaz_, drs_params_.kaz * gain_coeff, bypass_filter, "kaz", updated);
    kqxy_  = calculateGainChange(dt, kqxy_, drs_params_.kqxy * gain_coeff, bypass_filter, "kqxy", updated);
    kqz_   = calculateGainChange(dt, kqz_, drs_params_.kqz * gain_coeff, bypass_filter, "kqz", updated);
    km_    = calculateGainChange(dt, km_, drs_params_.km * gain_coeff, bypass_filter, "km", updated);

    kiwxy_lim_ = calculateGainChange(dt, kiwxy_lim_, drs_params_.kiwxy_lim, false, "kiwxy_lim", updated);
    kibxy_lim_ = calculateGainChange(dt, kibxy_lim_, drs_params_.kibxy_lim, false, "kibxy_lim", updated);
    km_lim_    = calculateGainChange(dt, km_lim_, drs_params_.km_lim, false, "km_lim", updated);

    // set the gains back to dynamic reconfigure
    // and only do it when some filtering occurs
    if (updated) {

      DrsConfig_t new_drs_params = drs_params_;

      new_drs_params.kpxy        = kpxy_;
      new_drs_params.kvxy        = kvxy_;
      new_drs_params.kaxy        = kaxy_;
      new_drs_params.kiwxy       = kiwxy_;
      new_drs_params.kibxy       = kibxy_;
      new_drs_params.kpz         = kpz_;
      new_drs_params.kvz         = kvz_;
      new_drs_params.kaz         = kaz_;
      new_drs_params.kqxy        = kqxy_;
      new_drs_params.kqz         = kqz_;
      new_drs_params.kiwxy_lim   = kiwxy_lim_;
      new_drs_params.kibxy_lim   = kibxy_lim_;
      new_drs_params.km          = km_;
      new_drs_params.km_lim      = km_lim_;
      new_drs_params.output_mode = output_mode_;

      drs_->updateConfig(new_drs_params);
    }
  }
}

//}

/* calculateGainChange() //{ */

double Se3CopyController::calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name,
                                          bool& updated) {

  double change = desired_value - current_value;

  double gains_filter_max_change = _gains_filter_change_rate_ * dt;
  double gains_filter_min_change = _gains_filter_min_change_rate_ * dt;

  if (!bypass_rate) {

    // if current value is near 0...
    double change_in_perc;
    double saturated_change;

    if (fabs(current_value) < 1e-6) {
      change *= gains_filter_max_change;
    } else {

      saturated_change = change;

      change_in_perc = (current_value + saturated_change) / current_value - 1.0;

      if (change_in_perc > gains_filter_max_change) {
        saturated_change = current_value * gains_filter_max_change;
      } else if (change_in_perc < -gains_filter_max_change) {
        saturated_change = current_value * -gains_filter_max_change;
      }

      if (fabs(saturated_change) < fabs(change) * gains_filter_min_change) {
        change *= gains_filter_min_change;
      } else {
        change = saturated_change;
      }
    }
  }

  if (fabs(change) > 1e-3) {
    ROS_INFO_THROTTLE(1.0, "[Se3CopyController]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

}  // namespace se3_copy_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::se3_copy_controller::Se3CopyController, mrs_uav_managers::Controller)
