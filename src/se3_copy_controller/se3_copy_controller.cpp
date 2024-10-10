/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <common.h> //#include <mrs_uav_controllers/common.h> // TODO: does not work as in https://github.com/ctu-mrs/mrs_uav_controllers/blob/master/src/se3_controller.cpp
#include <pid.hpp> // TODO same comment as above

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <controllers_brubotics/se3_copy_controllerConfig.h>

#include <mrs_lib/profiler.h>
#include <mrs_lib/utils.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>

#include <geometry_msgs/Vector3Stamped.h>

// our includes: 
#include <se3_copy_controller/se3_copy_controller.h>
#include <mrs_lib/param_loader.h>
#include <math.h> 
// | ----------------- custom publisher---------------- |
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mrs_msgs/BoolStamped.h>
#include <vector>
// | ----------------- Load---------------- |
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>   // for the position
#include <geometry_msgs/Twist.h> //for the velocity
#include <mrs_modules_msgs/BacaProtocol.h>
#include <std_msgs/UInt8.h>
// | --------------------------------- |

//}

#define OUTPUT_ACTUATORS 0
#define OUTPUT_CONTROL_GROUP 1
#define OUTPUT_ATTITUDE_RATE 2
#define OUTPUT_ATTITUDE 3

namespace controllers_brubotics
{

namespace se3_copy_controller
{

/* //{ class Se3CopyController */

class Se3CopyController : public mrs_uav_managers::Controller {

public: 
  bool initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);
  
  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command);

  ControlOutput updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  ros::NodeHandle nh_;  // Se3CopyController
  ros::NodeHandle nh2_; // DergbryanTracker // TODO: choose better names

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;
  
  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;
  
  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                   mutex_drs_;
  typedef controllers_brubotics::se3_copy_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>         Drs_t;
  boost::shared_ptr<Drs_t>                                 drs_;
  void                                                     callbackDrs(controllers_brubotics::se3_copy_controllerConfig& config, uint32_t level);
  DrsConfig_t                                              drs_params_;


  // | ----------------------- controllers ---------------------- |

  void positionPassthrough(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  void PIDVelocityOutput(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command, const common::CONTROL_OUTPUT& control_output,
                         const double& dt);

  void SE3Controller(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command, const double& dt,
                     const common::CONTROL_OUTPUT& output_modality);

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;


  // | --------- throttle generation and mass estimation -------- |
  double _uav_mass_; // feedforward uav mass
  double uav_mass_difference_; // total mass difference (integral error over uav position)
  
  
  // | -------------------------- gains ------------------------- |
  Gains_t gains_;

  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs

  ros::Timer timer_gains_;
  void       timerGains(const ros::TimerEvent& event);

  double _gain_filtering_rate_;

  // | ----------------------- gain muting ---------------------- |

  std::atomic<bool> mute_gains_            = false;
  std::atomic<bool> mute_gains_by_tracker_ = false;
  double            _gain_mute_coefficient_;

  // | --------------------- gain filtering --------------------- |

  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool& updated);

  double getHeadingSafely(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;

  // | ------------ controller limits and saturations ----------- |

  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;


  double _throttle_saturation_; // total thrust limit in [0,1] // TODO: make tracker and controller same in how type of _throttle_saturation_ is chosen

 
   // | ------------------ activation and output ----------------- |
  ControlOutput last_control_output_; 
  ControlOutput activation_control_output_;

  ros::Time last_update_time_;
  bool      first_iteration_ = true;

  // | ------------------ profiler_ ----------------- |

  mrs_lib::Profiler profiler_;
  bool   _profiler_enabled_ = false;

  // | ------------------------ integrals ----------------------- |

  Eigen::Vector2d Ib_b_;  // body error integral in the body frame
  Eigen::Vector2d Iw_w_;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;
 

  // | ------------------------- rampup ------------------------- |
  bool   _rampup_enabled_ = false;
  double _rampup_speed_;

  bool      rampup_active_ = false;
  double    rampup_throttle_;
  int       rampup_direction_;
  double    rampup_duration_;
  ros::Time rampup_start_time_;
  ros::Time rampup_last_time_;

  // | ---------------------- position pid ---------------------- |

  double _pos_pid_p_;
  double _pos_pid_i_;
  double _pos_pid_d_;

  double _hdg_pid_p_;
  double _hdg_pid_i_;
  double _hdg_pid_d_;

  PIDController position_pid_x_;
  PIDController position_pid_y_;
  PIDController position_pid_z_;
  PIDController position_pid_heading_;



  // TODO: clean below------------
  double dt_ = 0.010; // DO NOT CHANGE! Hardcoded controller sample time = controller sample time TODO: obtain via loop rate, see MpcTracker
  double uav_heading_;

  // | ------------------- declaring env (session/bashrc) parameters ------------------- |
  
  std::string _uav_name_;         // uavID
  std::string _leader_uav_name_;  // leader uavID for 2uavs payload transport
  std::string _follower_uav_name_;// follower uavID for 2uavs payload transport
  std::string _run_type_;         // set to "simulation" (for Gazebo simulation) OR "uav" (for hardware testing) defined in bashrc or session.yaml. Used for payload transport as payload position comes from two different callbacks depending on how the test is ran (in sim or on real UAV).
  std::string _type_of_system_;   // defines the dynamic system model to simulate in the prediction using the related controller: can be 1uav_no_payload, 1uav_payload or 2uavs_payload. Set in session.yaml file.
  double _cable_length_;          // length of the cable between payload COM / anchoring point and COM of the UAV
  double _cable_length_offset_; // accounts for the fact that the cable is attached below the UAV's COM
  double _load_mass_;             // feedforward load mass per uav defined in the session.yaml of every test file (session variable also used by the xacro for Gazebo simulation)
  double _load_length_;         // length of bar load transported by 2uavs
  bool _baca_in_simulation_=false;// Used to validate the encoder angles, and the FK without having to make the UAV fly. Gains related to payload must be set on 0 to perform this. Set on false by default.
  //emulate nimbro
  bool emulate_nimbro_ = false;
  double emulate_nimbro_delay_;
  double emulate_nimbro_time_ = 0;

  // | ------------------- declaring .yaml parameters (and some related vars & funs) ------------------- |
 
  double _Epl_min_; // [m], below this payload error norm, the payload error is disabled
  bool _Epl_max_failsafe_enabled_;
  double _Epl_max_scaling_; // [m]
  bool max_swing_angle_failsafe_enabled_;
  double max_swing_angle_;

  // ---------------
  // ROS Publishers:
  // ---------------
  // TODO: check that publishers use msgs with time stamp info and correctly used for plotting. Never use ros time now as not synchronized.
  //|-----------------------------UAV--------------------------------|//
  // TODO: document
  ros::Publisher uav_state_publisher_; // TODO: already publishing to this in Tracker!
  ros::Publisher projected_thrust_publisher_;
  ros::Publisher thrust_publisher_;
  ros::Publisher thrust_satlimit_publisher_;
  ros::Publisher thrust_satlimit_physical_publisher_;
  ros::Publisher thrust_satval_publisher_;
  ros::Publisher hover_thrust_publisher_;
  ros::Publisher tilt_angle_publisher_;
  //|-----------------------------LOAD--------------------------------|//
  ros::Publisher load_pose_publisher_; //Publisher used to publish the absolute position of payload, for both simulation and hardware
  geometry_msgs::Pose anchoring_pt_pose_; //msg to be published as the position of the payload.
  Eigen::Vector3d anchoring_pt_pose_position_; //Vector of the absolute position of the payload.
  ros::Publisher load_vel_publisher_; //Publisher that publish the absolute velocity of the payload, both for simulation and hardware.
  geometry_msgs::Twist anchoring_pt_velocity_; //Msg used to be published as the velocity of payload
  Eigen::Vector3d anchoring_pt_lin_vel_; //Vector of the absolute velocity of the payload.
  ros::Publisher load_position_errors_publisher_; // Publisher that will publish the error on the payload (Epl). 
  ros::Publisher encoder_angle_1_publisher_; //Publishers used to publish the angles received from the bacaprotocol/Arduino.
  ros::Publisher encoder_angle_2_publisher_;
  //|-----------------------------2UAVs safety communication--------------------------------|//
  ros::Publisher Eland_controller_leader_to_follower_pub_;
  ros::Publisher Eland_controller_follower_to_leader_pub_;
  ros::Publisher ros_delay_pub_;
  ros::Publisher ros_time_l_to_f_pub_;
  ros::Publisher ros_time_delay_pub_;
  ros::Publisher Eland_time_pub_;
  // ros::Publisher ros_time_pub_;
  // ros::Publisher ros_time_trigger_l_pub_;
  ros::Publisher time_delay_Eland_controller_leader_to_follower_pub_;
  ros::Publisher time_delay_Eland_controller_follower_to_leader_pub_;
  // Distance between the two UAVs
  ros::Publisher distance_uavs_pub_;

  // ---------------
  // ROS Subscribers:
  // ---------------
  // TODO: 
  ros::Subscriber load_state_sub_; //Subscriber used to find the state of the payload, from Gazebo (during simulations only).
  void GazeboLoadStatesCallback(const gazebo_msgs::LinkStatesConstPtr& loadmsg); // TODO: bryan think how we can make library to use these function in both controller and tracker if exactly same
  bool payload_spawned_ = false; // inititally not spawned
  bool payload_once_spawned_ = false;
  double time_last_payload_message = 0;
  ros::Subscriber data_payload_sub_;  //Subscriber used to get the state of the payload, from Arduino data/baca protocol. (during hardware testing only)
  void BacaLoadStatesCallback(const mrs_modules_msgs::BacaProtocolConstPtr& msg); // TODO: bryan think how we can make library to use these function in both controller and tracker if exactly same
  float encoder_angle_1_; //Angles returned by Arduino/Encoders via Bacaprotocol
  float encoder_angle_2_;
  float encoder_velocity_1_;//Angular velocities returned by Arduino/Encoders via Bacaprotocol
  float encoder_velocity_2_;

  // 2UAVs safety communication
  void RosTimeDifference(void);
  void SafetyCommunication(void);
  ros::Subscriber Eland_controller_leader_to_follower_sub_;
  void ElandLeaderToFollowerCallback(const mrs_msgs::BoolStamped& msg);
  mrs_msgs::BoolStamped Eland_controller_leader_to_follower_;
  std_msgs::Float64 time_delay_Eland_controller_leader_to_follower_out_;
  ros::Subscriber Eland_controller_follower_to_leader_sub_; 
  void ElandFollowerToLeaderCallback(const mrs_msgs::BoolStamped& msg);
  mrs_msgs::BoolStamped Eland_controller_follower_to_leader_;
  std_msgs::Float64 time_delay_Eland_controller_follower_to_leader_out_;
  // ros::Subscriber ros_time_trigger_l_sub_;
  // void rosTimeTriggerCallback(const std_msgs::Bool& msg);
  // std_msgs::Float64 ros_time_out_;
  // std_msgs::Bool ros_time_trigger_l_;
  ros::Subscriber ros_time_l_to_f_sub_;
  void rosTimeLeaderToFollowerCallback(const std_msgs::Float64& msg);
  std_msgs::Float64 ros_time_l_to_f_;
  std_msgs::Float64 ros_time_delay_;
  std_msgs::Float64 Eland_time_;
  double _max_time_delay_eland_;
  bool Eland_status_ = false;
  // double _max_time_delay_on_callback_data_follower_;
  // double _max_time_delay_on_callback_data_leader_;
  bool both_uavs_connected_ = false;
  double start_time_ = 0;
  double connection_time_ = 0;
  double ros_time_difference_delay_;
  double ros_time_difference_duration_;
  bool ros_time_difference_enabled_;
  bool busy_ros_time_l_to_f_ = false;
  double ready_delay_;
  bool both_uavs_ready_ = false;
  bool distance_uavs_failsafe_enabled_ = false;
  bool distance_uavs_ready_ = false;
  double distance_uavs_max_error_;
    // Distance between the two UAVs
  std_msgs::Float64 distance_UAVs_out_;
  ros::Subscriber uav_state_follower_for_leader_sub_;
  void uav_state_follower_for_leader_callback(const mrs_msgs::UavState::ConstPtr& msg);
  mrs_msgs::UavState uav_state_follower_for_leader_;

  // | ----------------- load ---------------- |
  geometry_msgs::Pose load_position_errors;


  //|----------------------------------------------|//
  // TODO: check if needs to be used?
  //Eigen::Vector3d load_pose_position_offset_ = Eigen::Vector3d::Zero(3); // TODO: document

  //|---------------------Bacacallback and encoder offset-------------------|//
  std_msgs::Float64 encoder_angle_1_to_publish_; //To publish the angles received from the Baca protocol. Used only for possible debugging.
  std_msgs::Float64 encoder_angle_2_to_publish_;
  Eigen::Vector3d offset_anchoring_pt_ = Eigen::Vector3d::Zero(3);//The payload position offset, that will be computed from the two average values above.

  // geometry_msgs::Vector3 load_pose_error;
  // geometry_msgs::Vector3 load_velocity_error;
  
  //Eigen::Matrix3d R;
 
  // std::string number_of_uav;
  // | -----------------------------Load transport----------------------------- |
  // geometry_msgs::Vector3 rel_load_pose_position_to_publish;
  // geometry_msgs::Vector3 Difference_load_drone_position_to_publish;
  // geometry_msgs::Vector3 sum_load_pose;
  // geometry_msgs::Vector3 average_load_pose;
  // geometry_msgs::Vector3 sum_drone_pose;
  // geometry_msgs::Vector3 offset;
  // geometry_msgs::Vector3 average_drone_pose;
  // Eigen::Vector3d rel_load_pose_position = Eigen::Vector3d::Zero(3);
  // Eigen::Vector3d Difference_load_drone_position = Eigen::Vector3d::Zero(3);

  // debug:
  double ROS_INFO_THROTTLE_PERIOD; // = 0.1;

};

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() */
bool Se3CopyController::initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                               std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {  
  
  ROS_INFO("[Se3CopyController]: start of initialize");

  // TODO: make global vars of these as in examples ctu. First make local and then set global equal to local.
  ros::NodeHandle nh_(nh, "se3_copy_controller"); // NodeHandle for Se3CopyController, used to load controller params
  ros::NodeHandle nh2_(nh, "dergbryan_tracker"); // NodeHandle 2 for DergbryanTracker, used to load tracker params
  
  common_handlers_ = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass(); // TODO: document which mass this is: the feedforward mass of the UAV?

  // last_update_time_ = ros::Time(0); // only in https://github.com/ctu-mrs/example_controller_plugin/blob/master/src/example_controller.cpp

  ros::Time::waitForValid();
  

  // --------------------------------------------------------------
  //            loading env (session/bashrc) parameters           |
  // --------------------------------------------------------------


  ROS_INFO("[Se3CopyController]: start loading environment (session/bashrc) parameters");
  
  _uav_name_       = getenv("UAV_NAME"); // TODO: ask ctu if they can allow uav_name as an argument of the inititalize function as done in the trackers.h. So then we do not need to specify the UAV IDs anymore to enable 2UAV cooperative load transport.
  //_uav_mass_       = std::stod(getenv("UAV_MASS")); // _uav_mass_       = uav_mass; // TODO: is the loaded uav_mass as argument of init same as uav_mass exported in session?
  _run_type_       = getenv("RUN_TYPE"); 
  _type_of_system_ = getenv("TYPE_OF_SYSTEM"); 
  if(_type_of_system_=="1uav_payload" || _type_of_system_=="2uavs_payload"){ // load the required load transportation paramters only if the test is configured for it
    _cable_length_      = std::stod(getenv("CABLE_LENGTH")); 
    _cable_length_offset_ = - std::stod(getenv("UAV_LOAD_OFFSET_Z")); 
    _cable_length_ = _cable_length_ + _cable_length_offset_;
    if (_type_of_system_=="1uav_payload"){
      _load_mass_         = std::stod(getenv("LOAD_MASS")); // LOAD_MASS is the total load mass of the to be transported point mass object
    }
    else if (_type_of_system_=="2uavs_payload"){ 
      _load_mass_ = 0.50 * std::stod(getenv("LOAD_MASS")); // in case of 2uavs, each uav takes only half of the total bar-type load
      _load_length_ = std::stod(getenv("LOAD_LENGTH"));
      _leader_uav_name_ = "uav"+std::to_string(std::stoi(getenv("LEADER_UAV_ID")));
      _follower_uav_name_ = "uav"+std::to_string(std::stoi(getenv("FOLLOWER_UAV_ID")));
      // Sanity check:
      if(_uav_name_ !=_leader_uav_name_ && _uav_name_ !=_follower_uav_name_){
        ROS_ERROR("[Se3CopyController]: _uav_name_ is different from _leader_uav_name_ and _follower_uav_name_!");
        ros::requestShutdown();
      }
    }
    // More sanity checks:
    if(_cable_length_ <=0){
      ROS_ERROR("[Se3CopyController]: _cable_length_ <=0, use a value > 0!");
      ros::requestShutdown();
    }
    if(_load_mass_<=0){
      ROS_ERROR("[Se3CopyController]: _load_mass_ <=0, use a value > 0!");
      ros::requestShutdown();
    }
    std::string BACA_IN_SIMULATION = getenv("BACA_IN_SIMULATION");
    if (BACA_IN_SIMULATION == "true" && _run_type_ == "simulation"){// "true" or "false" as string, then changed into a boolean. 
      _baca_in_simulation_ = true;
      ROS_INFO("[Se3CopyController]: Use Baca in simulation: true");
    }
    else{
      _baca_in_simulation_ = false;
      ROS_INFO("[Se3CopyController]: Use Baca in simulation: false");
    }
  }

  ROS_INFO("[Se3CopyController]: finished loading environment (session/bashrc) parameters");

  // --------------------------------------------------------------
  // |                     loading parameters                     |
  // --------------------------------------------------------------

  // | ---------- loading params using the parent's nh ---------- |

  mrs_lib::ParamLoader param_loader_parent(common_handlers->parent_nh, "ControlManager");

  param_loader_parent.loadParam("enable_profiler", _profiler_enabled_);

  if (!param_loader_parent.loadedSuccessfully()) {
    ROS_ERROR("[Se3CopyController]: Could not load all parameters!");
    return false;
  }

  else {
    ROS_INFO("[Se3CopyController]: Could DO load all parameters!");
  }

  // | ------------------- loading my params ------------------- |
  /* TODO: method below does not work as in https://github.com/ctu-mrs/mrs_uav_controllers/blob/master/src/se3_controller.cpp
     private_handlers->param_loader->addYamlFile(ros::package::getPath("controllers_brubotics") + "/config/private/se3_copy_controller.yaml");
     private_handlers->param_loader->addYamlFile(ros::package::getPath("controllers_brubotics") + "/config/public/se3_copy_controller.yaml");
     use method as in https://github.com/ctu-mrs/example_controller_plugin/blob/master/src/example_controller.cpp
  */
  bool success = true;
  // FYI
  // This method will load the file using `rosparam get`
  //   Pros: you can the full power of the official param loading
  //   Cons: it is slower
  //
  // Alternatives:
  //   You can load the file directly into the ParamLoader as shown below.
  success *= private_handlers->loadConfigFile(ros::package::getPath("controllers_brubotics") + "/config/private/se3_copy_controller.yaml");
  if (!success) {
    ROS_ERROR("[Se3CopyController]: Error in loading config file: controllers_brubotics/config/private/se3_copy_controller.yaml");
    return false;
  }
  ROS_INFO("[Se3CopyController]: Successfully loading config file: controllers_brubotics/config/private/se3_copy_controller.yaml");
  success *= private_handlers->loadConfigFile(ros::package::getPath("controllers_brubotics") + "/config/public/se3_copy_controller.yaml");
  if (!success) {
    ROS_ERROR("[Se3CopyController]: Error in loading config file: controllers_brubotics/config/public/se3_copy_controller.yaml");
    return false;
  }
  ROS_INFO("[Se3CopyController]: Successfully loading config file: controllers_brubotics/config/public/se3_copy_controller.yaml");
  // TODO: how is the link between the yaml file choice and below??? now we don't have launch file anymore

  mrs_lib::ParamLoader param_loader(nh_, "Se3CopyController");

  ROS_INFO("[Se3CopyController]: Loading Private Yaml File of the Controller using addYamlFile...");
  param_loader.addYamlFile(ros::package::getPath("controllers_brubotics") + "/config/private/se3_copy_controller.yaml");
  ROS_INFO("[Se3CopyController]: Successfully loaded Private Yaml File of the Controller using addYamlFile.");

  ROS_INFO("[Se3CopyController]: Loading Public Yaml File of the Controller using addYamlFile...");
  param_loader.addYamlFile(ros::package::getPath("controllers_brubotics") + "/config/public/se3_copy_controller.yaml");
  ROS_INFO("[Se3CopyController]: Successfully loaded Public Yaml File of the Controller using addYamlFile.");


  const std::string yaml_prefix = "controllers_brubotics/se3_copy_controller/";
  
  // TODO: understand which params to store in private and which in public
  // lateral gains
  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/kp", gains_.kpxy);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/kv", gains_.kvxy);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/ka", gains_.kaxy);

  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/kiw", gains_.kiwxy);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/kib", gains_.kibxy);

  // | ------------------------- rampup ------------------------- |

  param_loader.loadParam(yaml_prefix + "se3/rampup/enabled", _rampup_enabled_);
  param_loader.loadParam(yaml_prefix + "se3/rampup/speed", _rampup_speed_);

  // height gains
  param_loader.loadParam(yaml_prefix + "se3/default_gains/vertical/kp", gains_.kpz);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/vertical/kv", gains_.kvz);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/vertical/ka", gains_.kaz);

  // attitude gains
  param_loader.loadParam(yaml_prefix + "se3/default_gains/attitude/kq_roll_pitch", gains_.kq_roll_pitch);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/attitude/kq_yaw", gains_.kq_yaw);

  // attitude rate gains
  param_loader.loadParam(yaml_prefix + "se3/attitude_rate_gains/kw_roll_pitch", gains_.kw_roll_pitch);
  param_loader.loadParam(yaml_prefix + "se3/attitude_rate_gains/kw_yaw", gains_.kw_yaw);

  // mass estimator
  param_loader.loadParam(yaml_prefix + "se3/default_gains/mass_estimator/km", gains_.km);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/mass_estimator/km_lim", gains_.km_lim);

  // integrator limits
  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/kiw_lim", gains_.kiwxy_lim);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/kib_lim", gains_.kibxy_lim);

  // constraints
  param_loader.loadParam(yaml_prefix + "se3/constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  param_loader.loadParam(yaml_prefix + "se3/constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);

  _tilt_angle_failsafe_ = M_PI * (_tilt_angle_failsafe_ / 180.0);

  if (_tilt_angle_failsafe_enabled_ && fabs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[Se3CopyController]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low");
    return false;
  }

  param_loader.loadParam(yaml_prefix + "se3/constraints/throttle_saturation", _throttle_saturation_);

  // gain filtering
  param_loader.loadParam(yaml_prefix + "se3/gain_filtering/perc_change_rate", _gains_filter_change_rate_);
  param_loader.loadParam(yaml_prefix + "se3/gain_filtering/min_change_rate", _gains_filter_min_change_rate_);
  param_loader.loadParam(yaml_prefix + "se3/gain_filtering/rate", _gain_filtering_rate_);
  param_loader.loadParam(yaml_prefix + "se3/gain_filtering/gain_mute_coefficient", _gain_mute_coefficient_);

  // output mode
  param_loader.loadParam(yaml_prefix + "se3/preferred_output", drs_params_.preferred_output_mode);

  param_loader.loadParam(yaml_prefix + "se3/rotation_matrix", drs_params_.rotation_type);

  // angular rate feed forward
  param_loader.loadParam(yaml_prefix + "se3/angular_rate_feedforward/parasitic_pitch_roll",
                                            drs_params_.pitch_roll_heading_rate_compensation);
  param_loader.loadParam(yaml_prefix + "se3/angular_rate_feedforward/jerk", drs_params_.jerk_feedforward);

  // | ------------------- position pid params ------------------ |

  param_loader.loadParam(yaml_prefix + "position_controller/translation_gains/p", _pos_pid_p_);
  param_loader.loadParam(yaml_prefix + "position_controller/translation_gains/i", _pos_pid_i_);
  param_loader.loadParam(yaml_prefix + "position_controller/translation_gains/d", _pos_pid_d_);

  param_loader.loadParam(yaml_prefix + "position_controller/heading_gains/p", _hdg_pid_p_);
  param_loader.loadParam(yaml_prefix + "position_controller/heading_gains/i", _hdg_pid_i_);
  param_loader.loadParam(yaml_prefix + "position_controller/heading_gains/d", _hdg_pid_d_);

  //TODO further clean below params
  // lateral gains for load state feedback:
  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/kpl", gains_.kplxy);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/horizontal/kvl", gains_.kvlxy);
  // load gains vertical for load state feedback:
  param_loader.loadParam(yaml_prefix + "se3/default_gains/vertical/kpl", gains_.kplz);
  param_loader.loadParam(yaml_prefix + "se3/default_gains/vertical/kvl", gains_.kvlz);
  // payload:
  param_loader.loadParam(yaml_prefix + "payload/Epl_min", _Epl_min_);
  param_loader.loadParam(yaml_prefix + "payload/Epl_max/failsafe_enabled", _Epl_max_failsafe_enabled_);
  param_loader.loadParam(yaml_prefix + "payload/Epl_max/scaling", _Epl_max_scaling_);
  param_loader.loadParam(yaml_prefix + "payload/swing_angle/failsafe_enabled", max_swing_angle_failsafe_enabled_);  
  param_loader.loadParam(yaml_prefix + "payload/swing_angle/max", max_swing_angle_);
  
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/max_time_delay_eland", _max_time_delay_eland_);
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/ready_delay", ready_delay_); 
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/ros_time_difference/ros_time_difference_delay", ros_time_difference_delay_);
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/ros_time_difference/ros_time_difference_duration", ros_time_difference_duration_);
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/ros_time_difference/enabled", ros_time_difference_enabled_);
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/nimbro/emulate_nimbro", emulate_nimbro_);
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/nimbro/emulate_nimbro_delay", emulate_nimbro_delay_);
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/distance_uavs/failsafe_enabled", distance_uavs_failsafe_enabled_);
  param_loader.loadParam(yaml_prefix + "two_uavs_payload/distance_uavs/max_error", distance_uavs_max_error_);
  param_loader.loadParam(yaml_prefix + "ros_info_throttle_period", ROS_INFO_THROTTLE_PERIOD);
 
  // | ------------------ finish loading params ----------------- |
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Se3CopyController]: could not load all parameters!");
    return false;
  }
  else{
    ROS_INFO("[Se3CopyController]: correctly loaded all Se3CopyController parameters!");
  }

  // | ------------------- create publishers ------------------- |
  // TODO bryan: change below to correct msg types (e.g., do not use PoseArray for a thrust or angle)
  // UAV: always loaded
  uav_state_publisher_   = nh_.advertise<mrs_msgs::UavState>("uav_state",1); // TODO: seems not LOAD specific, why are both tracker and controller publishig on this topic??? Bryan: not issue as will be publish on different topic due to name of controller/tracker before topic name. So can be used to check if the same
  projected_thrust_publisher_ = nh_.advertise<std_msgs::Float64>("custom_projected_thrust",1);
  thrust_publisher_           = nh_.advertise<std_msgs::Float64>("custom_thrust",1);
  thrust_satlimit_physical_publisher_           = nh_.advertise<std_msgs::Float64>("thrust_satlimit_physical",1);
  thrust_satlimit_publisher_           = nh_.advertise<std_msgs::Float64>("thrust_satlimit",1);
  thrust_satval_publisher_           = nh_.advertise<std_msgs::Float64>("thrust_satval",1);
  hover_thrust_publisher_            = nh_.advertise<std_msgs::Float64>("hover_thrust",1);
  tilt_angle_publisher_            = nh_.advertise<std_msgs::Float64>("tilt_angle",1);
  // LOAD (1 & 2 UAVs):
  if(_type_of_system_=="1uav_payload" || _type_of_system_=="2uavs_payload"){
    load_pose_publisher_   = nh_.advertise<geometry_msgs::Pose>("load_pose",1);
    load_vel_publisher_   = nh_.advertise<geometry_msgs::Twist>("load_vel",1);
    load_position_errors_publisher_ = nh_.advertise<geometry_msgs::Pose>("load_position_errors",1);
    if((_run_type_ == "uav") || _baca_in_simulation_){
      encoder_angle_1_publisher_ = nh_.advertise<std_msgs::Float64>("encoder_angle_1",1); // theta'
      encoder_angle_2_publisher_ = nh_.advertise<std_msgs::Float64>("encoder_angle_2",1); // phi'
    }
  }

  // 2UAVs safety communication
  if(_type_of_system_=="2uavs_payload"){
    Eland_time_pub_ = nh_.advertise<std_msgs::Float64>("Eland_time",1);
    // ros_time_pub_ = nh_.advertise<std_msgs::Float64>("ros_time",1);
    if (_uav_name_ == _leader_uav_name_){  // leader
      Eland_controller_leader_to_follower_pub_ = nh_.advertise<mrs_msgs::BoolStamped>("Eland_contr_l_to_f",1);
      time_delay_Eland_controller_follower_to_leader_pub_ = nh_.advertise<std_msgs::Float64>("time_delay_Eland_controller_follower_to_leader",1);
      // ros_time_trigger_l_pub_ = nh_.advertise<std_msgs::Bool>("ros_time_trigger_l",1);
      ros_time_l_to_f_pub_ = nh_.advertise<std_msgs::Float64>("ros_time_l_to_f",1);

      distance_uavs_pub_ = nh_.advertise<std_msgs::Float64>("distance_uavs", 1);
    }
    else if(_uav_name_ == _follower_uav_name_){
      Eland_controller_follower_to_leader_pub_ = nh_.advertise<mrs_msgs::BoolStamped>("Eland_contr_f_to_l",1);
      time_delay_Eland_controller_leader_to_follower_pub_ = nh_.advertise<std_msgs::Float64>("time_delay_Eland_controller_leader_to_follower",1);
      ros_time_delay_pub_ = nh_.advertise<std_msgs::Float64>("ros_time_delay",1);
    }  
  }

  ROS_INFO("[Se3CopyController]: advertised all publishers.");

  // | ------------------- create subscribers ------------------- |
  if(_type_of_system_=="1uav_payload" || _type_of_system_=="2uavs_payload"){
    // this uav subscribes to own (i.e., of this uav) load states:
    if (_run_type_ == "simulation" && !_baca_in_simulation_ ){ // subscriber of the load model spawned in the gazebo simulation
      load_state_sub_ =  nh_.subscribe("/gazebo/link_states", 1, &Se3CopyController::GazeboLoadStatesCallback, this, ros::TransportHints().tcpNoDelay());
    }
    else if (_run_type_ == "uav" || (_baca_in_simulation_ && _run_type_ == "simulation") ){ // subscriber of the hardware encoders, if real test or if simulation-based validation of the bacaprotocol and FK of the encoder are done.
      std::string slash = "/";
      std::string _uav_name_copy_ = _uav_name_;
      // ROS_INFO_STREAM("[Se3CopyController]: uav_name_ = " << _uav_name_);
      data_payload_sub_ = nh_.subscribe(slash.append(_uav_name_copy_.append("/serial/received_message")), 1, &Se3CopyController::BacaLoadStatesCallback, this, ros::TransportHints().tcpNoDelay()); // TODO: explain how this is used for 2 uav hardware
      // ROS_INFO_STREAM("[Se3CopyController]: uav_name_ after subscribe BacaLoadStatesCallback = " << _uav_name_);
    }
    else{ // undefined
      ROS_ERROR("[Se3CopyController]: undefined _run_type_ used for uav with payload!");
      ros::requestShutdown(); // TODO or return false?
    }
  }

  // 2UAVs safety communication
  if(_type_of_system_=="2uavs_payload"){
    if (_uav_name_ == _leader_uav_name_){  // leader
      Eland_controller_follower_to_leader_sub_ = nh_.subscribe("/"+_follower_uav_name_+"/control_manager/se3_copy_controller/Eland_contr_f_to_l", 1, &Se3CopyController::ElandFollowerToLeaderCallback, this, ros::TransportHints().tcpNoDelay());
      uav_state_follower_for_leader_sub_ = nh2_.subscribe("/"+_follower_uav_name_+"/control_manager/dergbryan_tracker/uav_state_f_for_l", 1, &Se3CopyController::uav_state_follower_for_leader_callback, this, ros::TransportHints().tcpNoDelay());
    }
    else if(_uav_name_ == _follower_uav_name_){
      Eland_controller_leader_to_follower_sub_ = nh_.subscribe("/"+_leader_uav_name_+"/control_manager/se3_copy_controller/Eland_contr_l_to_f", 1, &Se3CopyController::ElandLeaderToFollowerCallback, this, ros::TransportHints().tcpNoDelay());
      // ros_time_trigger_l_sub_ = nh_.subscribe("/"+_leader_uav_name_+"/control_manager/se3_copy_controller/ros_time_trigger_l", 1, &Se3CopyController::rosTimeTriggerCallback, this, ros::TransportHints().tcpNoDelay());
      ros_time_l_to_f_sub_ = nh_.subscribe("/"+_leader_uav_name_+"/control_manager/se3_copy_controller/ros_time_l_to_f", 1, &Se3CopyController::rosTimeLeaderToFollowerCallback, this, ros::TransportHints().tcpNoDelay());
    }  
  }

  ROS_INFO("[Se3CopyController]: linked all subscribers to their callbacks.");

  // | ---------------- prepare stuff from params --------------- |
  
  if (!(drs_params_.preferred_output_mode == OUTPUT_ACTUATORS || drs_params_.preferred_output_mode == OUTPUT_CONTROL_GROUP ||
        drs_params_.preferred_output_mode == OUTPUT_ATTITUDE_RATE || drs_params_.preferred_output_mode == OUTPUT_ATTITUDE)) {
    ROS_ERROR("[Se3CopyController]: preferred output mode has to be {0, 1, 2, 3}!");
    return false;
  }

  // initialize the integrals
  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2); // world integral
  Ib_b_                = Eigen::Vector2d::Zero(2); // body integral

  // | --------------- dynamic reconfigure server --------------- |
  
  drs_params_.kpxy             = gains_.kpxy;
  drs_params_.kvxy             = gains_.kvxy;
  drs_params_.kaxy             = gains_.kaxy;
  drs_params_.kiwxy            = gains_.kiwxy;
  drs_params_.kibxy            = gains_.kibxy;
  drs_params_.kpz              = gains_.kpz;
  drs_params_.kvz              = gains_.kvz;
  drs_params_.kaz              = gains_.kaz;
  drs_params_.kq_roll_pitch    = gains_.kq_roll_pitch;
  drs_params_.kq_yaw           = gains_.kq_yaw;
  drs_params_.kiwxy_lim        = gains_.kiwxy_lim;
  drs_params_.kibxy_lim        = gains_.kibxy_lim;
  drs_params_.km               = gains_.km;
  drs_params_.km_lim           = gains_.km_lim;
  drs_params_.jerk_feedforward = true;
  // load gains
  // TODO: adding thesegives error, trace back definition of typedef controllers_brubotics::se3_copy_controllerConfig DrsConfig_t;
  // drs_params_.kplxy            = gains_.kplxy;
  // drs_params_.kvlxy            = gains_.kvlxy;
  // drs_params_.kplz             = gains_.kplz;
  // drs_params_.kvlz             = gains_.kvlz;
  // TOOD: need to add more load gains?

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&Se3CopyController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ------------------------- timers ------------------------- |

  timer_gains_ = nh_.createTimer(ros::Rate(_gain_filtering_rate_), &Se3CopyController::timerGains, this, false, false);

  // | ---------------------- position pid ---------------------- |

  position_pid_x_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_y_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_z_.setParams(_pos_pid_p_, _pos_pid_d_, _pos_pid_i_, -1, 1.0);
  position_pid_heading_.setParams(_hdg_pid_p_, _hdg_pid_d_, _hdg_pid_i_, -1, 0.1);
  // TOOD: need to add load gains here?

  // | ------------------------ profiler ------------------------ |

  profiler_ = mrs_lib::Profiler(common_handlers_->parent_nh, "Se3CopyController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[Se3CopyController]: initialized");

  is_initialized_ = true;

  return true;
}

/* //{ activate() */

bool Se3CopyController::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;

  double activation_mass = _uav_mass_;

  if (activation_control_output_.diagnostics.mass_estimator) {
    uav_mass_difference_ = activation_control_output_.diagnostics.mass_difference;
    activation_mass += uav_mass_difference_;
    ROS_INFO("[Se3CopyController]: setting mass difference from the last control output: %.2f kg", uav_mass_difference_);
  }

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  if (activation_control_output_.diagnostics.disturbance_estimator) {
    Ib_b_[0] = -activation_control_output_.diagnostics.disturbance_bx_b;
    Ib_b_[1] = -activation_control_output_.diagnostics.disturbance_by_b;

    Iw_w_[0] = -activation_control_output_.diagnostics.disturbance_wx_w;
    Iw_w_[1] = -activation_control_output_.diagnostics.disturbance_wy_w;

    ROS_INFO(
        "[Se3CopyController]: setting disturbances from the last control output: Ib_b_: %.2f, %.2f N, Iw_w_: "
        "%.2f, %.2f N",
        Ib_b_[0], Ib_b_[1], Iw_w_[0], Iw_w_[1]);
  }

  // did the last controller use manual throttle control?
  auto throttle_last_controller = common::extractThrottle(activation_control_output_);

  // rampup check
  if (_rampup_enabled_ && throttle_last_controller) {

    double hover_throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, activation_mass * common_handlers_->g);

    double throttle_difference = hover_throttle - throttle_last_controller.value();

    if (throttle_difference > 0) {
      rampup_direction_ = 1;
    } else if (throttle_difference < 0) {
      rampup_direction_ = -1;
    } else {
      rampup_direction_ = 0;
    }

    ROS_INFO("[Se3CopyController]: activating rampup with initial throttle: %.4f, target: %.4f", throttle_last_controller.value(), hover_throttle);

    rampup_active_     = true;
    rampup_start_time_ = ros::Time::now();
    rampup_last_time_  = ros::Time::now();
    rampup_throttle_   = throttle_last_controller.value();

    rampup_duration_ = fabs(throttle_difference) / _rampup_speed_;
  }

  first_iteration_ = true;
  mute_gains_      = true;

  timer_gains_.start();

  // | ------------------ finish the activation ----------------- |

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

  timer_gains_.stop();

  ROS_INFO("[Se3CopyController]: deactivated");
}

//}

/* updateInactive() //{ */

void Se3CopyController::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateWhenActive() */

Se3CopyController::ControlOutput Se3CopyController::updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  // ROS_INFO_THROTTLE(0.1,"[Se3CopyController]: Start of updateActive()");
  
  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("Se3CopyController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  try {
    uav_state_publisher_.publish(uav_state_);
  }
  catch (...) {
    ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", uav_state_publisher_.getTopic().c_str());
  }

  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};
  
  // added -----------------------

  Se3CopyController::ControlOutput empty_control_output; // TODO: check if this works like before mrs_msgs::AttitudeCommand::ConstPtr(); I got this idea from https://github.com/ctu-mrs/mrs_uav_controllers/blob/master/src/failsafe_controller.cpp where they give an empty command
  empty_control_output.diagnostics.controller = "Se3CopyController";
  
  // Emulates nimbro communication
  if(emulate_nimbro_){
    emulate_nimbro_time_ = emulate_nimbro_time_ + dt_;
    if(emulate_nimbro_time_ >= emulate_nimbro_delay_){
      emulate_nimbro_time_ = 0;
    }
  }

  // | ----------------- 2UAVs safety communication --------------|
  if(_type_of_system_ == "2uavs_payload"){
    if(is_active_ || !is_active_){ // Safety communication after activation
      SafetyCommunication();
      if(Eland_status_ && is_active_){
        ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Returning empty command to trigger Eland");
        return empty_control_output; 
      }
    }
    else{ // Before activation, communication to determine ros time difference
      if(ros_time_difference_enabled_){
        RosTimeDifference();
      }
    }
  }

  // | ------------------ payload safety check ------------------|
  if(_type_of_system_=="1uav_payload" || _type_of_system_=="2uavs_payload"){
    if(payload_spawned_ && !payload_once_spawned_){
      payload_once_spawned_ = true;
    }

    if(payload_once_spawned_){
      // ROS_INFO_STREAM("time baca"<< ros::Time::now().toSec()-time_last_payload_message);
      if(ros::Time::now().toSec()-time_last_payload_message>0.1){
        ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: time since last 'payload_swpawned_ = true' is bigger than 0.1 seconds. Problem with the BACA protocol => trigger eland");
        return empty_control_output;
      }
    }
  }

  // distance between both UAVs
  if(_type_of_system_=="2uavs_payload"){
    if(distance_uavs_failsafe_enabled_){
      if(both_uavs_ready_ && distance_uavs_ready_){
        Eigen::Vector3d pos_leader(uav_state_.pose.position.x, uav_state_.pose.position.y, uav_state_.pose.position.z);
        Eigen::Vector3d pos_follower(uav_state_follower_for_leader_.pose.position.x, uav_state_follower_for_leader_.pose.position.y, uav_state_follower_for_leader_.pose.position.z);
        // ROS_INFO_STREAM("[Se3CopyController]: position leader" <<pos_leader);
        // ROS_INFO_STREAM("[Se3CopyController]: position follower" <<pos_follower);
        distance_UAVs_out_.data = (pos_leader - pos_follower).norm();
        // ROS_INFO_STREAM("[Se3CopyController]: distance_uavs = " << distance_UAVs_out_.data);
        if(distance_UAVs_out_.data > _load_length_ + distance_uavs_max_error_){
          ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Distance between both UAvs %f > %f ==> Eland",distance_UAVs_out_.data,_load_length_ + distance_uavs_max_error_);
          return empty_control_output;
        }
        else if(distance_UAVs_out_.data < _load_length_ - distance_uavs_max_error_){
          ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Distance between both UAvs %f < %f ==> Eland",distance_UAVs_out_.data,_load_length_ - distance_uavs_max_error_);
          return empty_control_output;
        }   
      }  
      try {
        distance_uavs_pub_.publish(distance_UAVs_out_);
      }
      catch (...) {
        ROS_ERROR("[DergbryanTracker]: Exception caught during publishing topic %s.", distance_uavs_pub_.getTopic().c_str());
      } 
    }
  }

  
//tot hier ----------------------------


  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {

    ROS_INFO("[Se3CopyController]: first iteration");
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (fabs(dt) < 0.001) {

    ROS_DEBUG("[Se3CopyController]: the last odometry message came too close (%.2f s)!", dt);

    dt = 0.01;
  }

  // | ----------- obtain the lowest possible modality ---------- |

  auto lowest_modality = common::getLowestOuput(common_handlers_->control_output_modalities);

  if (!lowest_modality) {

    ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: output modalities are empty! This error should never appear.");

    return last_control_output_;
  }

  // | ----- we might prefer some output mode over the other ---- |

  if (drs_params.preferred_output_mode == OUTPUT_ATTITUDE_RATE && common_handlers_->control_output_modalities.attitude_rate) {
    ROS_DEBUG_THROTTLE(1.0, "[Se3CopyController]: prioritizing attitude rate output");
    lowest_modality = common::ATTITUDE_RATE;
  } else if (drs_params.preferred_output_mode == OUTPUT_ATTITUDE && common_handlers_->control_output_modalities.attitude) {
    ROS_DEBUG_THROTTLE(1.0, "[Se3CopyController]: prioritizing attitude output");
    lowest_modality = common::ATTITUDE;
  } else if (drs_params.preferred_output_mode == OUTPUT_CONTROL_GROUP && common_handlers_->control_output_modalities.control_group) {
    ROS_DEBUG_THROTTLE(1.0, "[Se3CopyController]: prioritizing control group output");
    lowest_modality = common::CONTROL_GROUP;
  } else if (drs_params.preferred_output_mode == OUTPUT_ACTUATORS && common_handlers_->control_output_modalities.actuators) {
    ROS_DEBUG_THROTTLE(1.0, "[Se3CopyController]: prioritizing actuators output");
    lowest_modality = common::ACTUATORS_CMD;
  }

  switch (lowest_modality.value()) {

    case common::POSITION: {
      positionPassthrough(uav_state, tracker_command);
      break;
    }

    case common::VELOCITY_HDG: {
      PIDVelocityOutput(uav_state, tracker_command, common::VELOCITY_HDG, dt);
      break;
    }

    case common::VELOCITY_HDG_RATE: {
      PIDVelocityOutput(uav_state, tracker_command, common::VELOCITY_HDG_RATE, dt);
      break;
    }

    case common::ACCELERATION_HDG: {
      SE3Controller(uav_state, tracker_command, dt, common::ACCELERATION_HDG);
      break;
    }

    case common::ACCELERATION_HDG_RATE: {
      SE3Controller(uav_state, tracker_command, dt, common::ACCELERATION_HDG_RATE);
      break;
    }

    case common::ATTITUDE: {
      SE3Controller(uav_state, tracker_command, dt, common::ATTITUDE);
      break;
    }

    case common::ATTITUDE_RATE: {
      SE3Controller(uav_state, tracker_command, dt, common::ATTITUDE_RATE);
      break;
    }

    case common::CONTROL_GROUP: {
      SE3Controller(uav_state, tracker_command, dt, common::CONTROL_GROUP);
      break;
    }

    case common::ACTUATORS_CMD: {
      SE3Controller(uav_state, tracker_command, dt, common::ACTUATORS_CMD);
      break;
    }

    default: {
    }
  }

  return last_control_output_;
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

void Se3CopyController::switchOdometrySource(const mrs_msgs::UavState& new_uav_state) {

  ROS_INFO("[Se3CopyController]: switching the odometry source");

  auto uav_state = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  // | ----- transform world disturabances to the new frame ----- |

  geometry_msgs::Vector3Stamped world_integrals;

  world_integrals.header.stamp    = ros::Time::now();
  world_integrals.header.frame_id = uav_state.header.frame_id;

  world_integrals.vector.x = Iw_w_[0];
  world_integrals.vector.y = Iw_w_[1];
  world_integrals.vector.z = 0;

  auto res = common_handlers_->transformer->transformSingle(world_integrals, new_uav_state.header.frame_id);

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

  ROS_INFO("[Se3CopyController]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                         controllers                        |
// --------------------------------------------------------------

/* SE3Controller() //{ */

void Se3CopyController::SE3Controller(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command, const double& dt,
                                  const common::CONTROL_OUTPUT& output_modality) {

  auto drs_params  = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);
  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
  auto gains       = mrs_lib::get_mutexed(mutex_gains_, gains_);

  // | ----------------- get the current heading ---------------- |

  double uav_heading = getHeadingSafely(uav_state, tracker_command);
  uav_heading_ = uav_heading_;

  // --------------------------------------------------------------
  // |          load the control reference and estimates          |
  // --------------------------------------------------------------

  // Rp - position reference in global frame
  // Rv - velocity reference in global frame
  // Ra - velocity reference in global frame

  Eigen::Vector3d Rp = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Rv = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d Ra = Eigen::Vector3d::Zero(3);

  if (tracker_command.use_position_vertical || tracker_command.use_position_horizontal) {

    if (tracker_command.use_position_horizontal) {
      Rp[0] = tracker_command.position.x;
      Rp[1] = tracker_command.position.y;
    } else {
      Rv[0] = 0;
      Rv[1] = 0;
    }

    if (tracker_command.use_position_vertical) {
      Rp[2] = tracker_command.position.z;
    } else {
      Rv[2] = 0;
    }
  }

  if (tracker_command.use_velocity_horizontal) {
    Rv[0] = tracker_command.velocity.x;
    Rv[1] = tracker_command.velocity.y;
  } else {
    Rv[0] = 0;
    Rv[1] = 0;
  }

  if (tracker_command.use_velocity_vertical) {
    Rv[2] = tracker_command.velocity.z;
  } else {
    Rv[2] = 0;
  }

  if (tracker_command.use_acceleration) {
    Ra << tracker_command.acceleration.x, tracker_command.acceleration.y, tracker_command.acceleration.z;
  } else {
    Ra << 0, 0, 0;
  }

  /* test streaming the references Rp, Rv, Ra when the uav is moving.*/
  // ROS_INFO_STREAM("Rp = \n" << Rp);
  // ROS_INFO_STREAM("Rv = \n" << Rv);
  // ROS_INFO_STREAM("Ra = \n" << Ra);

  // | ------ store the estimated values from the uav state ----- |

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);
  Eigen::Vector3d Ov(uav_state.velocity.linear.x, uav_state.velocity.linear.y, uav_state.velocity.linear.z);

  // R - current uav attitude
  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state.pose.orientation);

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state.velocity.angular.x, uav_state.velocity.angular.y, uav_state.velocity.angular.z);

  // payload anchoring point state:
  // Opl - position load in global frame
  // Ovl - velocity load in global frame
  
  // TODO: currently Opl and Ovl are only known once the payload has been spawned, otherwise inititlized on zero.
  Eigen::Vector3d Opl = anchoring_pt_pose_position_;
  Eigen::Vector3d Ovl = anchoring_pt_lin_vel_;

  // | -------------- calculate the control errors -------------- |

  // position control error
  Eigen::Vector3d Ep(0, 0, 0);

  if (tracker_command.use_position_horizontal || tracker_command.use_position_vertical) {
    Ep = Rp - Op;
  }

  // velocity control error
  Eigen::Vector3d Ev(0, 0, 0);

  if (tracker_command.use_velocity_horizontal || tracker_command.use_velocity_vertical ||
      tracker_command.use_position_vertical) {  // use_position_vertical = true, not a mistake, this provides dampening
    Ev = Rv - Ov;
  }

  /*VALIDATE: test streaming the errors.*/
  // ROS_INFO_STREAM("Ep = \n" << Ep);
  // ROS_INFO_STREAM("Ev = \n" << Ev);

  // | --------------------------LOAD--------------------------|
  // --------------------------------------------------------------
  // |          load the control reference and errors             |
  // --------------------------------------------------------------
  // TODO: Rpl and Rvl not defined, nor used. Should be the uav ref cable length down. Not used in actual error of Pandolfo.
  // Rpl - position reference load in global frame
  // Rvl - velocity reference load in global frame

  Eigen::Vector3d Epl = Eigen::Vector3d::Zero(3); // load position control error
  Eigen::Vector3d Evl = Eigen::Vector3d::Zero(3); // load velocity control error
  // ROS_INFO_STREAM("Usebaca \n" << _baca_in_simulation_);
  if (payload_spawned_){
    // load position control error
    if (tracker_command.use_position_horizontal || tracker_command.use_position_vertical) {
      Eigen::Vector3d e3(0.0, 0.0, 1.0);
      Epl = Op - _cable_length_*e3 - Opl; // relative to uav base frame poiting from the anchoring point to the stable equilibrium beneuth the uav, according to Pandolfo / thesis Raphael: Anchoring point realignment
      // 2021 student said this : Op - Opl is super unstable!! However, this is the way Pandolfo thesis explained it. And I think what they did (Rp - Opl) will always be more unstable, as for a very far references, the control actions of the error of the UAV and the one of the Payload will superpose and generate a huge Td, which can easilly saturates the actuators and creates instability. 
      // TODO: I thought Raphael only looks to the posiiton error in xy (ignoring z)? check this.
    }
    // load velocity control error
    if (tracker_command.use_velocity_horizontal || tracker_command.use_velocity_vertical ||
      tracker_command.use_position_vertical) {  // even when use_position_vertical to provide dampening
      Evl = Ov - Ovl; // speed relative to base frame
    }
    
    // Sanity + safety checks: 
    // ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Epl = %.02fm and Epl_max = %.02f", Epl.norm(),_Epl_max_scaling_*_cable_length_*sqrt(2));
    if (Epl.norm()> _Epl_max_scaling_*_cable_length_*sqrt(2)){ // Largest possible error when cable is oriented 90°.
      if(_run_type_!="uav"){
        ROS_ERROR("[Se3CopyController]: Control error of the anchoring point Epl was larger than expected (%.02fm> _cable_length_*sqrt(2)= %.02fm).", Epl.norm(), _Epl_max_scaling_*_cable_length_*sqrt(2));
      }
      else{
        ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Control error of the anchoring point Epl was larger than expected (%.02fm> _cable_length_*sqrt(2)= %.02fm).", Epl.norm(), _Epl_max_scaling_*_cable_length_*sqrt(2));
      }
      // Epl = Eigen::Vector3d::Zero(3);
      if (_Epl_max_failsafe_enabled_){
        return; // TODO: check if this still elands sa before we did mrs_msgs::AttitudeCommand::ConstPtr(); // trigger eland
      }
    }
    // Ignore small load position errors to deal with small, but non-zero load offsets in steady-state and prevent aggressive actions on small errors 
    if(Epl.norm() < _Epl_min_){ // When the payload is very close to equilibrium vertical position, the error is desactivated so the UAV doesn't try to compensate and let it damp naturally.
      ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Control error of the anchoring point Epl = %.02fm < _Epl_min_ = %.02fm, hence it has been set to zero", Epl.norm(), _Epl_min_);
      Epl = Eigen::Vector3d::Zero(3);
    }

    // Check max swing angle
    Eigen::Vector3d uav_position(uav_state.pose.position.x,uav_state.pose.position.y,uav_state.pose.position.z); //get a vector of the UAV position to ease the following computations.
    Eigen::Vector3d mu; //Unit vector indicating cable orientation.
    Eigen::Vector3d zB; //unit vector z_B of the UAV body frame
    mu = (uav_position-Opl).normalized();
    zB = R.col(2);
    double swing_angle = acos((mu.dot(zB))/(mu.norm()*zB.norm()));
    // ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: swing angle = %f ",swing_angle);
    if(swing_angle > max_swing_angle_){
      ROS_ERROR("[Se3CopyController]: Swing angle is larger than allowed (%.02f rad >  %.02f rad).", swing_angle, max_swing_angle_);
      if (max_swing_angle_failsafe_enabled_){
        return; // TODO: check if this still elands sa before we did mrs_msgs::AttitudeCommand::ConstPtr(); // trigger eland
      }
    }
  }

  // publish the load_position_errors
  // TODO: check every to add header.stamp info of uav_state for all published data. Don't use ros time now everywhere as not synced.
  load_position_errors.position.x=Epl[0];
  load_position_errors.position.y=Epl[1];
  load_position_errors.position.z=Epl[2];

  try {
    load_position_errors_publisher_.publish(load_position_errors);
  }
  catch (...) {
    ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", load_position_errors_publisher_.getTopic().c_str());
  }
  
  // | --------------------- load the gains --------------------- |

  mute_gains_by_tracker_ = tracker_command.disable_position_gains;

  Eigen::Vector3d Ka(0, 0, 0);
  Eigen::Array3d  Kp(0, 0, 0);
  Eigen::Array3d  Kv(0, 0, 0);
  Eigen::Array3d  Kq(0, 0, 0);
  Eigen::Array3d  Kw(0, 0, 0);

  {
    std::scoped_lock lock(mutex_gains_);

    if (tracker_command.use_position_horizontal) {
      Kp[0] = gains.kpxy;
      Kp[1] = gains.kpxy;
    } else {
      Kp[0] = 0;
      Kp[1] = 0;
    }

    if (tracker_command.use_position_vertical) {
      Kp[2] = gains.kpz;
    } else {
      Kp[2] = 0;
    }

    if (tracker_command.use_velocity_horizontal) {
      Kv[0] = gains.kvxy;
      Kv[1] = gains.kvxy;
    } else {
      Kv[0] = 0;
      Kv[1] = 0;
    }

    // special case: if want to control z-pos but not the velocity => at least provide z dampening, therefore kvz_
    if (tracker_command.use_velocity_vertical || tracker_command.use_position_vertical) {
      Kv[2] = gains.kvz;
    } else {
      Kv[2] = 0;
    }

    if (tracker_command.use_acceleration) {
      Ka << gains.kaxy, gains.kaxy, gains.kaz;
    } else {
      Ka << 0, 0, 0;
    }

    if (!tracker_command.use_attitude_rate) {
      Kq << gains.kq_roll_pitch, gains.kq_roll_pitch, gains.kq_yaw;
    }

    Kw[0] = gains.kw_roll_pitch;
    Kw[1] = gains.kw_roll_pitch;
    Kw[2] = gains.kw_yaw;
  }

  // Print to test if the gains are loaded correctly:
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
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kw_x = %f", Kw(0));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kw_y = %f", Kw(1));
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kw_z = %f", Kw(2));

  // | --------------------------LOAD--------------------------|
  // Load the gains for the Anchoring point realignment method of Pandolfo: 
  Eigen::Array3d  Kpl = Eigen::Array3d::Zero(3); 
  Eigen::Array3d  Kdl = Eigen::Array3d::Zero(3); 

  if (_type_of_system_ == "1uav_payload" ||_type_of_system_ == "2uavs_payload" ){ 
    if (tracker_command.use_position_horizontal) {
      Kpl[0] = gains_.kplxy;
      Kpl[1] = gains_.kplxy;
    } else {
      Kpl[0] = 0;
      Kpl[1] = 0;
    }

    if (tracker_command.use_position_vertical) {
      Kpl[2] = gains_.kplz;
    } else {
      Kpl[2] = 0;
    }

    if (tracker_command.use_velocity_horizontal) {
      Kdl[0] = gains_.kvlxy;
      Kdl[1] = gains_.kvlxy;
    } else {
      Kdl[0] = 0;
      Kdl[1] = 0;
    }

    if (tracker_command.use_velocity_vertical) {
      Kdl[2] = gains_.kvlz;
    } 
    else if (tracker_command.use_position_vertical) {  // special case: want to control z-pos but not the velocity => at least provide z dampening
      Kdl[2] = gains_.kvlz;
    } else {
      Kdl[2] = 0;
    }
  } 

// | ------------------------compute total_mass --------------------------------------|
  //ROS_INFO_STREAM("_uav_mass_ \n" << _uav_mass_ ); // _uav_mass_ is loaded in initialize function

  //Compute the total mass of the system (depending on if there is a payload being transported or not). Note that in the 2UAV case, _load_mass_ is half of the mass of the beam payload. Which means it is the feedforward mass each UAV will have to lift additionally. 
  double total_mass;
  if(_type_of_system_ == "1uav_payload" || _type_of_system_ == "2uavs_payload" ){
    if(payload_spawned_){ // for simulation, but also on the hardware, when the controller activates, the load mass is always already suspended by the uav. For harware, if encoders values are finite, payload_spawn will be true.
      total_mass = _uav_mass_ + _load_mass_ + uav_mass_difference_ ;
      //ROS_INFO_STREAM("payload spwaned" << std::endl << total_mass);
    }else{
      total_mass = _uav_mass_ + uav_mass_difference_ ;
      //ROS_INFO_STREAM("payload NOT spwaned" << std::endl << total_mass);
    }
  }else{ // no load case
    total_mass = _uav_mass_ + uav_mass_difference_;
  }

  // Scale the gains with the total estimated mass of the system
  Kp = Kp * total_mass;
  Kv = Kv * total_mass;
  Kpl = Kpl * total_mass;
  Kdl = Kdl * total_mass;

  // gains after being mutiplied with total_mass
  if(_run_type_ != "uav"){ // Only printed in simulation
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_x*m = %f", Kp(0));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_y*m = %f", Kp(1));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kp_z*m = %f", Kp(2));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_x*m = %f", Kv(0));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_y*m = %f", Kv(1));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kv_z*m = %f", Kv(2));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kpl_x*m = %f", Kpl(0));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kpl_y*m = %f", Kpl(1));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kpl_z*m = %f", Kpl(2));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kdl_x*m = %f", Kdl(0));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kdl_y*m = %f", Kdl(1));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kdl_z*m = %f", Kdl(2));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kq_x = %f", Kq(0));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kq_y = %f", Kq(1));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kq_z = %f", Kq(2));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kw_x = %f", Kw(0));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kw_y = %f", Kw(1));
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: Kw_z = %f", Kw(2));

    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: _uav_mass_ = %f", _uav_mass_);
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: uav_mass_difference_ (estimated)= %f", uav_mass_difference_);
    if(_type_of_system_ == "1uav_payload" || _type_of_system_ == "2uavs_payload" ){
      ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: _load_mass_ = %f", _load_mass_);
    }
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: total_mass (estimated)= %f", total_mass);
    
    // TODO: replace below with new syntax of throttle_model
    // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: n_motors = %d", common_handlers_->motor_params.n_motors);
    // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: motor_params.A = %f", common_handlers_->motor_params.A);
    // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: motor_params.B = %f", common_handlers_->motor_params.B);
  }

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

    auto res = common_handlers_->transformer->transformSingle(Ib_b_stamped, uav_state_.header.frame_id);

    if (res) {
      Ib_w[0] = res.value().vector.x;
      Ib_w[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(1.0, "[Se3CopyController]: could not transform the Ib_b_ to the world frame");
    }
  }

  // construct the desired force vector
  
  // Compute control actions 
  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, common_handlers_->g) + Ra);
  Eigen::Vector3d position_feedback = Kp * Ep.array();
  Eigen::Vector3d velocity_feedback = Kv * Ev.array();
    
  // Load errors were initialized to zero if load not spawned
  Eigen::Vector3d load_position_feedback = -Kpl * Epl.array(); // pushes uav in opposed direction of the position error from anchoring point to load at equilibrium
  Eigen::Vector3d load_velocity_feedback = -Kdl * Evl.array(); // pushes uav in opposed direction of the velocity error from anchoring point to load at equilibrium

  Eigen::Vector3d integral_feedback;
  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w_[0], Ib_w[1] + Iw_w_[1], 0;
  }

  // --------------------------------------------------------------
  // |                 integrators and estimators                 |
  // --------------------------------------------------------------

  /* world error integrator //{ */

  // --------------------------------------------------------------
  // |                  integrate the world error                 |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_, mutex_integrals_);

    Eigen::Vector3d integration_switch(1, 1, 0);

    // integrate the world error
    if (tracker_command.use_position_horizontal) {
      Iw_w_ += gains.kiwxy * Ep.head(2) * dt;
    } else if (tracker_command.use_velocity_horizontal) {
      Iw_w_ += gains.kiwxy * Ev.head(2) * dt;
    }

    // saturate the world X
    bool world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[0])) {
      Iw_w_[0] = 0;
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'Iw_w_[0]', setting it to 0!!!");
    } else if (Iw_w_[0] > gains.kiwxy_lim) {
      Iw_w_[0]                 = gains.kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w_[0] < -gains.kiwxy_lim) {
      Iw_w_[0]                 = -gains.kiwxy_lim;
      world_integral_saturated = true;
    }

    if (gains.kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: SE3's world X integral is being saturated!");
    }

    // saturate the world Y
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[1])) {
      Iw_w_[1] = 0;
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'Iw_w_[1]', setting it to 0!!!");
    } else if (Iw_w_[1] > gains.kiwxy_lim) {
      Iw_w_[1]                 = gains.kiwxy_lim;
      world_integral_saturated = true;
    } else if (Iw_w_[1] < -gains.kiwxy_lim) {
      Iw_w_[1]                 = -gains.kiwxy_lim;
      world_integral_saturated = true;
    }

    if (gains.kiwxy_lim >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: SE3's world Y integral is being saturated!");
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

      auto res = common_handlers_->transformer->transformSingle(Ep_stamped, "fcu_untilted");

      if (res) {
        Ep_fcu_untilted[0] = res.value().vector.x;
        Ep_fcu_untilted[1] = res.value().vector.y;
      } else {
        ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: could not transform the position error to fcu_untilted");
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

      auto res = common_handlers_->transformer->transformSingle(Ev_stamped, "fcu_untilted");

      if (res) {
        Ev_fcu_untilted[0] = res.value().vector.x;
        Ev_fcu_untilted[1] = res.value().vector.x;
      } else {
        ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: could not transform the velocity error to fcu_untilted");
      }
    }

    // integrate the body error
    if (tracker_command.use_position_horizontal) {
      Ib_b_ += gains.kibxy * Ep_fcu_untilted * dt;
    } else if (tracker_command.use_velocity_horizontal) {
      Ib_b_ += gains.kibxy * Ev_fcu_untilted * dt;
    }

    // saturate the body
    bool body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[0])) {
      Ib_b_[0] = 0;
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'Ib_b_[0]', setting it to 0!!!");
    } else if (Ib_b_[0] > gains.kibxy_lim) {
      Ib_b_[0]                = gains.kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b_[0] < -gains.kibxy_lim) {
      Ib_b_[0]                = -gains.kibxy_lim;
      body_integral_saturated = true;
    }

    if (gains.kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: SE3's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[1])) {
      Ib_b_[1] = 0;
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'Ib_b_[1]', setting it to 0!!!");
    } else if (Ib_b_[1] > gains.kibxy_lim) {
      Ib_b_[1]                = gains.kibxy_lim;
      body_integral_saturated = true;
    } else if (Ib_b_[1] < -gains.kibxy_lim) {
      Ib_b_[1]                = -gains.kibxy_lim;
      body_integral_saturated = true;
    }

    if (gains.kibxy_lim > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: SE3's body roll integral is being saturated!");
    }
  }

  //}

  if (output_modality == common::ACCELERATION_HDG || output_modality == common::ACCELERATION_HDG_RATE) {

    Eigen::Vector3d des_acc = (position_feedback + velocity_feedback + integral_feedback) / total_mass + Ra;

    if (output_modality == common::ACCELERATION_HDG) {

      mrs_msgs::HwApiAccelerationHdgCmd cmd;

      cmd.acceleration.x = des_acc[0];
      cmd.acceleration.y = des_acc[1];
      cmd.acceleration.z = des_acc[2];

      cmd.heading = tracker_command.heading;

      last_control_output_.control_output = cmd;

    } else {

      double des_hdg_ff = 0;

      if (tracker_command.use_heading_rate) {
        des_hdg_ff = tracker_command.heading_rate;
      }

      mrs_msgs::HwApiAccelerationHdgRateCmd cmd;

      cmd.acceleration.x = des_acc[0];
      cmd.acceleration.y = des_acc[1];
      cmd.acceleration.z = des_acc[2];

      position_pid_heading_.setSaturation(constraints.heading_speed);

      double hdg_err = mrs_lib::geometry::sradians::diff(tracker_command.heading, uav_heading);

      double des_hdg_rate = position_pid_heading_.update(hdg_err, dt) + des_hdg_ff;

      cmd.heading_rate = des_hdg_rate;

      last_control_output_.desired_heading_rate = des_hdg_rate;

      last_control_output_.control_output = cmd;
    }

    // | -------------- unbiased desired acceleration ------------- |

    Eigen::Vector3d unbiased_des_acc(0, 0, 0);

    {

      Eigen::Vector3d unbiased_des_acc_world = (position_feedback + velocity_feedback) / total_mass + Ra;

      geometry_msgs::Vector3Stamped world_accel;

      world_accel.header.stamp    = ros::Time::now();
      world_accel.header.frame_id = uav_state.header.frame_id;
      world_accel.vector.x        = unbiased_des_acc_world[0];
      world_accel.vector.y        = unbiased_des_acc_world[1];
      world_accel.vector.z        = unbiased_des_acc_world[2];

      auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

      if (res) {
        unbiased_des_acc << res.value().vector.x, res.value().vector.y, res.value().vector.z;
      }
    }

    // fill the unbiased desired accelerations
    last_control_output_.desired_unbiased_acceleration = unbiased_des_acc;

    // | ----------------- fill in the diagnostics ---------------- |

    last_control_output_.diagnostics.ramping_up = false;

    last_control_output_.diagnostics.mass_estimator  = false;
    last_control_output_.diagnostics.mass_difference = 0;
    last_control_output_.diagnostics.total_mass      = total_mass;

    last_control_output_.diagnostics.disturbance_estimator = true;

    last_control_output_.diagnostics.disturbance_bx_b = -Ib_b_[0];
    last_control_output_.diagnostics.disturbance_by_b = -Ib_b_[1];

    last_control_output_.diagnostics.disturbance_bx_w = -Ib_w[0];
    last_control_output_.diagnostics.disturbance_by_w = -Ib_w[1];

    last_control_output_.diagnostics.disturbance_wx_w = -Iw_w_[0];
    last_control_output_.diagnostics.disturbance_wy_w = -Iw_w_[1];

    last_control_output_.diagnostics.controller_enforcing_constraints = false;

    last_control_output_.diagnostics.controller = "Se3CopyController";

    return;
  }

  /* mass estimatior //{ */

  // --------------------------------------------------------------
  // |                integrate the mass difference               |
  // --------------------------------------------------------------

  {
    std::scoped_lock lock(mutex_gains_);

    if (tracker_command.use_position_vertical && !rampup_active_) {
      uav_mass_difference_ += gains.km * Ep[2] * dt;
    }

    // saturate the mass estimator
    bool uav_mass_saturated = false;
    if (!std::isfinite(uav_mass_difference_)) {
      uav_mass_difference_ = 0;
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > gains.km_lim) {
      uav_mass_difference_ = gains.km_lim;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -gains.km_lim) {
      uav_mass_difference_ = -gains.km_lim;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  //}

  // | -------------------------------se3+load------------------------ | 
  Eigen::Vector3d f = position_feedback + velocity_feedback + load_position_feedback + load_velocity_feedback + integral_feedback + feed_forward;
  ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: fx= %.2f, fy= %.2f, fz=%.2f ",f[0],f[1],f[2]);

  // | ----------- limiting the downwards acceleration ---------- |
  // the downwards force produced by the position and the acceleration feedback should not be larger than the gravity
  // if the downwards part of the force is close to counter-act the gravity acceleration
  if (f[2] < 0) {

    ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", f[2]);

    f << 0, 0, 1;
    // TODO: check with MPC guidage code if this actually makes sense as i remember to have improved it
    // f << f[0], f[1], 0.0; // saturate the z-component on zero such that the desired tilt angle stays in the upper hemisphere
  }

  // | ------------------- sanitize tilt angle ------------------ |

  double tilt_safety_limit = _tilt_angle_failsafe_enabled_ ? _tilt_angle_failsafe_ : std::numeric_limits<double>::max();
  // TODO: solve issue with outputting a struct
  auto output = common::sanitizeDesiredForce(f.normalized(), tilt_safety_limit, constraints.tilt, "Se3CopyController");
  Eigen::Vector3d f_normed_sanitized = output.value().force;
  double tilt = output.value().tilt; // TODO: this is the desired tilt angle, also compute the actual tilt angle
  
  //double tilt = 0.0; // TODO: remove when solved
  //auto f_normed_sanitized = common::sanitizeDesiredForce(f.normalized(), tilt_safety_limit, constraints.tilt, "Se3Controller");

  // TODO: streamline theta using simple double and Float64 if needed. See ComputeSe3Controller.
  std_msgs::Float64 theta_msg;
  theta_msg.data = tilt;
  // publish the tilt angle
  tilt_angle_publisher_.publish(theta_msg); 

  // if (!f_normed_sanitized) {
  if (!output) {
    ROS_INFO("[Se3CopyController]: position feedback: [%.2f, %.2f, %.2f]", position_feedback[0], position_feedback[1], position_feedback[2]);
    ROS_INFO("[Se3CopyController]: velocity feedback: [%.2f, %.2f, %.2f]", velocity_feedback[0], velocity_feedback[1], velocity_feedback[2]);
    ROS_INFO("[Se3CopyController]: integral feedback: [%.2f, %.2f, %.2f]", integral_feedback[0], integral_feedback[1], integral_feedback[2]);
    ROS_INFO("[Se3CopyController]: load position feedback: [%.2f, %.2f, %.2f]", load_position_feedback[0], load_position_feedback[1], load_position_feedback[2]);
    ROS_INFO("[Se3CopyController]: load velocity feedback: [%.2f, %.2f, %.2f]", load_velocity_feedback[0], load_velocity_feedback[1], load_velocity_feedback[2]);
    ROS_INFO("[Se3CopyController]: tracker_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", tracker_command.position.x, tracker_command.position.y,
             tracker_command.position.z, tracker_command.heading);
    ROS_INFO("[Se3CopyController]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", uav_state.pose.position.x, uav_state.pose.position.y,
             uav_state.pose.position.z, uav_heading);

    return;
  }

  Eigen::Vector3d f_normed = f_normed_sanitized;//f_normed_sanitized.value();

  // --------------------------------------------------------------
  // |               desired orientation + throttle               |
  // --------------------------------------------------------------

  // | ------------------- desired orientation ------------------ |

  Eigen::Matrix3d Rd;

  if (tracker_command.use_orientation) {

    // fill in the desired orientation based on the desired orientation from the control command
    Rd = mrs_lib::AttitudeConverter(tracker_command.orientation);

    if (tracker_command.use_heading) {
      try {
        Rd = mrs_lib::AttitudeConverter(Rd).setHeading(tracker_command.heading);
      }
      catch (...) {
        if(_run_type_!="uav"){
          ROS_ERROR("[Se3CopyController]: could not set the desired heading");
        }
        else{
          ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: could not set the desired heading");
        }
      }
    }

  } else {

    Eigen::Vector3d bxd;  // desired heading vector

    if (tracker_command.use_heading) {
      bxd << cos(tracker_command.heading), sin(tracker_command.heading), 0;
    } else {
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: desired heading was not specified, using current heading instead!");
      bxd << cos(uav_heading), sin(uav_heading), 0;
    }

    Rd = common::so3transform(f_normed, bxd, drs_params.rotation_type == 1);
  }

  // | -------------------- desired throttle -------------------- |

  double desired_thrust_force = f.dot(R.col(2));
  std_msgs::Float64 thrust_force_msg;
  thrust_force_msg.data = desired_thrust_force;
  // custom publisher
  projected_thrust_publisher_.publish(thrust_force_msg);
  std_msgs::Float64 thrust_norm_msg;
  thrust_norm_msg.data = sqrt(f(0)*f(0)+f(1)*f(1)+f(2)*f(2)); // norm of f ( not projected on the z axis of the UAV frame)
  thrust_publisher_.publish(thrust_norm_msg);
  // TODO: print below 
  // print _motor_params_.A and _motor_params_.B
  // ROS_INFO_STREAM("_motor_params_.A = \n" << _motor_params_.A);
  // ROS_INFO_STREAM("_motor_params_.B = \n" << _motor_params_.B);
  // ROS_INFO_STREAM("_throttle_saturation_ = \n" << _throttle_saturation_);
  // OLD double thrust_saturation_physical = pow((_throttle_saturation_-_motor_params_.B)/_motor_params_.A, 2);
  std_msgs::Float64 thrust_saturation_physical_msg;
  thrust_saturation_physical_msg.data = mrs_lib::quadratic_throttle_model::throttleToForce(common_handlers_->throttle_model, _throttle_saturation_);
  ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: thrust_saturation_physical (N) = %f", thrust_saturation_physical_msg.data);
  thrust_satlimit_physical_publisher_.publish(thrust_saturation_physical_msg);

  std_msgs::Float64 hover_thrust_msg;
  hover_thrust_msg.data = total_mass*common_handlers_->g;
  hover_thrust_publisher_.publish(hover_thrust_msg);
  
  std_msgs::Float64 throttle_saturation_msg; // TODO: make tracker and controller same in how type of _throttle_saturation_ is chosen
  throttle_saturation_msg.data = _throttle_saturation_;
  thrust_satlimit_publisher_.publish(throttle_saturation_msg);
  
  double throttle             = 0;

  if (tracker_command.use_throttle) {

    // the throttle is overriden from the tracker command
    throttle = tracker_command.throttle;

  } else if (rampup_active_) {

    // deactivate the rampup when the times up
    if (fabs((ros::Time::now() - rampup_start_time_).toSec()) >= rampup_duration_) {

      rampup_active_ = false;

      ROS_INFO("[Se3CopyController]: rampup finished");

    } else {

      double rampup_dt = (ros::Time::now() - rampup_last_time_).toSec();

      rampup_throttle_ += double(rampup_direction_) * _rampup_speed_ * rampup_dt;

      rampup_last_time_ = ros::Time::now();

      throttle = rampup_throttle_;

      ROS_INFO_THROTTLE(0.1, "[Se3CopyController]: ramping up throttle, %.4f", throttle);
    }

  } else {

    if (desired_thrust_force >= 0) {
      throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, desired_thrust_force);
    } else {
      ROS_WARN_THROTTLE(1.0, "[Se3CopyController]: just so you know, the desired throttle force is negative (%.2f)", desired_thrust_force);
    }
  }

  // | ------------------- throttle saturation ------------------ |

  bool throttle_saturated = false;

  if (!std::isfinite(throttle)) {

    ROS_ERROR("[Se3CopyController]: NaN detected in variable 'throttle'!!!");
    // TODO: check old code commented:
    // thrust = 0;
    // if(_run_type_!="uav"){
    //   ROS_ERROR("[Se3CopyController]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");
    // }
    // else{
    //   ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");
    // }
    return;

  } else if (throttle > _throttle_saturation_) {
    throttle = _throttle_saturation_;
    ROS_WARN_THROTTLE(0.1*ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: saturating throttle to %.2f", _throttle_saturation_);
  } else if (throttle < 0.0) {
    throttle = 0.0;
    ROS_WARN_THROTTLE(0.1*ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: saturating throttle to 0.0");
  }

  // TODO: this if seems false all the time, change throttle_saturated in the above else ifs 
  if (throttle_saturated) {
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: ---------------------------");
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: desired state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command.position.x, tracker_command.position.y,
                      tracker_command.position.z, tracker_command.heading);
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: desired state: vel [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command.velocity.x, tracker_command.velocity.y,
                      tracker_command.velocity.z, tracker_command.heading_rate);
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: desired state: acc [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command.acceleration.x,
                      tracker_command.acceleration.y, tracker_command.acceleration.z, tracker_command.heading_acceleration);
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: desired state: jerk [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", tracker_command.jerk.x, tracker_command.jerk.y,
                      tracker_command.jerk.z, tracker_command.heading_jerk);
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: ---------------------------");
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: current state: pos [x: %.2f, y: %.2f, z: %.2f, hdg: %.2f]", uav_state.pose.position.x, uav_state.pose.position.y,
                      uav_state.pose.position.z, uav_heading);
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: current state: vel [x: %.2f, y: %.2f, z: %.2f, yaw rate: %.2f]", uav_state.velocity.linear.x,
                      uav_state.velocity.linear.y, uav_state.velocity.linear.z, uav_state.velocity.angular.z);
    ROS_WARN_THROTTLE(0.1, "[Se3CopyController]: ---------------------------");
  }

  // custom publisher
  std_msgs::Float64 thrust_physical_saturated_msg;
  thrust_physical_saturated_msg.data = mrs_lib::quadratic_throttle_model::throttleToForce(common_handlers_->throttle_model, throttle);
  //ROS_INFO_STREAM("thrust_physical_saturated_msg = \n" << thrust_physical_saturated_msg);
  thrust_satval_publisher_.publish(thrust_physical_saturated_msg);

  // | -------------- unbiased desired acceleration ------------- |

  Eigen::Vector3d unbiased_des_acc(0, 0, 0);

  {
    Eigen::Vector3d unbiased_des_acc_world = (position_feedback + velocity_feedback) / total_mass + Ra;

    geometry_msgs::Vector3Stamped world_accel;

    world_accel.header.stamp    = ros::Time::now();
    world_accel.header.frame_id = uav_state.header.frame_id;
    world_accel.vector.x        = unbiased_des_acc_world[0];
    world_accel.vector.y        = unbiased_des_acc_world[1];
    world_accel.vector.z        = unbiased_des_acc_world[2];

    auto res = common_handlers_->transformer->transformSingle(world_accel, "fcu");

    if (res) {
      unbiased_des_acc << res.value().vector.x, res.value().vector.y, res.value().vector.z;
    }
  }

  // | --------------- fill the resulting command --------------- |

  // fill the desired orientation for the tilt error check
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(Rd);

  // fill the unbiased desired accelerations
  last_control_output_.desired_unbiased_acceleration = unbiased_des_acc;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.ramping_up = rampup_active_;

  last_control_output_.diagnostics.mass_estimator  = true;
  last_control_output_.diagnostics.mass_difference = uav_mass_difference_;
  last_control_output_.diagnostics.total_mass      = total_mass;

  last_control_output_.diagnostics.disturbance_estimator = true;

  last_control_output_.diagnostics.disturbance_bx_b = -Ib_b_[0];
  last_control_output_.diagnostics.disturbance_by_b = -Ib_b_[1];

  last_control_output_.diagnostics.disturbance_bx_w = -Ib_w[0];
  last_control_output_.diagnostics.disturbance_by_w = -Ib_w[1];

  last_control_output_.diagnostics.disturbance_wx_w = -Iw_w_[0];
  last_control_output_.diagnostics.disturbance_wy_w = -Iw_w_[1];

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  last_control_output_.diagnostics.controller = "Se3CopyController";

  // | ------------ construct the attitude reference ------------ |

  mrs_msgs::HwApiAttitudeCmd attitude_cmd;

  attitude_cmd.stamp       = ros::Time::now();
  attitude_cmd.orientation = mrs_lib::AttitudeConverter(Rd);
  attitude_cmd.throttle    = throttle;

  if (output_modality == common::ATTITUDE) {

    last_control_output_.control_output = attitude_cmd;

    return;
  }

  // --------------------------------------------------------------
  // |                      attitude control                      |
  // --------------------------------------------------------------

  Eigen::Vector3d rate_feedforward = Eigen::Vector3d::Zero(3);

  if (tracker_command.use_attitude_rate) {

    rate_feedforward << tracker_command.attitude_rate.x, tracker_command.attitude_rate.y, tracker_command.attitude_rate.z;

  } else if (tracker_command.use_heading_rate) {

    // to fill in the feed forward yaw rate
    double desired_yaw_rate = 0;

    try {
      desired_yaw_rate = mrs_lib::AttitudeConverter(Rd).getYawRateIntrinsic(tracker_command.heading_rate);
    }
    catch (...) { 
      if(_run_type_!="uav"){
        ROS_ERROR("[Se3CopyController]: exception caught while calculating the desired_yaw_rate feedforward");
      }
      else{
        ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: exception caught while calculating the desired_yaw_rate feedforward");
      }
    }

    rate_feedforward << 0, 0, desired_yaw_rate;
  }

  // | ------------ jerk feedforward -> angular rate ------------ |

  Eigen::Vector3d jerk_feedforward = Eigen::Vector3d(0, 0, 0);

  if (tracker_command.use_jerk && drs_params.jerk_feedforward) {

    ROS_DEBUG_THROTTLE(1.0, "[Se3CopyController]: using jerk feedforward");

    Eigen::Matrix3d I;
    I << 0, 1, 0, -1, 0, 0, 0, 0, 0;
    Eigen::Vector3d desired_jerk = Eigen::Vector3d(tracker_command.jerk.x, tracker_command.jerk.y, tracker_command.jerk.z);
    jerk_feedforward             = (I.transpose() * Rd.transpose() * desired_jerk) / (desired_thrust_force / total_mass);
  }

  // | --------------- run the attitude controller -------------- |

  Eigen::Vector3d attitude_rate_saturation(constraints.roll_rate, constraints.pitch_rate, constraints.yaw_rate);
  ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: constraints.roll_rate = %f", constraints.roll_rate);
  ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: constraints.pitch_rate = %f", constraints.pitch_rate);
  ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: constraints.yaw_rate = %f", constraints.yaw_rate);


  auto attitude_rate_command = common::attitudeController(uav_state, attitude_cmd, jerk_feedforward + rate_feedforward, attitude_rate_saturation, Kq,
                                                          drs_params.pitch_roll_heading_rate_compensation);

  if (!attitude_rate_command) {
    return;
  }

  // | --------- fill in the already known attitude rate -------- |

  {
    try {
      last_control_output_.desired_heading_rate = mrs_lib::AttitudeConverter(R).getHeadingRate(attitude_rate_command->body_rate);
    }
    catch (...) {
    }
  }

  // | ---------- construct the attitude rate reference --------- |

  if (output_modality == common::ATTITUDE_RATE) {

    last_control_output_.control_output = attitude_rate_command;

    return;
  }

  // --------------------------------------------------------------
  // |                    Attitude rate control                   |
  // --------------------------------------------------------------

  Kw = common_handlers_->detailed_model_params->inertia.diagonal().array() * Kw;

  auto control_group_command = common::attitudeRateController(uav_state, attitude_rate_command.value(), Kw);

  if (!control_group_command) {
    return;
  }

  if (output_modality == common::CONTROL_GROUP) {

    last_control_output_.control_output = control_group_command;

    return;
  }

  // --------------------------------------------------------------
  // |                        output mixer                        |
  // --------------------------------------------------------------

  mrs_msgs::HwApiActuatorCmd actuator_cmd = common::actuatorMixer(control_group_command.value(), common_handlers_->detailed_model_params->control_group_mixer);

  last_control_output_.control_output = actuator_cmd;

  return;
}

//}

/* positionPassthrough() //{ */

void Se3CopyController::positionPassthrough(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  if (!tracker_command.use_position_vertical || !tracker_command.use_position_horizontal || !tracker_command.use_heading) {
    ROS_ERROR("[Se3CopyController]: the tracker did not provide position+hdg reference");
    return;
  }

  mrs_msgs::HwApiPositionCmd cmd;

  cmd.header.frame_id = uav_state.header.frame_id;
  cmd.header.stamp    = ros::Time::now();

  cmd.position = tracker_command.position;
  cmd.heading  = tracker_command.heading;

  last_control_output_.control_output = cmd;

  // fill the unbiased desired accelerations
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_heading_rate          = {};

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.ramping_up = false;

  last_control_output_.diagnostics.mass_estimator  = false;
  last_control_output_.diagnostics.mass_difference = 0;

  last_control_output_.diagnostics.disturbance_estimator = false;

  last_control_output_.diagnostics.disturbance_bx_b = 0;
  last_control_output_.diagnostics.disturbance_by_b = 0;

  last_control_output_.diagnostics.disturbance_bx_w = 0;
  last_control_output_.diagnostics.disturbance_by_w = 0;

  last_control_output_.diagnostics.disturbance_wx_w = 0;
  last_control_output_.diagnostics.disturbance_wy_w = 0;

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  last_control_output_.diagnostics.controller = "Se3CopyController";
}

//}

/* PIDVelocityOutput() //{ */

void Se3CopyController::PIDVelocityOutput(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command,
                                      const common::CONTROL_OUTPUT& control_output, const double& dt) {

  if (!tracker_command.use_position_vertical || !tracker_command.use_position_horizontal || !tracker_command.use_heading) {
    ROS_ERROR("[Se3CopyController]: the tracker did not provide position+hdg reference");
    return;
  }

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
  auto gains       = mrs_lib::get_mutexed(mutex_gains_, gains_);

  Eigen::Vector3d pos_ref = Eigen::Vector3d(tracker_command.position.x, tracker_command.position.y, tracker_command.position.z);
  Eigen::Vector3d pos     = Eigen::Vector3d(uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z);

  double hdg_ref = tracker_command.heading;
  double hdg     = getHeadingSafely(uav_state, tracker_command);

  // | ------------------ velocity feedforward ------------------ |

  Eigen::Vector3d vel_ff(0, 0, 0);

  if (tracker_command.use_velocity_horizontal && tracker_command.use_velocity_vertical) {
    vel_ff = Eigen::Vector3d(tracker_command.velocity.x, tracker_command.velocity.y, tracker_command.velocity.z);
  }

  // | -------------------------- gains ------------------------- |

  Eigen::Vector3d Kp;

  {
    std::scoped_lock lock(mutex_gains_);

    Kp << gains.kpxy, gains.kpxy, gains.kpz;
  }

  // | --------------------- control errors --------------------- |

  Eigen::Vector3d Ep = pos_ref - pos;

  // | --------------------------- pid -------------------------- |

  position_pid_x_.setSaturation(constraints.horizontal_speed);
  position_pid_y_.setSaturation(constraints.horizontal_speed);
  position_pid_z_.setSaturation(std::min(constraints.vertical_ascending_speed, constraints.vertical_descending_speed));

  double des_vel_x = position_pid_x_.update(Ep[0], dt);
  double des_vel_y = position_pid_y_.update(Ep[1], dt);
  double des_vel_z = position_pid_z_.update(Ep[2], dt);

  // | -------------------- position feedback ------------------- |

  Eigen::Vector3d des_vel = Eigen::Vector3d(des_vel_x, des_vel_y, des_vel_z) + vel_ff;

  if (control_output == common::VELOCITY_HDG) {

    // | --------------------- fill the output -------------------- |

    mrs_msgs::HwApiVelocityHdgCmd cmd;

    cmd.header.frame_id = uav_state.header.frame_id;
    cmd.header.stamp    = ros::Time::now();

    cmd.velocity.x = des_vel[0];
    cmd.velocity.y = des_vel[1];
    cmd.velocity.z = des_vel[2];

    cmd.heading = tracker_command.heading;

    last_control_output_.control_output = cmd;

  } else if (control_output == common::VELOCITY_HDG_RATE) {

    position_pid_heading_.setSaturation(constraints.heading_speed);

    double hdg_err = mrs_lib::geometry::sradians::diff(hdg_ref, hdg);

    double des_hdg_rate = position_pid_heading_.update(hdg_err, dt);

    // | --------------------------- ff --------------------------- |

    double des_hdg_ff = 0;

    if (tracker_command.use_heading_rate) {
      des_hdg_ff = tracker_command.heading_rate;
    }

    // | --------------------- fill the output -------------------- |

    mrs_msgs::HwApiVelocityHdgRateCmd cmd;

    cmd.header.frame_id = uav_state.header.frame_id;
    cmd.header.stamp    = ros::Time::now();

    cmd.velocity.x = des_vel[0];
    cmd.velocity.y = des_vel[1];
    cmd.velocity.z = des_vel[2];

    cmd.heading_rate = des_hdg_rate + des_hdg_ff;

    last_control_output_.control_output = cmd;
  } else {

    ROS_ERROR("[Se3CopyController]: the required output of the position PID is not supported");
    return;
  }

  // fill the unbiased desired accelerations
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_heading_rate          = {};

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.ramping_up = false;

  last_control_output_.diagnostics.mass_estimator  = false;
  last_control_output_.diagnostics.mass_difference = 0;

  last_control_output_.diagnostics.disturbance_estimator = false;

  last_control_output_.diagnostics.disturbance_bx_b = 0;
  last_control_output_.diagnostics.disturbance_by_b = 0;

  last_control_output_.diagnostics.disturbance_bx_w = 0;
  last_control_output_.diagnostics.disturbance_by_w = 0;

  last_control_output_.diagnostics.disturbance_wx_w = 0;
  last_control_output_.diagnostics.disturbance_wy_w = 0;

  last_control_output_.diagnostics.controller_enforcing_constraints = false;

  last_control_output_.diagnostics.controller = "Se3CopyController";
}

//}



//
void Se3CopyController::RosTimeDifference(void){
  if(_uav_name_==_leader_uav_name_){
    if(start_time_ == 0){
      start_time_ = ros::Time::now().toSec();
    }

    if(ros::Time::now().toSec()-start_time_ < ros_time_difference_delay_ + ros_time_difference_duration_){ // Sending ros time to follower only for duration_ros_time_l_to_f_, determine_ros_time_delay_ after the start
      if(ros::Time::now().toSec()-start_time_ > ros_time_difference_delay_){
        if(!busy_ros_time_l_to_f_){
          busy_ros_time_l_to_f_ = true;
          ROS_INFO("[Se3CopyController]: Starting ros_time_l_to_f");
        }
        if(!emulate_nimbro_ || (emulate_nimbro_time_ == 0)){
          ros_time_l_to_f_.data = ros::Time::now().toSec();
          try {
            ros_time_l_to_f_pub_.publish(ros_time_l_to_f_);
          }
          catch (...) {
            ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", ros_time_l_to_f_pub_.getTopic().c_str());
          }
        }
      }
    }
    else{
      if(busy_ros_time_l_to_f_){
        busy_ros_time_l_to_f_ = false;
        ROS_INFO("[Se3CopyController]: Finished ros_time_l_to_f");
      }
    }
  }
}

void Se3CopyController::SafetyCommunication(void) {
  if(_uav_name_==_leader_uav_name_){

    // Determine time_delay safety communication
    time_delay_Eland_controller_follower_to_leader_out_.data = (ros::Time::now() - Eland_controller_follower_to_leader_.stamp).toSec();
    ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: time_delay_Eland_controller_follower_to_leader_ = %f",time_delay_Eland_controller_follower_to_leader_out_.data);

    // Detection start of communication
    if(time_delay_Eland_controller_follower_to_leader_out_.data < _max_time_delay_eland_ && !both_uavs_connected_){ 
      connection_time_ = ros::Time::now().toSec();
      ROS_INFO("[Se3CopyController]: Both UAVs are connected");
      both_uavs_connected_ = true;
    }
    
    // Eland because of time_delay > max_time_delay enabled "ready_delay_" seconds after connection
    if(both_uavs_connected_ && !both_uavs_ready_){
      if(ros::Time::now().toSec() - connection_time_ > ready_delay_){
        ROS_INFO("[Se3CopyController]: Both UAVs are ready");
        both_uavs_ready_ = true;
      }
    }

    // 3 reasons why Eland_status_ should be true
    // // New way
    bool Eland_status = false; // local variable
    if(!is_active_){ // if controller is deactivated by controlManager
      Eland_status = true;
    }
    if(Eland_controller_follower_to_leader_.data){ // if Eland_status_ of follower is true
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (1)");
      Eland_status = true;
    }
    if(time_delay_Eland_controller_follower_to_leader_out_.data > _max_time_delay_eland_ && both_uavs_ready_){ // Eland if time since last received message from follower is > _max_time_delay_eland_
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (2)");
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: time_delay_Eland_controller_follower_to_leader_ = %f",time_delay_Eland_controller_follower_to_leader_out_.data);
      Eland_status = true;
    }

    // Time at which Eland occurs the first time
    if(Eland_status && !Eland_status_){
      Eland_time_.data = ros::Time::now().toSec();
      ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland time leader = %f",Eland_time_.data);
      try {
        Eland_time_pub_.publish(Eland_time_);
      }
      catch (...) {
        ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", Eland_time_pub_.getTopic().c_str());
      }
    }
    Eland_status_ = Eland_status;

    // old way
    // if(!is_active_){ // if controller is deactivated by controlManager
    //   Eland_status_ = true;
    // }
    // if(Eland_controller_follower_to_leader_.data){ // if Eland_status_ of follower is true
    //   ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (1)");
    //   Eland_status_ = true;
    // }
    // if(time_delay_Eland_controller_follower_to_leader_out_.data > _max_time_delay_eland_ && both_uavs_ready_){ // Eland if time since last received message from follower is > _max_time_delay_eland_
    //   ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (2)");
    //   ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: time_delay_Eland_controller_follower_to_leader_ = %f",time_delay_Eland_controller_follower_to_leader_out_.data);
    //   Eland_status_ = true;
    // }


    if(Eland_status_){
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Sending Eland to follower");
    }
    Eland_controller_leader_to_follower_.data = Eland_status_;
    
    if(!emulate_nimbro_ || (emulate_nimbro_time_ == 0)){
      // Sending Eland_status_ to follower
      try {
        Eland_controller_leader_to_follower_pub_.publish(Eland_controller_leader_to_follower_);
      }
      catch (...) {
        ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", Eland_controller_leader_to_follower_pub_.getTopic().c_str());
      }
    }
    // Publishing time_delay for rosbag
    try {
      time_delay_Eland_controller_follower_to_leader_pub_.publish(time_delay_Eland_controller_follower_to_leader_out_);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", time_delay_Eland_controller_follower_to_leader_pub_.getTopic().c_str());
    }
  }
  else if(_uav_name_==_follower_uav_name_){

    // Determine time_delay safety communication
    time_delay_Eland_controller_leader_to_follower_out_.data = (ros::Time::now() - Eland_controller_leader_to_follower_.stamp).toSec();
    ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: time_delay_Eland_controller_leader_to_follower_ = %f",time_delay_Eland_controller_leader_to_follower_out_.data);

    // Detection start of communication
    if(time_delay_Eland_controller_leader_to_follower_out_.data < _max_time_delay_eland_ && !both_uavs_connected_){
      connection_time_ = ros::Time::now().toSec();
      ROS_INFO_STREAM("[Se3CopyController]: Both UAVs are connected");
      both_uavs_connected_ = true;
    }

    // Eland because of time_delay > max_time_delay enabled "ready_delay_" seconds after connection
    if(both_uavs_connected_ && !both_uavs_ready_){
      if(ros::Time::now().toSec() - connection_time_ > ready_delay_){
        ROS_INFO_STREAM("[Se3CopyController]: Both UAVs are ready");
        both_uavs_ready_ = true;
      }
    }

    // 3 reasons why Eland_status_ should be true
    // // New way
    bool Eland_status = false; // local variable
    if(!is_active_){
      Eland_status = true;
    }
    if(Eland_controller_leader_to_follower_.data){ // Eland if Eland message from leader is true
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (1)");
      Eland_status = true;
    }
    if(time_delay_Eland_controller_leader_to_follower_out_.data > _max_time_delay_eland_ && both_uavs_ready_ && is_active_){ // Eland if time since last received message from follower is > _max_time_delay_eland_
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (2)");
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: time_delay_Eland_controller_leader_to_follower_ = %f",time_delay_Eland_controller_leader_to_follower_out_.data);
      Eland_status = true;
    }

    // Time at which Eland occurs the first time
    if(Eland_status && !Eland_status_){
      Eland_time_.data = ros::Time::now().toSec();
      ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland time follower = %f",Eland_time_.data);
      try {
        Eland_time_pub_.publish(Eland_time_);
      }
      catch (...) {
        ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", Eland_time_pub_.getTopic().c_str());
      }
    }
    Eland_status_ = Eland_status;

    // old way
    // if(!is_active_){
    //   Eland_status_ = true;
    // }
    // if(Eland_controller_leader_to_follower_.data){ // Eland if Eland message from leader is true
    //   ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (1)");
    //   Eland_status_ = true;
    // }
    // if(time_delay_Eland_controller_leader_to_follower_out_.data > _max_time_delay_eland_ && both_uavs_ready_ && is_active_){ // Eland if time since last received message from follower is > _max_time_delay_eland_
    //   ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (2)");
    //   ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: time_delay_Eland_controller_leader_to_follower_ = %f",time_delay_Eland_controller_leader_to_follower_out_.data);
    //   Eland_status_ = true;
    // }

    if(Eland_status_){
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Sending Eland to leader");
    }
    Eland_controller_follower_to_leader_.data = Eland_status_;

    if(!emulate_nimbro_ || (emulate_nimbro_time_ == 0)){
      // Sending Eland_status_ to leader
      try {
        Eland_controller_follower_to_leader_pub_.publish(Eland_controller_follower_to_leader_);
      }
      catch (...) {
        ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", Eland_controller_follower_to_leader_pub_.getTopic().c_str());
      }
    }
    // Publishing time_delay for rosbag
    try {
      time_delay_Eland_controller_leader_to_follower_pub_.publish(time_delay_Eland_controller_leader_to_follower_out_);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", time_delay_Eland_controller_leader_to_follower_pub_.getTopic().c_str());
    }
  }
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

// | ----------------- LOAD ----------------------------------------- | 
// | ----------------- load subscribtion callback --------------|
// TODO: make single callback called from library as (almost) same callback used in tracker
// TODO: moreover, as this callback changes global load state variables at asynchronous and higher rates than the tracker update function, one needs to ensure that the global variables used for the state are always the same everywhere (avoid using different global information as the update function is sequentially executed). For this I think at the start of the update function one can "freeze" those global variables. So create 2 sets of global load variables and use everywhere except in this callabck the frozen variables.
void Se3CopyController::GazeboLoadStatesCallback(const gazebo_msgs::LinkStatesConstPtr& loadmsg) {
    // ROS_INFO_STREAM("GazeboLoadStatesCallback is starting");
    // This callback function is only triggered when doing simulation, and will be used to unpack the data coming from the Gazebo topics to determine the load state (i.e., in terms of anchoring point position and velocity).
    int anchoring_pt_index; // Stores the index at which the anchoring point appears in the message that is received from Gazebo. 
    std::vector<std::string> link_names = loadmsg->name; // Take a vector containing the name of each link in the simulation. Among these there is the links that are related to the payload. 
    for(size_t i = 0; i < link_names.size(); i++){ // Go through all the link names
      if (_type_of_system_ == "1uav_payload"){
        if(link_names[i] == "point_mass::link_01"){ //link_01 is the point mass payload link (see mass_point.xacro). When the link_name corresponds to the one of the payload, defined in simulation/models/suspended_payload xacro files of the testing folder.
          anchoring_pt_index = i; // Store the index of the name, as it will be used as the index to access all the states of this link, in the loadmsg later.
          payload_spawned_ = true; // Notify that the payload has spawned. This will only be triggered once and allow predictions to start.
        }
      }
      if (_type_of_system_ == "2uavs_payload"){ // 2UAVs transporting beam payload case. Need to return different link if this UAV is the leader uav or follower uav. 
        if (_uav_name_==_leader_uav_name_){
          if(link_names[i] == "bar::link_04"){ //link_04 corresponds to the anchoring point of the leader uav (see bar.xacro)
              anchoring_pt_index = i;
              payload_spawned_ = true;
          }
        }
        else if (_uav_name_==_follower_uav_name_) { // for the follower uav
          if(link_names[i] == "bar::link_01"){ //link_01 corresponds to the anchoring point of the leader uav (see bar.xacro)
              anchoring_pt_index = i;
              payload_spawned_ = true;
          }
        }
      }

      if(payload_spawned_){
        time_last_payload_message = ros::Time::now().toSec();
      }
    }
    // Extract the value from the received loadmsg. 
    anchoring_pt_pose_= loadmsg->pose[anchoring_pt_index]; // Now that we know which index refers to the anchoring point we search for (depending on which system we have), we can use it to get the actual state of this point.  
    anchoring_pt_pose_position_[0] = anchoring_pt_pose_.position.x;
    anchoring_pt_pose_position_[1] = anchoring_pt_pose_.position.y;
    anchoring_pt_pose_position_[2] = anchoring_pt_pose_.position.z;

    anchoring_pt_velocity_ = loadmsg->twist[anchoring_pt_index];
    anchoring_pt_lin_vel_[0]= anchoring_pt_velocity_.linear.x;
    anchoring_pt_lin_vel_[1]= anchoring_pt_velocity_.linear.y;
    anchoring_pt_lin_vel_[2]= anchoring_pt_velocity_.linear.z;

    // TODO: if we want to measure other variables (e.g., 2uav bar load COM linear and rotatinal velocity)

    //-------------------------------------------------//
    // if we don't print something, we get an error. TODO: figure out why, see emails with Raphael.
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: publish this here or you get strange error");

    try {
      load_pose_publisher_.publish(anchoring_pt_pose_);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", load_pose_publisher_.getTopic().c_str());
    }
    try {
      load_vel_publisher_.publish(anchoring_pt_velocity_);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", load_vel_publisher_.getTopic().c_str());
    }
}

// TODO: document and test if this works on hardware
// TODO: combine with BacaLoadStatesCallback of controller as (almost) same code
void Se3CopyController::BacaLoadStatesCallback(const mrs_modules_msgs::BacaProtocolConstPtr& msg) {
  // ROS_INFO_STREAM("BacaLoadStatesCallback is starting");
  // TODO: similar to GazeboLoadStatesCallback, update the variable payload_spawned_ if this the data is correctly received from arduino
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
    encoder_angle_1_ = encoder_output; //theta '
    //ROS_INFO_STREAM("theta " << encoder_angle_1_);
  }else if (message_id == 25)
  {
    encoder_angle_2_ = encoder_output; // Phi '
    //ROS_INFO_STREAM("phi " << encoder_angle_2_);
  }else if (message_id == 32)
  {
    encoder_velocity_1_ = encoder_output;
  }else if (message_id == 33)
  {
    encoder_velocity_2_ = encoder_output;
  }

  // Sanity checks
  /* in theory, the encoder angles would be possibe to have in the range [-M_PI/2.0, M_PI/2.0], but in practice
  the encoder fixation is results in different offsets for each UAV. 
  The maximum offsets of the asymtric encoder mddule are given in comments*/
  double encoder_angle_1_max = M_PI;//1.24;
  double encoder_angle_1_min = -M_PI;//-2.04;
  double encoder_angle_2_max = M_PI;//2.13;
  double encoder_angle_2_min = -M_PI;//-1.61;
  double msg_time_delay = std::abs(msg->stamp.toSec() - uav_state_.header.stamp.toSec());
  int bound_num_samples_delay = 2;
  if (!std::isfinite(encoder_angle_1_)||!std::isfinite(encoder_angle_2_)) {
    if(_run_type_!="uav"){
      ROS_ERROR("[Se3CopyController]: NaN detected in encoder angles");
    }
    else{
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD*10,"[Se3CopyController]: NaN detected in encoder angles");
    }
    payload_spawned_ = false; //Put payload_spawned back to false in case the encoder stops giving finite values during a flight. Epl stays equal to zero when this flag is false, avoiding strange behaviors or non finite Epl. 
  }
  else if (!std::isfinite(encoder_velocity_1_)||!std::isfinite(encoder_velocity_2_)) {
    if(_run_type_!="uav"){
      ROS_ERROR("[Se3CopyController]: NaN detected in encoder angular velocities");
    }
    else{
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD*10,"[Se3CopyController]: NaN detected in encoder angular velocities");
    }
    payload_spawned_ = false;  
  }
  else if ((encoder_angle_1_>encoder_angle_1_max && encoder_angle_1_< encoder_angle_1_min) || (encoder_angle_2_>encoder_angle_2_max && encoder_angle_2_< encoder_angle_2_min)) {
    if(_run_type_!="uav"){
      ROS_ERROR("[Se3CopyController]: Out of expected range [-pi/2, pi/2] detected in encoder angles");
    }
    else{
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD*10,"[Se3CopyController]: Out of expected range [-pi/2, pi/2] detected in encoder angles");
    }
    payload_spawned_ = false; 
  }
  else if (msg_time_delay > dt_*bound_num_samples_delay) {
    if(_run_type_!="uav"){
      ROS_ERROR("[Se3CopyController]: Encoder msg is delayed by at least %d samples and is = %f", bound_num_samples_delay, msg_time_delay);
    }
    else{
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD*10,"[Se3CopyController]: Encoder msg is delayed by at least %d samples and is = %f", bound_num_samples_delay, msg_time_delay);
    }
    payload_spawned_ = false; 
  }
  else{
    ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD*5,"[Se3CopyController]: Encoder angles and angular velocities returned are finite values and the angles are within the expected range.");
    payload_spawned_ = true; // Values are finite and withing the expect range and thus can be used in the computations
  }

  if (payload_spawned_) {
    time_last_payload_message = ros::Time::now().toSec();

    Eigen::Vector3d anchoring_pt_pose_position_rel ; // position of the payload in the body frame B
    Eigen::Vector3d anchoring_pt_lin_vel_rel; //Velocity of the payload in the body frame B.

    //Compute position of the anchoring point in body base B. (i.e., relative to drone COM)
    anchoring_pt_pose_position_rel[0]=_cable_length_*sin(encoder_angle_1_)*cos(encoder_angle_2_);
    anchoring_pt_pose_position_rel[1]=_cable_length_*sin(encoder_angle_2_);
    anchoring_pt_pose_position_rel[2]=-_cable_length_*cos(encoder_angle_1_)*cos(encoder_angle_2_);
    
    //Compute absolute position of the payload.
    Eigen::Vector3d Op(uav_state_.pose.position.x, uav_state_.pose.position.y, uav_state_.pose.position.z);
    Eigen::Vector3d Ov(uav_state_.velocity.linear.x, uav_state_.velocity.linear.y, uav_state_.velocity.linear.z);
    Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state_.pose.orientation);

    anchoring_pt_pose_position_ = Op + R*anchoring_pt_pose_position_rel;
    
    //Compute absolute velocity of the anchoring point.
    Eigen::Vector3d Ow(uav_state_.velocity.angular.x, uav_state_.velocity.angular.y, uav_state_.velocity.angular.z); 
    Eigen::Matrix3d skew_Ow;
    skew_Ow << 0.0     , -Ow(2), Ow(1),
              Ow(2) , 0.0,       -Ow(0),
              -Ow(1), Ow(0),  0.0;
    Eigen::Matrix3d Rdot = skew_Ow*R;
    anchoring_pt_lin_vel_rel[0] =_cable_length_*(cos(encoder_angle_1_)*cos(encoder_angle_2_)*encoder_velocity_1_
                                  - sin(encoder_angle_1_)*sin(encoder_angle_2_)*encoder_velocity_2_);
    anchoring_pt_lin_vel_rel[1] =_cable_length_*cos(encoder_angle_2_)*encoder_velocity_2_;
    anchoring_pt_lin_vel_rel[2] =_cable_length_*(sin(encoder_angle_2_)*cos(encoder_angle_1_)*encoder_velocity_2_+
                                  sin(encoder_angle_1_)*cos(encoder_angle_2_)*encoder_velocity_1_);
    
    anchoring_pt_lin_vel_ = Ov + Rdot*anchoring_pt_pose_position_rel + R*anchoring_pt_lin_vel_rel;

    //Store values in msgs
    encoder_angle_1_to_publish_.data = encoder_angle_1_;
    encoder_angle_2_to_publish_.data = encoder_angle_2_;

    anchoring_pt_pose_.position.x = anchoring_pt_pose_position_[0];
    anchoring_pt_pose_.position.y = anchoring_pt_pose_position_[1];
    anchoring_pt_pose_.position.z = anchoring_pt_pose_position_[2];
    anchoring_pt_velocity_.linear.x = anchoring_pt_lin_vel_[0];
    anchoring_pt_velocity_.linear.y = anchoring_pt_lin_vel_[1];
    anchoring_pt_velocity_.linear.z = anchoring_pt_lin_vel_[2];
    
    //Publish values
    try {
      load_pose_publisher_.publish(anchoring_pt_pose_);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", load_pose_publisher_.getTopic().c_str());
    }
    try {
      load_vel_publisher_.publish(anchoring_pt_velocity_);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", load_vel_publisher_.getTopic().c_str());
    }
    try {
      encoder_angle_1_publisher_.publish(encoder_angle_1_to_publish_);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", encoder_angle_1_publisher_.getTopic().c_str());
    }
    try {
      encoder_angle_2_publisher_.publish(encoder_angle_2_to_publish_);
    }
    catch (...) {
      ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", encoder_angle_2_publisher_.getTopic().c_str());
    }
  } 
  else{
    if(_run_type_!="uav"){
      ROS_ERROR("[Se3CopyController]: Something is wrong with the encoder msg as payload_spawned_ = false and therefor the encoder and the anchoring point data are NOT globally updated or published.");
    }
    else{
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Something is wrong with the encoder msg as payload_spawned_ = false and therefore the encoder and the anchoring point data are NOT globally updated or published.");
    }
  }
}

// | ----------------- 2UAVs safety communication --------------|

void Se3CopyController::ElandLeaderToFollowerCallback(const mrs_msgs::BoolStamped& msg){
  Eland_controller_leader_to_follower_ = msg;
  Eland_controller_leader_to_follower_.stamp = ros::Time::now();
}

void Se3CopyController::ElandFollowerToLeaderCallback(const mrs_msgs::BoolStamped& msg){
  Eland_controller_follower_to_leader_ = msg;
  Eland_controller_follower_to_leader_.stamp = ros::Time::now();
}

// void Se3CopyController::rosTimeTriggerCallback(const std_msgs::Bool& msg){
//   ros_time_out_.data = ros::Time::now().toSec();
//   ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: ROS-time follower = %f ",ros_time_out_.data);
//   try {
//     ros_time_pub_.publish(ros_time_out_);
//   }
//   catch (...) {
//     ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", ros_time_pub_.getTopic().c_str());
//   }
// }

void Se3CopyController::rosTimeLeaderToFollowerCallback(const std_msgs::Float64& msg){
  double ros_time_leader = msg.data;
  double ros_time_follower = ros::Time::now().toSec();
  // ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: ROS time leader = %f ",ros_time_leader);
  // ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: ROS time follower = %f ",ros_time_follower);

  ros_time_delay_.data = ros_time_leader - ros_time_follower;
  // ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: ROS time delay leader - follower = %f ",ros_time_delay_.data);
  try {
    ros_time_delay_pub_.publish(ros_time_delay_);
  }
  catch (...) {
    ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", ros_time_delay_pub_.getTopic().c_str());
  }
}

// Used to compute distance between UAVs (safety feature)
void Se3CopyController::uav_state_follower_for_leader_callback(const mrs_msgs::UavState::ConstPtr& msg){
  // ROS_INFO_STREAM("[Se3CopyController]: Leader received UAV state of follower in controller");
  if(!distance_uavs_ready_){
    distance_uavs_ready_ = true;
  }
  uav_state_follower_for_leader_ = *msg;
}

// | ------------------------------------------------------------------- | 

/* //{ callbackDrs() */

void Se3CopyController::callbackDrs(controllers_brubotics::se3_copy_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[Se3CopyController]: DRS updated gains");
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerGains() //{ */

void Se3CopyController::timerGains(const ros::TimerEvent& event) {

  mrs_lib::Routine    profiler_routine = profiler_.createRoutine("timerGains", _gain_filtering_rate_, 1.0, event);
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("ControlManager::timerHwApiCapabilities", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);
  auto gains      = mrs_lib::get_mutexed(mutex_gains_, gains_);

  // When muting the gains, we want to bypass the filter,
  // so it happens immediately.
  bool   bypass_filter = (mute_gains_ || mute_gains_by_tracker_);
  double gain_coeff    = (mute_gains_ || mute_gains_by_tracker_) ? _gain_mute_coefficient_ : 1.0;

  mute_gains_ = false;

  const double dt = (event.current_real - event.last_real).toSec();

  bool updated = false;

  gains.kpxy          = calculateGainChange(dt, gains.kpxy, drs_params.kpxy * gain_coeff, bypass_filter, "kpxy", updated);
  gains.kvxy          = calculateGainChange(dt, gains.kvxy, drs_params.kvxy * gain_coeff, bypass_filter, "kvxy", updated);
  gains.kaxy          = calculateGainChange(dt, gains.kaxy, drs_params.kaxy * gain_coeff, bypass_filter, "kaxy", updated);
  gains.kiwxy         = calculateGainChange(dt, gains.kiwxy, drs_params.kiwxy * gain_coeff, bypass_filter, "kiwxy", updated);
  gains.kibxy         = calculateGainChange(dt, gains.kibxy, drs_params.kibxy * gain_coeff, bypass_filter, "kibxy", updated);
  gains.kpz           = calculateGainChange(dt, gains.kpz, drs_params.kpz * gain_coeff, bypass_filter, "kpz", updated);
  gains.kvz           = calculateGainChange(dt, gains.kvz, drs_params.kvz * gain_coeff, bypass_filter, "kvz", updated);
  gains.kaz           = calculateGainChange(dt, gains.kaz, drs_params.kaz * gain_coeff, bypass_filter, "kaz", updated);
  gains.kq_roll_pitch = calculateGainChange(dt, gains.kq_roll_pitch, drs_params.kq_roll_pitch * gain_coeff, bypass_filter, "kq_roll_pitch", updated);
  gains.kq_yaw        = calculateGainChange(dt, gains.kq_yaw, drs_params.kq_yaw * gain_coeff, bypass_filter, "kq_yaw", updated);
  gains.km            = calculateGainChange(dt, gains.km, drs_params.km * gain_coeff, bypass_filter, "km", updated);
  // TODO: add load gains here

  // do not apply muting on these gains
  gains.kiwxy_lim = calculateGainChange(dt, gains.kiwxy_lim, drs_params.kiwxy_lim, false, "kiwxy_lim", updated);
  gains.kibxy_lim = calculateGainChange(dt, gains.kibxy_lim, drs_params.kibxy_lim, false, "kibxy_lim", updated);
  gains.km_lim    = calculateGainChange(dt, gains.km_lim, drs_params.km_lim, false, "km_lim", updated);

  mrs_lib::set_mutexed(mutex_gains_, gains, gains_);

  // set the gains back to dynamic reconfigure
  // and only do it when some filtering occurs
  if (updated) {

    drs_params.kpxy          = gains.kpxy;
    drs_params.kvxy          = gains.kvxy;
    drs_params.kaxy          = gains.kaxy;
    drs_params.kiwxy         = gains.kiwxy;
    drs_params.kibxy         = gains.kibxy;
    drs_params.kpz           = gains.kpz;
    drs_params.kvz           = gains.kvz;
    drs_params.kaz           = gains.kaz;
    drs_params.kq_roll_pitch = gains.kq_roll_pitch;
    drs_params.kq_yaw        = gains.kq_yaw;
    drs_params.kiwxy_lim     = gains.kiwxy_lim;
    drs_params.kibxy_lim     = gains.kibxy_lim;
    drs_params.km            = gains.km;
    drs_params.km_lim        = gains.km_lim;
    // TODO: add load gains here

    drs_->updateConfig(drs_params);

    ROS_INFO_THROTTLE(10.0, "[Se3CopyController]: gains have been updated");
  }
}

//}

// --------------------------------------------------------------
// |                       other routines                       |
// --------------------------------------------------------------

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
    ROS_DEBUG("[Se3CopyController]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

/* getHeadingSafely() //{ */

double Se3CopyController::getHeadingSafely(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  try {
    return mrs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: could not calculate the UAV heading");
  }

  try {
    return mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: could not calculate the UAV heading");
  }

  if (tracker_command.use_heading) {
    return tracker_command.heading;
  }

  return 0;
}

//}

}  // namespace se3_copy_controller

}  // namespace controllers_brubotics

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(controllers_brubotics::se3_copy_controller::Se3CopyController, mrs_uav_managers::Controller)
