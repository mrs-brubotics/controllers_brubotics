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
#include <std_msgs/Bool.h>
#include <mrs_msgs/BoolStamped.h>

#include <vector>

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
  ros::NodeHandle                                    nh_;
  ros::NodeHandle                                    nh2_;
  std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers_;

  // | ------------------- declaring env (session/bashrc) parameters ------------------- |
  std::string _uav_name_;         // uavID
  std::string _leader_uav_name_;  // leader uavID for 2uavs payload transport
  std::string _follower_uav_name_;// follower uavID for 2uavs payload transport
  std::string _run_type_;         // set to "simulation" (for Gazebo simulation) OR "uav" (for hardware testing) defined in bashrc or session.yaml. Used for payload transport as payload position comes from two different callbacks depending on how the test is ran (in sim or on real UAV).
  std::string _type_of_system_;   // defines the dynamic system model to simulate in the prediction using the related controller: can be 1uav_no_payload, 1uav_payload or 2uavs_payload. Set in session.yaml file.
  double _cable_length_;          // length of the cable between payload COM / anchoring point and COM of the UAV
  double _cable_length_offset_; // accounts for the fact that the cable is attached below the UAV's COM
  double _load_mass_;             // feedforward load mass per uav defined in the session.yaml of every test file (session variable also used by the xacro for Gazebo simulation)
  bool _baca_in_simulation_=false;// Used to validate the encoder angles, and the FK without having to make the UAV fly. Gains related to payload must be set on 0 to perform this. Set on false by default.
  //emulate nimbro
  bool emulate_nimbro_ = false;
  double emulate_nimbro_delay_;
  double emulate_nimbro_time_ = 0;


  // | ------------------- declaring .yaml parameters (and some related vars & funs) ------------------- |
  // Se3CopyController:
  std::string _version_;
  bool   _profiler_enabled_ = false;
  double kpxy_;       // position xy gain
  double kvxy_;       // velocity xy gain
  double kplxy_;      // load position xy gain
  double kvlxy_;      // load velocity xy gain
  double kaxy_;       // acceleration xy gain (feed forward, =1)
  double kiwxy_;      // world xy integral gain
  double kibxy_;      // body xy integral gain
  double kiwxy_lim_;  // world xy integral limit
  double kibxy_lim_;  // body xy integral limit
  double kpz_;        // position z gain
  double kvz_;        // velocity z gain
  double kplz_;       // load position z gain
  double kvlz_;       // load velocity z gain
  double kaz_;        // acceleration z gain (feed forward, =1)
  double km_;         // mass estimator gain
  double km_lim_;     // mass estimator limit
  double kqxy_;       // pitch/roll attitude gain
  double kqz_;        // yaw attitude gain
  bool   _tilt_angle_failsafe_enabled_;
  double _tilt_angle_failsafe_;
  //double _thrust_saturation_; // total thrust limit in [0,1]
  std_msgs::Float64 _thrust_saturation_; // TODO: make tracker and controller same in how type of _thrust_saturation_ is chosen
  // rampup:
  bool   _rampup_enabled_ = false;
  double _rampup_speed_;
  bool      rampup_active_ = false;
  double    rampup_thrust_;
  int       rampup_direction_;
  double    rampup_duration_;
  ros::Time rampup_start_time_;
  ros::Time rampup_last_time_;
  // gains filtering:
  void filterGains(const bool mute_gains, const double dt);
  double calculateGainChange(const double dt, const double current_value, const double desired_value, const bool bypass_rate, std::string name, bool& updated);
  double _gains_filter_change_rate_;
  double _gains_filter_min_change_rate_;
  // output mode:
  int    output_mode_;  // attitude_rate / quaternion
  std::mutex mutex_output_mode_;
  // gain muting:
  bool   gains_muted_ = false;  // the current state (may be initialized in activate())
  double _gain_mute_coefficient_;
  // dynamic reconfigure server:
  boost::recursive_mutex                            mutex_drs_;
  typedef mrs_uav_controllers::se3_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>  Drs_t;
  boost::shared_ptr<Drs_t>                          drs_;
  void                                              callbackDrs(mrs_uav_controllers::se3_controllerConfig& config, uint32_t level);
  DrsConfig_t                                       drs_params_;
  std::mutex mutex_gains_;       // locks the gains the are used and filtered
  std::mutex mutex_drs_params_;  // locks the gains that came from the drs
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
  void BacaLoadStatesCallback(const mrs_msgs::BacaProtocolConstPtr& msg); // TODO: bryan think how we can make library to use these function in both controller and tracker if exactly same
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
  double _max_time_delay_safety_communication_;
  bool Eland_status_ = false;
  // double _max_time_delay_on_callback_data_follower_;
  // double _max_time_delay_on_callback_data_leader_;
  bool both_uavs_connected_ = false;
  double start_time_ = 0;
  double connection_time_ = 0;
  double ros_time_difference_delay_;
  double ros_time_difference_duration_;
  bool busy_ros_time_l_to_f_ = false;
  double ready_delay_;
  bool both_uavs_ready_ = false;
  bool deactivated_ = false;
  // int n_ros_time_triggers_;
  // int i_ros_time_triggers_ = 0;
  // bool determine_ros_time_delay_;


  // | ------------------------ integrals ----------------------- |
  Eigen::Vector2d Ib_b_;  // body error integral in the body frame
  Eigen::Vector2d Iw_w_;  // world error integral in the world_frame
  std::mutex      mutex_integrals_;

  // | ------------------ profiler_ ----------------- |
  mrs_lib::Profiler profiler_;
  
  // | ------------------ initialization ----------------- |
  bool is_initialized_ = false;
  double dt_ = 0.010; // DO NOT CHANGE! Hardcoded controller sample time = controller sample time TODO: obtain via loop rate, see MpcTracker

  // | ------------------ activation and output ----------------- |
  mrs_msgs::AttitudeCommand::ConstPtr last_attitude_cmd_;
  mrs_msgs::AttitudeCommand           activation_attitude_cmd_;
  ros::Time last_update_time_;
  bool      first_iteration_ = true;
  bool is_active_      = false;
  
  // | ------------------------ update ----------------------- |
  // | ------------------------ uav state ----------------------- |
  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;
  double uav_heading_;

  // | ----------------- load ---------------- |
  geometry_msgs::Pose load_position_errors;

  // | ---------- thrust generation and mass estimation --------- |
  double _uav_mass_; // feedforward uav mass
  double uav_mass_difference_; // total mass difference (integral error over uav position)
  
  // | ----------------------- constraints ---------------------- |
  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;
  bool                          got_constraints_ = false;

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
void Se3CopyController::initialize(const ros::NodeHandle& parent_nh, [[maybe_unused]] const std::string name, const std::string name_space, const double uav_mass,
                               std::shared_ptr<mrs_uav_managers::CommonHandlers_t> common_handlers) {
  ROS_INFO("[Se3CopyController]: start of initialize");
  _uav_mass_ = uav_mass;
  common_handlers_ = common_handlers;
  ros::NodeHandle nh_(parent_nh, name_space); // NodeHandle for Se3CopyController, used to load controller params
  ros::NodeHandle nh2_(parent_nh, "dergbryan_tracker"); // NodeHandle 2 for DergbryanTracker, used to load tracker params
  ros::Time::waitForValid();
  
  // | ------------------- loading env (session/bashrc) parameters ------------------- |
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

  // | ------------------- loading .yaml parameters ------------------- |
  mrs_lib::ParamLoader param_loader(nh_, "Se3CopyController");
  param_loader.loadParam("version", _version_);
  if (_version_ != VERSION) {
    ROS_ERROR("[Se3CopyController]: the version of the binary (%s) does not match the config file (%s), please build me!", VERSION, _version_.c_str());
    ros::requestShutdown();
  }
  param_loader.loadParam("enable_profiler", _profiler_enabled_);
  // lateral gains and limits:
  param_loader.loadParam("default_gains/horizontal/kp", kpxy_);
  param_loader.loadParam("default_gains/horizontal/kv", kvxy_);
  param_loader.loadParam("default_gains/horizontal/ka", kaxy_);
  param_loader.loadParam("default_gains/horizontal/kiw", kiwxy_);
  param_loader.loadParam("default_gains/horizontal/kib", kibxy_);
  param_loader.loadParam("default_gains/horizontal/kiw_lim", kiwxy_lim_);
  param_loader.loadParam("default_gains/horizontal/kib_lim", kibxy_lim_);
  // lateral gains for load damping part of the controller:
  param_loader.loadParam("default_gains/horizontal/kpl", kplxy_);
  param_loader.loadParam("default_gains/horizontal/kvl", kvlxy_);
  // vertical gains:
  param_loader.loadParam("default_gains/vertical/kp", kpz_);
  param_loader.loadParam("default_gains/vertical/kv", kvz_);
  param_loader.loadParam("default_gains/vertical/ka", kaz_);
  // load gains vertical:
  param_loader.loadParam("default_gains/vertical/kpl", kplz_);
  param_loader.loadParam("default_gains/vertical/kvl", kvlz_);
  // mass estimator (vertical) gains and limits:
  param_loader.loadParam("default_gains/mass_estimator/km", km_);
  param_loader.loadParam("default_gains/mass_estimator/km_lim", km_lim_);
  // attitude gains:
  param_loader.loadParam("default_gains/horizontal/attitude/kq", kqxy_);
  param_loader.loadParam("default_gains/vertical/attitude/kq", kqz_);
  // constraints:
  param_loader.loadParam("constraints/tilt_angle_failsafe/enabled", _tilt_angle_failsafe_enabled_);
  param_loader.loadParam("constraints/tilt_angle_failsafe/limit", _tilt_angle_failsafe_);
  if (_tilt_angle_failsafe_enabled_ && fabs(_tilt_angle_failsafe_) < 1e-3) {
    ROS_ERROR("[Se3CopyController]: constraints/tilt_angle_failsafe/enabled = 'TRUE' but the limit is too low");
    ros::requestShutdown();
  }
  param_loader.loadParam("constraints/thrust_saturation", _thrust_saturation_.data);
  // rampup:
  param_loader.loadParam("rampup/enabled", _rampup_enabled_);
  param_loader.loadParam("rampup/speed", _rampup_speed_);
  // gain filtering:
  param_loader.loadParam("gains_filter/perc_change_rate", _gains_filter_change_rate_);
  param_loader.loadParam("gains_filter/min_change_rate", _gains_filter_min_change_rate_);
  // gain muting:
  param_loader.loadParam("gain_mute_coefficient", _gain_mute_coefficient_);
  // output mode:
  param_loader.loadParam("output_mode", output_mode_);
  if (!(output_mode_ == OUTPUT_ATTITUDE_RATE || output_mode_ == OUTPUT_ATTITUDE_QUATERNION)) {
    ROS_ERROR("[Se3CopyController]: output mode has to be {0, 1}!");
    ros::requestShutdown();
  }
  param_loader.loadParam("rotation_matrix", drs_params_.rotation_type);
  // angular rate feed forward:
  param_loader.loadParam("angular_rate_feedforward/parasitic_pitch_roll", drs_params_.pitch_roll_heading_rate_compensation);
  param_loader.loadParam("angular_rate_feedforward/jerk", drs_params_.jerk_feedforward);
  // payload:
  param_loader.loadParam("payload/Epl_min", _Epl_min_);
  param_loader.loadParam("payload/Epl_max/failsafe_enabled", _Epl_max_failsafe_enabled_);
  param_loader.loadParam("payload/Epl_max/scaling", _Epl_max_scaling_);
  param_loader.loadParam("payload/swing_angle/failsafe_enabled", max_swing_angle_failsafe_enabled_);  
  param_loader.loadParam("payload/swing_angle/max", max_swing_angle_);
  param_loader.loadParam("two_uavs_payload/max_time_delay_safety_communication", _max_time_delay_safety_communication_);
  // param_loader.loadParam("two_uavs_payload/n_ros_time_triggers",n_ros_time_triggers_);
  // param_loader.loadParam("two_uavs_payload/ros_time_delay/n_ros_time_l_to_f",n_ros_time_l_to_f_);
  // param_loader.loadParam("two_uavs_payload/ros_time_delay/ros_time_l_to_f_period",ros_time_l_to_f_period_);
  param_loader.loadParam("two_uavs_payload/ready_delay", ready_delay_); 
  param_loader.loadParam("two_uavs_payload/ros_time_difference/ros_time_difference_delay", ros_time_difference_delay_);
  param_loader.loadParam("two_uavs_payload/ros_time_difference/ros_time_difference_duration", ros_time_difference_duration_);
  param_loader.loadParam("two_uavs_payload/nimbro/emulate_nimbro", emulate_nimbro_);
  param_loader.loadParam("two_uavs_payload/nimbro/emulate_nimbro_delay", emulate_nimbro_delay_);
  param_loader.loadParam("ros_info_throttle_period", ROS_INFO_THROTTLE_PERIOD);
 
  
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Se3CopyController]: could not load all parameters!");
    ros::requestShutdown();
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
      ros::requestShutdown();
    }
  }

  // 2UAVs safety communication
  if(_type_of_system_=="2uavs_payload"){
    if (_uav_name_ == _leader_uav_name_){  // leader
      Eland_controller_follower_to_leader_sub_ = nh_.subscribe("/"+_follower_uav_name_+"/control_manager/se3_copy_controller/Eland_contr_f_to_l", 1, &Se3CopyController::ElandFollowerToLeaderCallback, this, ros::TransportHints().tcpNoDelay());
    }
    else if(_uav_name_ == _follower_uav_name_){
      Eland_controller_leader_to_follower_sub_ = nh_.subscribe("/"+_leader_uav_name_+"/control_manager/se3_copy_controller/Eland_contr_l_to_f", 1, &Se3CopyController::ElandLeaderToFollowerCallback, this, ros::TransportHints().tcpNoDelay());
      // ros_time_trigger_l_sub_ = nh_.subscribe("/"+_leader_uav_name_+"/control_manager/se3_copy_controller/ros_time_trigger_l", 1, &Se3CopyController::rosTimeTriggerCallback, this, ros::TransportHints().tcpNoDelay());
      ros_time_l_to_f_sub_ = nh_.subscribe("/"+_leader_uav_name_+"/control_manager/se3_copy_controller/ros_time_l_to_f", 1, &Se3CopyController::rosTimeLeaderToFollowerCallback, this, ros::TransportHints().tcpNoDelay());
    }  
  }

  ROS_INFO("[Se3CopyController]: linked all subscribers to their callbacks.");

  // | ---------------- prepare stuff from params --------------- |
  // initialize the integrals
  uav_mass_difference_ = 0;
  Iw_w_                = Eigen::Vector2d::Zero(2); //World integral
  Ib_b_                = Eigen::Vector2d::Zero(2); //Body integral, see ctu-mrs paper for more details.

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

  deactivated_ = true;

  ROS_INFO("[Se3CopyController]: deactivated");
}

//}
/* //{ update() */

const mrs_msgs::AttitudeCommand::ConstPtr Se3CopyController::update(const mrs_msgs::UavState::ConstPtr&        uav_state,
                                                                const mrs_msgs::PositionCommand::ConstPtr& control_reference) {
                                                                  
  // ROS_INFO_THROTTLE(0.1,"[Se3CopyController]: Start of update()");
  mrs_lib::Routine profiler_routine = profiler_.createRoutine("update");
  mrs_lib::ScopeTimer timer = mrs_lib::ScopeTimer("Se3CopyController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  // uav_state_:
  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = *uav_state;
  }
  try {
    uav_state_publisher_.publish(uav_state_);
  }
  catch (...) {
    ROS_ERROR("[Se3CopyController]: Exception caught during publishing topic %s.", uav_state_publisher_.getTopic().c_str());
  }

  // Emulates nimbro communication
  if(emulate_nimbro_){
    emulate_nimbro_time_ = emulate_nimbro_time_ + dt_;
    if(emulate_nimbro_time_>emulate_nimbro_delay_){
      emulate_nimbro_time_ = 0;
    }
  }

  // | ----------------- 2UAVs safety communication --------------|
  if(_type_of_system_=="2uavs_payload"){
    if(is_active_ || deactivated_){ // Safety communication after activation
      SafetyCommunication();
      if(Eland_status_ && is_active_){
        ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Returning empty command to trigger Eland");
        return mrs_msgs::AttitudeCommand::ConstPtr();
      }
    }
    else{ // Before activation, communication to determine ros time difference
      RosTimeDifference();
    }
  }

  // | ----------------- payload safety check --------------|
  if(_type_of_system_=="1uav_payload" || _type_of_system_=="2uavs_payload"){
    if(payload_spawned_ && !payload_once_spawned_){
      payload_once_spawned_ = true;
    }

    if(payload_once_spawned_){
      // ROS_INFO_STREAM("time baca"<< ros::Time::now().toSec()-time_last_payload_message);
      if(ros::Time::now().toSec()-time_last_payload_message>0.1){
        ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: time since last 'payload_swpawned_ = true' is bigger than 0.1 seconds. Problem with the BACA protocol => trigger eland");
        return mrs_msgs::AttitudeCommand::ConstPtr();
      }
    }
  }

  if (!is_active_) {
    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // Op - position in global frame
  // Ov - velocity in global frame
  Eigen::Vector3d Op(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
  Eigen::Vector3d Ov(uav_state->velocity.linear.x, uav_state->velocity.linear.y, uav_state->velocity.linear.z);

  // R - current uav attitude
  Eigen::Matrix3d R = mrs_lib::AttitudeConverter(uav_state->pose.orientation);

  // Ow - UAV angular rate
  Eigen::Vector3d Ow(uav_state->velocity.angular.x, uav_state->velocity.angular.y, uav_state->velocity.angular.z); // TODO: actually not used (see Se3Controller CTU)

  // payload anchoring point state:

  // Opl - position load in global frame
  // Ovl - velocity load in global frame
  
  // TODO: currently Opl and Ovl are only known once the payload has been spawned, otherwise inititlized on zero.
  Eigen::Vector3d Opl = anchoring_pt_pose_position_;
  Eigen::Vector3d Ovl = anchoring_pt_lin_vel_;

  // DRS:
  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);
  
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

  // | ----------------- get the current heading ---------------- |
  uav_heading_ = 0;
  try {
    uav_heading_ = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();
  }
  catch (...) {
    ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: could not calculate the UAV heading");
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
  /* test streaming the references Rp, Rv, Ra when the uav is moving.*/
  // ROS_INFO_STREAM("Rp = \n" << Rp);
  // ROS_INFO_STREAM("Rv = \n" << Rv);
  // ROS_INFO_STREAM("Ra = \n" << Ra);
 
  // | -------------- calculate the control errors -------------- |
  // position control error
  Eigen::Vector3d Ep = Eigen::Vector3d::Zero(3);

  if (control_reference->use_position_horizontal || control_reference->use_position_vertical) {
    Ep = Op - Rp;
  }

  // velocity control error
  Eigen::Vector3d Ev = Eigen::Vector3d::Zero(3);

  if (control_reference->use_velocity_horizontal || control_reference->use_velocity_vertical ||
      control_reference->use_position_vertical) {  // even when use_position_vertical to provide dampening
    Ev = Ov - Rv;
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
    if (control_reference->use_position_horizontal || control_reference->use_position_vertical) {
      Eigen::Vector3d e3(0.0, 0.0, 1.0);
      Epl = Op - _cable_length_*e3 - Opl; // relative to uav base frame poiting from the anchoring point to the stable equilibrium beneuth the uav, according to Pandolfo / thesis Raphael: Anchoring point realignment
      // 2021 student said this : Op - Opl is super unstable!! However, this is the way Pandolfo thesis explained it. And I think what they did (Rp - Opl) will always be more unstable, as for a very far references, the control actions of the error of the UAV and the one of the Payload will superpose and generate a huge Td, which can easilly saturates the actuators and creates instability. 
      // TODO: I thought Raphael only looks to the posiiton error in xy (ignoring z)? check this.
    }
    // load velocity control error
    if (control_reference->use_velocity_horizontal || control_reference->use_velocity_vertical ||
      control_reference->use_position_vertical) {  // even when use_position_vertical to provide dampening
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
        return mrs_msgs::AttitudeCommand::ConstPtr(); // trigger eland
      }
    }
    // Ignore small load position errors to deal with small, but non-zero load offsets in steady-state and prevent aggressive actions on small errors 
    if(Epl.norm() < _Epl_min_){ // When the payload is very close to equilibrium vertical position, the error is desactivated so the UAV doesn't try to compensate and let it damp naturally.
      ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Control error of the anchoring point Epl = %.02fm < _Epl_min_ = %.02fm, hence it has been set to zero", Epl.norm(), _Epl_min_);
      Epl = Eigen::Vector3d::Zero(3);
    }

    // Check max swing angle
    Eigen::Vector3d uav_position(uav_state->pose.position.x,uav_state->pose.position.y,uav_state->pose.position.z); //get a vector of the UAV position to ease the following computations.
    Eigen::Vector3d mu; //Unit vector indicating cable orientation.
    Eigen::Vector3d zB; //unit vector z_B of the UAV body frame
    mu = (uav_position-Opl).normalized();
    zB = R.col(2);
    double swing_angle = acos((mu.dot(zB))/(mu.norm()*zB.norm()));
    // ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: swing angle = %f ",swing_angle);
    if(swing_angle > max_swing_angle_){
      ROS_ERROR("[Se3CopyController]: Swing angle is larger than allowed (%.02f rad >  %.02f rad).", swing_angle, max_swing_angle_);
      if (max_swing_angle_failsafe_enabled_){
        return mrs_msgs::AttitudeCommand::ConstPtr(); // trigger eland
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

    if (control_reference->use_velocity_vertical) {
      Kv[2] = kvz_;
    } 
    else if (control_reference->use_position_vertical) {  // special case: want to control z-pos but not the velocity => at least provide z dampening
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

  // | --------------------------LOAD--------------------------|
  // Load the gains for the Anchoring point realignment method of Pandolfo: 
  Eigen::Array3d  Kpl = Eigen::Array3d::Zero(3); 
  Eigen::Array3d  Kdl = Eigen::Array3d::Zero(3); 

  if (_type_of_system_ == "1uav_payload" ||_type_of_system_ == "2uavs_payload" ){ 
    if (control_reference->use_position_horizontal) {
      Kpl[0] = kplxy_;
      Kpl[1] = kplxy_;
    } else {
      Kpl[0] = 0;
      Kpl[1] = 0;
    }

    if (control_reference->use_position_vertical) {
      Kpl[2] = kplz_;
    } else {
      Kpl[2] = 0;
    }

    if (control_reference->use_velocity_horizontal) {
      Kdl[0] = kvlxy_;
      Kdl[1] = kvlxy_;
    } else {
      Kdl[0] = 0;
      Kdl[1] = 0;
    }

    if (control_reference->use_velocity_vertical) {
      Kdl[2] = kvlz_;
    } 
    else if (control_reference->use_position_vertical) {  // special case: want to control z-pos but not the velocity => at least provide z dampening
      Kdl[2] = kvlz_;
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
  if(_run_type_!="uav"){ // Only printed in simulation
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

    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: _uav_mass_ = %f", _uav_mass_);
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: uav_mass_difference_ (estimated)= %f", uav_mass_difference_);
    if(_type_of_system_ == "1uav_payload" || _type_of_system_ == "2uavs_payload" ){
      ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: _load_mass_ = %f", _load_mass_);
    }
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: total_mass (estimated)= %f", total_mass);
    
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: n_motors = %d", common_handlers_->motor_params.n_motors);
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: motor_params.A = %f", common_handlers_->motor_params.A);
    ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: motor_params.B = %f", common_handlers_->motor_params.B);
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

    auto res = common_handlers_->transformer->transformSingle(Ib_b_stamped,uav_state_.header.frame_id);

    if (res) {
      Ib_w[0] = res.value().vector.x;
      Ib_w[1] = res.value().vector.y;
    } else {
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: could not transform the Ib_b_ to the world frame");
    }
  }

  // construct the desired force vector

  // Compute control actions 
  Eigen::Vector3d feed_forward      = total_mass * (Eigen::Vector3d(0, 0, common_handlers_->g) + Ra);
  Eigen::Vector3d position_feedback = -Kp * Ep.array(); // note: Ep was defined as current - desired
  Eigen::Vector3d velocity_feedback = -Kv * Ev.array(); // note: Ev was defined as current - desired
  Eigen::Vector3d integral_feedback;
  {
    std::scoped_lock lock(mutex_integrals_);

    integral_feedback << Ib_w[0] + Iw_w_[0], Ib_w[1] + Iw_w_[1], 0;
  }
  // Load errors were initialized to zero if load not spawned
  Eigen::Vector3d load_position_feedback = -Kpl * Epl.array(); // pushes uav in opposed direction of the position error from anchoring point to load at equilibrium
  Eigen::Vector3d load_velocity_feedback = -Kdl * Evl.array(); // pushes uav in opposed direction of the velocity error from anchoring point to load at equilibrium

  // | -------------------------------se3+load------------------------ | 
  Eigen::Vector3d f = position_feedback + velocity_feedback + load_position_feedback + load_velocity_feedback + integral_feedback + feed_forward;
  ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: fx= %.2f, fy= %.2f, fz=%.2f ",f[0],f[1],f[2]);

  // | ----------- limiting the desired downwards acceleration and maximum tilt angle ---------- |
  // TODO: check with MPC guidage code if this actually makes sense as i remember to have improved it
  if (f[2] < 0) {
    ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: the calculated downwards desired force is negative (%.2f) -> mitigating flip", f[2]);
    f << 0, 0, 1; // ctu original
    // f << f[0], f[1], 0.0; // saturate the z-component on zero such that the desired tilt angle stays in the upper hemisphere
  }

  // | ------------------ limit the tilt angle ------------------ |

  Eigen::Vector3d f_norm = f.normalized();

  // calculate the force in spherical coordinates
  // TODO: streamline theta using simple double and Float64 if needed. See ComputeSe3Controller.
  std_msgs::Float64 theta;
  theta.data = acos(f_norm[2]);
  double phi   = atan2(f_norm[1], f_norm[0]);

  // ROS_INFO_STREAM("theta (deg) = \n" << theta*180/3.1415);
  // ROS_INFO_STREAM("phi (deg) = \n" << phi*180/3.1415);

  // check for the failsafe limit
  if (!std::isfinite(theta.data)) {
    ROS_ERROR("[Se3CopyController]: NaN detected in variable 'theta', returning null");

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  if (_tilt_angle_failsafe_enabled_ && theta.data > _tilt_angle_failsafe_) {

    ROS_ERROR("[Se3CopyController]: the produced tilt angle (%.2f deg) would be over the failsafe limit (%.2f deg), returning null", (180.0 / M_PI) * theta.data,
              (180.0 / M_PI) * _tilt_angle_failsafe_);

    if(_run_type_!="uav"){ // Only print in simulation
      ROS_INFO("[Se3CopyController]: f = [%.2f, %.2f, %.2f]", f[0], f[1], f[2]);
      ROS_INFO("[Se3CopyController]: position feedback: [%.2f, %.2f, %.2f]", position_feedback[0], position_feedback[1], position_feedback[2]);
      ROS_INFO("[Se3CopyController]: velocity feedback: [%.2f, %.2f, %.2f]", velocity_feedback[0], velocity_feedback[1], velocity_feedback[2]);
      ROS_INFO("[Se3CopyController]: load position feedback: [%.2f, %.2f, %.2f]", load_position_feedback[0], load_position_feedback[1], load_position_feedback[2]);
      ROS_INFO("[Se3CopyController]: load velocity feedback: [%.2f, %.2f, %.2f]", load_velocity_feedback[0], load_velocity_feedback[1], load_velocity_feedback[2]);
      ROS_INFO("[Se3CopyController]: integral feedback: [%.2f, %.2f, %.2f]", integral_feedback[0], integral_feedback[1], integral_feedback[2]);
      ROS_INFO("[Se3CopyController]: position_cmd: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", control_reference->position.x, control_reference->position.y,
              control_reference->position.z, control_reference->heading);
      ROS_INFO("[Se3CopyController]: odometry: x: %.2f, y: %.2f, z: %.2f, heading: %.2f", uav_state->pose.position.x, uav_state->pose.position.y,
              uav_state->pose.position.z, uav_heading_);
    }

    return mrs_msgs::AttitudeCommand::ConstPtr();
  }

  // saturate the angle

  auto constraints = mrs_lib::get_mutexed(mutex_constraints_, constraints_);
  // ROS_INFO_THROTTLE(15.0,"[Se3CopyController]: constraints.tilt = %f", constraints.tilt);
  // ROS_INFO_STREAM("[Se3CopyController]: theta.data = " << theta.data);
  if (fabs(constraints.tilt) > 1e-3 && theta.data > constraints.tilt) {
    ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: tilt is being saturated, desired: %.2f deg, saturated %.2f deg", (theta.data / M_PI) * 180.0,
                      (constraints.tilt / M_PI) * 180.0);
    theta.data = constraints.tilt;
  }

  // reconstruct the vector
  f_norm[0] = sin(theta.data) * cos(phi);
  f_norm[1] = sin(theta.data) * sin(phi);
  f_norm[2] = cos(theta.data);

  // publish the tilt angle
  tilt_angle_publisher_.publish(theta);

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

    if (control_reference->use_heading) {
      bxd << cos(control_reference->heading), sin(control_reference->heading), 0;
    } else {
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: desired heading was not specified, using current heading instead!");
      bxd << cos(uav_heading_), sin(uav_heading_), 0;
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

  Eigen::Matrix3d E = Eigen::Matrix3d::Zero();

  if (!control_reference->use_attitude_rate) {
    E = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
    //ROS_INFO_STREAM("orientation error");
  }

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
  projected_thrust_publisher_.publish(thrust_force);
  std_msgs::Float64 thrust_norm;
  thrust_norm.data = sqrt(f(0,0)*f(0,0)+f(1,0)*f(1,0)+f(2,0)*f(2,0)); // norm of f ( not projected on the z axis of the UAV frame)
  thrust_publisher_.publish(thrust_norm);
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
  thrust_satlimit_physical_publisher_.publish(thrust_saturation_physical);
  thrust_satlimit_publisher_.publish(_thrust_saturation_);
  hover_thrust_publisher_.publish(hover_thrust);

  if (!control_reference->use_thrust) {
    if (thrust_force.data >= 0) {
      thrust = mrs_lib::quadratic_thrust_model::forceToThrust(common_handlers_->motor_params, thrust_force.data);
    } else {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: just so you know, the desired thrust force is negative (%.2f)", thrust_force.data);
    }
  } else {
    // the thrust is overriden from the tracker command
    thrust = control_reference->thrust;
  }

  // saturate the thrust
  if (!std::isfinite(thrust)) {
    thrust = 0;
    if(_run_type_!="uav"){
      ROS_ERROR("[Se3CopyController]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");
    }
    else{
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: NaN detected in variable 'thrust', setting it to 0 and returning!!!");
    }
  } else if (thrust > _thrust_saturation_.data) {

    thrust = _thrust_saturation_.data;
    ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: saturating thrust to %.2f", _thrust_saturation_.data);

  } else if (thrust < 0.0) {

    thrust = 0.0;
    ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: saturating thrust to 0");
  }
  

  // custom publisher
  std_msgs::Float64 thrust_physical_saturated;
  thrust_physical_saturated.data = mrs_lib::quadratic_thrust_model::thrustToForce(common_handlers_->motor_params, thrust);
  //ROS_INFO_STREAM("thrust_physical_saturated = \n" << thrust_physical_saturated);
  thrust_satval_publisher_.publish(thrust_physical_saturated);

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
      if(_run_type_!="uav"){
        ROS_ERROR("[Se3CopyController]: exception caught while calculating the desired_yaw_rate feedforward");
      }
      else{
        ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: exception caught while calculating the desired_yaw_rate feedforward");
      }
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
      if(_run_type_!="uav"){
        ROS_ERROR("[Se3CopyController]: exception caught while calculating the parasitic heading rate!");
      }
      else{
        ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: exception caught while calculating the parasitic heading rate!");
      }
    }

    try {
      rp_heading_rate_compensation(2) = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getYawRateIntrinsic(-parasitic_heading_rate);
    }
    catch (...) {
      if(_run_type_!="uav"){
        ROS_ERROR("[Se3CopyController]: exception caught while calculating the parasitic heading rate compensation!");
      }
      else{
        ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: exception caught while calculating the parasitic heading rate compensation!");
      }
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
    bool world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[0])) {
      Iw_w_[0] = 0;
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'Iw_w_[0]', setting it to 0!!!");
    } else if (Iw_w_[0] > kiwxy_lim_) {
      Iw_w_[0]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[0] < -kiwxy_lim_) {
      Iw_w_[0]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: SE3's world X integral is being saturated!");
    }

    // saturate the world Y
    world_integral_saturated = false;
    if (!std::isfinite(Iw_w_[1])) {
      Iw_w_[1] = 0;
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'Iw_w_[1]', setting it to 0!!!");
    } else if (Iw_w_[1] > kiwxy_lim_) {
      Iw_w_[1]                 = kiwxy_lim_;
      world_integral_saturated = true;
    } else if (Iw_w_[1] < -kiwxy_lim_) {
      Iw_w_[1]                 = -kiwxy_lim_;
      world_integral_saturated = true;
    }

    if (kiwxy_lim_ >= 0 && world_integral_saturated) {
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

      auto res = common_handlers_->transformer->transformSingle(Ep_stamped,"fcu_untilted");

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

      auto res = common_handlers_->transformer->transformSingle(Ev_stamped,"fcu_untilted");

      if (res) {
        Ev_fcu_untilted[0] = res.value().vector.x;
        Ev_fcu_untilted[1] = res.value().vector.x;
      } else {
        ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: could not transform the velocity error to fcu_untilted");
      }
    }

    // integrate the body error
    if (control_reference->use_position_horizontal) {
      Ib_b_ -= kibxy_ * Ep_fcu_untilted * dt;
    } else if (control_reference->use_velocity_horizontal) {
      Ib_b_ -= kibxy_ * Ev_fcu_untilted * dt;
    }

    // saturate the body
    bool body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[0])) {
      Ib_b_[0] = 0;
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'Ib_b_[0]', setting it to 0!!!");
    } else if (Ib_b_[0] > kibxy_lim_) {
      Ib_b_[0]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[0] < -kibxy_lim_) {
      Ib_b_[0]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: SE3's body pitch integral is being saturated!");
    }

    // saturate the body
    body_integral_saturated = false;
    if (!std::isfinite(Ib_b_[1])) {
      Ib_b_[1] = 0;
      ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'Ib_b_[1]', setting it to 0!!!");
    } else if (Ib_b_[1] > kibxy_lim_) {
      Ib_b_[1]                = kibxy_lim_;
      body_integral_saturated = true;
    } else if (Ib_b_[1] < -kibxy_lim_) {
      Ib_b_[1]                = -kibxy_lim_;
      body_integral_saturated = true;
    }

    if (kibxy_lim_ > 0 && body_integral_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: SE3's body roll integral is being saturated!");
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
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: NaN detected in variable 'uav_mass_difference_', setting it to 0 and returning!!!");
    } else if (uav_mass_difference_ > km_lim_) {
      uav_mass_difference_ = km_lim_;
      uav_mass_saturated   = true;
    } else if (uav_mass_difference_ < -km_lim_) {
      uav_mass_difference_ = -km_lim_;
      uav_mass_saturated   = true;
    }

    if (uav_mass_saturated) {
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: The UAV mass difference is being saturated to %.2f!", uav_mass_difference_);
    }
  }

  //}

  // --------------------------------------------------------------
  // |                 produce the control output                 |
  // --------------------------------------------------------------

  mrs_msgs::AttitudeCommand::Ptr output_command(new mrs_msgs::AttitudeCommand);
  output_command->header.stamp = ros::Time::now(); // TODO: why ros::Time::now used and not synced with controller update?

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

    auto res = common_handlers_->transformer->transformSingle(world_accel,"fcu");

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
    ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: missing dynamics constraints");
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

    ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: outputting desired orientation (this is not normal)");
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

      ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: ramping up thrust, %.4f", output_command->thrust);
    }

  } else {
    output_command->thrust = thrust;
  }

  // ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: thrust = %f ",thrust);

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

  auto res = common_handlers_->transformer->transformSingle(world_integrals,new_uav_state->header.frame_id);

  if (res) {

    std::scoped_lock lock(mutex_integrals_);

    Iw_w_[0] = res.value().vector.x;
    Iw_w_[1] = res.value().vector.y;

  } else {

    ROS_ERROR_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: could not transform world integral to the new frame");

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
    if(time_delay_Eland_controller_follower_to_leader_out_.data < _max_time_delay_safety_communication_ && !both_uavs_connected_){ 
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
    if(deactivated_){ // if controller is deactivated by controlManager
      Eland_status = true;
    }
    if(Eland_controller_follower_to_leader_.data){ // if Eland_status_ of follower is true
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (1)");
      Eland_status = true;
    }
    if(time_delay_Eland_controller_follower_to_leader_out_.data > _max_time_delay_safety_communication_ && both_uavs_ready_){ // Eland if time since last received message from follower is > _max_time_delay_safety_communication_
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
    // if(deactivated_){ // if controller is deactivated by controlManager
    //   Eland_status_ = true;
    // }
    // if(Eland_controller_follower_to_leader_.data){ // if Eland_status_ of follower is true
    //   ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (1)");
    //   Eland_status_ = true;
    // }
    // if(time_delay_Eland_controller_follower_to_leader_out_.data > _max_time_delay_safety_communication_ && both_uavs_ready_){ // Eland if time since last received message from follower is > _max_time_delay_safety_communication_
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
    if(time_delay_Eland_controller_leader_to_follower_out_.data < _max_time_delay_safety_communication_ && !both_uavs_connected_){
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
    if(deactivated_){
      Eland_status = true;
    }
    if(Eland_controller_leader_to_follower_.data){ // Eland if Eland message from leader is true
      ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (1)");
      Eland_status = true;
    }
    if(time_delay_Eland_controller_leader_to_follower_out_.data > _max_time_delay_safety_communication_ && both_uavs_ready_ && is_active_){ // Eland if time since last received message from follower is > _max_time_delay_safety_communication_
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
    // if(deactivated_){
    //   Eland_status_ = true;
    // }
    // if(Eland_controller_leader_to_follower_.data){ // Eland if Eland message from leader is true
    //   ROS_WARN_THROTTLE(ROS_INFO_THROTTLE_PERIOD,"[Se3CopyController]: Eland (1)");
    //   Eland_status_ = true;
    // }
    // if(time_delay_Eland_controller_leader_to_follower_out_.data > _max_time_delay_safety_communication_ && both_uavs_ready_ && is_active_){ // Eland if time since last received message from follower is > _max_time_delay_safety_communication_
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
void Se3CopyController::BacaLoadStatesCallback(const mrs_msgs::BacaProtocolConstPtr& msg) {
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
    // TODO: add those of load when this dynamic reconfigure function is tested

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
    ROS_INFO_THROTTLE(ROS_INFO_THROTTLE_PERIOD, "[Se3CopyController]: changing gain '%s' from %.2f to %.2f", name.c_str(), current_value, desired_value);
    updated = true;
  }

  return current_value + change;
}

//}

}  // namespace se3_copy_controller

}  // namespace mrs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_controllers::se3_copy_controller::Se3CopyController, mrs_uav_managers::Controller)
