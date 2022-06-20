/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>

/* this header file is created during compilation from python script dynparam.cfg */
#include <voo_wp/dynparamConfig.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/BoolStamped.h>
#include <mrs_msgs/Vec4.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

/* for operations with matrices */
#include <Eigen/Dense>

#include <pluginlib/class_list_macros.h>

//}

using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;

namespace voo_wp
{

/* class WPFlier //{ */

class WPFlier : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;

  /* ros parameters */
  bool        _simulation_;
  std::string _uav_name_;

  // | ---------------------- msg callbacks --------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_ground_truth_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  void callbackControlManagerDiag(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& sh);
  bool have_goal_ = false;
  bool is_arrived_ = false;

  // | --------------------- timer callbacks -------------------- |

  void           callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te);
  ros::Publisher pub_dist_to_waypoint_;
  ros::Timer     timer_publish_dist_to_waypoint_;
  int            _rate_timer_publish_dist_to_waypoint_;

  void           callbackTimerPublishSetReference(const ros::TimerEvent& te);
  ros::Publisher pub_reference_;
  ros::Timer     timer_publisher_reference_;
  int            _rate_timer_publisher_reference_;

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  void           callbackTimerPublishArrived(const ros::TimerEvent& te);
  ros::Publisher pub_arrived_;
  ros::Timer     timer_publisher_arrived_;
  int            _rate_timer_publish_arrived_;

  // | ---------------- service server callbacks ---------------- |

  bool               callbackStartWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_waypoints_following_;

  bool               callbackStopWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_stop_waypoints_following_;

  bool               callbackFlyToFirstWaypoint(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_fly_to_first_waypoint_;

  bool               callbackFlyToGivenWaypoint(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res);
  ros::ServiceServer srv_server_fly_to_given_waypoint_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_land_;
  bool               _land_end_;

  // | -------------------- loading waypoints ------------------- |

  std::vector<mrs_msgs::Reference> waypoints_;
  std::string                      _waypoints_frame_;
  bool                             waypoints_loaded_ = false;
  mrs_msgs::Reference              current_waypoint_;
  std::mutex                       mutex_current_waypoint_;
  int                              idx_current_waypoint_;
  int                              n_waypoints_;
  int                              _n_loops_;
  int                              c_loop_;
  std::mutex                       mutex_waypoint_idle_time_;
  Eigen::MatrixXd                  _offset_;

  // | ------------------- dynamic reconfigure ------------------ |

  typedef voo_wp::dynparamConfig                              Config;
  typedef dynamic_reconfigure::Server<voo_wp::dynparamConfig> ReconfigureServer;
  boost::recursive_mutex                                              mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer>                                reconfigure_server_;
  void                                                                callbackDynamicReconfigure(Config& config, uint32_t level);
  voo_wp::dynparamConfig                                      last_drs_config_;

  // | --------------------- waypoint idling -------------------- |

  bool       is_idling_ = false;
  ros::Timer timer_idling_;
  double     _waypoint_idle_time_;
  void       callbackTimerIdling(const ros::TimerEvent& te);

  // | -------------------- support functions ------------------- |

  std::vector<mrs_msgs::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);

  void offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset);

  double distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose);
};

//}


// ------------------------------------ TUDO ACIMA SERIA O .h ------------------------------------//

/* onInit() //{ */

void WPFlier::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here

  have_goal_        = false;
  is_idling_        = false;
  waypoints_loaded_ = false;
  is_arrived_   = false;

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(nh, "WPFlier");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("simulation", _simulation_);
  param_loader.loadParam("land_at_the_end", _land_end_);
  param_loader.loadParam("n_loops", _n_loops_);
  param_loader.loadParam("waypoint_idle_time", _waypoint_idle_time_);
  param_loader.loadParam("waypoints_frame", _waypoints_frame_);
  param_loader.loadParam("rate/publish_dist_to_waypoint", _rate_timer_publish_dist_to_waypoint_);
  param_loader.loadParam("rate/publish_reference", _rate_timer_publisher_reference_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("rate/publish_arrived", _rate_timer_publish_arrived_);

  /* load waypoints as a half-dynamic matrix from config file */
  Eigen::MatrixXd waypoint_matrix;
  param_loader.loadMatrixDynamic("waypoints", waypoint_matrix, -1, 4);  // -1 indicates the dynamic dimension
  waypoints_            = matrixToPoints(waypoint_matrix);
  n_waypoints_          = waypoints_.size();
  waypoints_loaded_     = true;
  idx_current_waypoint_ = 0;
  c_loop_               = 0;
  ROS_INFO_STREAM_ONCE("[WPFlier]: " << n_waypoints_ << " waypoints loaded");
  ROS_INFO_STREAM_ONCE("[WPFlier]: " << _n_loops_ << " loops requested");

  /* load offset of all waypoints as a static matrix from config file */
  param_loader.loadMatrixStatic("offset", _offset_, 1, 4);
  offsetPoints(waypoints_, _offset_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WPFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ------------------ initialize subscribers ----------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh;
  shopts.node_name          = "WPFlier";
  shopts.no_message_timeout = ros::Duration(1.0);
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_odometry_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_uav_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in",
                                                                                            &WPFlier::callbackControlManagerDiag, this);

  /* subscribe ground truth only in simulation, where it is available */
  if (_simulation_) {
    sh_ground_truth_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_gt_in");
  }

  // | ------------------ initialize publishers ----------------- |

  pub_dist_to_waypoint_ = nh.advertise<mrs_msgs::Float64Stamped>("dist_to_waypoint_out", 1);
  pub_reference_        = nh.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);
  pub_arrived_          = nh.advertise<mrs_msgs::BoolStamped>("arrived_in_pose", 1);

  // | -------------------- initialize timers ------------------- |

  timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(_rate_timer_publish_dist_to_waypoint_), &WPFlier::callbackTimerPublishDistToWaypoint, this);

  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &WPFlier::callbackTimerCheckSubscribers, this);

  // you can disable autostarting of the timer by the last argument
  timer_publisher_reference_ =
      nh.createTimer(ros::Rate(_rate_timer_publisher_reference_), &WPFlier::callbackTimerPublishSetReference, this, false, false);

  timer_publisher_arrived_ = nh.createTimer(ros::Rate(_rate_timer_publish_arrived_), &WPFlier::callbackTimerPublishArrived, this);


  // | --------------- initialize service servers --------------- |

  srv_server_start_waypoints_following_ = nh.advertiseService("start_waypoints_following_in", &WPFlier::callbackStartWaypointFollowing, this);
  srv_server_stop_waypoints_following_  = nh.advertiseService("stop_waypoints_following_in", &WPFlier::callbackStopWaypointFollowing, this);
  srv_server_fly_to_first_waypoint_     = nh.advertiseService("fly_to_first_waypoint_in", &WPFlier::callbackFlyToFirstWaypoint, this);

  srv_server_fly_to_given_waypoint_     = nh.advertiseService("fly_to_given_waypoint_in", &WPFlier::callbackFlyToGivenWaypoint, this);

  // | --------------- initialize service clients --------------- |

  srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("land_out");

  // | ---------- initialize dynamic reconfigure server --------- |

  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&WPFlier::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  /* set the default value of dynamic reconfigure server to the value of parameter with the same name */
  {
    std::scoped_lock lock(mutex_waypoint_idle_time_);
    last_drs_config_.waypoint_idle_time = _waypoint_idle_time_;
  }
  reconfigure_server_->updateConfig(last_drs_config_);

  ROS_INFO_ONCE("[WPFlier]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackControlManagerDiag() //{ */

void WPFlier::callbackControlManagerDiag(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& sh) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_)
    return;

  mrs_msgs::ControlManagerDiagnosticsConstPtr diagnostics = sh.getMsg();

  ROS_INFO_ONCE("[WPFlier]: Received first control manager diagnostics msg");

  if (have_goal_ && !diagnostics->tracker_status.have_goal) {

    ROS_INFO("[WPFlier]: Waypoint reached.");
    have_goal_ = false;

    /* start idling at the reached waypoint */
    is_idling_ = true;

    ros::NodeHandle nh("~");
    timer_idling_ = nh.createTimer(ros::Duration(_waypoint_idle_time_), &WPFlier::callbackTimerIdling, this,
                                   true);  // the last boolean argument makes the timer run only once

    ROS_INFO("[WPFlier]: Idling for %.2f seconds.", _waypoint_idle_time_);
  }
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerPublishSetReference() //{ */

void WPFlier::callbackTimerPublishSetReference([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  /* return if the uav is still flying to the previous waypoints */
  if (have_goal_)
    return;

  /* return if the UAV is idling at a waypoint */
  if (is_idling_)
    return;

  /* shutdown node after flying through all the waypoints (call land service before) */
  if (idx_current_waypoint_ >= n_waypoints_) {

    c_loop_++;

    ROS_INFO("[WPFlier]: Finished loop %d/%d", c_loop_, _n_loops_);

    if (c_loop_ >= _n_loops_) {

      ROS_INFO("[WPFlier]: Finished %d loops of %d waypoints.", _n_loops_, n_waypoints_);

      if (_land_end_) {
        ROS_INFO("[WPFlier]: Calling land service.");
        std_srvs::Trigger srv_land_call;
        srv_client_land_.call(srv_land_call);
      }

      //ROS_INFO("[WPFlier]: Shutting down.");
      //ros::shutdown();
      return;

    } else {
      ROS_INFO("[WPFlier]: Starting loop %d/%d", c_loop_ + 1, _n_loops_);
      idx_current_waypoint_ = 0;
    }
  }

  /* create new waypoint msg */
  mrs_msgs::ReferenceStamped new_waypoint;

  // it is important to set the frame id correctly !!
  // -- "" means the frame currently used for control
  // other options:
  // -- "gps_origin"
  // -- "local_origin" (0, 0) where the drone starts
  // ...
  new_waypoint.header.frame_id = _waypoints_frame_;
  new_waypoint.header.stamp    = ros::Time::now();

  new_waypoint.reference = waypoints_.at(idx_current_waypoint_);

  // set the variable under the mutex
  mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(idx_current_waypoint_), current_waypoint_);

  ROS_INFO("[WPFlier]: Flying to waypoint %d: x: %.2f y: %.2f z: %.2f heading: %.2f", idx_current_waypoint_ + 1, new_waypoint.reference.position.x,
           new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.heading);

  try {
    pub_reference_.publish(new_waypoint);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
  }

  idx_current_waypoint_++;

  have_goal_ = true;
}

//}

/* callbackTimerPublishDistToWaypoint() //{ */

void WPFlier::callbackTimerPublishDistToWaypoint([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_) {
    return;
  }

  /* do not publish distance to waypoint when the uav is not flying towards a waypoint */
  if (!have_goal_) {
    return;
  }

  // this routine can not work without the odometry
  if (!sh_odometry_.hasMsg()) {
    return;
  }

  // get the variable under the mutex
  mrs_msgs::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::Pose current_pose = mrs_lib::getPose(sh_odometry_.getMsg());

  double dist = distance(current_waypoint, current_pose);
  ROS_INFO("[WPFlier]: Distance to waypoint: %.2f", dist);

  mrs_msgs::Float64Stamped dist_msg;
  // it is important to set the frame id correctly !!
  dist_msg.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  dist_msg.header.stamp    = ros::Time::now();
  dist_msg.value           = dist;

  try {
    pub_dist_to_waypoint_.publish(dist_msg);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_dist_to_waypoint_.getTopic().c_str());
  }
}


/* callbackTimerPublishArrived() */

void WPFlier::callbackTimerPublishArrived([[maybe_unused]] const ros::TimerEvent& te) {
  
  mrs_msgs::BoolStamped is_arrived;

  if (!is_initialized_) {
    return;
  }

  /* do not publish distance to waypoint when the uav is not flying towards a waypoint */
  if (!have_goal_) {
    is_arrived_ = true;
  }else{
    is_arrived_ = false;
  }
  
  is_arrived.data = is_arrived_;

  try {
    pub_arrived_.publish(is_arrived);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_arrived_.getTopic().c_str());
  }
}

//}

/* callbackTimerCheckSubscribers() //{ */

void WPFlier::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  if (!sh_odometry_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[WPFlier]: Not received uav odom msg since node launch.");
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[WPFlier]: Not received tracker diagnostics msg since node launch.");
  }

  if (_simulation_) {
    if (!sh_ground_truth_.hasMsg()) {
      ROS_WARN_THROTTLE(1.0, "[WPFlier]: Not received ground truth odom msg since node launch.");
    }
  }
}

//}

/* callbackTimerIdling() //{ */

void WPFlier::callbackTimerIdling([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO("[WPFlier]: Idling finished");
  is_idling_ = false;
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStartWaypointFollowing() */

bool WPFlier::callbackStartWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[WPFlier]: Cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (waypoints_loaded_) {

    timer_publisher_reference_.start();

    ROS_INFO("[WPFlier]: Starting waypoint following.");

    res.success = true;
    res.message = "Starting waypoint following.";

  } else {

    ROS_WARN("[WPFlier]: Cannot start waypoint following, waypoints are not set.");
    res.success = false;
    res.message = "Waypoints not set.";
  }

  return true;
}

//}

/* //{ callbackStopWaypointFollowing() */

bool WPFlier::callbackStopWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[WPFlier]: Cannot stop waypoint following, nodelet is not initialized.");
    return true;
  }

  timer_publisher_reference_.stop();

  ROS_INFO("[WPFlier]: Waypoint following stopped.");

  res.success = true;
  res.message = "Waypoint following stopped.";

  return true;
}

//}

/* //{ callbackFlyToFirstWaypoint() */

bool WPFlier::callbackFlyToFirstWaypoint([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[WPFlier]: Cannot start waypoint following, nodelet is not initialized.");

    return true;
  }

  if (waypoints_loaded_) {

    /* create new waypoint msg */
    mrs_msgs::ReferenceStamped new_waypoint;

    // it is important to set the frame id correctly !!
    new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
    new_waypoint.header.stamp    = ros::Time::now();
    new_waypoint.reference       = waypoints_.at(0);

    mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(0), current_waypoint_);

    // set the variable under the mutex

    idx_current_waypoint_ = 0;
    c_loop_               = 0;

    have_goal_ = true;

    try {
      pub_reference_.publish(new_waypoint);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
    }

    std::stringstream ss;
    ss << "Flying to first waypoint: x: " << new_waypoint.reference.position.x << ", y: " << new_waypoint.reference.position.y
       << ", z: " << new_waypoint.reference.position.z << ", heading: " << new_waypoint.reference.heading;

    ROS_INFO_STREAM_THROTTLE(1.0, "[WPFlier]: " << ss.str());

    res.success = true;
    res.message = ss.str();

  } else {

    ROS_WARN("[WPFlier]: Cannot fly to first waypoint, waypoints not loaded!");

    res.success = false;
    res.message = "Waypoints not loaded";
  }

  return true;
}



/* //{ callbackFlyToGivenWaypoint() */

bool WPFlier::callbackFlyToGivenWaypoint(mrs_msgs::Vec4::Request& req, mrs_msgs::Vec4::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[WPFlier]: Cannot start waypoint following, nodelet is not initialized.");

    return true;
  }

    /* create new waypoint msg */
    mrs_msgs::ReferenceStamped new_waypoint;

    // it is important to set the frame id correctly !!
    new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
    new_waypoint.header.stamp    = ros::Time::now();
    
    /* fill in reference */
    new_waypoint.reference.position.x = req.goal[0];
    new_waypoint.reference.position.y = req.goal[1];
    new_waypoint.reference.position.z = req.goal[2];
    new_waypoint.reference.heading = req.goal[3];

    c_loop_               = 0;

    have_goal_ = true;

    try {
      pub_reference_.publish(new_waypoint);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
    }

    std::stringstream ss;
    ss << "Flying to given waypoint: x: " << new_waypoint.reference.position.x << ", y: " << new_waypoint.reference.position.y
       << ", z: " << new_waypoint.reference.position.z << ", heading: " << new_waypoint.reference.heading;

    ROS_INFO_STREAM_THROTTLE(1.0, "[WPFlier]: " << ss.str());

    res.success = true;
    res.message = ss.str();

  return true;
}


//}

// | -------------- dynamic reconfigure callback -------------- |

/* //{ callbackDynamicReconfigure() */

void WPFlier::callbackDynamicReconfigure([[maybe_unused]] Config& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_)
    return;

  ROS_INFO(
      "[WPFlier]:"
      "Reconfigure Request: "
      "Waypoint idle time: %.2f",
      config.waypoint_idle_time);

  {
    std::scoped_lock lock(mutex_waypoint_idle_time_);

    _waypoint_idle_time_ = config.waypoint_idle_time;
  }
}

//}

// | -------------------- support functions ------------------- |

/* matrixToPoints() //{ */

std::vector<mrs_msgs::Reference> WPFlier::matrixToPoints(const Eigen::MatrixXd& matrix) {

  std::vector<mrs_msgs::Reference> points;

  for (int i = 0; i < matrix.rows(); i++) {

    mrs_msgs::Reference point;
    point.position.x = matrix.row(i)(0);
    point.position.y = matrix.row(i)(1);
    point.position.z = matrix.row(i)(2);
    point.heading    = matrix.row(i)(3);

    points.push_back(point);
  }

  return points;
}

//}

/* offsetPoints() //{ */

void WPFlier::offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset) {

  for (size_t i = 0; i < points.size(); i++) {

    points.at(i).position.x += offset(0);
    points.at(i).position.y += offset(1);
    points.at(i).position.z += offset(2);
    points.at(i).heading += offset(3);
  }
}

//}

/* distance() //{ */

double WPFlier::distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

//}

}  // namespace voo_wp

/* every nodelet must include macros which export the class as a nodelet plugin */
PLUGINLIB_EXPORT_CLASS(voo_wp::WPFlier, nodelet::Nodelet);
