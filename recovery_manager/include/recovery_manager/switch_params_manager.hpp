/*********************************************************************
 *
 *  Copyright (c) 2024, LexxPluss, Inc.
 *  All rights reserved.
 *
 *********************************************************************/
#ifndef SWITCH_PARAMS_MANAGER_HPP_
#define SWITCH_PARAMS_MANAGER_HPP_

#include <ros/ros.h>
#include <nav_core/recovery_behavior.h>
#include <lexxauto_msgs/CarryingInformation.h>
#include <lexxauto_msgs/ActuatorStatus.h>

// namespace
namespace carrying_manager
{

// class
class SwitchParamsManager 
{
public:
  SwitchParamsManager(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~SwitchParamsManager();

private:
  typedef boost::shared_ptr<nav_core::RecoveryBehavior> BehPtr;

  bool loadRecoveryBehaviors();
  void loadDefaultRecoveryBehaviors();
  bool loadGivenRecoveryBehavior(const std::string & field_name, 
                                              const std::string & loop_count_str, 
                                              std::vector<std::string> &behaviors);   

  bool createRecoveryBehaviors( XmlRpc::XmlRpcValue behavior_list,
                                              std::vector<std::string> & behaviors);

  void carryingStatusCB(const lexxauto_msgs::ActuatorStatus::ConstPtr& msg);

  void carryingInfoCB(const lexxauto_msgs::CarryingInformation::ConstPtr& msg);

  void reloadRecoveryBehavior(const bool& is_carrying);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber carrying_status_sub_;
  ros::Subscriber carrying_info_sub_;


  std::map<std::string, std::string> behavior_definitions_;

  std::vector<std::string> recovery_behaviors_;
  std::vector<std::string> recovery_behaviors_carrying_;

 };

}  // namespace carrying_manager

#endif  // CARRYING_MANAGER_STATUS_CONTROLLER_HPP_
