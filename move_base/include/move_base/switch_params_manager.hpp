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

  bool loadRecoveryBehaviors(ros::NodeHandle node);

  void addRecoveryBehavior(std::string name, boost::shared_ptr<std::vector<BehPtr>> behaviors,std::vector<std::string> &recovery_behavior_names);

  bool createRecoveryBehaviors(
    XmlRpc::XmlRpcValue behavior_list,
    boost::shared_ptr<std::vector<BehPtr>> behaviors,
    std::vector<std::string> &recovery_behavior_names,
    int& idx, int depth = 0);

  void carryingStatusCB(const lexxauto_msgs::ActuatorStatus::ConstPtr& msg);

  void carryingInfoCB(const lexxauto_msgs::CarryingInformation::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber carrying_status_sub_;
  ros::Subscriber carrying_info_sub_;

  int outer_loop_recovery_count_;
  int inner_loop_recovery_count_;

  std::map<std::string, std::string> behavior_definitions_;
  std::map<std::string, BehPtr> recovery_behaviors_cache_;

  boost::shared_ptr<std::vector<BehPtr>> recovery_behaviors_;
  boost::shared_ptr<std::vector<BehPtr>> recovery_behaviors_carrying_;
  std::vector<std::string> recovery_behavior_names_;
  std::vector<std::string> recovery_behavior_names_carrying_;

 };

}  // namespace carrying_manager

#endif  // CARRYING_MANAGER_STATUS_CONTROLLER_HPP_
