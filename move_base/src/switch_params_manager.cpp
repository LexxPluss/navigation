/*********************************************************************
 *
 *  Copyright (c) 2024, LexxPluss, Inc.
 *  All rights reserved.
 *
 *********************************************************************/

#include <carrying_manager/switch_params_manager.hpp>
#include <lexxauto_msgs/CarryingInformation.h>

namespace carrying_manager
{
SwitchParamsManager::SwitchParamsManager(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  pnh_(pnh),
  (nh)
{
  bool use_carrying_manager = false;
  nh.param<bool>("carrying_manager/use_carrying_manager", use_carrying_manager, false);

  if (use_carrying_manager)
  {
    carrying_info_sub_ = nh.subscribe("carrying_manager/carrying_info", 1, &MoveBase::carryingInfoCB, this);
  }
  else
  {
    carrying_status_sub_ = nh.subscribe<lexxauto_msgs::ActuatorStatus>("actuator_position", 1, boost::bind(&MoveBase::carryingStatusCB, this, _1));
  }
  this->recovery_behaviors_ = boost::make_shared<std::vector<BehPtr>>();
  this->recovery_behaviors_carrying_ = boost::make_shared<std::vector<BehPtr>>();
  this->current_recovery_behaviors_ = this->recovery_behaviors_;

  //could be directly parsed from config, but will leave it for backward compatibility now
  behavior_definitions_ = {
  {"conservative_reset", "clear_costmap_recovery/ClearCostmapRecovery"},
  {"aggressive_reset", "clear_costmap_recovery/ClearCostmapRecovery"},
  {"safety_direction_recovery", "safety_direction_recovery/SafetyDirectionRecovery"},
  {"rotate_recovery", "rotate_recovery/RotateRecovery"},
  {"go_back_recovery", "go_back_recovery/GoBackRecovery"},
  {"rotate_small_recovery", "rotate_small_recovery/RotateSmallRecovery"},
  {"remove_virtual_obstacle_recovery", "remove_virtual_obstacle_recovery/RemoveVirtualObstacleRecovery"}
  };

  if(!loadRecoveryBehaviors())
    loadDefaultRecoveryBehaviors();
}

SwitchParamsManager::~SwitchParamsManager()
{
}

//we'll load our default recovery behaviors here
void SwitchParamsManager::loadDefaultRecoveryBehaviors()
{
  recovery_behaviors_->clear();
  recovery_behaviors_carrying_->clear();
  try{
    //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
    ros::NodeHandle n("~");
    n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
    n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

    for (int i = 0; i < this->outer_loop_recovery_count_; i++)
    {
      if (conservative_clearing_map_allowed_)
      {
        addRecoveryBehavior("conservative_reset", recovery_behaviors_, recovery_behavior_names_);
        addRecoveryBehavior("conservative_reset", recovery_behaviors_carrying_, recovery_behavior_names_carrying_);
      }
      if (aggressive_clearing_map_allowed_)
      {
        addRecoveryBehavior("aggressive_reset", recovery_behaviors_, recovery_behavior_names_);
        addRecoveryBehavior("aggressive_reset", recovery_behaviors_carrying_, recovery_behavior_names_carrying_);
      }

      for (int j=0; j < this->inner_loop_recovery_count_; j++)
      {
        addRecoveryBehavior("safety_direction_recovery", recovery_behaviors_, recovery_behavior_names_);
        if (use_safety_direction_recovery_in_towing_)
        {
          addRecoveryBehavior("safety_direction_recovery", recovery_behaviors_carrying_, recovery_behavior_names_carrying_);
        }
        else
        {
          if (backward_recovery_allowed_)
          {
            addRecoveryBehavior("go_back_recovery", recovery_behaviors_, recovery_behavior_names_);
          }

          if (clearing_rotation_allowed_ && rotate_small_angle_ != 0.0)
          {
            addRecoveryBehavior("rotate_small_recovery", recovery_behaviors_,recovery_behavior_names_);
          }
        }
      }
      if (clearing_rotation_allowed_)
      {
        addRecoveryBehavior("rotate_recovery", recovery_behaviors_,recovery_behavior_names_);
        if (use_rotate_recovery_in_towing_)
        {
          addRecoveryBehavior("rotate_recovery", recovery_behaviors_carrying_,recovery_behavior_names_carrying_);
        }
      }

      if (remove_virtual_obstacle_recovery_allowed_)
      {
        addRecoveryBehavior("remove_virtual_obstacle_recovery", recovery_behaviors_,recovery_behavior_names_);
        addRecoveryBehavior("remove_virtual_obstacle_recovery", recovery_behaviors_carrying_,recovery_behavior_names_carrying_);
      }
    }

  }
  catch(pluginlib::PluginlibException& ex){
    ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
  }

  return;
}

bool SwitchParamsManager::loadRecoveryBehaviors() 
{
  XmlRpc::XmlRpcValue behavior_list;
  if (!nh_.getParam("recovery_behaviors", behavior_list))
  {
    ROS_ERROR_STREAM("Failed to get 'recovery_behaviors' parameter.");
    return false;
  }
  
  if (loadRecoveryBehavior("recovery_behaviors", "recovery_loop_count", recovery_behaviors_))
    return false;
  
  if (loadRecoveryBehavior("recovery_behaviors_carrying", "recovery_loop_count_carrying", recovery_behaviors_carrying_))
    return false;

  return true;
}

bool SwitchParamsManager::loadRecoveryBehavior(const std::string & field_name, 
                                              const std::string & loop_count_str, 
                                              boost::shared_ptr<std::vector<BehPtr>> behaviors) 
{
  XmlRpc::XmlRpcValue behavior_list;
  if (!nh_.getParam(field_name, behavior_list))
  {
    ROS_ERROR_STREAM("Failed to get " << field_name << " parameter.");
    return false;
  }

  if (behavior_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_STREAM("Recovery behaviors should be specified as a list.");
    return false;
  }

  int recovery_loop_count;
  nh_.param(loop_count_str, recovery_loop_count, 1);

  for (int i = 0; i < recovery_loop_count; i++)
  {
    int behavior_index = 0;
    if (!createRecoveryBehaviors(behavior_list, behaviors))
    {
      recovery_behaviors_->clear(); //TODO replace ?
      recovery_behaviors_carrying_->clear();
      return false;
    }
  }
}

bool SwitchParamsManager::createRecoveryBehaviors( XmlRpc::XmlRpcValue behavior_list,
                                                  boost::shared_ptr<std::vector<BehPtr>> behaviors)
{
  for (auto & current_behavior : behavior_list)
  {
    if (current_behavior.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Each recovery behavior must be a struct.");
      continue;
    }

    std::string type = static_cast<std::string>(current_behavior["type"]);

    if (behavior_definitions_.find(type) != behavior_definitions_.end())
    {
        ROS_INFO("Adding behavior '%s' of type '%s'", name.c_str(), behavior_definitions_[type].c_str());
        behaviors.push_back(type);
    }
    else
    {
      ROS_WARN("Unknown recovery behavior type: %s", type.c_str());
      return false;
    }
  }
  return true;
}

void SwitchParamsManager::carryingStatusCB(const lexxauto_msgs::ActuatorStatus::ConstPtr& msg)
{
  ReloadRecoveryBehavior(msg->connect);
}

void SwitchParamsManager::CarryingInfoCB(const lexxauto_msgs::CarryingInformation::ConstPtr& msg)
{ 
  ReloadRecoveryBehavior(msg->weight_applied);
}

void SwitchParamsManager::ReloadRecoveryBehavior(const bool& is_carrying)
{
  auto recovery_list = is_carrying ? recovery_behaviors_carrying_ : recovery_behaviors_;

  XmlRpc::XmlRpcValue recovery_behaviors_param;
  XmlRpc::XmlRpcValue current_recovery_param;

  for(auto & recovery : recovery_list)
  {
    current_recovery_param["type"] = recovery; 
    current_recovery_param["name"] =  behavior_definitions_[recovery];
    recovery_behaviors_param.push_back(recovery_behaviors_param);
  }

  nh_.setParam("move_base/recovery_behaviors",recovery_behaviors_param);

  ROS_INFO("reload recovery behavior parameters"); 
}

}  // namespace carrying_manager
