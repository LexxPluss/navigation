/*********************************************************************
 *
 *  Copyright (c) 2024, LexxPluss, Inc.
 *  All rights reserved.
 *
 *********************************************************************/

#include <recovery_manager/switch_params_manager.hpp>


namespace carrying_manager
{
SwitchParamsManager::SwitchParamsManager(ros::NodeHandle& nh, ros::NodeHandle& pnh):
  pnh_(pnh),
  nh_(nh)
{
  bool use_carrying_manager = false;
  // feature switch flag
  nh_.param<bool>("carrying_manager/use_carrying_manager", use_carrying_manager, false);

  if (use_carrying_manager)
  {
    carrying_info_sub_ = nh.subscribe("carrying_manager/carrying_info", 1, &SwitchParamsManager::carryingInfoCB, this);
  }
  else
  {
    carrying_status_sub_ = nh.subscribe<lexxauto_msgs::ActuatorStatus>("actuator_position", 1, boost::bind(&SwitchParamsManager::carryingStatusCB, this, _1));
  }

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
  int outer_loop_recovery_count, inner_loop_recovery_count;
  bool conservative_clearing_map_allowed, aggressive_clearing_map_allowed;
  bool clearing_rotation_allowed, backward_recovery_allowed, abort_after_recovery_allowed;
  bool remove_virtual_obstacle_recovery_allowed, use_safety_direction_recovery_in_towing, use_rotate_recovery_in_towing;
  double rotate_small_angle;

  recovery_behaviors_.clear();
  recovery_behaviors_carrying_.clear();
  
  nh_.param("move_base/outer_loop_recovery_count", outer_loop_recovery_count, 1);
  nh_.param("move_base/inner_loop_recovery_count", inner_loop_recovery_count, 2);
  nh_.param("move_base/conservative_clearing_map_allowed", conservative_clearing_map_allowed, false);
  nh_.param("move_base/aggressive_clearing_map_allowed", aggressive_clearing_map_allowed, false);
  nh_.param("move_base/use_safety_direction_recovery_in_towing", use_safety_direction_recovery_in_towing, true);
  nh_.param("move_base/use_rotate_recovery_in_towing", use_rotate_recovery_in_towing, true);
  nh_.param("move_base/remove_virtual_obstacle_recovery_allowed", remove_virtual_obstacle_recovery_allowed, false);
  nh_.param("move_base/backward_recovery_allowed", backward_recovery_allowed, false);
  nh_.param("move_base/clearing_rotation_allowed", clearing_rotation_allowed, true);
  nh_.param("move_base/rotate_small_angle", rotate_small_angle, 0.0);

  try{
    //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility

    // nh_.setParam("move_base/conservative_reset/reset_distance", conservative_reset_dist_); //TODO
    // nh_.setParam("move_base/aggressive_reset/reset_distance", circumscribed_radius_ * 4);

    for (int i = 0; i < outer_loop_recovery_count; i++)
    {
      if (conservative_clearing_map_allowed)
      {
        recovery_behaviors_.push_back("conservative_reset");
        recovery_behaviors_carrying_.push_back("conservative_reset");
      }
      if (aggressive_clearing_map_allowed)
      {
        recovery_behaviors_.push_back("aggressive_reset");
        recovery_behaviors_carrying_.push_back("aggressive_reset");
      }

      for (int j=0; j < inner_loop_recovery_count; j++)
      {
        recovery_behaviors_.push_back("safety_direction_recovery");
        if (use_safety_direction_recovery_in_towing)
        {
          recovery_behaviors_carrying_.push_back("safety_direction_recovery");
        }
        else
        {
          if (backward_recovery_allowed)
          {
            recovery_behaviors_.push_back("go_back_recovery");
          }

          if (clearing_rotation_allowed && rotate_small_angle != 0.0)
          {
            recovery_behaviors_.push_back("rotate_small_recovery");
          }
        }
      }
      if (clearing_rotation_allowed)
      {
        recovery_behaviors_.push_back("rotate_recovery");
        if (use_rotate_recovery_in_towing)
        {
          recovery_behaviors_carrying_.push_back("rotate_recovery");
        }
      }

      if (remove_virtual_obstacle_recovery_allowed)
      {
        recovery_behaviors_.push_back("remove_virtual_obstacle_recovery");
        recovery_behaviors_carrying_.push_back("remove_virtual_obstacle_recovery");
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
  
  if (loadGivenRecoveryBehavior("recovery_behaviors", "recovery_loop_count", recovery_behaviors_))
    return false;
  
  if (loadGivenRecoveryBehavior("recovery_behaviors_carrying", "recovery_loop_count_carrying", recovery_behaviors_carrying_))
    return false;

  return true;
}

bool SwitchParamsManager::loadGivenRecoveryBehavior(const std::string & field_name, 
                                              const std::string & loop_count_str, 
                                              std::vector<std::string> & behaviors) 
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
      behaviors.clear();
      return false;
    }
  }
}

bool SwitchParamsManager::createRecoveryBehaviors(XmlRpc::XmlRpcValue behavior_list,
                                              std::vector<std::string> & behaviors) 
{
  for (int idx = 0; idx < behavior_list.size(); idx++) // & current_behavior : behavior_list)
  {
    if (behavior_list[idx].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Each recovery behavior must be a struct.");
      continue;
    }
    std::string type = behavior_list[idx]["type"];

    if (behavior_definitions_.find(type) != behavior_definitions_.end())
    {
        ROS_INFO("Adding behavior '%s' of type '%s'",type, behavior_definitions_[type].c_str());
        behaviors.push_back(type);
    }
    else
    {
      ROS_WARN("Unknown recovery behavior type: %s", type);
      return false;
    }
  }
  return true;
}

void SwitchParamsManager::carryingStatusCB(const lexxauto_msgs::ActuatorStatus::ConstPtr& msg)
{
  reloadRecoveryBehavior(msg->connect);
}

void SwitchParamsManager::carryingInfoCB(const lexxauto_msgs::CarryingInformation::ConstPtr& msg)
{ 
  reloadRecoveryBehavior(msg->weight_applied);
}

void SwitchParamsManager::reloadRecoveryBehavior(const bool& is_carrying)
{
  auto recovery_list = is_carrying ? recovery_behaviors_carrying_ : recovery_behaviors_;

  XmlRpc::XmlRpcValue recovery_behaviors_param;
  XmlRpc::XmlRpcValue current_recovery_param;

  recovery_behaviors_param.setSize(recovery_list.size());
  int index = 0;
  for(auto & recovery : recovery_list)
  {
    current_recovery_param["type"] = recovery; 
    current_recovery_param["name"] =  behavior_definitions_[recovery];
    recovery_behaviors_param[index] = recovery_behaviors_param;
    index++;
  }

  nh_.setParam("move_base/recovery_behaviors",recovery_behaviors_param);

  ROS_INFO("reload recovery behavior parameters"); 
}

}  // namespace carrying_manager
