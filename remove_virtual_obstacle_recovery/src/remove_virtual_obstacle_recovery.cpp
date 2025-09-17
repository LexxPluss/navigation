#include <remove_virtual_obstacle_recovery/remove_virtual_obstacle_recovery.h>
#include <nav_core/parameter_magic.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(remove_virtual_obstacle_recovery::RemoveVirtualObstacleRecovery, nav_core::RecoveryBehavior)

namespace remove_virtual_obstacle_recovery{
RemoveVirtualObstacleRecovery::RemoveVirtualObstacleRecovery(): initialized_(false)
{
}

void RemoveVirtualObstacleRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
    if(!initialized_)
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~/" + name);
        this->virtual_obstacle_enabled_pub_ = nh.advertise<std_msgs::Bool>("virtual_obstacle_map/enable", 1, true);
        this->virtual_obstacle_map_sub_ = nh.subscribe("virtual_obstacle_map", 1, &RemoveVirtualObstacleRecovery::virtualObstacleMapCallback, this);

        initialized_ = true;
    }
    else
    {
        ROS_WARN("You should not call initialize twice on this object, doing nothing");
    }
}

RemoveVirtualObstacleRecovery::~RemoveVirtualObstacleRecovery()
{
}

void RemoveVirtualObstacleRecovery::virtualObstacleMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    this->virtual_obstacle_map_received_ = true;
}

void RemoveVirtualObstacleRecovery::runBehavior()
{
    std_msgs::Bool msg;
    msg.data = false;
    this->virtual_obstacle_enabled_pub_.publish(msg);

    if (this->virtual_obstacle_map_received_)
    {
        ros::Rate r(10);
        this->virtual_obstacle_map_received_ = false;
        while(ros::ok() && !this->virtual_obstacle_map_received_)
        {
            r.sleep();
        }
    }

    return;
}
};  // namespace remove_virtual_obstacle_recovery
