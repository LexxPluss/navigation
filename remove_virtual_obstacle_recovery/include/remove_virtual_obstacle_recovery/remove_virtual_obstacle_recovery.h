#ifndef REMOVE_VIRTUAL_OBSTACLE_RECOVERY_H
#define REMOVE_VIRTUAL_OBSTACLE_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace remove_virtual_obstacle_recovery
{
class RemoveVirtualObstacleRecovery : public nav_core::RecoveryBehavior
{
    public:
        RemoveVirtualObstacleRecovery();

        void initialize(std::string name, tf2_ros::Buffer*,
                        costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

        void runBehavior();

        ~RemoveVirtualObstacleRecovery();
    private:
        bool initialized_;
        bool virtual_obstacle_map_received_;
        ros::Publisher virtual_obstacle_enabled_pub_;
        ros::Subscriber virtual_obstacle_map_sub_;

        void virtualObstacleMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};
};  // namespace remove_virtual_obstacle_recovery

#endif