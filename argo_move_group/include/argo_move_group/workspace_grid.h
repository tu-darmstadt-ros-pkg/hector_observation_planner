#ifndef _WORKSPACE_GRID_H_
#define _WORKSPACE_GRID_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <argo_move_group/camera_pose.h>
#include <hector_simox_ros/WorkspaceRepresentationROS.h>

namespace argo_move_group {

class WorkspaceGridMap
{
public:
    static const std::string BASE_POSE_LAYER;
    static const std::string OCCUPANCY_LAYER;
    static const std::string REACH_COST_LAYER;

    WorkspaceGridMap(std::string wsNS, const Eigen::Vector2d& footprint);
    virtual ~WorkspaceGridMap();

    void dynamicOccupancyUpdate(const nav_msgs::OccupancyGrid &updateMsg);

    void computeReachCostMap(const Eigen::Vector2d &origin);

    void fillData(const Eigen::Affine3d &eefPoseGlobal, double yaw, bool freeRollAngle = false);
    void fillData(const std::vector<Eigen::Affine3d> &eefPoseList, double yaw, bool freeRollAngle = false);

    void fillData(const std::vector<CameraPose> &cameraPoseList, double yaw, double height, bool freeRollAngle = false);

    float getValue(const Eigen::Affine3d& poseGlobal) const;
    float getValue(float x, float y) const;

    float getMaxPosition(Eigen::Vector2d &pos);

    nav_msgs::OccupancyGrid getGridMsg(std::string layer);

private:
    VirtualRobot::WorkspaceRepresentationROSPtr ws_;

    grid_map::GridMap map_;
    grid_map::Polygon footprint_;
    grid_map::Index maxIdx_;
    float maxVal_;

};

typedef boost::shared_ptr<WorkspaceGridMap> WorkspaceGridMapPtr;

}

#endif // _ROS_WORKSPACE_GRID_H_
