#ifndef ARGO_MOVE_GROUP_H_
#define ARGO_MOVE_GROUP_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <argo_move_group/camera_pose.h>
#include <argo_move_group/workspace_grid.h>
#include <argo_move_group_msgs/ArgoCombinedPlanAction.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/Constraints.h>
#include <nav_msgs/OccupancyGrid.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_proc/grid_map_polygon_tools.h>

namespace argo_move_group {

class ArgoMoveGroupBasePlanner : public move_group::MoveGroupCapability
{
public:
    ArgoMoveGroupBasePlanner();

    void initialize();

private:
    // --- callbacks for subscribed topics and actions ---
    void combinedPlanActionCB(const argo_move_group_msgs::ArgoCombinedPlanGoalConstPtr &msg);

    void dynamicMapCB(const nav_msgs::OccupancyGrid &msg);

    void sampleOnlyCB(const ObjectTypeParams &params, argo_move_group_msgs::ArgoCombinedPlanResult &result);
    void armPlanRequestCB(const ObjectTypeParams &params, argo_move_group_msgs::ArgoCombinedPlanResult &result);
    void armMoveRequestCB(const ObjectTypeParams &params, argo_move_group_msgs::ArgoCombinedPlanResult &result);
    void basePlanRequestCB(const argo_move_group_msgs::ArgoCombinedPlanGoalConstPtr &request, argo_move_group_msgs::ArgoCombinedPlanResult &result);

    // --- functions ---

    bool sampleCameraPoses(const Eigen::Affine3d &target, ObjectTypeParams params, size_t max_num_samples, bool do_ik = true, ros::Duration max_time = ros::Duration(5.0));

    bool stateCheckerFN(moveit::core::RobotState *robot_state, const moveit::core::JointModelGroup *joint_group, const double *joint_group_variable_values);

    void clearTargetArea(Eigen::Affine3d target, Eigen::Vector3d margin);

    bool castRay(const boost::shared_ptr<const octomap::OcTree> &octree, const Eigen::Vector3d &origin, const Eigen::Vector3d &target);

    bool planUsingPlanningPipeline(const planning_interface::MotionPlanRequest &req,
                                   plan_execution::ExecutableMotionPlan &plan);

    void readObjectTypes();
    ObjectTypeParams getParams(std::string object_type, std::string object_id);

    // --- class member variables ---
    ros::NodeHandle nh_combined_planner_;

    ros::Publisher baseBaseGridPub_;
    ros::Publisher cameraPoseesPub_;
    ros::Publisher dbgPosePub_;
    ros::Publisher dbgMarkerPub_;
    ros::Publisher jointStatePub_;

    ros::Subscriber dynamicMapSub_;

    boost::scoped_ptr<actionlib::SimpleActionServer<argo_move_group_msgs::ArgoCombinedPlanAction> > armPlanMoveServer_;

    moveit::core::GroupStateValidityCallbackFn stateCheckerCB;

    planning_scene::PlanningScenePtr scene_;
    random_numbers::RandomNumberGenerator rand_;

    std::vector<CameraPose> samples_;

    moveit_msgs::Constraints constraints_;
    visualization_msgs::Marker marker_;

    std::string cam_frame_;
    ObjectTypeParamsMap object_type_parms_;

    WorkspaceGridMapPtr wsGridMap_;

    Eigen::Affine3d target_;

}; // class ArgoMoveGroupBasePlanner



} // namespace argo_move_group

#endif // ARGO_MOVE_GROUP_H_
