#ifndef ARGO_MOVE_GROUP_H_
#define ARGO_MOVE_GROUP_H_

#include <argo_move_group_msgs/ArgoArmPlanMoveAction.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/String.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/Constraints.h>

namespace argo_move_group {

struct CheckpointParams
{
    float angle_x_high;
    float angle_x_low;
    float angle_y_high;
    float angle_y_low;
    float dist_min;
    float dist_max;
    bool from_back;
    std::string group;
};
const u_int8_t DEFAULT_OBJECT_TYPE = 255;
typedef std::map<u_int8_t, CheckpointParams> CheckpointParamsMap;

class ArgoMoveGroupBasePlanner : public move_group::MoveGroupCapability
{
public:
    ArgoMoveGroupBasePlanner();

    void initialize();

private:
    // --- callbacks for subscribed topics ---
    void armPlanRequestCB(const argo_move_group_msgs::ArgoArmPlanMoveGoalConstPtr &msg);
    void basePlanRequestCB(const geometry_msgs::PoseStampedConstPtr &msg);

    // --- functions ---
    moveit::core::GroupStateValidityCallbackFn stateCheckerCB;

    bool sampleCameraPoses(const Eigen::Affine3d &target, const CheckpointParams &params,
                           size_t max_num_samples, std::vector<Eigen::Affine3d> &samples,
                           std::vector< boost::shared_array<double> > *joint_positions, ros::Duration timeout = ros::Duration(5.0));

    void clearTargetArea(Eigen::Affine3d target, Eigen::Vector3d margin);

    bool castRay(const boost::shared_ptr<const octomap::OcTree> &octree, const Eigen::Vector3d &origin, const Eigen::Vector3d &target);

    bool planUsingPlanningPipeline(const planning_interface::MotionPlanRequest &req,
                                   plan_execution::ExecutableMotionPlan &plan);

    void readCheckpointParams();
    CheckpointParams getParams(u_int8_t object_type, std::string object_id);

    // --- class member variables ---
    ros::NodeHandle nh_param_;
    ros::Publisher debugPosePub_;
    ros::Publisher markerPub_;
    ros::Subscriber basePlanRequestSub_;
    ros::Subscriber armPlanRequestSub_;

    boost::scoped_ptr<actionlib::SimpleActionServer<argo_move_group_msgs::ArgoArmPlanMoveAction> > armPlanMoveServer_;

    random_numbers::RandomNumberGenerator rand_;
    moveit_msgs::Constraints constraints_;
    visualization_msgs::Marker marker_;
    CheckpointParamsMap checkpoint_parms_;

}; // class ArgoMoveGroupBasePlanner



} // namespace argo_move_group

#endif // ARGO_MOVE_GROUP_H_
