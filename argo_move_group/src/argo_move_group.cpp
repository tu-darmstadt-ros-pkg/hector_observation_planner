#include <argo_move_group/argo_move_group.h>

#include <Eigen/Geometry>
#include <boost/foreach.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/VisibilityConstraint.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <random_numbers/random_numbers.h>

using namespace argo_move_group_msgs;

namespace argo_move_group {
    bool stateCheckerFn(planning_scene::PlanningSceneConstPtr &scene, const moveit_msgs::Constraints &constraints,
            moveit::core::RobotState *robot_state, const moveit::core::JointModelGroup *joint_group, const double *joint_group_variable_values)
    {
        robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
        robot_state->update();
        bool collision = scene->isStateColliding(*robot_state, joint_group->getName());
        bool constrained = scene->isStateConstrained(*robot_state, constraints);
        //ROS_INFO_STREAM("state checker: " << (collision ? "C" : " ") << (constrained ? " " : "V") );
        return !collision && constrained;
    }
}

argo_move_group::ArgoMoveGroupBasePlanner::ArgoMoveGroupBasePlanner() :
        move_group::MoveGroupCapability("ArgoMoveGroupBasePlanner")
{
    // create the visibility constraint msg
    moveit_msgs::VisibilityConstraint visibility;
    visibility.target_radius = 0.1;
    visibility.cone_sides = 8;
    visibility.sensor_pose.header.frame_id = "arm_zoom_cam_optical_frame";
    visibility.sensor_pose.pose.position.z = 0.05;
    visibility.sensor_pose.pose.orientation.w = 1;
    //visibility.max_view_angle = 2 * M_PI; // ignored, defined by samples
    visibility.max_range_angle = M_PI * 5.0 / 180.0;
    visibility.sensor_view_direction = moveit_msgs::VisibilityConstraint::SENSOR_Z;
    visibility.weight = 1.0;
    constraints_.visibility_constraints.push_back(visibility);

    // create the debug marker msg
    marker_.ns = "visibility_ray";
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.pose.orientation.w = 1;
    marker_.scale.x = 0.025;
    marker_.scale.y = 0.025;
    marker_.scale.z = 0.025;
    marker_.color.r = 1.0;
    marker_.color.g = 0.0;
    marker_.color.b = 0.0;
    marker_.color.a = 1.0;
    //marker_.lifetime = ros::Duration(120.0);
}

void argo_move_group::ArgoMoveGroupBasePlanner::initialize()
{
    ROS_INFO("Initialize ArgoMoveGroupBasePlanner");

    // --- initialize all the ROS stuff
    nh_param_ = ros::NodeHandle("/combined_planner");

    armPlanRequestSub_ = node_handle_.subscribe("/move_group/arm_plan_request",
            1, &ArgoMoveGroupBasePlanner::armPlanRequestCB, this);
    basePlanRequestSub_ = node_handle_.subscribe("/move_group/base_plan_request",
            1, &ArgoMoveGroupBasePlanner::basePlanRequestCB, this);

    debugPosePub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/debug/arm_planner", 0);
    markerPub_ = node_handle_.advertise<visualization_msgs::Marker>("/visibility_ray", 0);

    actionlib::SimpleActionServer<ArgoArmPlanMoveAction>::ExecuteCallback cb =
            boost::bind(&ArgoMoveGroupBasePlanner::armPlanRequestCB, this, _1);

    armPlanMoveServer_.reset(new actionlib::SimpleActionServer<ArgoArmPlanMoveAction>(
                                 node_handle_, "/arm_plan_move_action", cb, false) );
    armPlanMoveServer_->start();

    readCheckpointParams();


    // --- initialize the private stuff
    constraints_.visibility_constraints.begin()->target_pose.header.frame_id = context_->planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
    marker_.header.frame_id = context_->planning_scene_monitor_->getPlanningScene()->getPlanningFrame();
}

void argo_move_group::ArgoMoveGroupBasePlanner::armPlanRequestCB(const ArgoArmPlanMoveGoalConstPtr &msg)
{
    ArgoArmPlanMoveResult result;

    planning_scene::PlanningScenePtr scene(context_->planning_scene_monitor_->getPlanningScene());
    Eigen::Affine3d target;

    CheckpointParams params = getParams(msg->object_type, msg->object_id.data);
    ROS_INFO_STREAM("Using sampling parameters: view angle " << params.angle_x_high << " dist " << params.dist_min << " " << params.dist_max);

    std::vector<Eigen::Affine3d> samples;
    std::vector<boost::shared_array<double> > joint_positions;

    { // Begin planning lock
    planning_scene_monitor::LockedPlanningSceneRW l_scene(context_->planning_scene_monitor_);
    ROS_INFO_STREAM("ArmPlanRequest received:");

    // get target pose in planning frame
    geometry_msgs::PoseStamped target_msg = msg->target;
    tf::poseMsgToEigen(target_msg.pose, target);
//    if (target_msg.header.frame_id.compare(scene->getPlanningFrame()))
//    {
//        if (scene->knowsFrameTransform(target_msg.header.frame_id))
//        {
//            target = scene->getFrameTransform(target_msg.header.frame_id) * target;
//            target_msg.header.frame_id = scene->getPlanningFrame();
//            tf::poseEigenToMsg(target, target_msg.pose);
//        }
//        else
//        {
//            ROS_ERROR_STREAM("Unknown reference frame: " << target_msg.header.frame_id);
//            return;
//        }
//    }    
//    debugPosePub_.publish(target_msg);

    target = target * Eigen::Translation3d(Eigen::Vector3d(0,0,0.1));
    tf::poseEigenToMsg(target, target_msg.pose);
    debugPosePub_.publish(target_msg);

    // clean target area in order to get a chance for positive rayCasts and visibilityConstraint checks
    // TODO: get the size of the area to clean from request or parameter
    //scene->getWorldNonConst()->addToObject("target_object", shapes::ShapeConstPtr(new shapes::Cylinder(0.075, 0.01)), target);
    //clearTargetArea(target, Eigen::Vector3d(0.076,0.076,0.051));

    sampleCameraPoses(target, params, 25, samples, &joint_positions);
    if (!samples.size() && params.from_back)
    {
        Eigen::Affine3d mirrored = target * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
        tf::poseEigenToMsg(mirrored, target_msg.pose);
        debugPosePub_.publish(target_msg);

        sampleCameraPoses(mirrored, params, 25, samples, &joint_positions);
    }


    ROS_INFO("End of planning part");
    } // End  planning lock

    if (!samples.size())
    {
        result.success = ArgoArmPlanMoveResult::SAMPLING_FAILED;
        armPlanMoveServer_->setAborted(result);
        return;
    }

    // create motion plan request
    planning_interface::MotionPlanRequest plan_req;
    plan_req.group_name = params.group;

    moveit_msgs::JointConstraint joint;
    joint.tolerance_above = 0.01;
    joint.tolerance_below = 0.01;
    joint.weight = 1.0;

    const std::vector<std::string> &joint_names = scene->getCurrentState().getJointModelGroup(params.group)->getActiveJointModelNames();
    BOOST_FOREACH(boost::shared_array<double> & q, joint_positions)
    {
        //std::stringstream ss;
        //ss << "q: ";
        for(int i = 0; i < joint_names.size(); i++)
        {
            joint.position = q[i];
            joint.joint_name = joint_names[i];
            constraints_.joint_constraints.push_back(joint);
            //ss << q[i] << "; ";
        }
        //ROS_INFO_STREAM(ss.str());
        plan_req.goal_constraints.push_back(constraints_);
        constraints_.joint_constraints.clear();
    }

    // initialize motion planner
    plan_execution::ExecutableMotionPlan plan;
    plan_execution::PlanExecution::Options opt;

    opt.replan_ = true;
    opt.replan_attempts_ = 10;
    opt.replan_delay_ = 0.01;
    opt.plan_callback_ = boost::bind(&ArgoMoveGroupBasePlanner::planUsingPlanningPipeline, this, boost::cref(plan_req), _1);
//    if (context_->plan_with_sensing_)
//    {
//        ROS_INFO_STREAM("Use plan with sensing!");
//        opt.plan_callback_ = boost::bind(&plan_execution::PlanWithSensing::computePlan, context_->plan_with_sensing_.get(), _1, opt.plan_callback_, 10, 0);
//    }

    // drive camera to desired pose
    context_->plan_execution_->planAndExecute(plan, opt);

    // clear target object
//    {
//        planning_scene_monitor::LockedPlanningSceneRW l_scene(context_->planning_scene_monitor_);
//        scene->getWorldNonConst()->removeObject("target_object");
//    }


    // get result
    switch (plan.error_code_.val) {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
        result.success = ArgoArmPlanMoveResult::SUCCESS;
        break;
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        result.success = ArgoArmPlanMoveResult::PLANNING_FAILED;
        break;
    default:
        result.success = ArgoArmPlanMoveResult::FAILED;
        break;
    }

    armPlanMoveServer_->setSucceeded(result);
}


void argo_move_group::ArgoMoveGroupBasePlanner::basePlanRequestCB(const geometry_msgs::PoseStampedConstPtr &msg)
{
    ROS_INFO("basePlanRequest received:");
    ROS_WARN("Base planner not implemented yet!");

    // sample camera poses
    // FOREACH sample
    //   cut IRM with floor and get manipulability values
    //   compute costmap from manipulability values and camera pose value
    // join costmaps of all samples --> base placement costmap
    // join move_base and base_placement costmap
    // choose new robot base pose
}

/*--- private functions ---*/


bool argo_move_group::ArgoMoveGroupBasePlanner::sampleCameraPoses(const Eigen::Affine3d &target, const CheckpointParams &params,
                                                             size_t max_num_samples, std::vector<Eigen::Affine3d> &samples,
                                                             std::vector<boost::shared_array<double> > *joint_positions, ros::Duration timeout)
{
    planning_scene::PlanningSceneConstPtr scene = context_->planning_scene_monitor_->getPlanningScene();

    // get octomap
    collision_detection::World::ObjectConstPtr octomap =scene->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
    if (octomap == NULL)
    {
        ROS_ERROR_STREAM("Octomap object could not be retrieved from planning scene.");
        return false;
    }
    boost::shared_ptr<const octomap::OcTree> octree(dynamic_cast<const shapes::OcTree *>(octomap->shapes_[0].get())->octree);

    moveit::core::RobotState state(scene->getCurrentState());
    // TODO: set camera link in visibility constraint.
    tf::poseEigenToMsg(target, constraints_.visibility_constraints.begin()->target_pose.pose);
    stateCheckerCB = boost::bind(stateCheckerFn, scene, constraints_, _1, _2, _3);
    kinematics::KinematicsQueryOptions opt;

    ros::Time _timeout = ros::Time::now() + timeout;
    while (samples.size() < max_num_samples && ros::Time::now() < _timeout)
    {
        // generate random sample
        double dist = rand_.uniformReal(params.dist_min, params.dist_max);
        double angle_horizontal = rand_.uniformReal(params.angle_x_low, params.angle_x_high);
        double angle_vertical = rand_.uniformReal(params.angle_y_low, params.angle_y_high);

        // compute camera position from sample
        Eigen::Affine3d sample_pose = target *
                Eigen::AngleAxisd(angle_horizontal, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(angle_vertical, Eigen::Vector3d::UnitY()) *
                Eigen::Translation3d(Eigen::Vector3d(0,0,dist)) *
                Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY());
        opt.return_approximate_solution = true;

        // coarse check for visibility
        if (castRay(octree, target.translation(), sample_pose.translation()))
        {
            state = scene->getCurrentState();
            // fine check for IK solution with collision and visibility constraints
            if (state.setFromIK(state.getJointModelGroup(params.group), sample_pose, 0, 0.005, stateCheckerCB, opt))
            {
                // rate this sample
//                double a = std::sqrt(std::cos(angle_horizontal) * std::cos(angle_horizontal) +
//                                     std::cos(angle_vertical) * std::cos(angle_vertical));
                double a = std::fabs(angle_horizontal) / std::max(std::fabs(params.angle_x_high), std::fabs(params.angle_x_low)) +
                        std::fabs(angle_vertical) / std::max(std::fabs(params.angle_y_high), std::fabs(params.angle_y_low));
                double d_j = state.distance(scene->getCurrentState());
                double d_c = scene->distanceToCollision(state);
//                double value = (2 - a) * d_j;
                double value = a * d_j;

                ROS_INFO_STREAM("Angle: " << a << " Distance Joints: " << d_j << " Distance Collision: " << d_c << " => " << value);

                // add valid camera pose to samples
                samples.push_back(sample_pose);
                // add debug marker to message
                geometry_msgs::Point tmp;
                tf::pointEigenToMsg(sample_pose.translation(), tmp);
                marker_.points.push_back(tmp);
                std_msgs::ColorRGBA col;
                col.a = 1.0;
                col.r = std::max(0.0, (15.0 - value)) / 15.0;
                marker_.colors.push_back(col);


                // add IK solution to output if requested
                if (joint_positions)
                {
                    //store joint positions for this solution
                    const std::vector<std::string> &joints = state.getJointModelGroup(params.group)->getActiveJointModelNames();
                    boost::shared_array<double> positions(new double[joints.size()]);
                    double *p = positions.get();
                    for (std::vector<std::string>::const_iterator i = joints.begin(); i != joints.end(); i++, p++)
                    {
                        *p = *state.getJointPositions(*i);
                    }
                    joint_positions->push_back(positions);
                }
            }
        }
    } // END sampling loop

    ROS_INFO_STREAM("Computed " << samples.size() << " samples in " << ros::Time::now() - _timeout + timeout << "seconds.");

    // send debug markers message
    markerPub_.publish(marker_);
    marker_.points.clear();
    marker_.colors.clear();

    return !samples.empty();
}

void argo_move_group::ArgoMoveGroupBasePlanner::clearTargetArea(Eigen::Affine3d target, Eigen::Vector3d margin)
{
    // get the current octomap
    moveit_msgs::PlanningScene scene_msg;
    moveit_msgs::PlanningSceneComponents pls_comps;
    pls_comps.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
    context_->planning_scene_monitor_->getPlanningScene()->getPlanningSceneMsg(scene_msg, pls_comps);
    boost::scoped_ptr<octomap::OcTree> octree(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(scene_msg.world.octomap.octomap)));

    // clean target area
    if (octree->size())
    {
        Eigen::Vector3d tmp = target.linear() * margin;
        octomap::point3d margin(std::abs(tmp[0]), std::abs(tmp[1]), std::abs(tmp[2]));
        octomap::point3d pos(target.translation()[0], target.translation()[1], target.translation()[2]);
        octomap::OcTree::leaf_bbx_iterator it, end;
        for (it = octree->begin_leafs_bbx(pos-margin, pos+margin), end = octree->end_leafs_bbx(); it!= end; ++it)
        {
            octree->updateNode(it.getKey(), -10000000000000.0f, true);
        }
        octree->updateInnerOccupancy();

        octomap_msgs::Octomap octree_msg;
        octomap_msgs::binaryMapToMsg(*octree, octree_msg);
        context_->planning_scene_monitor_->getPlanningScene()->processOctomapMsg(octree_msg);
    }
}

//double cameraPoseValue(const Affine3dPtr &cp, const Eigen::Vector3d &target)
//{
//    // compute rating of CP for the generating parameters
//    double dist_value = (cp->getDist() < 1.0) ? 1.0 : (cpf_.getMaxDist() - cp->getDist()) / cpf_.getMaxDist();
//    //double angle_value = (cpf_.getMaxViewAngle() - cp->getViewAngle()) / cpf_.getMaxViewAngle();
//    double angle_value = std::cos(cp->getViewAngle());

//    // compute rating of CP according to distance to the octomap
//    octomap::point3d cam, tgt;
//    tf::pointEigenToOctomap(cp->translation(), cam);
//    tf::pointEigenToOctomap(target, tgt);
//    // get ray
//    octomap::KeyRay keys;
//    octree_->computeRayKeys(cam, tgt, keys);

//    bool occ;
//    double oc_dist_value = 1.0;
//    BOOST_FOREACH(octomap::OcTreeKey k, keys)
//    {
//        if ((tgt-octree_->keyToCoord(k)).norm() < cpf_.getMinDist())
//        {
//            // finished
//            break;
//        }

//        for (size_t i = 1; i <= 3; i++)
//        {
//            octomap::OcTreeNode *node = octree_->search(k, octree_->getTreeDepth()-i);
//            occ = node ? octree_->isNodeOccupied(*node) : false;
//            if (occ)
//            {
//                oc_dist_value = 0.1 * i;
//                break;
//            }
//        }

//    }
//    ROS_INFO_STREAM("(" << cam.x() << " " << cam.y() << " " << cam.z() << "): D: "
//                    << dist_value << " A: " << angle_value << " O: " << oc_dist_value << " ==> " << dist_value * angle_value * oc_dist_value);
//    return dist_value * angle_value * oc_dist_value;
//}

bool argo_move_group::ArgoMoveGroupBasePlanner::castRay(const boost::shared_ptr<const octomap::OcTree> &octree,
                                                   const Eigen::Vector3d &origin, const Eigen::Vector3d &target)
{
    octomap::point3d ori(origin[0], origin[1], origin[2]);
    octomap::point3d dir(target[0]-origin[0], target[1]-origin[1], target[2]-origin[2]);
    octomap::point3d end;

    return !octree->castRay(ori, dir, end, true, dir.norm());
}

bool argo_move_group::ArgoMoveGroupBasePlanner::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest &req, plan_execution::ExecutableMotionPlan &plan)
{
  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
  bool solved = false;
  planning_interface::MotionPlanResponse res;
  try
  {
    solved = context_->planning_pipeline_->generatePlan(plan.planning_scene_, req, res);
  }
  catch(std::runtime_error &ex)
  {
    ROS_ERROR("Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  catch(...)
  {
    ROS_ERROR("Planning pipeline threw an exception");
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan.plan_components_.resize(1);
    plan.plan_components_[0].trajectory_ = res.trajectory_;
    plan.plan_components_[0].description_ = "plan";
  }
  plan.error_code_ = res.error_code_;
  return solved;
}

void argo_move_group::ArgoMoveGroupBasePlanner::readCheckpointParams()
{
    CheckpointParams params;
    // default params
    params.angle_x_high = 30.0 * M_PI/180.0;
    params.angle_x_low = -30.0 * M_PI/180.0;
    params.angle_y_high = 30.0 * M_PI/180.0;
    params.angle_y_low = -30.0 * M_PI/180.0;
    params.dist_min = 0.5;
    params.dist_max = 3.5;
    params.from_back = false;
    params.group = "arm_group";
    checkpoint_parms_[DEFAULT_OBJECT_TYPE] = params;


    typedef std::map<std::string, u_int8_t> _type_map;
    _type_map types = {{"dial_gauge",ArgoArmPlanMoveGoal::DIAL_GAUGE}, {"hotspot",ArgoArmPlanMoveGoal::HOTSPOT},
                       {"level_gauge",ArgoArmPlanMoveGoal::LEVEL_GAUGE}, {"valve",ArgoArmPlanMoveGoal::VALVE},
                       {"unknown",ArgoArmPlanMoveGoal::NO_TYPE}};
    BOOST_FOREACH(_type_map::value_type t, types)
    {
        nh_param_.param(t.first+"/max_angle_diff/x_high", params.angle_x_high, (float)(30.0 * M_PI/180.0));
        nh_param_.param(t.first+"/max_angle_diff/x_low", params.angle_x_low, (float)(30.0 * M_PI/180.0));
        nh_param_.param(t.first+"/max_angle_diff/y_high", params.angle_y_high, (float)(30.0 * M_PI/180.0));
        nh_param_.param(t.first+"/max_angle_diff/y_low", params.angle_y_low, (float)(30.0 * M_PI/180.0));
        nh_param_.param(t.first+"/min_dist", params.dist_min, 0.5F);
        nh_param_.param(t.first+"/max_dist", params.dist_max, 3.5F);
        nh_param_.param(t.first+"/from_back", params.from_back, false);
        nh_param_.param(t.first+"/joint_group", params.group, std::string("arm_group"));
        ROS_INFO_STREAM(t.first << " " << params.angle_x_high << " " << params.from_back << " " << params.group);
        checkpoint_parms_[t.second] = params;
    }
}

argo_move_group::CheckpointParams argo_move_group::ArgoMoveGroupBasePlanner::getParams(u_int8_t object_type, std::string object_id)
{
    // get Checkpoint Type default parameters
    CheckpointParamsMap::iterator params_it = checkpoint_parms_.find(object_type);
    if (params_it == checkpoint_parms_.end())
    {
        ROS_WARN_STREAM("Unknown object type " << (int)(object_type) << "! Using default params.");
        params_it = checkpoint_parms_.find(DEFAULT_OBJECT_TYPE);
    }
    CheckpointParams params = params_it->second;

    ros::NodeHandle nh("/object_tracker/objects");

    ROS_INFO_STREAM("Get param overrides from: " << nh.getNamespace()+object_id+"/view_params/y_low");
    // get overrieds for current checkoint
    if (nh.hasParam(object_id + "/view_params/x_high"))
    {
        nh.param(object_id + "/view_params/x_high", params.angle_x_high, params.angle_x_high);
    }
    if (nh.hasParam(object_id + "/view_params/x_low"))
    {
        nh.param(object_id + "/view_params/x_low", params.angle_x_low, params.angle_x_low);
    }
    if (nh.hasParam(object_id + "/view_params/y_high"))
    {
        nh.param(object_id + "/view_params/y_high", params.angle_y_high, params.angle_y_high);
    }
    if (nh.hasParam(object_id + "/view_params/y_low"))
    {
        nh.param(object_id + "/view_params/y_low", params.angle_y_low, params.angle_y_low);
    }

    if (nh.hasParam(object_id + "/view_params/min_dist"))
    {
        nh.param(object_id + "/view_params/min_dist", params.dist_min, params.dist_min);
    }
    if (nh.hasParam(object_id + "/view_params/max_dist"))
    {
        nh.param(object_id + "/view_params/max_dist", params.dist_max, params.dist_max);
    }

    if (nh.hasParam(object_id + "/view_params/from_back"))
    {
        nh.param(object_id + "/view_params/from_back", params.from_back, params.from_back);
    }

    if (nh.hasParam(object_id + "/view_params/joint_group"))
    {
        nh.param(object_id + "/view_params/joint_group", params.group, params.group);
    }

    return params;
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(argo_move_group::ArgoMoveGroupBasePlanner,
        move_group::MoveGroupCapability)
