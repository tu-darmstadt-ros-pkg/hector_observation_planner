#include <argo_move_group/argo_move_group.h>

#include <Eigen/Geometry>
#include <boost/foreach.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/VisibilityConstraint.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <random_numbers/random_numbers.h>
#include <argo_move_group/tf_octomap_eigen.h>
#include <sensor_msgs/JointState.h>


using namespace argo_move_group;
using namespace argo_move_group_msgs;
using namespace Eigen;

ArgoMoveGroupBasePlanner::ArgoMoveGroupBasePlanner() :
        move_group::MoveGroupCapability("ArgoMoveGroupBasePlanner")
{
    stateCheckerCB = boost::bind(&ArgoMoveGroupBasePlanner::stateCheckerFN, this, _1, _2, _3);

    // create the visibility constraint msg
    moveit_msgs::VisibilityConstraint visibility;
    visibility.cone_sides = 8;
    visibility.sensor_pose.pose.position.x = 0.05;
    visibility.sensor_pose.pose.orientation.w = 1;
    visibility.max_range_angle = M_PI * 5.0 / 180.0;
    visibility.sensor_view_direction = moveit_msgs::VisibilityConstraint::SENSOR_X;
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

void ArgoMoveGroupBasePlanner::initialize()
{
    ROS_INFO("Initialize ArgoMoveGroupBasePlanner");

    // --- initialize all the ROS stuff
    nh_combined_planner_ = ros::NodeHandle("/combined_planner");

    baseBaseGridPub_ = nh_combined_planner_.advertise<nav_msgs::OccupancyGrid>("base_pose_grid", 0);
    cameraPoseesPub_ = nh_combined_planner_.advertise<geometry_msgs::PoseArray>("camera_poses", 0);
    dbgPosePub_ = nh_combined_planner_.advertise<geometry_msgs::PoseStamped>("debug_pose", 0);
    dbgMarkerPub_ = nh_combined_planner_.advertise<visualization_msgs::Marker>("debug_marker", 0);
    jointStatePub_ = nh_combined_planner_.advertise<sensor_msgs::JointState>("joint_positions", 0);

    armPlanMoveServer_.reset(new actionlib::SimpleActionServer<ArgoCombinedPlanAction>(nh_combined_planner_, "",
                                 boost::bind(&ArgoMoveGroupBasePlanner::combinedPlanActionCB, this, _1), false) );

    // --- initialize data members
    readObjectTypes();
    scene_ = context_->planning_scene_monitor_->getPlanningScene();
    constraints_.visibility_constraints.begin()->target_pose.header.frame_id = scene_->getPlanningFrame();
    nh_combined_planner_.param<std::string>("camera_frame", cam_frame_, "arm_zoom_cam_link");
    constraints_.visibility_constraints.begin()->sensor_pose.header.frame_id = cam_frame_;
    marker_.header.frame_id = scene_->getPlanningFrame();

    double footprint_x, footprint_y;
    root_node_handle_.param("map_combiner_node/footprint_x", footprint_x, 0.4);
    root_node_handle_.param("map_combiner_node/footprint_y", footprint_y, 0.3);
    wsGridMap_.reset(new WorkspaceGridMap(nh_combined_planner_.getNamespace()+"/simox", Vector2d(footprint_x+0.15, footprint_y+0.025)));

    // --- start ROS action server and subscribers
    armPlanMoveServer_->start();
    dynamicMapSub_ = node_handle_.subscribe("/dynamic_map", 1, &WorkspaceGridMap::dynamicOccupancyUpdate, wsGridMap_);
}

void ArgoMoveGroupBasePlanner::combinedPlanActionCB(const ArgoCombinedPlanGoalConstPtr &request)
{
    ArgoCombinedPlanResult result;
    result.success.val = ErrorCodes::FAILED;

    // get target pose in planning frame
    geometry_msgs::PoseStamped target_msg = request->target;
    tf::poseMsgToEigen(target_msg.pose, target_);
    if (target_msg.header.frame_id[0] != '/')
    {
        target_msg.header.frame_id = "/" + target_msg.header.frame_id;
    }
    if (target_msg.header.frame_id.compare(scene_->getPlanningFrame()))
    {
        if (scene_->knowsFrameTransform(target_msg.header.frame_id))
        {
            ROS_INFO_STREAM("Transform target into planning frame: target-frame: " <<
                            target_msg.header.frame_id << " planning-frame: " << scene_->getPlanningFrame());
            target_ = scene_->getFrameTransform(target_msg.header.frame_id) * target_;
            target_msg.header.frame_id = scene_->getPlanningFrame();
        }
        else
        {
            ROS_ERROR_STREAM("Unknown reference frame: " << target_msg.header.frame_id);
            return;
        }
    }
    tf::poseEigenToMsg(target_, target_msg.pose);
    dbgPosePub_.publish(target_msg);

    // get params for current checkpoint
    ObjectTypeParams params = getParams(request->object_type.data, request->object_id.data);

    ROS_INFO_STREAM("Argo CombinedPlan Request received:" << " action: " << 0 + request->action_type.val << " object: " << request->object_id.data << " " << request->object_type.data << std::endl
                    << "   target p: (" << target_msg.pose.position.x << " " << target_msg.pose.position.y << " " << target_msg.pose.position.z << ")" << std::endl
                    << "          m: (" << target_msg.pose.orientation.x << " " << target_msg.pose.orientation.y << " " << target_msg.pose.orientation.z << " " << target_msg.pose.orientation.w << ")");


    // perform requested action
    switch (request->action_type.val) {
    case ActionCodes::SAMPLE:
        sampleOnlyCB(params, result);
        break;

    case ActionCodes::PLAN_BASE:
        basePlanRequestCB(request, result);
        break;

    case ActionCodes::SAMPLE_MOVE_ARM:
        armPlanRequestCB(params, result);
        break;

    case ActionCodes::MOVE_ARM:
        armMoveRequestCB(params, result);
        break;

    default:
        break;
    }

    armPlanMoveServer_->setSucceeded(result);
}

void ArgoMoveGroupBasePlanner::armPlanRequestCB(const ObjectTypeParams &params, ArgoCombinedPlanResult &result)
{
    ROS_INFO_STREAM("ArmPlanRequest received:");

    { // Begin planning lock
        planning_scene_monitor::LockedPlanningSceneRW l_scene(context_->planning_scene_monitor_);

        sampleCameraPoses(target_, params, 25, true);

        ROS_INFO("End of planning part");
    } // End  planning lock

    if (!samples_.size())
    {
        result.success.val = ErrorCodes::SAMPLING_FAILED;
        return;
    }

    armMoveRequestCB(params, result);
}

void ArgoMoveGroupBasePlanner::armMoveRequestCB(const ObjectTypeParams &params, ArgoCombinedPlanResult &result)
{
    // create motion plan request
    planning_interface::MotionPlanRequest plan_req;
    plan_req.group_name = params.group;

    moveit_msgs::JointConstraint joint;
    joint.tolerance_above = 0.01;
    joint.tolerance_below = 0.01;
    joint.weight = 1.0;

    auto& s = samples_.back();
    ROS_INFO("%f", s.getValue());
    for(auto &q : s.getJointValues())
    {
        joint.joint_name = q.first;
        joint.position = q.second;
        constraints_.joint_constraints.push_back(joint);
    }
    //ROS_INFO_STREAM(ss.str());
    plan_req.goal_constraints.push_back(constraints_);
    constraints_.joint_constraints.clear();
    samples_.pop_back();

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

    // get result
    switch (plan.error_code_.val)
    {
    case moveit_msgs::MoveItErrorCodes::SUCCESS:
        result.success.val = ErrorCodes::SUCCESS;
        break;
    case moveit_msgs::MoveItErrorCodes::PLANNING_FAILED:
    case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
    case moveit_msgs::MoveItErrorCodes::INVALID_MOTION_PLAN:
        result.success.val = ErrorCodes::PLANNING_FAILED;
        break;
    default:
        result.success.val = ErrorCodes::FAILED;
        break;
    }
}

void ArgoMoveGroupBasePlanner::basePlanRequestCB(const ArgoCombinedPlanGoalConstPtr &request, ArgoCombinedPlanResult &result)
{
    ROS_INFO("basePlanRequest received: yaw %f", request->base_yaw);

    // get params for current checkpoint
    ObjectTypeParams params = getParams(request->object_type.data, request->object_id.data);

    // sample camera poses
    { // planning scene lock only for sampling
        planning_scene_monitor::LockedPlanningSceneRO lScene(context_->planning_scene_monitor_);
        sampleCameraPoses(target_, params, 1000, false);
    }

    // compute reachable area for current robot base pose
    Affine3d base_pose = scene_->getFrameTransform("base_link");
    wsGridMap_->computeReachCostMap(grid_map::Position(base_pose.translation()(0), base_pose.translation()(1)));
    baseBaseGridPub_.publish(wsGridMap_->getGridMsg(WorkspaceGridMap::REACH_COST_LAYER));

    // fill GridMap with Base Pose Quality Values
    wsGridMap_->fillData(samples_, request->base_yaw, base_pose.translation()(2), true);
    // visualize base pose map
    baseBaseGridPub_.publish(wsGridMap_->getGridMsg(WorkspaceGridMap::BASE_POSE_LAYER));

    // get Max Pose
    grid_map::Position maxP;
    if (wsGridMap_->getMaxPosition(maxP) < 0)
    {
        ROS_WARN("Could not find a reachable Base Position for target.");
        result.success.val = ErrorCodes::PLANNING_FAILED;
        return;
    }

    Affine3d maxPose(Translation3d(Vector3d(maxP[0], maxP[1], base_pose.translation()(2))) * AngleAxisd(request->base_yaw, Vector3d::UnitZ()));
    result.waypoint.header.frame_id = scene_->getPlanningFrame();
    tf::poseEigenToMsg(maxPose, result.waypoint.pose);
    result.success.val = ErrorCodes::SUCCESS;
}

void ArgoMoveGroupBasePlanner::sampleOnlyCB(const ObjectTypeParams &params, ArgoCombinedPlanResult &result)
{
    // sample camera poses
    { // planning scene lock only for sampling
        planning_scene_monitor::LockedPlanningSceneRO lScene(context_->planning_scene_monitor_);
        sampleCameraPoses(target_, params, 50);
    }

    if (!samples_.size())
    {
        result.success.val = ErrorCodes::SAMPLING_FAILED;
        return;
    }

    result.success.val = ErrorCodes::SUCCESS;

    if (jointStatePub_.getNumSubscribers())
    {
        for (auto &s : samples_)
        {
            sensor_msgs::JointState msg;
            for(auto &q : s.getJointValues())
            {
                msg.name.push_back(q.first);
                msg.position.push_back(q.second);
            }
            jointStatePub_.publish(msg);
        }
    }
}


/*--- private functions ---*/

bool ArgoMoveGroupBasePlanner::sampleCameraPoses(const Affine3d &target, ObjectTypeParams params, size_t max_num_samples, bool do_ik, ros::Duration max_time)
{
    samples_.clear();

    geometry_msgs::PoseArray posesMsg;

    // only sample around current position
    if (do_ik)
    {
        Vector3d pos_cam = scene_->getFrameTransform(cam_frame_).translation();
        Vector3d pos_tgt = target.translation();
        double d = Vector3d(pos_cam - pos_tgt).lpNorm<2>();
        params.dist_min = std::max(d - 1.25, params.dist_min);
        params.dist_max = std::min(d + 1.25, params.dist_max);
        ROS_INFO_STREAM("Sampling Dist:" << params.dist_min << " => " << params.dist_max);
    }

    // get octomap
    collision_detection::World::ObjectConstPtr octo_obj =scene_->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
    if (octo_obj == NULL)
    {
        ROS_ERROR_STREAM("Octomap object could not be retrieved from planning scene.");
        return false;
    }
    boost::shared_ptr<const octomap::OcTree> octree(dynamic_cast<const shapes::OcTree *>(octo_obj->shapes_[0].get())->octree);


    const moveit::core::JointModelGroup* group = scene_->getRobotModel()->getJointModelGroup(params.group);
    const std::vector<std::string> link_names = group->getLinkModelNames();

    Eigen::Affine3d group_root_link_transform (scene_->getFrameTransform(link_names[0]));
    int num_root_distance_rejects = 0;

    // LOOP until enough samples are generated or timeout occurs
    ros::Time timeout = ros::Time::now() + max_time;
    while (samples_.size() < max_num_samples
           && ros::Time::now() < timeout)
    {
        Affine3d _target = target;
        // draw sample
        double angleX = rand_.uniformReal(params.angle_x_low, params.angle_x_high);
        double angleY = rand_.uniformReal(params.angle_y_low, params.angle_y_high);
        double dist = rand_.uniformReal(params.dist_min, params.dist_max);
        if (params.from_back && rand_.uniformInteger(0,1))
        {
            _target = target * AngleAxisd(M_PI, Vector3d::UnitX());
        }

        //ROS_INFO_STREAM("Sample: X " << angleX << " Y " << angleY << " D " << dist);

        // generate sample pose
        CameraPose cp(_target, angleX, angleY, dist);

        double distance_from_group_root = (cp.translation() - group_root_link_transform.translation()).norm();

        if (do_ik && (distance_from_group_root > 1.4))
        {
          ++num_root_distance_rejects;
          continue;
        }

        cp.computeValue(params);

        // check if sample is valid
        if (castRay(octree, cp.translation(), _target.translation()))
        {
            bool add_cp = false;
            if (do_ik)
            {
                moveit::core::RobotState state = scene_->getCurrentState();
                kinematics::KinematicsQueryOptions opt;
                opt.return_approximate_solution = true;
                // move target few cm ahead and set this position as target for visibility constraint
                tf::poseEigenToMsg(_target * Translation3d(Vector3d(0,0,octree->getResolution()*1.5)),
                                   constraints_.visibility_constraints.begin()->target_pose.pose);
                //tf::poseEigenToMsg(_target, constraints_.visibility_constraints.begin()->target_pose.pose);
                constraints_.visibility_constraints.begin()->target_radius = params.radius;
                if (state.setFromIK(state.getJointModelGroup(params.group), cp, 0, 0.005, stateCheckerCB, opt))
                {
                    //store joint positions for this solution
                    const std::vector<std::string> &names = state.getJointModelGroup(params.group)->getActiveJointModelNames();
                    for (auto &n : names)
                    {
                        cp.addJointValue(n, state.getVariablePosition(n));
                    }
                    double d = state.distance(scene_->getCurrentState());
                    double v = 1.0 - (std::min(names.size()*M_PI, d) / (names.size()*M_PI));
                    cp.setValue(cp.getValue()*v*v);
                    //ROS_INFO("N %d CP v: %.3f d: %.3f %.3f", names.size(), cp.getValue(), d, v);
                    // add sample
                    samples_.push_back(cp);
                    add_cp = true;
                }
            }
            else
            {
                //ROS_INFO_STREAM("P " << p[0] << " " << p[1] << ": " << map_.atPosition("occupancy", p));
                samples_.push_back(cp);
                add_cp = true;
            }

            if (add_cp && cameraPoseesPub_.getNumSubscribers())
            {
                geometry_msgs::Pose pMsg;
                tf::poseEigenToMsg(cp, pMsg);
                posesMsg.poses.push_back(pMsg);
            }
        } //END IF  castRay()
    } // END LOOP
    ROS_INFO_STREAM("Generated " << samples_.size() << " samples in " << ros::Time::now() - timeout + max_time << "seconds. Rejected " << num_root_distance_rejects << " dist from root link exceed samples");

    // sort best samples at the end of the vector
    std::sort(samples_.begin(), samples_.end(), [](const CameraPose& a, const CameraPose& b){return a.getValue() < b.getValue();});

    if (cameraPoseesPub_.getNumSubscribers() && posesMsg.poses.size())
    {
        posesMsg.header.frame_id = scene_->getPlanningFrame();
        cameraPoseesPub_.publish(posesMsg);
    }

    return true;
}

bool ArgoMoveGroupBasePlanner::stateCheckerFN(moveit::core::RobotState *robot_state, const moveit::core::JointModelGroup *joint_group, const double *joint_group_variable_values)
{
    robot_state->setJointGroupPositions(joint_group, joint_group_variable_values);
    robot_state->update();
    bool collision = scene_->isStateColliding(*robot_state, joint_group->getName());
    bool constrained = scene_->isStateConstrained(*robot_state, constraints_);
//    ROS_INFO_STREAM("state checker: " << (collision ? "C" : " ") << (constrained ? " " : "V") );
    return !collision && constrained;
}

void ArgoMoveGroupBasePlanner::clearTargetArea(Affine3d target, Vector3d margin)
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
        Vector3d tmp = target.linear() * margin;
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

bool ArgoMoveGroupBasePlanner::castRay(const boost::shared_ptr<const octomap::OcTree> &octree,
                                       const Vector3d &origin, const Vector3d &target)
{
    octomap::point3d ori;
    tf::pointEigenToOctomap(origin, ori);

    octomap::point3d tgt;
    tf::pointEigenToOctomap(target, tgt);

    octomap::point3d dir = tgt - ori;

//    std::cout << "ori:" << ori.x() << " " << ori.y() << " " << ori.z() << std::endl;
//    std::cout << "tgt:" << tgt.x() << " " << tgt.y() << " " << tgt.z() << std::endl;
//    std::cout << "dir:" << dir.x() << " " << dir.y() << " " << dir.z() << std::endl;

    return !octree->castRay(ori, dir, tgt, true, dir.norm() - 1.414 * octree->getResolution());
}

bool ArgoMoveGroupBasePlanner::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest &req, plan_execution::ExecutableMotionPlan &plan)
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

void ArgoMoveGroupBasePlanner::readObjectTypes()
{
    ObjectTypeParams params;
    // default params
    params.radius = 0.1;
    params.angle_x_high = M_PI_2;
    params.angle_x_low = -M_PI_2;
    params.angle_y_high = M_PI_2;
    params.angle_y_low = -M_PI_2;
    params.dist_min = 0.25;
    params.dist_max = 10.0;
    params.from_back = true;
    params.group = "arm_with_head_group";
    object_type_parms_[DEFAULT_OBJECT_TYPE] = params;

    ROS_INFO("%s: {r: %.3f x: %.3f %.3f y: %.3f %.3f d: %.3f %.3f b: %d %s}",
             DEFAULT_OBJECT_TYPE.c_str(), params.radius,
             params.angle_x_high, params.angle_x_low, params.angle_y_high, params.angle_y_low,
             params.dist_min, params.dist_max, params.from_back, params.group.c_str());

    XmlRpc::XmlRpcValue objectTypes;
    nh_combined_planner_.getParam("object_types", objectTypes);
    ROS_ASSERT(objectTypes.getType() == XmlRpc::XmlRpcValue::TypeArray)

    for (int i = 0; i < objectTypes.size(); i++)
    {
        XmlRpc::XmlRpcValue& t = objectTypes[i];

        if (!t.hasMember("name")) continue;

        if (t.hasMember("radius")) params.radius = (double)t["radius"];
        if (t.hasMember("angle_x_high")) params.angle_x_high = (double)t["angle_x_high"];
        if (t.hasMember("angle_x_low")) params.angle_x_low = (double)t["angle_x_low"];
        if (t.hasMember("angle_y_high")) params.angle_y_high = (double)t["angle_y_high"];
        if (t.hasMember("angle_y_low")) params.angle_y_low = (double)t["angle_y_low"];
        if (t.hasMember("dist_min")) params.dist_min = (double)t["dist_min"];
        if (t.hasMember("dist_max")) params.dist_max = (double)t["dist_max"];
        if (t.hasMember("from_back")) params.from_back = (bool)t["from_back"];
        if (t.hasMember("joint_group")) params.group = std::string(t["joint_group"]);

        ROS_INFO("%s: {r: %.3f x: %.3f %.3f y: %.3f %.3f d: %.3f %.3f b: %d %s}",
                 std::string(t["name"]).c_str(), params.radius,
                 params.angle_x_high, params.angle_x_low, params.angle_y_high, params.angle_y_low,
                 params.dist_min, params.dist_max, params.from_back, params.group.c_str());

        object_type_parms_[std::string(t["name"])] = params;
        params = object_type_parms_[DEFAULT_OBJECT_TYPE];
    }
}

ObjectTypeParams ArgoMoveGroupBasePlanner::getParams(std::string object_type, std::string object_id)
{
    // get Checkpoint Type default parameters
    auto params_it = object_type_parms_.find(object_type);
    if (params_it == object_type_parms_.end())
    {
        ROS_WARN_STREAM("Unknown object type \"" << object_type << "\"! Using default params.");
        params_it = object_type_parms_.find(DEFAULT_OBJECT_TYPE);
    }
    ObjectTypeParams params = params_it->second;

    ros::NodeHandle nh("/object_tracker/objects");

    // get overrieds for current checkoint
    if (nh.hasParam(object_id + "/dial_properties/diameter"))
    {
        nh.param(object_id + "/dial_properties/diameter", params.radius, params.radius);
        params.radius *= 0.75;
    }
    if (nh.hasParam(object_id + "/view_params/radius"))
    {
        nh.param(object_id + "/view_params/radius", params.radius, params.radius);
    }
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

    ROS_INFO("%s %s: {r: %.3f x: %.3f %.3f y: %.3f %.3f d: %.3f %.3f b: %d %s}",
             object_id.c_str(), object_type.c_str(), params.radius,
             params.angle_x_high, params.angle_x_low, params.angle_y_high, params.angle_y_low,
             params.dist_min, params.dist_max, params.from_back, params.group.c_str());


    return params;
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(argo_move_group::ArgoMoveGroupBasePlanner, move_group::MoveGroupCapability)
