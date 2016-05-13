#include <argo_move_group/camera_pose.h>

/************
 *  Implementation of CameraPose
 */

using namespace argo_move_group;
using namespace Eigen;

CameraPose::CameraPose(const Affine3d &target, double aX, double aY, double dist, const CheckpointParams &params) :
    Affine3d(target *                                 // goto target pose
             AngleAxisd(aY, Vector3d::UnitY()) *      // second rotation about fixed Y-Axis
             AngleAxisd(aX, Vector3d::UnitX()) *      // first rotation about fixed X-Axis
             Translation3d(Vector3d(0, 0, dist)) *    // third move by distance in current Z-Dir
             AngleAxisd(M_PI, Vector3d::UnitX()) *    // fourth rotate about current X-Axis by 180 Degree
             AngleAxisd(-M_PI_2, Vector3d::UnitY())), // last rotate to use cam link instead of optical frame)
    angleX_(aX),
    angleY_(aY),
    dist_(dist)
{
    computeValue(params);
}

boost::shared_array<double> CameraPose::getJointValues()
{
    return jointValues_;
}

void CameraPose::setJointValues(boost::shared_array<double> &jointValues)
{
    jointValues_ = jointValues;
}



void CameraPose::computeValue(const CheckpointParams &params)
{
//    value_ = std::sqrt(std::cos(angle_horizontal) * std::cos(angle_horizontal) +
//                       std::cos(angle_vertical) * std::cos(angle_vertical));

    value_ = 1.0 - (std::pow(std::fabs(angleX_) / std::max(std::fabs(params.angle_x_high), std::fabs(params.angle_x_low)), 2) +
                    std::pow(std::fabs(angleY_) / std::max(std::fabs(params.angle_y_high), std::fabs(params.angle_y_low)), 2)) / 2.0;
}
