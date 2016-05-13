#include <argo_move_group/workspace_grid.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <grid_map_proc/grid_map_polygon_tools.h>

using namespace argo_move_group;
using namespace grid_map;
using namespace Eigen;
using namespace VirtualRobot;

const std::string WorkspaceGridMap::BASE_POSE_LAYER("basePoseLayer");
const std::string WorkspaceGridMap::OCCUPANCY_LAYER("occupancyLayer");
const std::string WorkspaceGridMap::REACH_COST_LAYER("reachCostLayer");

WorkspaceGridMap::WorkspaceGridMap(std::string wsNS, Vector2d footprint)
{
    ws_ = WorkspaceRepresentationROSPtr(new WorkspaceRepresentationROS(wsNS));
    grid_map_polygon_tools::setFootprintPoly(footprint(0), footprint(1), footprint_);

    map_ = GridMap({BASE_POSE_LAYER, OCCUPANCY_LAYER, REACH_COST_LAYER});
    maxVal_ = NAN;
    maxIdx_ = Index(-1,-1);
}

WorkspaceGridMap::~WorkspaceGridMap()
{
}

void
WorkspaceGridMap::dynamicOccupancyUpdate(const nav_msgs::OccupancyGrid &updateMsg)
{
    GridMapRosConverter::fromOccupancyGrid(updateMsg, OCCUPANCY_LAYER, map_);
}

void
WorkspaceGridMap::fillData(const Affine3d &eefPoseGlobal, double yaw, bool freeRollAngle)
{
    std::vector<Affine3d> poseList{eefPoseGlobal};

    fillData(poseList, yaw, freeRollAngle);
}

void
WorkspaceGridMap::fillData(const std::vector<Affine3d> &eefPoseList, double yaw, bool freeRollAngle)
{
    if (!map_.isValid(Index(0,0), OCCUPANCY_LAYER))
    {
        ROS_ERROR("Did not yet receive dynamic occupancy map! Cannot fill GridMap.");
        return;
    }

    map_.clear(BASE_POSE_LAYER);
    maxVal_ = NAN;
    maxIdx_ = Index(-1,-1);

    Affine3d tmpPose(AngleAxisd(yaw, Vector3d::UnitZ()));
    for (GridMapIterator mapIt(map_); !mapIt.isPastEnd(); ++mapIt)
    {
        // get position of current pose
        Position3 p3;
        map_.getPosition3(OCCUPANCY_LAYER, *mapIt, p3);
        tmpPose.translation() = p3;
        //ROS_INFO("tmpPose: (%.2f %.2f) %3f", p3[0], p3[1], p3[2]);

        // check for free footprint at current pose
        bool freeFootprint = true;
        Polygon poly = grid_map_polygon_tools::getTransformedPoly(footprint_, tmpPose);
        for (PolygonIterator polyIt(map_, poly); !polyIt.isPastEnd(); ++polyIt)
        {
            if (map_.at(OCCUPANCY_LAYER, *polyIt) > 65.0)
            {
                freeFootprint = false;
                break;
            }
        }
        if (!freeFootprint)
        {
            continue;
        }

        // set workspace pose to current pose
        ws_->setWSPose(tmpPose);
        float &entry = map_.at(BASE_POSE_LAYER, *mapIt);
        entry = 0.0;
        BOOST_FOREACH(Affine3d eefPose, eefPoseList)
        {
            // get and add value of this enrty
            if (freeRollAngle)
            {
                // sum over free roll angle assuming ZYX intrinsic representation
                entry += ws_->getEntrySum(eefPose, 5);
            }
            else
            {
                entry += ws_->getEntry(eefPose);
            }
        }

        // remember max pose
        if (!std::isfinite(maxVal_) || maxVal_ < entry)
        {
            maxVal_ = entry;
            maxIdx_ = *mapIt;
        }
    } // END Loop over all grid entries

    // achieve values between 0 and 1
    map_.get(BASE_POSE_LAYER) /= maxVal_;
    Position3 p3;
    map_.getPosition3(BASE_POSE_LAYER, maxIdx_, p3);
    ROS_INFO("MaxValue: %.1f, at (%.2f %.2f) %.1f", maxVal_, p3[0], p3[1], p3[2]);
}

void
WorkspaceGridMap::fillData(const std::vector<CameraPose> &cameraPoseList, double yaw, double height, bool freeRollAngle)
{
    if (false)
    {
        ROS_ERROR("Did not yet receive dynamic occupancy map! Cannot fill GridMap.");
        return;
    }

    // reset data
    map_.clear(BASE_POSE_LAYER);
    maxVal_ = NAN;
    maxIdx_ = Index(-1,-1);

    // find appropriate submap
    Position minBB, maxBB;
    map_.getPosition(Index(0,0), minBB);
    map_.getPosition(map_.getSize(), maxBB);
    BOOST_FOREACH(CameraPose cp, cameraPoseList)
    {
        double x = cp(0,3);
        double y = cp(1,3);
        if (minBB(0) > x) minBB(0) = x;
        if (minBB(1) > y) minBB(1) = y;
        if (maxBB(0) < x) maxBB(0) = x;
        if (maxBB(1) < y) maxBB(1) = y;
    }
    bool succ;
    SubmapGeometry subMap(map_, minBB + 0.5 * (maxBB - minBB), Length(maxBB - minBB) + Length(3,3), succ);

    // workspace pose when iterating through Map
    Affine3d tmpPose(AngleAxisd(yaw, Vector3d::UnitZ()));
    tmpPose.translation()(2) = height;

    // Fill BasePose GridMap Layer
    for (SubmapIterator mapIt(subMap); !mapIt.isPastEnd(); ++mapIt)
    {
        // get position and value of current pose
        Position3 currPos;
        map_.getPosition3(REACH_COST_LAYER, *mapIt, currPos);
        if (currPos(2) < 1e-5)
        {
            continue;
        }

        tmpPose.translation()(0) = currPos(0);
        tmpPose.translation()(1) = currPos(1);
        //ROS_INFO("tmpPose: (%.2f %.2f) %3f", currPos[0], currPos[1], currPos[2]);

        // check for free footprint at current pose
        bool freeFootprint = true;
        Polygon poly = grid_map_polygon_tools::getTransformedPoly(footprint_, tmpPose);
        for (PolygonIterator polyIt(map_, poly); !polyIt.isPastEnd(); ++polyIt)
        {
            if (map_.at(REACH_COST_LAYER, *polyIt) < 1e-5)
            {
                freeFootprint = false;
                break;
            }
        }
        if (!freeFootprint)
        {
            continue;
        }

        // set workspace pose to current pose
        ws_->setWSPose(tmpPose);
        float &entry = map_.at(BASE_POSE_LAYER, *mapIt);
        entry = 0.0;
        BOOST_FOREACH(CameraPose cameraPose, cameraPoseList)
        {
            Position camPos(cameraPose.translation()(0), cameraPose.translation()(1));
            if (!map_.isInside(camPos))
                continue;

            // get and add value of this enrty
            if (freeRollAngle)
            {
                // sum over free roll angle assuming ZYX intrinsic representation
                entry += cameraPose.getValue() * map_.atPosition(REACH_COST_LAYER, camPos)
                        * ws_->getEntrySum(cameraPose, 5);
            }
            else
            {
                entry += cameraPose.getValue() * map_.atPosition(REACH_COST_LAYER, camPos)
                        * ws_->getEntry(cameraPose);
            }
        }
        // multiply rating with base placement costs (squared to give higher weight)
        entry *= currPos(2);// * currPos(2);

        // remember max pose
        if ( entry > 1e-5 &&
            (!std::isfinite(maxVal_) || maxVal_ < entry) )
        {
            maxVal_ = entry;
            maxIdx_ = *mapIt;
        }
    } // END Loop over all grid entries

    // achieve values between 0 and 1
    map_.get(BASE_POSE_LAYER) /= maxVal_;
    Position3 p3;
    map_.getPosition3(BASE_POSE_LAYER, maxIdx_, p3);
    ROS_INFO("MaxValue: %.1f, at (%.2f %.2f) %.1f", maxVal_, p3[0], p3[1], p3[2]);
}

float
WorkspaceGridMap::getValue(Affine3d poseGlobal) const
{
    return 0.0;
}

float
WorkspaceGridMap::getValue(float x, float y) const
{
    if (map_.isInside(Position(x, y)))
    {
        return map_.atPosition(BASE_POSE_LAYER, Position(x, y));
    }

    ROS_ERROR("Position not in GridMap!");
    return 0.0;
}

float
WorkspaceGridMap::getMaxPosition(Vector2d &pos)
{
    if (!std::isfinite(maxVal_))
    {
        ROS_WARN("There are no Entries in this Grid!");
        return -1;
    }

    map_.getPosition(maxIdx_, pos);
    return maxVal_;
}

nav_msgs::OccupancyGrid
WorkspaceGridMap::getGridMsg(std::string layer)
{
    nav_msgs::OccupancyGrid gridMsg;
    grid_map::GridMapRosConverter::toOccupancyGrid(map_, layer, 0.0, 1.0, gridMsg);

    return gridMsg;
}

void
WorkspaceGridMap::computeReachCostMap(const Vector2d &origin)
{
    Index originIdx;
    if (!map_.getIndex(origin, originIdx))
    {
        ROS_ERROR("Start position for flood fill outside of GridMap: (%.2f %.2f)", origin(0), origin(1));
        return;
    }

    // get reachable area
    cv::Mat occupancyMat;
    GridMapRosConverter::toCvImage(map_, OCCUPANCY_LAYER, occupancyMat);

    cv::Mat occupancyMatMono;
    cv::cvtColor(occupancyMat, occupancyMatMono, CV_BGRA2GRAY);

    cv::Mat reachableMask = cv::Mat::zeros(occupancyMatMono.rows + 2, occupancyMatMono.cols + 2, CV_8U);

    cv::Point seed(originIdx(1), originIdx(0));
    cv::floodFill(occupancyMatMono, reachableMask, seed, 1, NULL, cv::Scalar(), cv::Scalar(), 4 + (1 << 8) + (cv::FLOODFILL_MASK_ONLY));

    // use linear filtering for generating a costMap
    u_int kSize = 24;
    cv::Mat reachCostMat;
    cv::Mat kernel = cv::Mat::ones(kSize, kSize, CV_64F) / (kSize * kSize);
    cv::filter2D(reachableMask, reachCostMat, CV_64F, kernel);
    //cv::Mat kernel = cv::getGaussianKernel(kSize, 10, CV_64F);
    //cv::sepFilter2D(reachableMask, reachCostMat, CV_64F, kernel, kernel);

    // normalize and square non zero elements
    cv::Mat reachCostMatTmp;
    cv::normalize(reachCostMat, reachCostMatTmp, 0.0, 1.0, cv::NORM_MINMAX, -1, reachableMask);
    cv::pow(reachCostMatTmp, 3.0, reachCostMat);

    // fill GridMap Layer with generated CostMap
    cv_bridge::CvImage cvImage;
    cvImage.encoding = sensor_msgs::image_encodings::MONO8;
    reachCostMat.convertTo(reachCostMatTmp, CV_8U, 255.0);
    cvImage.image = reachCostMatTmp(cv::Rect(1, 1, occupancyMatMono.cols, occupancyMatMono.rows));
    sensor_msgs::ImagePtr rosImage = cvImage.toImageMsg();
    GridMapRosConverter::addLayerFromImage(*rosImage.get(), REACH_COST_LAYER, map_);
}
