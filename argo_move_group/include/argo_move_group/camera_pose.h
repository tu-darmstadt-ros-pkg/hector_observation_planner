#ifndef CAMERA_POSE_H_
#define CAMERA_POSE_H_

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <map>

#include <Eigen/Geometry>

namespace argo_move_group
{

struct CheckpointParams
{
    double angle_x_high;
    double angle_x_low;
    double angle_y_high;
    double angle_y_low;
    double dist_min;
    double dist_max;
    bool from_back;
    std::string group;
};

const u_int8_t DEFAULT_OBJECT_TYPE = 255;
typedef std::map<u_int8_t, CheckpointParams> CheckpointParamsMap;

class CameraPose : public Eigen::Affine3d
{
public:
    CameraPose(const Eigen::Affine3d &target, double aX, double aY, double dist, const CheckpointParams &params);
    virtual ~CameraPose() {}

    inline double getValue() const {return value_;}
    inline void setValue(double value) {value_ = value;}

    boost::shared_array<double> getJointValues();
    void setJointValues(boost::shared_array<double> &jointValues);

    inline double getDist() const {return dist_;}
    inline double getAngleX() const {return angleX_;}
    inline double getAngleY() const {return angleY_;}

private:
    void computeValue(const CheckpointParams &params);

private:
    double value_;

    // defining variables
    double dist_;
    double angleX_;
    double angleY_;

    // IK solution
    boost::shared_array<double> jointValues_;
};

typedef boost::shared_ptr<CameraPose> CameraPosePtr;
typedef boost::shared_ptr<const CameraPose> CameraPoseConstPtr;

}

#endif // CAMERA_POSE_H_
