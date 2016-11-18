#ifndef CAMERA_POSE_H_
#define CAMERA_POSE_H_

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <map>

#include <Eigen/Geometry>

namespace argo_move_group
{

struct ObjectTypeParams
{
    double radius;
    double angle_x_high;
    double angle_x_low;
    double angle_y_high;
    double angle_y_low;
    double dist_min;
    double dist_max;
    bool from_back;
    std::string group;
};

const std::string DEFAULT_OBJECT_TYPE = "default";
typedef std::map<std::string, ObjectTypeParams> ObjectTypeParamsMap;

class CameraPose : public Eigen::Affine3d
{
public:
    CameraPose(const Eigen::Affine3d &target, double aX, double aY, double dist, const ObjectTypeParams &params);
    virtual ~CameraPose() {}

    inline double getValue() const {return value_;}
    inline void setValue(double value) {value_ = value;}

    boost::shared_array<double> getJointValues();
    void setJointValues(boost::shared_array<double> &jointValues);

    inline double getDist() const {return dist_;}
    inline double getAngleX() const {return angleX_;}
    inline double getAngleY() const {return angleY_;}

private:
    void computeValue(const ObjectTypeParams &params);

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
