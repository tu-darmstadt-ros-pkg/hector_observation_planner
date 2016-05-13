#ifndef TF_OCTOMAP_EIGEN_H_
#define TF_OCTOMAP_EIGEN_H_

#include <Eigen/Geometry>
#include <octomap/octomap.h>

namespace tf
{

inline void pointEigenToOctomap(const Eigen::Vector3d &eigPoint, octomap::point3d &ocPoint)
{
    ocPoint.x() = eigPoint[0];
    ocPoint.y() = eigPoint[1];
    ocPoint.z() = eigPoint[2];
}

inline void pointOctomapToEigen(const octomap::point3d &ocPoint, Eigen::Vector3d &eigPoint)
{
    eigPoint[0] = ocPoint.x();
    eigPoint[1] = ocPoint.y();
    eigPoint[2] = ocPoint.z();
}

}

#endif // TF_OCTOMAP_EIGEN_H_
