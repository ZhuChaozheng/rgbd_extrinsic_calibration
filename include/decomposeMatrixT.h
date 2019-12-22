#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

extern Eigen::Matrix4d T;
extern Eigen::Matrix3d R;
extern Eigen::Vector3d t;

void decomposeMatrixT(Eigen::Isometry3d transform);
