
#include "decomposeMatrixT.h"
Eigen::Matrix4d T;
Eigen::Matrix3d R;
Eigen::Vector3d t;
void decomposeMatrixT(Eigen::Isometry3d transform) {
    // extract R from T, firstly translate Isometry3d to matrix4d
    T = transform.matrix();
    R = T.block(0, 0, 3, 3);
    // extract t from T
    /*
     * firstly, extract the fourth column to become vector4d, then
     * fetch head(3) to become vector3d
     *
     */
    t = (T.col(3)).head(3);
}
