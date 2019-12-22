//
// Created by jason on 12/7/19.
//

// tform = getExternalParameterFromPlane(plane)
//     * tformC2W = invert(tform)

#include "getExternalParameterFromPlane.h"

Eigen::Matrix3d translate2RigidMatrix(Eigen::Matrix3d R) {
    // translate2RigidMatrix translate matrix to rigid matrix
    // note: it is an experience to translate the matrix to rigid matrix. firstly
    // we translate the matrix to eul angle, then we translate euler angle to
    //matrix again.
    // R: the translating matrix

    Eigen::Vector3d euler_angle = R.eulerAngles(2, 1, 0);
    // std::cout << "yaw pitch roll = " << euler_angle.transpose() << std::endl;
    Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond q = yaw_angle * pitch_angle * roll_angle;
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    // std::cout << rotation_matrix << std::endl;
    return rotation_matrix;
}



Eigen::Vector3d getProjectedPoint(Eigen::Vector3d v, pcl::ModelCoefficients::Ptr coefficients) {
    // store coefficients in temp
    double coe0 = coefficients->values[0];
    double coe1 = coefficients->values[1];
    double coe2 = coefficients->values[2];
    double coe3 = coefficients->values[3];

    //
    Eigen::Matrix<double, 4, 4> A;
    A << coe0, coe1, coe2, 0, 1, 0, 0, coe0, 0, 1, 0, coe1, 0, 0, 1, coe2;
    // std::cout << "A" << A << std::endl;
    Eigen::Matrix<double, 4, 1> b;
    // std::cout << v(0, 0) << v(0, 1) << std::endl;
    b << -coe3, v(0), v(1), v(2);
    // std::cout << "b" << b << std::endl;

    // p = A\b
    Eigen::Vector4d p;
    p = A.inverse()*b;

    // std::cout << "p " << p << std::endl;
    // std::cout << "p.head(3): " << p.head(3) << std::endl;

    return p.head(3);
}

/*
 * getExternalParameterFromPlane
 * @para: coefficients
 * return Isometry3d (Transformation matrix)
 */
Eigen::Isometry3d getExternalParameterFromPlane (pcl::ModelCoefficients::Ptr coefficients) {

    // get world axis in camera coordinate
    Eigen::Vector3d oc(0, 0, 0);
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();;
    Eigen::Vector3d ow = getProjectedPoint(oc, coefficients);
    std::cout << "ow" << ow << std::endl;
    Eigen::Vector3d zc(0, 0, 1);
    Eigen::Vector3d yw = getProjectedPoint(zc, coefficients) - ow;
    std::cout << "yw" << yw << std::endl;

    if (yw.norm() == 0) //Euclid norm
    {
        T = Eigen::Isometry3d::Identity();
        return T;
    }

    yw = yw / yw.norm();

    // solve unit vector
    Eigen::Vector3d zw;
    zw = ow - oc;
    zw = zw / zw.norm();

    Eigen::Vector3d xw;
    xw = yw.cross(zw); // cross(yw,zw)

    // world coordinate points to camera coordinate points
    // R = [xw, yw, zw]
    Eigen::Matrix<double, 3, 3> R;
    R << xw, yw, zw;
    std::cout << "R_origin\n" << R << std::endl;

    Eigen::Vector3d t;
    t = -ow.transpose() * R;
    std::cout << "t_origin\n" << t << std::endl;

    // from matrix to euler, then euler to matrix
    R = translate2RigidMatrix(R);

    // merge T from R, t
    // Now, T is the tranformation matrix from world to camera frame
    T.rotate(R);
    T.pretranslate(t);


    // std::cout << "T_origin & T_W^{C}\n" << T.matrix() << std::endl;

    // camera coordinate points to world coordinate points
    // T = T.inverse();
    // std::cout << "T.inverse()\n" << T.matrix() << std::endl;

    return T;
}
