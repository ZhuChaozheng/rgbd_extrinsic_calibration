//
// Created by jason on 12/8/19.
//
/*
 * getCameraOrientation
 * @para: R
 * return az
 */
#include "getCameraOrientation.h"

double getCameraOrientation(Eigen::Matrix3d R) {

    Eigen::Vector3d nw (0, 0, 1); // the ground norm vector
    Eigen::Vector3d zw (0, 0, 1); // orientation

    double az;
    az = atan2((nw.cross(zw)).norm(), nw.dot(zw)); // cross(nw,zw)

    Eigen::Vector3d xw (1, 0, 0);
    double ax;
    ax = atan2((nw.cross(xw)).norm(), nw.dot(zw)) - M_PI/2;

    return az;

}
