//
// Created by jason on 12/8/19.
//

#ifndef RGBDEXTRINSICCALIBRATION_GETCAMERAORIENTATION_H
#define RGBDEXTRINSICCALIBRATION_GETCAMERAORIENTATION_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Geometry>

double getCameraOrientation(Eigen::Matrix3d R);

#endif //RGBDEXTRINSICCALIBRATION_GETCAMERAORIENTATION_H
