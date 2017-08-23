//
// Created by t on 8/20/17.
//

#ifndef DISSERTATION_FEATURE_H
#define DISSERTATION_FEATURE_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>


using namespace Eigen;

namespace Dissertation {

    class Frame;
    class MapPoint;

    class Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Frame* mpFrame; // associate frame
        MapPoint* mpPoint; // associate point
        Eigen::Vector2d mvGrad = Eigen::Vector2d(1.0,0.0);
        Eigen::Vector3d mvBearing;
        cv::KeyPoint mvUV;

        Feature(Frame* _frame, cv::KeyPoint _px, Vector3d _bearing):
                mpFrame(_frame),
                mvUV(_px),
                mvBearing(_bearing),
                mpPoint(nullptr),
                mvGrad(1.0,0.0)
        {};

        Feature(Frame* _frame, MapPoint* _point, cv::KeyPoint _px, Vector3d _f):
                mpFrame(_frame),
                mpPoint(_point),
                mvUV(_px),
                mvBearing(_f),
                mvGrad(1.0,0.0)
        {};
    };
};


#endif //DISSERTATION_FEATURE_H
