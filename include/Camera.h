//
// Created by t on 8/20/17.
//

#ifndef DISSERTATION_CAMERA_H
#define DISSERTATION_CAMERA_H



#include <Eigen/Dense>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

namespace Dissertation{

    class Camera
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        double fx;
        double fy;
        double invfx;
        double invfy;
        double cx;
        double cy;
        double d0;
        double d1;
        double d2;
        double d3;
        double d4;
        double width;
        double height;
        double ThDepth;
        //# stereo baseline times fx
        //Camera.bf: 387.5744
        //# Close/Far threshold. Baseline times.
        //ThDepth: 40

        double mbf;// Stereo baseline in meters.
        double mb;

        bool need_dist = false;
        cv::Mat K,K_inv;
        cv::Mat undist_map1_, undist_map2_;
        cv::Mat dist_K, dist_D;

        Camera(double width, double height, double fx, double fy, double cx, double cy,
               double d0, double d1, double d2, double d3, double d4,double ThDepth, double bf):
                width(width),height(height),fx(fx), fy(fy), cx(cx), cy(cy),
                d0(d0),d1(d1),d2(d2),d3(d3),d4(d4),ThDepth(ThDepth),mbf(bf)
        {
            dist_D = (cv::Mat_<double>(1, 5) << d0, d1, d2, d3, d4);
            dist_K = (cv::Mat_<float>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
            cv::initUndistortRectifyMap(dist_K, dist_D, cv::Mat_<double>::eye(3,3), dist_K, cv::Size(width, height), CV_16SC2, undist_map1_, undist_map2_);
            K = (cv::Mat_<double>(3,3)<< fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
            K_inv = K.inv();
            invfx = 1.0/fx;
            invfy = 1.0/fy;
            mb = mbf / fx;
            need_dist = (d0 > 0.0000001);
        };

        ~Camera(){};

        inline Eigen::Vector2d project2d(Eigen::Vector3d& v)
        {
            return v.head<2>()/v[2];
        }

        Eigen::Vector3d cam2world(const double& u, const double& v) const
        {
            Eigen::Vector3d xyz;
            if(!need_dist)
            {
                xyz[0] = (u - cx)/fx;
                xyz[1] = (v - cy)/fy;
                xyz[2] = 1.0;
            }
            else
            {
                cv::Point2f uv(u,v), px;
                const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
                cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
                cv::undistortPoints(src_pt, dst_pt, dist_K, dist_D);
                xyz[0] = px.x;
                xyz[1] = px.y;
                xyz[2] = 1.0;
            }
            return xyz.normalized();
        }

        Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz) const
        {
            return xyz.head<2>()/xyz[2];
        }

        Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const
        {
            Eigen::Vector2d px;
            if(!need_dist)
            {
                px[0] = fx*uv[0] + cx;
                px[1] = fy*uv[1] + cy;
            }
            else
            {
                double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
                x = uv[0];
                y = uv[1];
                r2 = x*x + y*y;
                r4 = r2*r2;
                r6 = r4*r2;
                a1 = 2*x*y;
                a2 = r2 + 2*x*x;
                a3 = r2 + 2*y*y;
                cdist = 1 + d0*r2 + d1*r4 + d4*r6;
                xd = x*cdist + d2*a1 + d3*a2;
                yd = y*cdist + d2*a3 + d3*a1;
                px[0] = xd*fx + cx;
                px[1] = yd*fy + cy;
            }
            return px;
        }
        virtual double errorMultiplier2() const
        {
            return fabs(fx);
        }

        virtual double errorMultiplier() const
        {
            return fabs(4.0*fx*fy);
        }
    };
}

#endif //DISSERTATION_CAMERA_H
