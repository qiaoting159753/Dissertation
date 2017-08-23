//
// Created by t on 8/22/17.
//

#ifndef DISSERTATION_QUASIDENSE_H
#define DISSERTATION_QUASIDENSE_H



#include <vector>
#include <queue>
#include <stdio.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#define BUFFER_SIZE		 4
#define NUM_MAX_FEATURES 500
#define NO_MATCH 0

using namespace std;

namespace Dissertation {
// Algorithm properties
    struct PropagationParameters {
        int WinSizeX;        // similarity window
        int WinSizeY;
        int aggWinSizeX;
        int aggWinSizeY;
        int BorderX;        // border to ignore
        int BorderY;
        float Ct;            // correlation threshold
        float Tt;            // texture threshold
        float Et;            // epipolar threshold
        int N;            // neighbourhood size
        int Dg;            // disparity gradient threshold
        int neighboursX;
        int neighboursY;
        bool aggregate;
        float minCostThreshold;
        bool savePropagationImages;
    };

// A basic match structure
    struct Match {
        cv::Point p0;
        cv::Point p1;
        float corr;

        bool operator<(const Match &rhs) const {
            return this->corr < rhs.corr;
        }
    };


    class QuasiDenseStereo {
    public:
        QuasiDenseStereo(void);

        ~QuasiDenseStereo(void);

        void initialize(cv::Size size);

        // the work
        void sparseMatching(cv::Mat *imgL, cv::Mat *imgR);

        void quasiDenseMatching(cv::Mat *imgL, cv::Mat *imgR, std::vector<cv::Point2f> *pointsL,
                                std::vector<cv::Point2f> *pointsR,
                                std::vector<uchar> *featureStatus);

        void process(cv::Mat imgL, cv::Mat imgR);

        // correlation function
        void buildTextureDescriptor(cv::Mat *img, int *descriptor);

        float iZNCC_c1(cv::Point p0, cv::Point p1, int wx = 1, int wy = 1);

        // access methods
        cv::Point getMatch(int x, int y);

        // return the matching point for a (x,y) cooridinate in the ref image
        void saveDisparityImage(char *name, int numDisparityLevels = -1, bool overlay = false);

        std::pair<std::vector<cv::Point>, std::vector<cv::Point>>
        getDisparityImage(cv::Mat &temp, cv::Mat *img, int numDisparityLevels = -1, bool overlay = false);

        std::pair<vector<cv::Point2f>, vector<cv::Point2f>> AssignDispToImage(cv::Mat &temp);

        cv::Mat mImageL;
        cv::Mat mImageR;
        cv::Mat mIntegralImageL;
        cv::Mat mIntegralImageR;
        cv::Mat mIntegralImageSqL;
        cv::Mat mIntegralImageSqR;

        std::vector<uchar> mStatusL;
        std::vector<uchar> mStatusR;
        std::vector<cv::Point2f> mPoints2DL;
        std::vector<cv::Point2f> mPoints2DR;

        std::vector<cv::Point> mRefMap;                // Reference image matching map
        std::vector<cv::Point> mMtcMap;                // Match image matching map

        int *mRefTextureDesc;                // Texture descriptor
        int *mDstTextureDesc;
        int MaxTextureVal;                    // Texture vals
        int MinTextureVal;

        unsigned char *data0;
        unsigned char *data1;

        int *sum0;
        int *sum1;

        double *ssum0;
        double *ssum1;
        int Step, Steps;

        float *costSpace, *costSpaceTemp;
        int costWidth, costHeight, costDepth;

        // buffer swaps
        char curr, past;

        PropagationParameters Param;
    };

}

#endif //DISSERTATION_QUASIDENSE_H
