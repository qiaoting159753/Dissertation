//
// Created by t on 8/20/17.
//

#ifndef DISSERTATION_FRAME_H
#define DISSERTATION_FRAME_H

#include <opencv2/core.hpp>
#include <vector>
#include <sophus/se3.hpp>
#include "ORBextractor.h"
#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>
#include "Camera.h"
#include <mutex>

using namespace std;
using namespace cv;

namespace Dissertation {

    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64

    class Feature;
    class MapPoint;

    typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> ORBVocabulary;

    class Frame {
    public:
        static long unsigned int nNextId;
        long unsigned int mnId;
        double mTimeStamp;
        cv::Mat imgLeft;
        cv::Mat imgRight;
        cv::Mat DepthMap;
        Camera* camera;
        bool mbBad;
        /*------------------------------------------------ Image Pyramid ---------------------------------------------*/
        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        vector<float> mvScaleFactors;
        vector<float> mvInvScaleFactors;
        vector<float> mvLevelSigma2;
        vector<float> mvInvLevelSigma2;
        vector<cv::Mat> imgPymLeft;
        vector<cv::Mat> imgPymRight;

        /*--------------------------Bag of words, Bag of Words Vector structures.-------------------------------------*/
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;
        ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
        ORBVocabulary* mpORBvocabulary;

        /*------------------------------------------Descriptors, Features---------------------------------------------*/
        float mThDepth;
        int N;
        cv::Mat mDescriptors, mDescriptorsRight;
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;
        std::vector<bool> mvbOutlier;
        std::vector<MapPoint*> mvpMapPoints;
        vector<Feature*> mvFtrLeft;
        vector<Feature*> mvFtrRight;

        /*------------------------------------------Positions --------------------------------------------------------*/
        cv::Mat mTwc;
        cv::Mat mTcw;                                       //Camera pose with respect world
        cv::Mat mRcw;                                       //Camera rotation with respect world
        cv::Mat mtcw;                                       //Camera translation
        cv::Mat mRwc;                                       //Rotation from camera to origin
        cv::Mat mOw;                                        //Translation from camera to origin in camera frame
        cv::Mat mCw;
        Sophus::SE3<double> msTcw;
        /*-------------------------------------------- Grid-- --------------------------------------------------------*/
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
        double mnMinX = 0.0f;
        double mnMaxX;
        double mnMinY = 0.0f;
        double mnMaxY;
        static bool mbInitialComputations;

        Frame(){};
        Frame(const Frame &frame);
        Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, Camera* camera);

        /*--------------------------------------Get, Set, Update------------------------------------------------------*/

        cv::Mat GetPoseInverse();
        cv::Mat GetCameraCenter();
        void AddMapPoint(MapPoint* point, size_t idx);
        void SetPose(const cv::Mat &Tcw_);
        void UpdatePoseMatrices(){
            mRcw = mTcw.rowRange(0,3).colRange(0,3);
            mRwc = mRcw.t();
            mtcw = mTcw.rowRange(0,3).col(3);
            mOw = -mRcw.t()*mtcw;

            mTwc = cv::Mat::eye(4,4,CV_64F);
            mTwc(Range(0,3),Range(0,3)) = mRwc;
            mTwc.rowRange(0,3).col(3) = mOw;
        };

        void ExtractORB(int flag, const cv::Mat &im);
        void ComputeStereoMatches();

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        cv::Mat UnprojectStereo(const int &i);
        bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

        /*-------------------------------------------- Grid-- --------------------------------------------------------*/
        // Compute the cell of a keypoint (return false if outside the grid)
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
        vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        bool isBad();
    };
}

#endif //DISSERTATION_FRAME_H
