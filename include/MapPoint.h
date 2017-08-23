//
// Created by t on 8/20/17.
//

#ifndef DISSERTATION_MAPPOINT_H
#define DISSERTATION_MAPPOINT_H

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <mutex>
#include <list>

namespace Dissertation {

    class Feature;
    class Frame;
    typedef Eigen::Matrix<double, 2, 3> Matrix23d;

    class MapPoint {
        public:

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            long int                    mnFirstFrame;//First Key frames' Frame ID;
            int                         nObs;      //Number of observations
            int                         mnAtImgLevel; //At which level of image pyramid
            long unsigned int           mnLastFrameSeen;
            int                         mnSucceededReproj; //How many projection I have
            int                         mnFailedReproj;    //How many failed projection
            int                         mnTrackScaleLevel; //??
            long int                    mnFirstKFid;//The First Keyframe ID;
            int                         mnVisible;
            int                         mnFound;
            long unsigned int    mnId;
            static long unsigned int    nNextId;
            long unsigned int           mnTrackReferenceForFrame;
            long unsigned int           mnLoopPointForKF;
            long unsigned int           mnFuseCandidateForKF;
            long unsigned int           mnBALocalForKF;
            long unsigned int           mnBAGlobalForKF;
            long unsigned int           mnCorrectedByKF;
            long unsigned int           mnCorrectedReference;
            bool                        mbTrackInView;
            bool                        mbHave_norm;
            bool                        mbBad;
            float                       mTrackProjX;
            float                       mTrackProjY;
            float                       mTrackProjXR;
            float                       mTrackViewCos;
            float                       mfMaxDistance;
            float                       mfMinDistance;
            cv::Mat                     mPosGBA;
            cv::Mat                     mNormalVector;
            cv::Mat                     mDescriptor;
            cv::Mat                     mWorldPos;
            Eigen::Vector3d             mvPos;
            Eigen::Vector3d             mvNorm;
            Eigen::Vector3d             mMeanNorm;
            Eigen::Matrix3d             mmNorm_info;

        //Key frame and index
        //Map*                        mpMap;
        MapPoint*                   mpReplaced;
        Frame*                      mpRefF;
        std::list<Feature*>         feature_observations;
        std::map<Frame*,size_t>     mObservations;


        std::mutex                  mMutexPos;
        std::mutex                  mMutexFeatures;
        static std::mutex           mGlobalMutex;

        MapPoint(const cv::Mat &Pos, Frame *pFrame, Feature* ftr);
        MapPoint(const cv::Mat &Pos, Frame *pFrame, const int &idxF, Feature* ftr);

        void SetWorldPos();
        void SetBadFlag();

        int                         GetIndexInKeyFrame(Frame *pKF);
        cv::Mat                     GetDescriptor();
        cv::Mat                     GetWorldPos();
        cv::Mat                     GetNormal();
        Frame*                      GetReferenceFrame();
        std::map<Frame*,size_t>     GetObservations();
        int                         GenIndexInKeyFrame();
        float                       GetMinDistanceInvariance();
        float                       GetMaxDistanceInvariance();
        float                       GetFoundRatio();
        void                        SetWorldPos(const cv::Mat &Pos);
        void                        Replace(MapPoint* pMP);
        MapPoint*                   GetReplaced();

        int                         Observations();
        void                        AddObservation(Frame* pKF,size_t idx,Feature* ftr);
        void                        EraseObservation(Frame* pKF);

        void                        IncreaseFound(int n);
        void                        IncreaseVisible(int n);

        bool                        isBad();
        bool                        IsInFrame(Frame *pKF);
        void                        ComputeDistinctiveDescriptors();
        void                        UpdateNormalAndDepth();
        //int                         PredictScale(float currentDist, Frame* pKF);
        int                         PredictScale(float currentDist, Frame* pF);
        void                        optimize(size_t n_iter);

        inline static void jacobian_xyz2uv(
                const Eigen::Vector3d& p_in_f,
                const Eigen::Matrix3d& R_f_w,
                Matrix23d& point_jac)
        {
            const double z_inv = 1.0/p_in_f[2];
            const double z_inv_sq = z_inv*z_inv;
            point_jac(0, 0) = z_inv;
            point_jac(0, 1) = 0.0;
            point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
            point_jac(1, 0) = 0.0;
            point_jac(1, 1) = z_inv;
            point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
            point_jac = - point_jac * R_f_w;
        }

        inline double norm_max(const Eigen::VectorXd & v)
        {
            double max = -1;
            for (int i=0; i<v.size(); i++)
            {
                double abs = fabs(v[i]);
                if(abs>max){
                    max = abs;
                }
            }
            return max;
        }
    };
}

#endif //DISSERTATION_MAPPOINT_H
