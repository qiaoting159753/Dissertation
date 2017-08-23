//
// Created by t on 8/20/17.
//

#include "../include/MapPoint.h"

#include "Converter.h"
#include "ORBmatcher.h"
#include "include/Feature.h"

using namespace std;

namespace Dissertation {

    long unsigned int MapPoint::nNextId=0;
    mutex MapPoint::mGlobalMutex;

    MapPoint::MapPoint(const cv::Mat &Pos, Frame *pRefF, Feature* ftr):
            mnFirstKFid(0), mnFirstFrame(0), nObs(0), mnTrackReferenceForFrame(0),
            mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefF(pRefF), mnVisible(1), mnFound(1), mbBad(false),
            mpReplaced(static_cast<MapPoint *>(NULL)), mfMinDistance(0), mfMaxDistance(0)
    {
        Pos.copyTo(mWorldPos);
        mNormalVector = cv::Mat::zeros(3, 1, CV_32F);
        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        //unique_lock <mutex> lock(mpMap->mMutexPointCreation);
        this->mnId = nNextId++;
        mvPos = Dissertation::Converter::toVector3d(Pos);
    }

    MapPoint::MapPoint(const cv::Mat &Pos,Frame *pFrame, const int &idxF, Feature* ftr) :
            mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
            mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
            mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefF(static_cast<Frame *>(NULL)), mnVisible(1),
            mnFound(1), mbBad(false), mpReplaced(NULL)
    {
        Pos.copyTo(mWorldPos);
        cv::Mat Ow = pFrame->GetCameraCenter();
        mNormalVector = mWorldPos - Ow;
        mNormalVector = mNormalVector / cv::norm(mNormalVector);

        cv::Mat PC = Pos - Ow;
        const float dist = cv::norm(PC);
        const int level = pFrame->mvKeys[idxF].octave;
        const float levelScaleFactor = pFrame->mvScaleFactors[level];
        const int nLevels = pFrame->mnScaleLevels;

        mfMaxDistance = dist * levelScaleFactor;
        mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

        pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

        // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
        //unique_lock <mutex> lock(mpMap->mMutexPointCreation);
        this->mnId = nNextId++;
        mvPos = Dissertation::Converter::toVector3d(Pos);
    }

    void MapPoint::SetWorldPos(const cv::Mat &Pos) {
        std::unique_lock <std::mutex> lock2(mGlobalMutex);
        unique_lock <mutex> lock(mMutexPos);
        Pos.copyTo(mWorldPos);
        mvPos = Dissertation::Converter::toVector3d(Pos);
    }

    cv::Mat MapPoint::GetWorldPos() {
        unique_lock <mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    cv::Mat MapPoint::GetNormal() {
        unique_lock <mutex> lock(mMutexPos);
        return mNormalVector.clone();
    }

    Frame* MapPoint::GetReferenceFrame() {
        unique_lock <mutex> lock(mMutexFeatures);
        return mpRefF;
    }


    void MapPoint::AddObservation(Frame *pKF, size_t idx,Feature* ftr) {
        unique_lock <mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;
        nObs+=2;
        feature_observations.push_front(ftr);
    }


    void MapPoint::EraseObservation(Frame *pKF) {
        bool bBad = false;

        {
            unique_lock <mutex> lock(mMutexFeatures);
            if (mObservations.count(pKF)) {
                int idx = mObservations[pKF];
                nObs-=2;

                mObservations.erase(pKF);

                if (mpRefF == pKF)
                    mpRefF = mObservations.begin()->first;

                // If only 2 observations or less, discard point
                if (nObs <= 2)
                    bBad = true;
            }

            for(auto it=feature_observations.begin(), ite=feature_observations.end(); it!=ite; ++it)
            {
                Feature* temp = (*it);

                if(temp->mpFrame == pKF)
                {
                    feature_observations.erase(it);
                }
            }


        }

        if (bBad)
            SetBadFlag();
    }

    map<Frame *, size_t> MapPoint::GetObservations() {
        unique_lock <mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int MapPoint::Observations() {
        unique_lock <mutex> lock(mMutexFeatures);
        return nObs;
    }

    void MapPoint::SetBadFlag() {
        map < Frame*, size_t > obs;
        {
            unique_lock <mutex> lock1(mMutexFeatures);
            unique_lock <mutex> lock2(mMutexPos);
            mbBad = true;
            obs = mObservations;
            mObservations.clear();
        }
        for (map<Frame *, size_t>::iterator mit = obs.begin(), mend = obs.end(); mit != mend; mit++) {
            Frame *pKF = mit->first;
            //pKF->EraseMapPointMatch(mit->second);
        }

        //mpMap->EraseMapPoint(this);
    }

    bool MapPoint::isBad() {
        unique_lock <mutex> lock(mMutexFeatures);
        unique_lock <mutex> lock2(mMutexPos);
        return mbBad;
    }

    void MapPoint::IncreaseVisible(int n) {
        unique_lock <mutex> lock(mMutexFeatures);
        mnVisible += n;
    }

    void MapPoint::IncreaseFound(int n) {
        unique_lock <mutex> lock(mMutexFeatures);
        mnFound += n;
    }

    float MapPoint::GetFoundRatio() {
        unique_lock <mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound) / mnVisible;
    }

    void MapPoint::ComputeDistinctiveDescriptors() {
        // Retrieve all observed descriptors
        vector <cv::Mat> vDescriptors;

        map < Frame * , size_t > observations;

        {
            unique_lock <mutex> lock1(mMutexFeatures);
            if (mbBad)
                return;
            observations = mObservations;
        }

        if (observations.empty())
            return;

        vDescriptors.reserve(observations.size());

        for (map<Frame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            Frame *pKF = mit->first;

            if (!pKF->isBad())
                vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
        }

        if (vDescriptors.empty())
            return;

        // Compute distances between them
        const size_t N = vDescriptors.size();

        float Distances[N][N];
        for (size_t i = 0; i < N; i++) {
            Distances[i][i] = 0;
            for (size_t j = i + 1; j < N; j++) {
                int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
                Distances[i][j] = distij;
                Distances[j][i] = distij;
            }
        }

        // Take the descriptor with least median distance to the rest
        int BestMedian = INT_MAX;
        int BestIdx = 0;
        for (size_t i = 0; i < N; i++) {
            vector<int> vDists(Distances[i], Distances[i] + N);
            sort(vDists.begin(), vDists.end());
            int median = vDists[0.5 * (N - 1)];

            if (median < BestMedian) {
                BestMedian = median;
                BestIdx = i;
            }
        }

        {
            unique_lock <mutex> lock(mMutexFeatures);
            mDescriptor = vDescriptors[BestIdx].clone();
        }
    }

    cv::Mat MapPoint::GetDescriptor() {
        unique_lock <mutex> lock(mMutexFeatures);
        return mDescriptor.clone();
    }

    int MapPoint::GetIndexInKeyFrame(Frame *pKF) {
        unique_lock <mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    bool MapPoint::IsInFrame(Frame *pKF) {
        unique_lock <mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));
    }

    void MapPoint::UpdateNormalAndDepth() {
        map < Frame * , size_t > observations;
        Frame *pRefF;
        cv::Mat Pos;
        {
            unique_lock <mutex> lock1(mMutexFeatures);
            unique_lock <mutex> lock2(mMutexPos);
            if (mbBad)
                return;
            observations = mObservations;
            pRefF = mpRefF;
            Pos = mWorldPos.clone();
        }

        if (observations.empty())
            return;

        cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
        int n = 0;

        for (map<Frame *, size_t>::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            Frame *pKF = mit->first;
            cv::Mat Owi = pKF->GetCameraCenter();
            cv::Mat normali = mWorldPos - Owi;
            normal = normal + normali / cv::norm(normali);
            n++;
        }

        cv::Mat PC = Pos - pRefF->GetCameraCenter();
        const float dist = cv::norm(PC);
        const int level = pRefF->mvKeys[observations[pRefF]].octave;
        const float levelScaleFactor = pRefF->mvScaleFactors[level];
        const int nLevels = pRefF->mnScaleLevels;

        {
            unique_lock <mutex> lock3(mMutexPos);
            mfMaxDistance = dist * levelScaleFactor;
            mfMinDistance = mfMaxDistance / pRefF->mvScaleFactors[nLevels - 1];
            mNormalVector = normal / n;
        }
    }

    float MapPoint::GetMinDistanceInvariance() {
        unique_lock <mutex> lock(mMutexPos);
        return 0.8f * mfMinDistance;
    }

    float MapPoint::GetMaxDistanceInvariance() {
        unique_lock <mutex> lock(mMutexPos);
        return 1.2f * mfMaxDistance;
    }

    int MapPoint::PredictScale(const float currentDist, Frame *pF) {
        float ratio;
        {
            unique_lock <mutex> lock(mMutexPos);
            ratio = mfMaxDistance / currentDist;
        }

        int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
        if (nScale < 0)
            nScale = 0;
        else if (nScale >= pF->mnScaleLevels)
            nScale = pF->mnScaleLevels - 1;

        return nScale;
    }

    void MapPoint::optimize(const size_t n_iter)
    {
        Eigen::Vector3d old_point = mvPos;
        double chi2 = 0.0;
        Eigen::Matrix3d A;
        Eigen::Vector3d b;

        for(size_t i=0; i<n_iter; i++)
        {
            A.setZero();
            b.setZero();
            double new_chi2 = 0.0;

            // compute residuals
            for(auto it=feature_observations.begin(); it!=feature_observations.end(); ++it)
            {
                Feature* ftemp = (*it);
                if(ftemp->mpFrame == nullptr)
                    continue;

                Matrix23d J;
                const Eigen::Vector3d p_in_f(ftemp->mpFrame->msTcw * mvPos);
                MapPoint::jacobian_xyz2uv(p_in_f, ftemp->mpFrame->msTcw.rotationMatrix(), J);

                Eigen::Vector3d temp1 = (ftemp)->mvBearing;
                Eigen::Vector2d b1 = temp1.head<2>()/temp1[2];

                const Eigen::Vector2d e(b1 - (temp1.head<2>()/temp1[2]));

                new_chi2 += e.squaredNorm();
                A += J.transpose() * J;
                b -= J.transpose() * e;
            }

            // solve linear system
            const Eigen::Vector3d dp(A.ldlt().solve(b));

            // check if error increased
            if((i > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dp[0]))
            {
                mvPos = old_point; // roll-back
                break;
            }

            // update the model
            Eigen::Vector3d new_point = mvPos + dp;
            old_point = mvPos;
            mvPos = new_point;
            chi2 = new_chi2;

            // stop when converged
            float EPS = 0.00000001;
            if(norm_max(dp) <= EPS)
                break;
        }
    }
}