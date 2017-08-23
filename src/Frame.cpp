//
// Created by t on 8/20/17.
//

#include "../include/Frame.h"
#include <thread>
#include "ORBmatcher.h"
#include <Converter.h>
#include "QuasiDenseStereo.h"
#include "Feature.h"

namespace Dissertation
{
    long unsigned int Frame::nNextId=0;
    bool Frame::mbInitialComputations=true;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    Frame::Frame(const Frame &frame):
            mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
            mTimeStamp(frame.mTimeStamp), mThDepth(frame.mThDepth),
            N(frame.N), mvKeys(frame.mvKeys), mvKeysRight(frame.mvKeysRight), mvuRight(frame.mvuRight), mvDepth(frame.mvDepth),
            mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec), mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
            mvpMapPoints(frame.mvpMapPoints),
            mvbOutlier(frame.mvbOutlier), mnId(frame.mnId), mnScaleLevels(frame.mnScaleLevels),
            mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
            mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
            mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),camera(frame.camera),
            imgLeft(frame.imgLeft),imgRight(frame.imgRight),imgPymLeft(frame.imgPymLeft),imgPymRight(frame.imgPymRight),
            mvFtrLeft(frame.mvFtrLeft),mvFtrRight(frame.mvFtrRight),DepthMap(frame.DepthMap)

    {
        msTcw = frame.msTcw;
        for(int i=0;i<FRAME_GRID_COLS;i++)
            for(int j=0; j<FRAME_GRID_ROWS; j++)
                mGrid[i][j]=frame.mGrid[i][j];
        if(!frame.mTcw.empty())
        {mTcw = frame.mTcw.clone();UpdatePoseMatrices();}
    }

    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc,Camera* camera) :
            mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight),
            mTimeStamp(timeStamp)
    {
        // Define object to store the stitched image
        this->camera = camera;
        mnMinX = camera->width;
        mnMinY = camera->height;
        this->imgLeft = imLeft.clone();
        this->imgRight = imRight.clone();
        mnId=nNextId++;//Why?

        mnScaleLevels = mpORBextractorLeft->GetLevels();mfScaleFactor = mpORBextractorLeft->GetScaleFactor();mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors(); mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors(); mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
        thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
        thread threadRight(&Frame::ExtractORB,this,1,imRight);
        threadLeft.join();threadRight.join();

        N = mvKeys.size();
        if(mvKeys.empty())return;

        ComputeStereoMatches();

        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N,false);

        //Feature to Grid
        if(mbInitialComputations)
        {
            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
            mbInitialComputations=false;
        }

        Eigen::Matrix4d temp2;
        temp2<< 1,0,0,0,
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;
        msTcw = Sophus::SE3<double>(temp2);

        AssignFeaturesToGrid();
    }

    void Frame::ExtractORB(int flag, const cv::Mat &im)
    {
        if(flag==0)
        {
            (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors);
            for(int i = 0; i < mpORBextractorLeft->mvImagePyramid.size();i++)
            {
                cv::Mat temp = mpORBextractorLeft->mvImagePyramid[i].clone();
                imgPymLeft.push_back(temp);
            }

            for(int i = 0; i < mvKeys.size();i++)
            {
                Eigen::Vector3d bearing = camera->cam2world(mvKeys[i].pt.x,mvKeys[i].pt.y);
                Feature* temp = new Feature(this,mvKeys[i],bearing);
                mvFtrLeft.push_back(temp);
            }
        }
        else{
            (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
            for(int i = 0; i < mpORBextractorRight->mvImagePyramid.size();i++)
            {
                cv::Mat temp = mpORBextractorRight->mvImagePyramid[i].clone();
                imgPymRight.push_back(temp);
            }
            for(int i = 0; i < mvKeysRight.size();i++)
            {
                Eigen::Vector3d bearing = camera->cam2world(mvKeysRight[i].pt.x,mvKeysRight[i].pt.y);
                Feature* temp = new Feature(this,mvKeysRight[i],bearing);
                mvFtrRight.push_back(temp);
            }
        }
    }

    void Frame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);

        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid[i][j].reserve(nReserve);


        for(int i=0;i<N;i++)
        {
            const cv::KeyPoint &kp = mvKeys[i];
            int nGridPosX, nGridPosY;

            if(PosInGrid(kp,nGridPosX,nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        pMP->mbTrackInView = false;
        cv::Mat P = pMP->GetWorldPos(); const cv::Mat Pc = mRcw*P+mtcw; const float &PcX = Pc.at<float>(0);const float &PcY= Pc.at<float>(1); const float &PcZ = Pc.at<float>(2);
        // Check positive depth
        if(PcZ<0.0f)return false;
        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ; const float u=camera->fx*PcX*invz+camera->cx; const float v=camera->fy*PcY*invz+camera->cy;
        if(u<mnMinX || u>mnMaxX)return false;
        if(v<mnMinY || v>mnMaxY)return false;
        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance(); const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P-mOw; const float dist = cv::norm(PO);
        if(dist<minDistance || dist>maxDistance)return false;
        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal(); const float viewCos = PO.dot(Pn)/dist;
        if(viewCos<viewingCosLimit)return false;
        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);
        // Data used by the tracking
        pMP->mbTrackInView = true;pMP->mTrackProjX = u;pMP->mTrackProjXR = u - camera->mbf*invz;pMP->mTrackProjY = v;pMP->mnTrackScaleLevel= nPredictedLevel;pMP->mTrackViewCos = viewCos;
        return true;
    }

    void Frame::SetPose(const cv::Mat &Tcw_)
    {
        //unique_lock<mutex> lock(mMutexPose);
        Tcw_.copyTo(mTcw);
        mRcw = Tcw_.rowRange(0,3).colRange(0,3);
        mtcw = Tcw_.rowRange(0,3).col(3);
        mRwc = mRcw.t();
        mOw = -mRwc*mtcw;

        mTwc = cv::Mat::eye(4,4,Tcw_.type());
        mRwc.copyTo(mTwc.rowRange(0,3).colRange(0,3));
        mOw.copyTo(mTwc.rowRange(0,3).col(3));
        cv::Mat center = (cv::Mat_<float>(4,1) << ((float)(camera->mb)/2.0), 0.0 , 0.0, 1.0);
        mCw = mTwc*center;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            return vIndices;

        const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            return vIndices;

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if(vCell.empty())
                    continue;

                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeys[vCell[j]];
                    if(bCheckLevels)
                    {
                        if(kpUn.octave<minLevel)
                            continue;
                        if(maxLevel>=0)
                            if(kpUn.octave>maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);
        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            return false;
        return true;
    }

    void Frame::ComputeStereoMatches() {
        cout << "Feature detected: " << N << endl;
//        QuasiDenseStereo qds;
//        //Texture threshold
//        qds.Param.Tt = 120;
//        qds.Param.BorderX = 40;
//        qds.Param.BorderY = 40;
//        //Searching area
//        qds.Param.N = 15;
//        //Correlation
//        qds.Param.Ct = 0.90;
//        //Gradient searching threshold
//        qds.Param.Dg = 2;
//        qds.Param.WinSizeY = 15;
//        qds.Param.WinSizeX = 15;
//        qds.process(imgLeft, imgRight);
//
//        std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> matches;
//        matches = qds.AssignDispToImage(DepthMap);

        mvuRight = vector<float>(N,-1.0f);
        mvDepth = vector<float>(N,-1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        //Assign keypoints to row table,candidates list
        vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

        for(int i=0; i<nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for(int iR=0; iR<Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY+r);
            const int minr = floor(kpY-r);

            for(int yi=minr;yi<=maxr;yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = camera->mb;
        const float minD = 0;
        const float maxD = camera->mbf/minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(N);

        int counter = 0;
        for(int iL=0; iL<N; iL++)
        {
            const cv::KeyPoint &kpL = mvKeys[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if(vCandidates.empty())
                continue;

            const float minU = uL-maxD;
            const float maxU = uL-minD;

            if(maxU<0)
                continue;

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptors.row(iL);

            // Compare descriptor to right keypoints
            for(size_t iC=0; iC<vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                    continue;

                const float &uR = kpR.pt.x;

                if(uR>=minU && uR<=maxU)
                {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                    if(dist<bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if(bestDist<thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x*scaleFactor);
                const float scaledvL = round(kpL.pt.y*scaleFactor);
                const float scaleduR0 = round(uR0*scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
                IL.convertTo(IL,CV_32F);
                IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2*L+1);

                const float iniu = scaleduR0+L-w;
                const float endu = scaleduR0+L+w+1;
                if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for(int incR=-L; incR<=+L; incR++)
                {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                    IR.convertTo(IR,CV_32F);
                    IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                    float dist = cv::norm(IL,IR,cv::NORM_L2);
                    if(dist<bestDist)
                    {
                        bestDist =  dist;
                        bestincR = incR;
                    }

                    vDists[L+incR] = dist;
                }

                if(bestincR==-L || bestincR==L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L+bestincR-1];
                const float dist2 = vDists[L+bestincR];
                const float dist3 = vDists[L+bestincR+1];

                const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

                if(deltaR<-1 || deltaR>1)
                    continue;

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
                float disparity = (uL-bestuR);

                if(disparity>=minD && disparity<maxD)
                {
                    if(disparity<=0)
                    {
                        disparity=0.01;
                        bestuR = uL-0.01;
                    }
                    mvDepth[iL]=camera->mbf/disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.push_back(pair<int,int>(bestDist,iL));
                }
            }
        }

        sort(vDistIdx.begin(),vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size()/2].first;
        const float thDist = 1.5f*1.4f*median;

        for(int i=vDistIdx.size()-1;i>=0;i--)
        {
            if(vDistIdx[i].first<thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second]=-1;
                mvDepth[vDistIdx[i].second]=-1;
            }
        }
    }

    cv::Mat Frame::UnprojectStereo(const int &i)
    {
        const float z = mvDepth[i];
        if(z>0)
        {
            const float u = mvKeys[i].pt.x;
            const float v = mvKeys[i].pt.y;
            const float x = (u-camera->cx)*z*camera->invfx;
            const float y = (v-camera->cy)*z*camera->invfy;

            cv::Mat x3Dc;
            //This x,y,z is in the camera's coordinate
            x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
            //Rwc is from world to camera
            //Ow is the camera center.
            cv::Mat temp = mRwc*x3Dc+mOw;

            return temp;
        }
        else
            return cv::Mat();
    }

    cv::Mat Frame::GetCameraCenter()
    {
        //unique_lock<mutex> lock(mMutexPose);
        return mOw.clone();
    }

    bool Frame::isBad()
    {
        //unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    cv::Mat Frame::GetPoseInverse()
    {
        //unique_lock<mutex> lock(mMutexPose);
        return mTwc.clone();
    }

    void Frame::AddMapPoint(MapPoint *point, size_t idx) {
        mvpMapPoints[idx] = point;
    }
}