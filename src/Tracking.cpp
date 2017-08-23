//
// Created by t on 8/20/17.
//

#include <include/Feature.h>
#include "../include/Tracking.h"
#include "ORBVocabulary.h"
#include "ORBmatcher.h"
#include "Viewer.h"
#include <thread>
namespace Dissertation {

    Tracking::Tracking(Camera* camera1):camera(camera1)
    {
        int nFeatures = 2000;
        float fScaleFactor = 1.2;
        int nLevels = 8;
        int fIniThFAST = 20;
        int fMinThFAST = 7;
        mState = NO_IMAGES_YET;

        mMMap = Map();
        mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        mThDepth = (camera1->mbf)*(float)camera1->ThDepth/camera1->fx;

        //std::thread* mptViewer;
        //mpViewer = new Viewer(mpFrameDrawer,mpMapDrawer,this);
        //mptViewer = new thread(&Viewer::Run, mpViewer);
    }

    cv::Mat Tracking::Track_Stereo(cv::Mat imgLeft, cv::Mat imgRight, double timestamp)
    {
        //RGB2Gray
        mImGray = imgLeft;
        cv::Mat imGrayRight = imgRight;
        bool mbRGB = true;
        if(mImGray.channels()==3)
        {
            if(mbRGB)
            {
                cvtColor(mImGray,mImGray,CV_RGB2GRAY);
                cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
            }
            else
            {
                cvtColor(mImGray,mImGray,CV_BGR2GRAY);
                cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
            }
        }
        else if(mImGray.channels()==4)
        {
            if(mbRGB)
            {
                cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
                cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
            }
            else
            {
                cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
                cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
            }
        }

        vector<cv::KeyPoint> mvKeys,mvKeys2;
        cv::Mat mDescriptors,mDescriptors2;
        (*mpORBextractorLeft)(mImGray, cv::Mat(), mvKeys, mDescriptors);
        (*mpORBextractorRight)(imGrayRight, cv::Mat(), mvKeys, mDescriptors);

        cout << camera->width << endl;
        mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpVocabulary,camera);
        Track();

        cv::Mat re = cv::Mat::eye(3,3,CV_32F);

        return re;
    }

    void Tracking::Track()
    {
        //Build the first frame
        if (!initialized)
        {
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            mMMap.AddFrame(&mCurrentFrame);

            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];

                if(z>0)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    Eigen::Vector3d temp;
                    temp << (double)x3D.at<float>(0,0),(double)x3D.at<float>(0,1),(double)x3D.at<float>(0,2);
                    Feature* ftemp = mCurrentFrame.mvFtrLeft[i];
                    MapPoint* pNewMP = new MapPoint(x3D,&mCurrentFrame,ftemp);
                    pNewMP->AddObservation(&mCurrentFrame,i,ftemp);
                    mCurrentFrame.AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mMMap.AddMapPoint(pNewMP);
                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }

            mLastFrame = Frame(mCurrentFrame);
            mMMap.AddFrame(&mCurrentFrame);
            cout << "cccccccccccccccccccccccccccccc" << endl;
            mpMapDrawer->SetCurrentCameraPose(cv::Mat::eye(4,4,CV_64F));
            mState=OK;
//            mnLastKeyFrameId=mCurrentFrame.mnId;
//            mpLastKeyFrame = pKFini;
//
//            mvpLocalKeyFrames.push_back(pKFini);
//            mvpLocalMapPoints=mpMap->GetAllMapPoints();
//
//            mpReferenceKF = pKFini;
//            mCurrentFrame.mpReferenceKF = pKFini;
//
//            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
//
//            mpMap->mvpKeyFrameOrigins.push_back(pKFini);
//
//            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
//
            mState=OK;

            initialized = true;
            return;
        }
        else
        {
            mState=OK;
            ORBmatcher matcher(0.9,true);
            //matcher.SearchByProjection(mCurrentFrame,mLastFrame,0.8,false);

        }
        
        cout << "All points: " << mMMap.GetAllMapPoints().size() << " All Frame: " << mMMap.GetAllFrames().size() << endl;
        //
    }
}