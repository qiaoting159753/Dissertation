//
// Created by t on 8/22/17.
//

#ifndef DISSERTATION_MAPDRAWER_H
#define DISSERTATION_MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"Frame.h"
#include<pangolin/pangolin.h>
#include<mutex>

namespace Dissertation {
    class MapDrawer {
        public:
            MapDrawer(Map *pMap);

            Map *mpMap;

            void DrawMapPoints();

            void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

            void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

            void SetCurrentCameraPose(const cv::Mat &Tcw);

            void SetReferenceKeyFrame(Frame *pKF);

            void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

        private:

            float mKeyFrameSize;
            float mKeyFrameLineWidth;
            float mGraphLineWidth;
            float mPointSize;
            float mCameraSize;
            float mCameraLineWidth;

            cv::Mat mCameraPose;

            std::mutex mMutexCamera;
    };
}

#endif //DISSERTATION_MAPDRAWER_H
