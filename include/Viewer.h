//
// Created by t on 8/22/17.
//

#ifndef DISSERTATION_VIEWER_H
#define DISSERTATION_VIEWER_H
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include <mutex>

namespace Dissertation {
    class Tracking;
    class FrameDrawer;

    class Viewer{
    public:
        Viewer(FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();

        private:

        bool Stop();


        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;
        Tracking* mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

    };
}
#endif //DISSERTATION_VIEWER_H
