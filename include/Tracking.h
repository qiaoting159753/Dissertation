//
// Created by t on 8/20/17.
//

#ifndef DISSERTATION_TRACKING_H
#define DISSERTATION_TRACKING_H

#include <opencv2/core.hpp>
#include "Camera.h"
#include "Frame.h"
#include "ORBextractor.h"
#include "ORBVocabulary.h"
#include "Map.h"
#include "Viewer.h"

namespace Dissertation {

    class Frame;
    class Viewer;

    class Tracking {
    public:

        // Tracking states
        enum eTrackingState{
            SYSTEM_NOT_READY=-1,
            NO_IMAGES_YET=0,
            NOT_INITIALIZED=1,
            OK=2,
            LOST=3
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState = OK;


        bool initialized = false;
        double mThDepth;

        ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
        ORBVocabulary* mpVocabulary = nullptr;

        cv::Mat mImGray,imGrayRight;
        Camera* camera;

        Frame mCurrentFrame;
        Frame mLastFrame;

        Map mMMap;

        Viewer* mpViewer;

        Tracking(Camera* camera1);

        cv::Mat Track_Stereo(cv::Mat imgLeft,cv::Mat imgRight, double timestamp);
        void Track();
    };
}

#endif //DISSERTATION_TRACKING_H
