//
// Created by t on 8/22/17.
//

#ifndef DISSERTATION_FRAMEDRAWER_H
#define DISSERTATION_FRAMEDRAWER_H


#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>

namespace Dissertation{
    class Tracking;
    class Viewer;

    class FrameDrawer {
    public:
        FrameDrawer(Map* pMap);

        // Update info from the last processed frame.
        void Update(Tracking *pTracker);

        // Draw last processed frame.
        cv::Mat DrawFrame();

    protected:

        void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

        // Info of the frame to be drawn
        cv::Mat mIm;
        int N;
        vector<cv::KeyPoint> mvCurrentKeys;
        vector<bool> mvbMap, mvbVO;
        bool mbOnlyTracking;
        int mnTracked, mnTrackedVO;
        vector<cv::KeyPoint> mvIniKeys;
        vector<int> mvIniMatches;
        int mState;

        Map* mpMap;

        std::mutex mMutex;
    };
}


#endif //DISSERTATION_FRAMEDRAWER_H