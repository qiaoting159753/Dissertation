//
// Created by t on 8/21/17.
//

#ifndef DISSERTATION_MAP_H
#define DISSERTATION_MAP_H


#include "MapPoint.h"
#include "Frame.h"
#include <set>
#include <mutex>

namespace Dissertation {
    class Map {
    public:
        Map();

        void AddFrame(Frame *pKF);

        void AddMapPoint(MapPoint *pMP);

        void EraseMapPoint(MapPoint *pMP);

        void EraseFrame(Frame *pKF);

        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        std::vector<Frame *> GetAllFrames();

        std::vector<MapPoint *> GetAllMapPoints();

        std::vector<MapPoint *> GetReferenceMapPoints();

        long unsigned int MapPointsInMap();

        long unsigned FramesInMap();

        long unsigned int GetMaxKFid();

        void clear();

        vector<Frame *> mvpKeyFrameOrigins;

        void Display();
        //std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        //std::mutex mMutexPointCreation;

        std::set<MapPoint *> mspMapPoints;
        std::set<Frame *> mspKeyFrames;
        std::vector<MapPoint *> mvpReferenceMapPoints;

        long unsigned int mnMaxKFid;
        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;

    };
}

#endif //DISSERTATION_MAP_H
