//
// Created by t on 8/22/17.
//

#ifndef DISSERTATION_VIEWER_H
#define DISSERTATION_VIEWER_H

#include "Tracking.h"

namespace Dissertation {
    class Tracking;

    class Viewer{
        public:
        Viewer(Tracking *pTracking){};

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

    };
}
#endif //DISSERTATION_VIEWER_H
